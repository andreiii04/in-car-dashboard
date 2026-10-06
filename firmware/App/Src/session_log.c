//
// session log - implementation
//
// - f_mount with the immediate flag runs the card init straight away, so a
//   missing card fails here and not on the first write
// - the file is one JSON document: a 512-byte prologue, then one 2 KB block
//   per drive. the block count is the file size minus the prologue, over the
//   block size, so finding this drive's slot needs no scanning
// - a new slot is claimed in two steps: the block before it has its closing
//   brackets rewritten into a comma, then this drive's block is written and
//   closes the document. a cut between the two leaves the previous block
//   ending in a comma, and the next boot claims the same slot again and
//   repairs it - the order is chosen so that it heals itself
// - the session number is the slot index plus one, checked against the number
//   stored in the last block; a slot claimed once per power cycle is reused if
//   the card drops out and comes back, so one drive stays one record
// - f_sync is what makes the data safe: it writes the file's dirty sectors and
//   the directory entry. closing and reopening would add a directory rewrite
//   and one more window for a cut to land in, and survive nothing more
//

#include "session_log.h"
#include "fatfs.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define LOG_FILE_NAME "SESSIONS.JSON"

static log_stats_t stats;
static uint32_t record_offset;      // where this drive's block sits in the file
static char block[SESSION_RECORD_LEN + 1];

// the slot this power cycle owns; a remount after a card error reuses it
// instead of claiming a new one, which is what keeps one drive to one record
static bool     claimed;
static uint32_t claimed_n;
static uint32_t claimed_offset;

// a failed write means the card is gone or corrupt; drop the mount so the
// loop's retry path can start again from scratch
static log_status_t log_fail(FRESULT fr, log_status_t status) {
    stats.fatfs_err = (uint8_t)fr;
    stats.errors++;
    if (stats.mounted) {
        f_close(&USERFile);
        f_mount(NULL, USERPath, 0);
        stats.mounted = false;
    }
    return status;
}

// write the prologue if the file is new or its opening is missing
static FRESULT write_prologue(void) {
    static char pro[SESSION_PROLOGUE_LEN + 1];
    FRESULT fr;
    UINT n;

    session_prologue_format(pro);
    fr = f_lseek(&USERFile, 0);
    if (fr != FR_OK) {
        return fr;
    }
    fr = f_write(&USERFile, pro, SESSION_PROLOGUE_LEN, &n);
    if ((fr == FR_OK) && (n != SESSION_PROLOGUE_LEN)) {
        return FR_DISK_ERR;
    }
    return fr;
}

// turn the closing brackets at the end of a block into a comma, so another
// block can follow it
static FRESULT open_previous_block(uint32_t offset) {
    static char term[SESSION_RECORD_TERM + 1];
    FRESULT fr;
    UINT n;

    session_record_term(term, false);
    fr = f_lseek(&USERFile, offset + SESSION_RECORD_LEN - SESSION_RECORD_TERM);
    if (fr != FR_OK) {
        return fr;
    }
    fr = f_write(&USERFile, term, SESSION_RECORD_TERM, &n);
    if ((fr == FR_OK) && (n != SESSION_RECORD_TERM)) {
        return FR_DISK_ERR;
    }
    return fr;
}

log_status_t session_log_init(void) {
    FRESULT fr;
    UINT n;
    uint32_t blocks;
    session_record_t first;

    stats.mount_attempts++;
    stats.mounted = false;
    stats.session_n = 0;

    // opt 1: mount now, which runs disk_initialize and so sd_spi_init
    fr = f_mount(&USERFatFS, USERPath, 1);
    if (fr == FR_NOT_READY) {
        stats.fatfs_err = (uint8_t)fr;
        return LOG_ERR_CARD;
    }
    if (fr != FR_OK) {
        stats.fatfs_err = (uint8_t)fr;
        return LOG_ERR_MOUNT;
    }
    stats.mounted = true;

    fr = f_open(&USERFile, LOG_FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
    if (fr != FR_OK) {
        return log_fail(fr, LOG_ERR_FILE);
    }

    // a file too short to hold the document's opening is a new one
    if (f_size(&USERFile) < SESSION_PROLOGUE_LEN) {
        fr = write_prologue();
        if (fr != FR_OK) {
            return log_fail(fr, LOG_ERR_WRITE);
        }
    }

    // whole blocks so far; a partial tail is a drive that was cut mid-claim
    // and gets overwritten by this one
    blocks = (uint32_t)((f_size(&USERFile) - SESSION_PROLOGUE_LEN) / SESSION_RECORD_LEN);

    // this power cycle already owns a slot: take it back rather than claiming
    // another, unless the file can no longer hold it - a swapped card
    if (claimed && (claimed_offset <= (SESSION_PROLOGUE_LEN + (blocks * SESSION_RECORD_LEN)))) {
        record_offset = claimed_offset;
        stats.session_n = claimed_n;
        return LOG_OK;
    }

    record_offset = SESSION_PROLOGUE_LEN + (blocks * SESSION_RECORD_LEN);
    stats.session_n = blocks + 1u;

    // the last block's own number wins if it disagrees - it does after an
    // earlier drive left a torn block behind
    if (blocks > 0u) {
        fr = f_lseek(&USERFile, record_offset - SESSION_RECORD_LEN);
        if (fr != FR_OK) {
            return log_fail(fr, LOG_ERR_FILE);
        }
        fr = f_read(&USERFile, block, SESSION_RECORD_LEN, &n);
        if ((fr != FR_OK) || (n != SESSION_RECORD_LEN)) {
            return log_fail(fr, LOG_ERR_FILE);
        }
        block[SESSION_RECORD_LEN] = '\0';
        uint32_t last_n = session_record_parse_n(block);
        if (last_n >= stats.session_n) {
            stats.session_n = last_n + 1u;
        }

        // let the document continue past that block; doing this before the
        // new block is written is what makes a cut here repairable
        fr = open_previous_block(record_offset - SESSION_RECORD_LEN);
        if (fr != FR_OK) {
            return log_fail(fr, LOG_ERR_WRITE);
        }
    }

    // claim the slot now with an empty record, so the file grows once, here,
    // while the car is still parked
    session_record_build(&first, stats.session_n, NULL, NULL, NULL);
    (void)session_record_format(&first, block);
    fr = f_lseek(&USERFile, record_offset);
    if (fr != FR_OK) {
        return log_fail(fr, LOG_ERR_FILE);
    }
    fr = f_write(&USERFile, block, SESSION_RECORD_LEN, &n);
    if ((fr != FR_OK) || (n != SESSION_RECORD_LEN)) {
        return log_fail(fr, LOG_ERR_WRITE);
    }
    fr = f_sync(&USERFile);
    if (fr != FR_OK) {
        return log_fail(fr, LOG_ERR_WRITE);
    }

    claimed = true;
    claimed_n = stats.session_n;
    claimed_offset = record_offset;
    return LOG_OK;
}

bool session_log_ready(void) {
    return stats.mounted;
}

uint32_t session_log_session_n(void) {
    return stats.mounted ? stats.session_n : 0u;
}

log_status_t session_log_write(const session_record_t *r) {
    FRESULT fr;
    UINT n;
    uint32_t t0;

    if (r == NULL) {
        return LOG_ERR_FILE;
    }
    if (!stats.mounted) {
        return LOG_ERR_NOT_READY;
    }

    // this drive's block is always the last one, so its own text closes the
    // document; nothing else in the file has to move
    (void)session_record_format(r, block);

    t0 = HAL_GetTick();
    fr = f_lseek(&USERFile, record_offset);
    if (fr != FR_OK) {
        return log_fail(fr, LOG_ERR_WRITE);
    }
    fr = f_write(&USERFile, block, SESSION_RECORD_LEN, &n);
    if ((fr != FR_OK) || (n != SESSION_RECORD_LEN)) {
        return log_fail(fr, LOG_ERR_WRITE);
    }
    fr = f_sync(&USERFile);
    if (fr != FR_OK) {
        return log_fail(fr, LOG_ERR_WRITE);
    }
    stats.last_write_ms = HAL_GetTick() - t0;
    if (stats.last_write_ms > stats.max_write_ms) {
        stats.max_write_ms = stats.last_write_ms;
    }
    stats.writes++;
    return LOG_OK;
}

void session_log_get_stats(log_stats_t *out) {
    if (out != NULL) {
        *out = stats;
    }
}
