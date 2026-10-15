//
// BSEC state on the card - implementation
//
// - the file is opened, written and closed on every save, every 10 min - not
//   kept open like SESSIONS.JSON, so a remount after a card error never finds
//   a stale handle. FatFs allows two open files here (_FS_LOCK 2), and this
//   one is only ever open for the length of one call
// - a save always goes into the slot that does not hold the newest state, so
//   the newest good one is never the one being overwritten
//

#include "air_store.h"
#include "air_slot.h"
#include "air.h"
#include "session_log.h"
#include "fatfs.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define AIR_FILE_NAME "BSEC.BIN"
#define AIR_FILE_LEN  (2u * AIR_SLOT_LEN)

static air_store_stats_t stats;
static FIL air_file;
static uint8_t slot[AIR_SLOT_LEN];
static uint32_t next_slot;   // 0 or 1, where the next save goes
static uint32_t next_seq;    // its sequence number

static void store_fail(FRESULT fr) {
    stats.fatfs_err = (uint8_t)fr;
    stats.errors++;
}

// read one slot and check it
static bool read_slot(uint32_t i, uint32_t *seq, uint8_t *state, uint16_t *len) {
    UINT n;

    if (f_lseek(&air_file, i * AIR_SLOT_LEN) != FR_OK) {
        return false;
    }
    if ((f_read(&air_file, slot, AIR_SLOT_LEN, &n) != FR_OK) || (n != AIR_SLOT_LEN)) {
        return false;
    }
    return air_slot_unpack(slot, seq, state, len, AIR_STATE_MAX);
}

air_store_boot_t air_store_load(uint8_t *state, uint16_t *len) {
    FRESULT fr;
    UINT n;
    uint32_t seq[2] = { 0u, 0u };
    uint16_t lens[2] = { 0u, 0u };
    bool ok[2];
    static uint8_t other[AIR_STATE_MAX];   // slot 1 goes here while slot 0 sits in state

    *len = 0u;
    stats.boot = AIR_STORE_NO_CARD;
    stats.armed = false;
    next_slot = 0u;
    next_seq = 1u;

    if (!session_log_ready()) {
        return stats.boot;
    }

    fr = f_open(&air_file, AIR_FILE_NAME, FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
    if (fr != FR_OK) {
        store_fail(fr);
        stats.boot = AIR_STORE_ERR;
        return stats.boot;
    }

    // new or too short: make it full length now, zeros in both slots, so the
    // file never grows while driving
    if (f_size(&air_file) < AIR_FILE_LEN) {
        memset(slot, 0, sizeof(slot));
        fr = f_lseek(&air_file, 0);
        for (uint32_t i = 0; (fr == FR_OK) && (i < 2u); i++) {
            fr = f_write(&air_file, slot, AIR_SLOT_LEN, &n);
            if ((fr == FR_OK) && (n != AIR_SLOT_LEN)) {
                fr = FR_DISK_ERR;
            }
        }
        if (fr == FR_OK) {
            fr = f_close(&air_file);
        } else {
            (void)f_close(&air_file);
        }
        if (fr != FR_OK) {
            store_fail(fr);
            stats.boot = AIR_STORE_ERR;
            return stats.boot;
        }
        stats.boot = AIR_STORE_NEW;
        stats.armed = true;
        return stats.boot;
    }

    ok[0] = read_slot(0u, &seq[0], state, &lens[0]);
    ok[1] = read_slot(1u, &seq[1], other, &lens[1]);
    (void)f_close(&air_file);

    int newest = air_slot_newest(ok[0], seq[0], ok[1], seq[1]);
    stats.armed = true;
    if (newest < 0) {
        // both torn or never written; the next save starts slot 0 over
        stats.boot = AIR_STORE_EMPTY;
        return stats.boot;
    }
    if (newest == 1) {
        memcpy(state, other, lens[1]);
    }
    *len = lens[newest];
    next_slot = (newest == 0) ? 1u : 0u;
    next_seq = seq[newest] + 1u;
    stats.boot = AIR_STORE_LOADED;
    return stats.boot;
}

bool air_store_save(const uint8_t *state, uint16_t len) {
    FRESULT fr;
    UINT n;

    if (!stats.armed || !session_log_ready()) {
        return false;
    }
    if (!air_slot_pack(slot, next_seq, state, len)) {
        stats.errors++;
        return false;
    }

    fr = f_open(&air_file, AIR_FILE_NAME, FA_WRITE | FA_OPEN_EXISTING);
    if (fr != FR_OK) {
        store_fail(fr);
        return false;
    }
    fr = f_lseek(&air_file, next_slot * AIR_SLOT_LEN);
    if (fr == FR_OK) {
        fr = f_write(&air_file, slot, AIR_SLOT_LEN, &n);
        if ((fr == FR_OK) && (n != AIR_SLOT_LEN)) {
            fr = FR_DISK_ERR;
        }
    }
    // closing is what flushes the sector and the directory entry
    if (fr == FR_OK) {
        fr = f_close(&air_file);
    } else {
        (void)f_close(&air_file);
    }
    if (fr != FR_OK) {
        // the same slot again next time; the other one still holds the last good state
        store_fail(fr);
        return false;
    }

    next_slot ^= 1u;
    next_seq++;
    stats.saves++;
    stats.last_save_tick = HAL_GetTick();
    return true;
}

void air_store_get_stats(air_store_stats_t *out) {
    if (out != NULL) {
        *out = stats;
    }
}
