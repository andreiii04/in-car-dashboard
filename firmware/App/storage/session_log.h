//
// session log - public interface
//
// - one file on the card, SESSIONS.JSON, a whole JSON document: a fixed
//   prologue then one 2 KB block per drive. this drive's block is claimed at
//   boot and rewritten in place every 10 s, so a power cut costs at most the
//   last 10 s of peaks
// - the file only ever grows once per drive, at boot. after that every write
//   lands on the same four sectors, which is the whole point: the FAT and the
//   directory are never touched while the car is moving
// - a slot is claimed once per power cycle and reused if the card drops out
//   and comes back, so one drive is always one record
// - sits on FatFs, which sits on FATFS/Target/user_diskio.c, which sits on
//   sd_spi; the record text itself comes from session_record.c
//

#ifndef SESSION_LOG_H
#define SESSION_LOG_H

#include <stdbool.h>
#include <stdint.h>
#include "session_record.h"

typedef enum {
    LOG_OK = 0,
    LOG_ERR_NOT_READY,   // no card mounted
    LOG_ERR_CARD,        // the card did not answer
    LOG_ERR_MOUNT,       // card answered but no usable filesystem
    LOG_ERR_FILE,        // could not open or read SESSIONS.JSON
    LOG_ERR_WRITE,       // a write or sync failed; the card is unmounted again
} log_status_t;

typedef struct {
    bool     mounted;
    uint32_t session_n;
    uint32_t mount_attempts;
    uint32_t writes;
    uint32_t errors;
    uint32_t last_write_ms;   // how long the last rewrite plus sync took
    uint32_t max_write_ms;    // the worst one, which is what the loop budget cares about
    uint8_t  fatfs_err;       // the last FRESULT, for the console
} log_stats_t;

// mount the card, open the file, work out this drive's session number and
// append its first record; call again later if it fails - there is no card
// detect pin, so a missing card only shows as a failed init
log_status_t session_log_init(void);

bool session_log_ready(void);
uint32_t session_log_session_n(void);

// rewrite this drive's record and flush it to the card
log_status_t session_log_write(const session_record_t *r);

void session_log_get_stats(log_stats_t *out);

#endif // SESSION_LOG_H
