//
// session record - public interface
//
// - the whole file is one JSON document: a fixed-size prologue that opens
//   { "sessions": [ , then one fixed-size block per drive, and the block that
//   happens to be last carries the closing ] } inside itself
// - one drive is one block, always exactly SESSION_RECORD_LEN bytes and always
//   a whole number of 512-byte sectors, which is what makes the every-10-second
//   rewrite safe: the block is overwritten in place, the file never changes
//   size mid-drive, and a power cut can only tear that one block
// - the last SESSION_RECORD_TERM bytes of every block are the separator. a
//   block that is last closes the document; when the next drive claims a block
//   the previous one's separator is rewritten into a comma, which is one sector
//   written once, at boot, with the car parked
// - no HAL and no FatFs in here - it turns numbers into text and back, and
//   tests/test_session_record.c checks it on the mac. session_log.c is the
//   part that touches the card
//

#ifndef SESSION_RECORD_H
#define SESSION_RECORD_H

#include <stdbool.h>
#include <stdint.h>
#include "trip.h"
#include "elevation.h"
#include "gps.h"
#include "imu.h"
#include "env.h"
#include "lcd.h"

// 4 sectors a block; the pretty-printed object needs about 1.7 KB and the rest
// is padding, so a new field does not change the file layout
#define SESSION_RECORD_LEN  2048u

// the separator that ends every block - a comma, or the closing brackets
#define SESSION_RECORD_TERM 16u

// one sector, holding the opening of the document
#define SESSION_PROLOGUE_LEN 512u

// every counter and init result the console would have shouted about, so a
// drive that went wrong says so in its own record instead of only on a
// terminal nobody was watching
typedef struct {
    imu_status_t imu_init;
    env_status_t env_init;
    gps_status_t gps_init;
    lcd_status_t display_init;
    bool         card_mounted;
    uint8_t      fatfs_error;        // the last FRESULT, 0 when nothing failed

    uint32_t imu_read_errors;
    uint32_t env_read_errors;        // BME680 reads that failed or never finished
    uint32_t gps_bad_checksums;
    uint32_t gps_ring_overruns;
    uint32_t display_errors;
    uint32_t card_crc_errors;
    uint32_t card_timeouts;
    uint32_t card_rejected;
    uint32_t log_write_errors;

    // how badly the loop stalled - a stall over ~2.1 s is what loses GPS bytes
    uint32_t longest_imu_gap_ms;     // worst gap between two IMU samples
    uint32_t slowest_log_write_ms;   // worst record write plus f_sync

    // a fix that came with a refused date; have false means it never happened
    bool     gps_date_refused_have;
    bool     gps_date_refused_empty; // the date field itself was empty
    uint8_t  gps_date_refused_day, gps_date_refused_month, gps_date_refused_year;
} session_faults_t;

typedef struct {
    uint32_t session_n;

    bool    start_utc_valid;
    uint8_t start_year, start_month, start_day;   // year is 2 digits
    uint8_t start_hour, start_minute, start_second;

    uint32_t duration_s;
    uint32_t moving_s;
    float    distance_km;
    float    speed_max_kmh;

    float accel_peak_pos_g, accel_peak_neg_g;
    float accel_peak_left_g, accel_peak_right_g;
    float pitch_max_up_deg, pitch_max_down_deg;
    float roll_max_right_deg, roll_max_left_deg;
    float lateral_load_worst, roll_load_worst;

    bool  env_valid;
    float temp_min_c, temp_max_c, temp_avg_c;
    float press_min_hpa, press_max_hpa, press_avg_hpa;
    float hum_min_pct, hum_max_pct, hum_avg_pct;

    bool  elev_valid;
    float elev_gain_m, elev_loss_m;

    bool             faults_valid;
    session_faults_t faults;
} session_record_t;

// fill a record from the trip and elevation snapshots; faults may be NULL,
// which leaves the section out
void session_record_build(session_record_t *r, uint32_t session_n,
                          const trip_stats_t *trip, const elevation_t *elev,
                          const session_faults_t *faults);

// write the record as one padded block into buf, which must hold
// SESSION_RECORD_LEN + 1 bytes. the block closes the document, because a
// freshly written record is always the last one in the file; false if the text
// would not fit, in which case buf still holds a valid but shortened object
bool session_record_format(const session_record_t *r, char *buf);

// the opening of the document, padded to SESSION_PROLOGUE_LEN; buf needs one
// byte more than that
void session_prologue_format(char *buf);

// the last SESSION_RECORD_TERM bytes of a block: last true closes the
// document, last false is the comma that lets another block follow. buf needs
// one byte more than SESSION_RECORD_TERM
void session_record_term(char *buf, bool last);

// pull the session number back out of a stored block; 0 if the text does not
// look like one of ours
uint32_t session_record_parse_n(const char *block);

// UTC plus the local offset in vehicle_axes.h, carried over month and year
// ends; used for the start_local field, where a reader wants wall-clock time
void session_record_local_time(const session_record_t *r,
                               uint16_t *year, uint8_t *month, uint8_t *day,
                               uint8_t *hour, uint8_t *minute, uint8_t *second);

// FAT timestamp from the GPS clock, packed the way FatFs wants it: year since
// 1980 in the top 7 bits, then month, day, hour, minute, seconds/2. when the
// GPS has no real time yet this gives a fixed date, so the file still carries
// something sane
uint32_t session_record_fattime(const gps_fix_t *fix);

#endif // SESSION_RECORD_H
