//
// GPS module - implementation
//
// - bytes come in one at a time; a sentence starts at $ and ends at CR or LF
// - RMC gives position, speed, course, date and time; GGA adds satellite count,
//   altitude and HDOP
// - talker ID is GP before a fix and GN after, so only the last three letters
//   of the sentence name are matched
//

#include "gps.h"
#include "uart_bus.h"
#include "nmea.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>
#include <string.h>

// NMEA caps a sentence at 82 characters, a bit of room on top
#define GPS_SENTENCE_MAX 96

// with no satellite heard yet the module still fills the time fields - it
// counts up from the GPS epoch, 6 jan 1980, from the first second on; so
// "fields not empty" is not "time is real"; only trust a year this firmware
// could actually see: from the year it was written up to the 1980 rollover
#define GPS_MIN_YEAR   26   // 2026, two-digit like the sentence
#define GPS_EPOCH_YEAR 80   // 1980; anything from here up is the counter

static char line[GPS_SENTENCE_MAX];
static uint16_t line_len;
static bool line_active;   // true once a $ has been seen
static gps_fix_t fix;
static gps_stats_t stats;

gps_status_t gps_init(void) {
    line_len = 0;
    line_active = false;
    memset(&fix, 0, sizeof(fix));
    memset(&stats, 0, sizeof(stats));

    if (uart_bus_start() != UART_BUS_OK) {
        return GPS_ERR_UART;
    }
    return GPS_OK;
}

static void gps_handle_sentence(const char *sentence) {
    stats.sentences++;

    // checked here so a failure is counted as noise, not as an unknown type
    if (!nmea_checksum_ok(sentence)) {
        stats.bad++;
        return;
    }

    switch (nmea_identify(sentence)) {
        case NMEA_RMC: {
            nmea_rmc_t rmc;
            if (nmea_parse_rmc(sentence, &rmc)) {
                stats.rmc++;
                fix.fix_valid  = rmc.valid;
                fix.time_valid = rmc.time_valid &&
                                 (rmc.year >= GPS_MIN_YEAR) &&
                                 (rmc.year < GPS_EPOCH_YEAR);

                if (fix.time_valid) {
                    fix.hour   = rmc.hour;
                    fix.minute = rmc.minute;
                    fix.second = rmc.second;
                    fix.day    = rmc.day;
                    fix.month  = rmc.month;
                    fix.year   = rmc.year;
                }

                // a fix with no usable date should not happen; keep what the
                // module actually sent so the record can show it
                if (rmc.valid && !fix.time_valid) {
                    stats.fix_no_date++;
                    stats.rejected_have = rmc.time_valid;
                    if (rmc.time_valid) {
                        stats.rejected_day   = rmc.day;
                        stats.rejected_month = rmc.month;
                        stats.rejected_year  = rmc.year;
                    }
                }

                if (rmc.valid) {
                    fix.lat_deg    = rmc.lat_deg;
                    fix.lon_deg    = rmc.lon_deg;
                    fix.speed_kmh  = rmc.speed_kmh;
                    fix.course_deg = rmc.course_deg;
                }

                // stamped on every RMC, valid or not - "the module is still
                // talking" is its own signal, separate from having a fix
                fix.updated_tick = HAL_GetTick();
            }
            break;
        }

        case NMEA_GGA: {
            nmea_gga_t gga;
            if (nmea_parse_gga(sentence, &gga)) {
                stats.gga++;
                fix.fix_quality = gga.fix_quality;
                fix.sats_used   = gga.sats_used;
                fix.hdop        = gga.hdop;
                fix.altitude_m  = gga.altitude_m;
            }
            break;
        }

        default:
            stats.other++;  // a sentence type this module does not parse
            break;
    }
}

void gps_poll(void) {
    uint8_t c;

    while (uart_bus_read_byte(&c)) {
        // a $ always starts a fresh sentence, whatever came before it
        if (c == '$') {
            line_len = 0;
            line_active = true;
        }

        if (!line_active) {
            continue;  // still mid-rubbish, waiting for the first $
        }

        if ((c == '\r') || (c == '\n')) {
            if (line_len > 0u) {
                line[line_len] = '\0';
                gps_handle_sentence(line);
            }
            line_len = 0;
            line_active = false;
            continue;
        }

        if (line_len < (GPS_SENTENCE_MAX - 1u)) {
            line[line_len] = (char)c;
            line_len++;
        } else {
            // longer than any real sentence, drop it and wait for the next $
            line_len = 0;
            line_active = false;
        }
    }
}

gps_status_t gps_get_fix(gps_fix_t *out) {
    if (out == NULL) {
        return GPS_ERR_ARG;
    }
    *out = fix;
    return GPS_OK;
}

gps_status_t gps_get_stats(gps_stats_t *out) {
    if (out == NULL) {
        return GPS_ERR_ARG;
    }
    *out = stats;
    return GPS_OK;
}

uint32_t gps_fix_age_ms(void) {
    return HAL_GetTick() - fix.updated_tick;
}