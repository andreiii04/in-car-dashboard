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

// the faster link. the module boots at 9600 and 1 Hz; 5 Hz of RMC + GGA is
// ~725 B/s, too close to the ~960 B/s 9600 can carry, so the baud goes up
// first. every value checked against the protocol spec on disk
#define GPS_BAUD_SLOW 9600u
#define GPS_BAUD_FAST 115200u
// PMTK251 - baud; one of the listed rates. back to default on a full cold
// restart or standby, and no PMTK001 answer for this one (p34, p45)
#define GPS_CMD_BAUD  "PMTK251,115200"
// PMTK314 - which sentences, as "once every n fixes" (p51): GLL, RMC, VTG,
// GGA, GSA, GSV, then 16 more that stay off. RMC and GGA every fix; GSV every
// 5th, once a second, so bytes per second still shows the module hearing
// satellites before the fix
#define GPS_CMD_NMEA  "PMTK314,0,1,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0"
// PMTK220 - ms between fixes, 100-1000 (p42); 200 is 5 Hz
#define GPS_CMD_RATE  "PMTK220,200"

// host choice - the spec gives no time for the module to switch its baud
#define GPS_SWITCH_WAIT_MS 100u
// good sentences needed at the new baud, and how long they get to show up;
// the module talks every 200 ms there, so 3 s is plenty
#define GPS_PROVE_SENTENCES 2u
#define GPS_PROVE_MS        3000u
// tries before staying at 9600 for the rest of the drive
#define GPS_SWITCH_TRIES    2u

static char line[GPS_SENTENCE_MAX];
static uint16_t line_len;
static bool line_active;   // true once a $ has been seen
static gps_fix_t fix;
static gps_stats_t stats;

static gps_link_t link_state;
static uint32_t switch_tick;    // when the last switch was made
static uint32_t good_at_switch; // good sentences counted before it

// build one command and send it
static void gps_send(const char *body) {
    char out[GPS_SENTENCE_MAX];
    size_t n = nmea_build_command(out, sizeof(out), body);
    if (n > 0u) {
        (void)uart_bus_send((const uint8_t *)out, (uint16_t)n);
    }
}

// a sentence cut in half by a baud change is rubbish, drop it
static void gps_line_reset(void) {
    line_len = 0;
    line_active = false;
}

// ask for 115200 and 5 Hz, then wait in gps_poll for proof. the baud command
// goes out twice: at 9600 for a module that just booted, then at 115200 for
// one that kept the faster baud through a warm start on a backup cell and
// never understood the first. ~230 ms, all of it at boot
static void gps_go_fast(void) {
    stats.switch_tries++;

    gps_send(GPS_CMD_BAUD);
    HAL_Delay(GPS_SWITCH_WAIT_MS);
    (void)uart_bus_set_baud(GPS_BAUD_FAST);
    gps_line_reset();

    gps_send(GPS_CMD_BAUD);
    HAL_Delay(GPS_SWITCH_WAIT_MS);
    gps_send(GPS_CMD_NMEA);
    gps_send(GPS_CMD_RATE);

    link_state = GPS_LINK_TRYING;
    switch_tick = HAL_GetTick();
    good_at_switch = stats.sentences - stats.bad;
}

gps_status_t gps_init(void) {
    gps_line_reset();
    memset(&fix, 0, sizeof(fix));
    memset(&stats, 0, sizeof(stats));
    link_state = GPS_LINK_SLOW;

    if (uart_bus_start() != UART_BUS_OK) {
        return GPS_ERR_UART;
    }
    gps_go_fast();
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

        case NMEA_PMTK_ACK: {
            uint16_t cmd;
            uint8_t flag;
            if (nmea_parse_ack(sentence, &cmd, &flag)) {
                if (flag == 3u) {
                    stats.acks_ok++;      // valid packet, and action succeeded
                } else {
                    stats.acks_failed++;
                }
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

    // proving the switch: good sentences at the new baud mean the module
    // moved with us. nothing good for 3 s means it did not - back to 9600,
    // where it still talks, and one more try
    if (link_state == GPS_LINK_TRYING) {
        if ((stats.sentences - stats.bad - good_at_switch) >= GPS_PROVE_SENTENCES) {
            link_state = GPS_LINK_FAST;
        } else if ((HAL_GetTick() - switch_tick) >= GPS_PROVE_MS) {
            (void)uart_bus_set_baud(GPS_BAUD_SLOW);
            gps_line_reset();
            if (stats.switch_tries < GPS_SWITCH_TRIES) {
                gps_go_fast();
            } else {
                link_state = GPS_LINK_SLOW;
            }
        }
    }
}

gps_link_t gps_link(void) {
    return link_state;
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