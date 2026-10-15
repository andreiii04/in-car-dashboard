//
// session record - implementation
//
// - the text is pretty-printed with full words on purpose: the card gets
//   pulled out and read on a laptop, and that matters more than the bytes. a
//   block is 2 KB of which the object uses about 1.7 KB, so there is room for
//   a few more fields before the layout has to move
// - null is used for anything not known: no fix means no start time, no
//   BME680 means no weather; a reader then sees a missing value instead of a
//   zero that looks real
// - iaq, eco2 and bvoc are written as null on purpose - BSEC is a later step
//   and the keys are reserved so the file format does not change then
// - the faults section is the console's evidence, kept with the drive it
//   belongs to; counters only, no message text - if something needs digging
//   into, the ST-Link is the tool for that
//

#include "session_record.h"
#include "vehicle_axes.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

// how much of a block the object may use; the rest is padding and the
// separator at the very end
#define RECORD_BODY_LEN (SESSION_RECORD_LEN - SESSION_RECORD_TERM)

// ---- init results as words -----------------------------------------------------

static const char *imu_word(imu_status_t s) {
    switch (s) {
    case IMU_OK:          return "ok";
    case IMU_ERR_ARG:     return "bad_argument";
    case IMU_ERR_BUS:     return "bus_error";
    case IMU_ERR_ID:      return "wrong_chip_id";
    case IMU_ERR_TIMEOUT: return "timeout";
    default:              return "unknown";
    }
}

static const char *env_word(env_status_t s) {
    switch (s) {
    case ENV_OK:          return "ok";
    case ENV_ERR_ARG:     return "bad_argument";
    case ENV_ERR_BUS:     return "bus_error";
    case ENV_ERR_ID:      return "wrong_chip_id";
    case ENV_ERR_NO_DATA: return "no_data";
    default:              return "unknown";
    }
}

static const char *gps_word(gps_status_t s) {
    switch (s) {
    case GPS_OK:       return "ok";
    case GPS_ERR_ARG:  return "bad_argument";
    case GPS_ERR_UART: return "uart_refused";
    default:           return "unknown";
    }
}

static const char *lcd_word(lcd_status_t s) {
    switch (s) {
    case LCD_OK:      return "ok";
    case LCD_ERR_ARG: return "bad_argument";
    case LCD_ERR_SPI: return "spi_error";
    case LCD_ERR_ID:  return "no_panel_id";
    default:          return "unknown";
    }
}

// ---- local time ----------------------------------------------------------------

static bool is_leap(uint16_t y) {
    return (((y % 4u) == 0u) && ((y % 100u) != 0u)) || ((y % 400u) == 0u);
}

static uint8_t days_in_month(uint16_t y, uint8_t m) {
    static const uint8_t len[12] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    if ((m < 1u) || (m > 12u)) {
        return 30u;
    }
    if ((m == 2u) && is_leap(y)) {
        return 29u;
    }
    return len[m - 1u];
}

void session_record_local_time(const session_record_t *r,
                               uint16_t *year, uint8_t *month, uint8_t *day,
                               uint8_t *hour, uint8_t *minute, uint8_t *second) {
    uint16_t y = (uint16_t)(2000u + r->start_year);
    uint8_t  mo = r->start_month;
    uint8_t  d = r->start_day;

    *second = r->start_second;

    // a date the module never filled in cannot be carried over a month end;
    // hand back the time as it came
    if ((mo < 1u) || (mo > 12u) || (d < 1u)) {
        *year = y;
        *month = mo;
        *day = d;
        *hour = r->start_hour;
        *minute = r->start_minute;
        return;
    }

    int32_t mins = ((int32_t)r->start_hour * 60) + (int32_t)r->start_minute +
                   (int32_t)VEHICLE_UTC_OFFSET_MIN;
    while (mins < 0) {
        mins += 1440;
        if (d > 1u) {
            d--;
        } else {
            mo = (mo > 1u) ? (uint8_t)(mo - 1u) : 12u;
            if (mo == 12u) {
                y--;
            }
            d = days_in_month(y, mo);
        }
    }
    while (mins >= 1440) {
        mins -= 1440;
        if (d < days_in_month(y, mo)) {
            d++;
        } else {
            d = 1u;
            mo = (mo < 12u) ? (uint8_t)(mo + 1u) : 1u;
            if (mo == 1u) {
                y++;
            }
        }
    }

    *year = y;
    *month = mo;
    *day = d;
    *hour = (uint8_t)(mins / 60);
    *minute = (uint8_t)(mins % 60);
}

// ---- build ---------------------------------------------------------------------

void session_record_build(session_record_t *r, uint32_t session_n,
                          const trip_stats_t *trip, const elevation_t *elev,
                          const session_faults_t *faults) {
    memset(r, 0, sizeof(*r));
    r->session_n = session_n;
    if (faults != NULL) {
        r->faults_valid = true;
        r->faults = *faults;
    }
    if (trip == NULL) {
        return;
    }
    r->start_utc_valid = trip->start_utc_valid;
    r->start_year = trip->start_year;
    r->start_month = trip->start_month;
    r->start_day = trip->start_day;
    r->start_hour = trip->start_hour;
    r->start_minute = trip->start_minute;
    r->start_second = trip->start_second;
    r->duration_s = trip->duration_s;
    r->moving_s = trip->moving_s;
    r->distance_km = trip->distance_km;
    r->speed_max_kmh = trip->speed_max_kmh;
    r->accel_peak_pos_g = trip->accel_peak_pos_g;
    r->accel_peak_neg_g = trip->accel_peak_neg_g;
    r->accel_peak_left_g = trip->accel_peak_left_g;
    r->accel_peak_right_g = trip->accel_peak_right_g;
    r->pitch_max_up_deg = trip->pitch_max_up_deg;
    r->pitch_max_down_deg = trip->pitch_max_down_deg;
    r->roll_max_right_deg = trip->roll_max_right_deg;
    r->roll_max_left_deg = trip->roll_max_left_deg;
    r->lateral_load_worst = trip->lateral_load_worst;
    r->roll_load_worst = trip->roll_load_worst;
    // trip has no "seen weather" flag; a max below a min is impossible, so a
    // still-zeroed pair means nothing arrived
    r->env_valid = (trip->temp_max_c >= trip->temp_min_c) &&
                   ((trip->temp_max_c != 0.0f) || (trip->temp_min_c != 0.0f) ||
                    (trip->press_max_hpa != 0.0f));
    r->temp_min_c = trip->temp_min_c;
    r->temp_max_c = trip->temp_max_c;
    r->temp_avg_c = trip->temp_avg_c;
    r->press_min_hpa = trip->press_min_hpa;
    r->press_max_hpa = trip->press_max_hpa;
    r->press_avg_hpa = trip->press_avg_hpa;
    r->hum_min_pct = trip->hum_min_pct;
    r->hum_max_pct = trip->hum_max_pct;
    r->hum_avg_pct = trip->hum_avg_pct;
    if (elev != NULL) {
        // climb comes from the barometer alone now, so it is known from the
        // first pressure sample, fix or no fix
        r->elev_valid = elev->counting;
        r->elev_gain_m = elev->gain_m;
        r->elev_loss_m = elev->loss_m;
    }
}

// ---- text ----------------------------------------------------------------------

// append to the block, tracking how much room is left; n is the running length
static void put(char *buf, size_t *n, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
static void put(char *buf, size_t *n, const char *fmt, ...) {
    if (*n >= RECORD_BODY_LEN) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(buf + *n, RECORD_BODY_LEN - *n, fmt, ap);
    va_end(ap);
    if (w < 0) {
        return;
    }
    *n += (size_t)w;
    if (*n > RECORD_BODY_LEN) {
        *n = RECORD_BODY_LEN;
    }
}

void session_prologue_format(char *buf) {
    size_t n = 0;
    int w = snprintf(buf, SESSION_PROLOGUE_LEN + 1u, "{\n  \"sessions\": [");
    if (w > 0) {
        n = (size_t)w;
    }
    // spaces up to the last byte, then a newline, so the first block starts on
    // a line of its own
    while (n < (SESSION_PROLOGUE_LEN - 1u)) {
        buf[n++] = ' ';
    }
    buf[n++] = '\n';
    buf[n] = '\0';
}

void session_record_term(char *buf, bool last) {
    size_t n = 0;
    if (last) {
        // spaces, then the brackets that close the document
        while (n < (SESSION_RECORD_TERM - 7u)) {
            buf[n++] = ' ';
        }
        memcpy(buf + n, "\n  ]\n}\n", 7);
        n += 7u;
    } else {
        buf[n++] = ',';
        while (n < (SESSION_RECORD_TERM - 1u)) {
            buf[n++] = ' ';
        }
        buf[n++] = '\n';
    }
    buf[n] = '\0';
}

bool session_record_format(const session_record_t *r, char *buf) {
    size_t n = 0;
    buf[0] = '\0';

    put(buf, &n, "    {\n");
    put(buf, &n, "      \"session\": %lu,\n", (unsigned long)r->session_n);

    put(buf, &n, "      \"time\": {\n");
    if (r->start_utc_valid) {
        uint16_t ly; uint8_t lmo, ld, lh, lmi, ls;
        session_record_local_time(r, &ly, &lmo, &ld, &lh, &lmi, &ls);
        int off = VEHICLE_UTC_OFFSET_MIN;
        char sign = (off < 0) ? '-' : '+';
        if (off < 0) {
            off = -off;
        }
        put(buf, &n, "        \"start_utc\": \"20%02u-%02u-%02uT%02u:%02u:%02uZ\",\n",
            (unsigned)r->start_year, (unsigned)r->start_month, (unsigned)r->start_day,
            (unsigned)r->start_hour, (unsigned)r->start_minute, (unsigned)r->start_second);
        put(buf, &n, "        \"start_local\": \"%04u-%02u-%02uT%02u:%02u:%02u%c%02d:%02d\",\n",
            (unsigned)ly, (unsigned)lmo, (unsigned)ld,
            (unsigned)lh, (unsigned)lmi, (unsigned)ls,
            sign, off / 60, off % 60);
    } else {
        put(buf, &n, "        \"start_utc\": null,\n");
        put(buf, &n, "        \"start_local\": null,\n");
    }
    put(buf, &n, "        \"duration_seconds\": %lu\n", (unsigned long)r->duration_s);
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"distance\": {\n");
    put(buf, &n, "        \"kilometres\": %.2f,\n", (double)r->distance_km);
    put(buf, &n, "        \"moving_seconds\": %lu\n", (unsigned long)r->moving_s);
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"speed\": {\n");
    put(buf, &n, "        \"maximum_kmh\": %.1f\n", (double)r->speed_max_kmh);
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"acceleration\": {\n");
    put(buf, &n, "        \"hardest_acceleration_g\": %.2f,\n", (double)r->accel_peak_pos_g);
    put(buf, &n, "        \"hardest_braking_g\": %.2f,\n", (double)r->accel_peak_neg_g);
    put(buf, &n, "        \"hardest_left_turn_g\": %.2f,\n", (double)r->accel_peak_left_g);
    put(buf, &n, "        \"hardest_right_turn_g\": %.2f,\n", (double)r->accel_peak_right_g);
    put(buf, &n, "        \"worst_lateral_load_fraction\": %.2f\n", (double)r->lateral_load_worst);
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"attitude\": {\n");
    put(buf, &n, "        \"max_pitch_up_degrees\": %.1f,\n", (double)r->pitch_max_up_deg);
    put(buf, &n, "        \"max_pitch_down_degrees\": %.1f,\n", (double)r->pitch_max_down_deg);
    put(buf, &n, "        \"max_roll_right_degrees\": %.1f,\n", (double)r->roll_max_right_deg);
    put(buf, &n, "        \"max_roll_left_degrees\": %.1f,\n", (double)r->roll_max_left_deg);
    put(buf, &n, "        \"worst_rollover_load_fraction\": %.2f\n", (double)r->roll_load_worst);
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"weather\": {\n");
    if (r->env_valid) {
        put(buf, &n, "        \"temperature_celsius\": "
                     "{ \"minimum\": %.1f, \"maximum\": %.1f, \"average\": %.1f },\n",
            (double)r->temp_min_c, (double)r->temp_max_c, (double)r->temp_avg_c);
        put(buf, &n, "        \"pressure_hpa\": "
                     "{ \"minimum\": %.1f, \"maximum\": %.1f, \"average\": %.1f },\n",
            (double)r->press_min_hpa, (double)r->press_max_hpa, (double)r->press_avg_hpa);
        put(buf, &n, "        \"humidity_percent\": "
                     "{ \"minimum\": %.0f, \"maximum\": %.0f, \"average\": %.0f }\n",
            (double)r->hum_min_pct, (double)r->hum_max_pct, (double)r->hum_avg_pct);
    } else {
        put(buf, &n, "        \"temperature_celsius\": null,\n");
        put(buf, &n, "        \"pressure_hpa\": null,\n");
        put(buf, &n, "        \"humidity_percent\": null\n");
    }
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"elevation\": {\n");
    if (r->elev_valid) {
        put(buf, &n, "        \"climb_metres\": %.0f,\n", (double)r->elev_gain_m);
        put(buf, &n, "        \"descent_metres\": %.0f\n", (double)r->elev_loss_m);
    } else {
        put(buf, &n, "        \"climb_metres\": null,\n");
        put(buf, &n, "        \"descent_metres\": null\n");
    }
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"air_quality\": {\n");
    put(buf, &n, "        \"iaq_index\": null,\n");
    put(buf, &n, "        \"eco2_ppm\": null,\n");
    put(buf, &n, "        \"breath_voc_ppm\": null\n");
    put(buf, &n, "      },\n");

    put(buf, &n, "      \"faults\": ");
    if (r->faults_valid) {
        const session_faults_t *f = &r->faults;
        put(buf, &n, "{\n");
        put(buf, &n, "        \"imu_init\": \"%s\",\n", imu_word(f->imu_init));
        put(buf, &n, "        \"environment_init\": \"%s\",\n", env_word(f->env_init));
        put(buf, &n, "        \"gps_init\": \"%s\",\n", gps_word(f->gps_init));
        put(buf, &n, "        \"display_init\": \"%s\",\n", lcd_word(f->display_init));
        put(buf, &n, "        \"card_mounted\": %s,\n", f->card_mounted ? "true" : "false");
        put(buf, &n, "        \"fatfs_error\": %u,\n", (unsigned)f->fatfs_error);
        put(buf, &n, "        \"imu_read_errors\": %lu,\n", (unsigned long)f->imu_read_errors);
        put(buf, &n, "        \"environment_read_errors\": %lu,\n",
            (unsigned long)f->env_read_errors);
        put(buf, &n, "        \"gps_bad_checksums\": %lu,\n", (unsigned long)f->gps_bad_checksums);
        put(buf, &n, "        \"gps_ring_overruns\": %lu,\n", (unsigned long)f->gps_ring_overruns);
        // the date as the module sent it, ddmmyy, only if a fix ever came
        // without a usable one - session 5 had exactly that and kept nothing
        if (!f->gps_date_refused_have) {
            put(buf, &n, "        \"gps_fix_date_refused\": null,\n");
        } else if (f->gps_date_refused_empty) {
            put(buf, &n, "        \"gps_fix_date_refused\": \"empty\",\n");
        } else {
            put(buf, &n, "        \"gps_fix_date_refused\": \"%02u%02u%02u\",\n",
                (unsigned)f->gps_date_refused_day, (unsigned)f->gps_date_refused_month,
                (unsigned)f->gps_date_refused_year);
        }
        put(buf, &n, "        \"longest_imu_gap_ms\": %lu,\n", (unsigned long)f->longest_imu_gap_ms);
        put(buf, &n, "        \"slowest_log_write_ms\": %lu,\n",
            (unsigned long)f->slowest_log_write_ms);
        put(buf, &n, "        \"display_errors\": %lu,\n", (unsigned long)f->display_errors);
        put(buf, &n, "        \"card_crc_errors\": %lu,\n", (unsigned long)f->card_crc_errors);
        put(buf, &n, "        \"card_timeouts\": %lu,\n", (unsigned long)f->card_timeouts);
        put(buf, &n, "        \"card_rejected_commands\": %lu,\n", (unsigned long)f->card_rejected);
        put(buf, &n, "        \"log_write_errors\": %lu\n", (unsigned long)f->log_write_errors);
        put(buf, &n, "      }\n");
    } else {
        put(buf, &n, "null\n");
    }

    put(buf, &n, "    }");

    // room for the closing brace is the fit test; if the text overran, cut it
    // back and close the object so the document stays parseable
    bool fits = (n < RECORD_BODY_LEN);
    if (!fits) {
        n = RECORD_BODY_LEN - 1u;
        buf[n++] = '}';
    }
    while (n < RECORD_BODY_LEN) {
        buf[n++] = ' ';
    }
    session_record_term(buf + n, true);
    return fits;
}

uint32_t session_record_parse_n(const char *block) {
    static const char key[] = "\"session\":";
    if (block == NULL) {
        return 0;
    }
    const char *p = strstr(block, key);
    if (p == NULL) {
        return 0;
    }
    p += sizeof(key) - 1u;
    while (*p == ' ') {
        p++;
    }
    if ((*p < '0') || (*p > '9')) {
        return 0;
    }
    uint32_t n = 0;
    while ((*p >= '0') && (*p <= '9')) {
        n = (n * 10u) + (uint32_t)(*p - '0');
        p++;
    }
    return n;
}

uint32_t session_record_fattime(const gps_fix_t *fix) {
    uint32_t year, month, day, hour, minute, second;
    if ((fix != NULL) && fix->time_valid) {
        year = 2000u + fix->year;
        month = fix->month;
        day = fix->day;
        hour = fix->hour;
        minute = fix->minute;
        second = fix->second;
    } else {
        // no real clock yet: a fixed date that is obviously not a real one
        year = 2026u;
        month = 1u;
        day = 1u;
        hour = 0u;
        minute = 0u;
        second = 0u;
    }
    return ((year - 1980u) << 25) | (month << 21) | (day << 16) |
           (hour << 11) | (minute << 5) | (second / 2u);
}
