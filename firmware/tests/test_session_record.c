//
// host test for the session record
//
// - the properties that matter are about bytes and structure, not numbers:
//   every block is exactly SESSION_RECORD_LEN, the prologue is exactly one
//   sector, the block that is last closes the document and any other block
//   ends in a comma, and a whole file assembled from them balances. those are
//   checked by counting and by scanning the text, not by calling the
//   formatter twice
// - the local-time conversion is checked against dates worked out by hand,
//   including the ones that carry over a month and a year end
// - the FAT timestamp is checked against the field layout written in the
//   FatFs documentation, packed by hand here
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/test_session_record.c
//  App/Src/session_record.c -o /tmp/test_session_record && /tmp/test_session_record

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "session_record.h"
#include "vehicle_axes.h"

static int fails = 0;

static void check(const char *what, int ok) {
    printf("  %-52s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        fails++;
    }
}

// walk the text tracking nesting, ignoring anything inside a string. a
// well-formed document never goes negative and comes back to zero
typedef struct {
    int  ok;
    int  max_depth;
    int  objects_at_depth3;   // one per session record
    int  arrays;
    long quotes;
} shape_t;

static shape_t json_shape(const char *s, size_t len) {
    shape_t r = { 1, 0, 0, 0, 0 };
    int depth = 0;
    int in_string = 0;
    for (size_t i = 0; i < len; i++) {
        char c = s[i];
        if (in_string) {
            if (c == '"') {
                in_string = 0;
                r.quotes++;      // count both ends, so a pair is two
            }
            continue;
        }
        if (c == '"') {
            in_string = 1;
            r.quotes++;
        } else if ((c == '{') || (c == '[')) {
            depth++;
            if (c == '[') {
                r.arrays++;
            }
            if ((c == '{') && (depth == 3)) {
                r.objects_at_depth3++;
            }
            if (depth > r.max_depth) {
                r.max_depth = depth;
            }
        } else if ((c == '}') || (c == ']')) {
            depth--;
            if (depth < 0) {
                r.ok = 0;
            }
        }
    }
    if ((depth != 0) || in_string) {
        r.ok = 0;
    }
    return r;
}

static void sample_trip(trip_stats_t *t) {
    memset(t, 0, sizeof(*t));
    t->duration_s = 3737;
    t->moving_s = 2520;
    t->start_utc_valid = true;
    t->start_year = 26;
    t->start_month = 9;
    t->start_day = 15;
    t->start_hour = 20;
    t->start_minute = 42;
    t->start_second = 17;
    t->distance_km = 45.678f;
    t->speed_max_kmh = 132.4f;
    t->accel_peak_pos_g = 0.45f;
    t->accel_peak_neg_g = -0.82f;
    t->accel_peak_left_g = 0.61f;
    t->accel_peak_right_g = -0.58f;
    t->pitch_max_up_deg = 12.3f;
    t->pitch_max_down_deg = -8.1f;
    t->roll_max_right_deg = 5.4f;
    t->roll_max_left_deg = -6.2f;
    t->lateral_load_worst = 0.42f;
    t->roll_load_worst = 0.11f;
    t->temp_min_c = 21.3f;
    t->temp_max_c = 27.8f;
    t->temp_avg_c = 24.1f;
    t->press_min_hpa = 1001.2f;
    t->press_max_hpa = 1006.8f;
    t->press_avg_hpa = 1004.3f;
    t->hum_min_pct = 31.2f;
    t->hum_max_pct = 45.6f;
    t->hum_avg_pct = 38.0f;
}

static void sample_faults(session_faults_t *f) {
    memset(f, 0, sizeof(*f));
    f->imu_init = IMU_OK;
    f->env_init = ENV_ERR_BUS;
    f->gps_init = GPS_OK;
    f->display_init = LCD_ERR_ID;
    f->card_mounted = true;
    f->fatfs_error = 3;
    f->imu_read_errors = 17;
    f->gps_bad_checksums = 1;
    f->card_crc_errors = 2;
    f->env_read_errors = 4;
    f->longest_imu_gap_ms = 525;
    f->slowest_log_write_ms = 233;
    f->gps_date_refused_have = true;
    f->gps_date_refused_day = 6;
    f->gps_date_refused_month = 1;
    f->gps_date_refused_year = 80;
}

// how many bytes of the block the object actually used, before the padding
static size_t object_bytes(const char *block) {
    size_t body = SESSION_RECORD_LEN - SESSION_RECORD_TERM;
    while ((body > 0u) && (block[body - 1u] == ' ')) {
        body--;
    }
    return body;
}

// check the local-time conversion against a date worked out by hand
static void check_local(const char *what, uint8_t y, uint8_t mo, uint8_t d,
                        uint8_t h, uint8_t mi,
                        int wy, int wmo, int wd, int wh, int wmi) {
    session_record_t r;
    uint16_t ly; uint8_t lmo, ld, lh, lmi, ls;
    memset(&r, 0, sizeof(r));
    r.start_year = y; r.start_month = mo; r.start_day = d;
    r.start_hour = h; r.start_minute = mi; r.start_second = 9;
    session_record_local_time(&r, &ly, &lmo, &ld, &lh, &lmi, &ls);
    int ok = (ly == wy) && (lmo == wmo) && (ld == wd) && (lh == wh) && (lmi == wmi) && (ls == 9);
    if (!ok) {
        printf("    got %04u-%02u-%02u %02u:%02u, want %04d-%02d-%02d %02d:%02d\n",
               ly, lmo, ld, lh, lmi, wy, wmo, wd, wh, wmi);
    }
    check(what, ok);
}

int main(void) {
    printf("session_record tests\n");

    trip_stats_t t;
    session_faults_t f;
    elevation_t e = { .altitude_m = 120.0f, .gain_m = 123.4f, .loss_m = 98.7f,
                      .valid = true, .counting = true };
    session_record_t r;
    static char block[SESSION_RECORD_LEN + 1];
    static char pro[SESSION_PROLOGUE_LEN + 1];
    char term[SESSION_RECORD_TERM + 1];

    // ---- the block --------------------------------------------------------
    sample_trip(&t);
    sample_faults(&f);
    session_record_build(&r, 12, &t, &e, &f);
    int fits = session_record_format(&r, block);

    check("formatter says it fits", fits);
    check("block is exactly SESSION_RECORD_LEN bytes", strlen(block) == SESSION_RECORD_LEN);
    check("block is a whole number of sectors", (SESSION_RECORD_LEN % 512u) == 0u);
    check("block ends with a newline", block[SESSION_RECORD_LEN - 1] == '\n');
    check("block starts indented inside the array", strncmp(block, "    {", 5) == 0);
    check("a fresh block closes the document",
          strstr(block, "\n  ]\n}\n") == &block[SESSION_RECORD_LEN - 7]);

    size_t used = object_bytes(block);
    printf("  object used %zu of %u bytes\n", used, SESSION_RECORD_LEN - SESSION_RECORD_TERM);
    check("at least 100 bytes of headroom",
          used + 100u <= (SESSION_RECORD_LEN - SESSION_RECORD_TERM));

    // ---- the words --------------------------------------------------------
    check("session written in full", strstr(block, "\"session\": 12,") != NULL);
    check("start_utc written in full",
          strstr(block, "\"start_utc\": \"2026-09-15T20:42:17Z\"") != NULL);
    check("duration written in full", strstr(block, "\"duration_seconds\": 3737") != NULL);
    check("distance written in full", strstr(block, "\"kilometres\": 45.68") != NULL);
    check("moving time kept in the record", strstr(block, "\"moving_seconds\": 2520") != NULL);
    check("braking written in full", strstr(block, "\"hardest_braking_g\": -0.82") != NULL);
    check("rollover load written in full",
          strstr(block, "\"worst_rollover_load_fraction\": 0.11") != NULL);
    check("temperature is a nested object",
          strstr(block, "\"temperature_celsius\": { \"minimum\": 21.3, "
                        "\"maximum\": 27.8, \"average\": 24.1 }") != NULL);
    check("climb written in full", strstr(block, "\"climb_metres\": 123") != NULL);
    // no GPS height yet but the barometer has spoken: climb is still known
    e.valid = false;
    session_record_build(&r, 12, &t, &e, &f);
    (void)session_record_format(&r, block);
    check("climb is written before the first fix",
          strstr(block, "\"climb_metres\": 123") != NULL);
    e.counting = false;
    session_record_build(&r, 12, &t, &e, &f);
    (void)session_record_format(&r, block);
    check("no pressure yet means null climb", strstr(block, "\"climb_metres\": null") != NULL);
    e.valid = true;
    e.counting = true;
    session_record_build(&r, 12, &t, &e, &f);
    (void)session_record_format(&r, block);
    check("iaq reserved as null", strstr(block, "\"iaq_index\": null") != NULL);
    check("session number reads back", session_record_parse_n(block) == 12);

    // ---- the faults section ----------------------------------------------
    check("a passing init is a word", strstr(block, "\"imu_init\": \"ok\"") != NULL);
    check("a bus failure is a word", strstr(block, "\"environment_init\": \"bus_error\"") != NULL);
    check("a missing panel id is a word",
          strstr(block, "\"display_init\": \"no_panel_id\"") != NULL);
    check("card_mounted is a JSON bool", strstr(block, "\"card_mounted\": true") != NULL);
    check("fatfs error carried", strstr(block, "\"fatfs_error\": 3") != NULL);
    check("imu read errors carried", strstr(block, "\"imu_read_errors\": 17") != NULL);
    check("card crc errors carried", strstr(block, "\"card_crc_errors\": 2") != NULL);
    check("env read errors carried", strstr(block, "\"environment_read_errors\": 4") != NULL);
    check("longest imu gap carried", strstr(block, "\"longest_imu_gap_ms\": 525") != NULL);
    check("slowest log write carried", strstr(block, "\"slowest_log_write_ms\": 233") != NULL);
    check("a refused fix date is kept as sent",
          strstr(block, "\"gps_fix_date_refused\": \"060180\"") != NULL);
    f.gps_date_refused_empty = true;
    session_record_build(&r, 12, &t, &e, &f);
    (void)session_record_format(&r, block);
    check("an empty refused date says so",
          strstr(block, "\"gps_fix_date_refused\": \"empty\"") != NULL);
    f.gps_date_refused_have = false;
    session_record_build(&r, 12, &t, &e, &f);
    (void)session_record_format(&r, block);
    check("no refused date is null", strstr(block, "\"gps_fix_date_refused\": null") != NULL);

    // ---- the prologue and the separators ---------------------------------
    session_prologue_format(pro);
    check("prologue is exactly one sector", strlen(pro) == SESSION_PROLOGUE_LEN);
    check("prologue opens the document", strncmp(pro, "{\n  \"sessions\": [", 17) == 0);
    check("prologue ends with a newline", pro[SESSION_PROLOGUE_LEN - 1] == '\n');

    session_record_term(term, false);
    check("a continuing separator is the right length", strlen(term) == SESSION_RECORD_TERM);
    check("a continuing separator is a comma", term[0] == ',');
    check("a continuing separator ends with a newline", term[SESSION_RECORD_TERM - 1] == '\n');
    session_record_term(term, true);
    check("a closing separator is the right length", strlen(term) == SESSION_RECORD_TERM);
    check("a closing separator closes both", strstr(term, "\n  ]\n}\n") != NULL);

    // ---- a whole file, assembled the way session_log does it -------------
    const uint32_t n_sessions = 3;
    size_t file_len = SESSION_PROLOGUE_LEN + (n_sessions * SESSION_RECORD_LEN);
    char *file = malloc(file_len);
    if (file == NULL) {
        printf("  out of memory\n");
        return 1;
    }
    memcpy(file, pro, SESSION_PROLOGUE_LEN);
    for (uint32_t i = 1; i <= n_sessions; i++) {
        t.duration_s = i * 100u;
        session_record_build(&r, i, &t, &e, &f);
        session_record_format(&r, block);
        if (i < n_sessions) {
            session_record_term(term, false);
            memcpy(block + SESSION_RECORD_LEN - SESSION_RECORD_TERM, term, SESSION_RECORD_TERM);
        }
        memcpy(file + SESSION_PROLOGUE_LEN + ((i - 1u) * SESSION_RECORD_LEN),
               block, SESSION_RECORD_LEN);
    }
    shape_t sh = json_shape(file, file_len);
    check("assembled file is balanced", sh.ok);
    check("assembled file has one array", sh.arrays == 1);
    check("assembled file holds one object per session",
          sh.objects_at_depth3 == (int)n_sessions);
    check("assembled file nests four deep or more", sh.max_depth >= 4);
    check("assembled file has paired quotes", (sh.quotes % 2) == 0);
    check("file size is the prologue plus whole blocks",
          ((file_len - SESSION_PROLOGUE_LEN) % SESSION_RECORD_LEN) == 0u);
    free(file);

    // ---- the boot record, written before any sensor spoke ----------------
    session_record_build(&r, 1, NULL, NULL, NULL);
    fits = session_record_format(&r, block);
    check("empty record fits", fits && strlen(block) == SESSION_RECORD_LEN);
    check("empty record has a null start time", strstr(block, "\"start_utc\": null") != NULL);
    check("empty record has null weather",
          strstr(block, "\"temperature_celsius\": null") != NULL);
    check("empty record has a null faults section", strstr(block, "\"faults\": null") != NULL);
    check("empty record parses n = 1", session_record_parse_n(block) == 1);

    // ---- text that is not one of ours ------------------------------------
    check("garbage parses as 0", session_record_parse_n("hello") == 0);
    check("null parses as 0", session_record_parse_n(NULL) == 0);
    check("the key without a number parses as 0",
          session_record_parse_n("  \"session\": x") == 0);

    // ---- worst-case widths -----------------------------------------------
    sample_trip(&t);
    t.duration_s = 999999;
    t.moving_s = 999999;
    t.distance_km = 99999.99f;
    t.speed_max_kmh = 999.9f;
    t.accel_peak_neg_g = -99.99f;
    t.accel_peak_right_g = -99.99f;
    t.pitch_max_down_deg = -999.9f;
    t.roll_max_left_deg = -999.9f;
    t.press_min_hpa = 99999.9f;
    e.gain_m = 999999.0f;
    e.loss_m = 999999.0f;
    f.imu_init = IMU_ERR_TIMEOUT;
    f.env_init = ENV_ERR_NO_DATA;
    f.gps_init = GPS_ERR_UART;
    f.card_mounted = false;
    f.fatfs_error = 255;
    f.imu_read_errors = 4294967295u;
    f.gps_bad_checksums = 4294967295u;
    f.gps_ring_overruns = 4294967295u;
    f.env_read_errors = 4294967295u;
    f.longest_imu_gap_ms = 4294967295u;
    f.slowest_log_write_ms = 4294967295u;
    f.gps_date_refused_have = true;          // six digits is longer than null
    f.gps_date_refused_empty = false;
    f.display_errors = 4294967295u;
    f.card_crc_errors = 4294967295u;
    f.card_timeouts = 4294967295u;
    f.card_rejected = 4294967295u;
    f.log_write_errors = 4294967295u;
    session_record_build(&r, 4294967295u, &t, &e, &f);
    fits = session_record_format(&r, block);
    printf("  worst case used %zu bytes\n", object_bytes(block));
    check("worst-case widths still fit", fits);
    check("worst-case block is still the right length", strlen(block) == SESSION_RECORD_LEN);
    check("worst-case block still closes the document",
          strstr(block, "\n  ]\n}\n") == &block[SESSION_RECORD_LEN - 7]);

    // ---- local time, worked out by hand ----------------------------------
    printf("  local offset under test: %+d min\n", VEHICLE_UTC_OFFSET_MIN);
    check("the offset is the one the notes describe", VEHICLE_UTC_OFFSET_MIN == 180);
    check_local("midday needs no carry", 26, 9, 16, 10, 34, 2026, 9, 16, 13, 34);
    check_local("late evening carries the day", 26, 9, 16, 22, 45, 2026, 9, 17, 1, 45);
    check_local("the last day of a month carries the month", 26, 9, 30, 23, 30, 2026, 10, 1, 2, 30);
    check_local("new year's eve carries the year", 26, 12, 31, 22, 45, 2027, 1, 1, 1, 45);
    check_local("28 February in a common year", 27, 2, 28, 23, 10, 2027, 3, 1, 2, 10);
    check_local("28 February in a leap year", 28, 2, 28, 23, 10, 2028, 2, 29, 2, 10);
    check_local("a date the module never filled in is left alone", 26, 0, 0, 5, 0, 2026, 0, 0, 5, 0);

    sample_trip(&t);
    t.start_hour = 22;
    t.start_minute = 45;
    session_record_build(&r, 5, &t, &e, &f);
    session_record_format(&r, block);
    check("start_local is written with its offset",
          strstr(block, "\"start_local\": \"2026-09-16T01:45:17+03:00\"") != NULL);

    // ---- FAT time ---------------------------------------------------------
    // 2026-09-15 20:42:17 -> year 46 since 1980, seconds stored /2
    gps_fix_t fix;
    memset(&fix, 0, sizeof(fix));
    fix.time_valid = true;
    fix.year = 26;
    fix.month = 9;
    fix.day = 15;
    fix.hour = 20;
    fix.minute = 42;
    fix.second = 17;
    uint32_t ft = session_record_fattime(&fix);
    uint32_t want = (46u << 25) | (9u << 21) | (15u << 16) | (20u << 11) | (42u << 5) | 8u;
    check("fattime packs the GPS clock", ft == want);
    fix.time_valid = false;
    ft = session_record_fattime(&fix);
    check("fattime without a clock is 2026-01-01", ft == ((46u << 25) | (1u << 21) | (1u << 16)));
    check("fattime with no fix at all is the same", session_record_fattime(NULL) == ft);

    printf("\n%s\n", fails ? "FAILURES" : "all checks passed");
    return fails ? 1 : 0;
}
