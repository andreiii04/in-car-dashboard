//
// NMEA parser - implementation
//
// - a sentence looks like $GNRMC,080608.000,A,3029.4614,N,...*09
// - fields are separated by commas and a field is left empty when the module
//   has no value for it, so every field has to be checked before use
// - the checksum is an XOR of everything between the $ and the *
//

#include "nmea.h"
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

// knots to km/h, the speed field is in knots
#define NMEA_KNOTS_TO_KMH 1.852

// find where field n starts; field 0 is the sentence type so it is not returned
static const char *nmea_field(const char *s, uint8_t index) {
    uint8_t count = 0;

    while ((*s != '\0') && (*s != '*')) {
        if (*s == ',') {
            count++;
            if (count == index) {
                return s + 1;
            }
        }
        s++;
    }
    return NULL;
}

// a missing field is just two commas next to each other
static bool nmea_empty(const char *f) {
    return (f == NULL) || (*f == ',') || (*f == '*') ||
           (*f == '\0') || (*f == '\r') || (*f == '\n');
}

static uint8_t nmea_two_digits(const char *s) {
    return (uint8_t)(((s[0] - '0') * 10) + (s[1] - '0'));
}

// NMEA glues degrees and minutes together as ddmm.mmmm, so split them apart;
// lat has 2 degree digits, lon has 3
static double nmea_to_degrees(const char *f, uint8_t degree_digits) {
    char deg_text[4] = { 0 };

    for (uint8_t i = 0; i < degree_digits; i++) {
        if ((f[i] < '0') || (f[i] > '9')) {
            return 0.0;  // not the format expected here
        }
        deg_text[i] = f[i];
    }

    double degrees = (double)atoi(deg_text);
    double minutes = strtod(f + degree_digits, NULL);
    return degrees + (minutes / 60.0);
}

// XOR every byte between $ and *, then compare against the two hex digits
bool nmea_checksum_ok(const char *s) {
    if (*s != '$') {
        return false;
    }
    s++;

    uint8_t sum = 0;
    while ((*s != '\0') && (*s != '*')) {
        sum ^= (uint8_t)*s;
        s++;
    }
    if (*s != '*') {
        return false;  // sentence was cut short
    }

    uint8_t given = 0;
    for (uint8_t i = 1; i <= 2; i++) {
        char c = s[i];
        uint8_t v;

        if ((c >= '0') && (c <= '9')) {
            v = (uint8_t)(c - '0');
        } else if ((c >= 'A') && (c <= 'F')) {
            v = (uint8_t)(c - 'A' + 10);
        } else if ((c >= 'a') && (c <= 'f')) {
            v = (uint8_t)(c - 'a' + 10);
        } else {
            return false;
        }
        given = (uint8_t)((given << 4) | v);
    }

    return given == sum;
}

nmea_type_t nmea_identify(const char *sentence) {
    if (sentence == NULL) {
        return NMEA_UNKNOWN;
    }
    if (strlen(sentence) < 6u) {
        return NMEA_UNKNOWN;
    }
    if (!nmea_checksum_ok(sentence)) {
        return NMEA_UNKNOWN;
    }

    // characters 1-2 are the talker, which flips between GP and GN depending on
    // which constellations gave the fix; only characters 3-5 matter here
    const char *type = sentence + 3;

    if (strncmp(type, "RMC", 3) == 0) {
        return NMEA_RMC;
    }
    if (strncmp(type, "GGA", 3) == 0) {
        return NMEA_GGA;
    }
    return NMEA_UNKNOWN;
}

bool nmea_parse_rmc(const char *sentence, nmea_rmc_t *out) {
    if ((sentence == NULL) || (out == NULL)) {
        return false;
    }

    const char *utc    = nmea_field(sentence, 1);
    const char *status = nmea_field(sentence, 2);
    const char *lat    = nmea_field(sentence, 3);
    const char *ns     = nmea_field(sentence, 4);
    const char *lon    = nmea_field(sentence, 5);
    const char *ew     = nmea_field(sentence, 6);
    const char *sog    = nmea_field(sentence, 7);
    const char *cog    = nmea_field(sentence, 8);
    const char *date   = nmea_field(sentence, 9);

    if (nmea_empty(status)) {
        return false;
    }

    out->valid = (*status == 'A');
    out->time_valid = false;

    // time and date arrive before the position fix does, so read them anyway
    if (!nmea_empty(utc) && !nmea_empty(date)) {
        out->hour   = nmea_two_digits(utc);
        out->minute = nmea_two_digits(utc + 2);
        out->second = nmea_two_digits(utc + 4);
        out->day    = nmea_two_digits(date);
        out->month  = nmea_two_digits(date + 2);
        out->year   = nmea_two_digits(date + 4);
        out->time_valid = true;
    }

    if (!out->valid) {
        return true;  // sentence was fine, there is just no usable fix yet
    }

    if (nmea_empty(lat) || nmea_empty(ns) || nmea_empty(lon) || nmea_empty(ew)) {
        out->valid = false;
        return true;
    }

    out->lat_deg = nmea_to_degrees(lat, 2);
    if (*ns == 'S') {
        out->lat_deg = -out->lat_deg;
    }

    out->lon_deg = nmea_to_degrees(lon, 3);
    if (*ew == 'W') {
        out->lon_deg = -out->lon_deg;
    }

    // the speed field is in knots, not km/h
    out->speed_kmh  = nmea_empty(sog) ? 0.0f
                                      : (float)(strtod(sog, NULL) * NMEA_KNOTS_TO_KMH);
    out->course_deg = nmea_empty(cog) ? 0.0f : (float)strtod(cog, NULL);

    return true;
}

bool nmea_parse_gga(const char *sentence, nmea_gga_t *out) {
    if ((sentence == NULL) || (out == NULL)) {
        return false;
    }

    const char *quality = nmea_field(sentence, 6);
    const char *sats    = nmea_field(sentence, 7);
    const char *hdop    = nmea_field(sentence, 8);
    const char *alt     = nmea_field(sentence, 9);

    out->fix_quality = nmea_empty(quality) ? 0u : (uint8_t)(*quality - '0');
    out->sats_used   = nmea_empty(sats) ? 0u : (uint8_t)atoi(sats);
    out->hdop        = nmea_empty(hdop) ? 0.0f : (float)strtod(hdop, NULL);
    out->altitude_m  = nmea_empty(alt) ? 0.0f : (float)strtod(alt, NULL);

    return true;
}