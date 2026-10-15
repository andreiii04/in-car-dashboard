//
// NMEA parser - public interface
//
// - NMEA 0183 is the text format the GPS speaks; each line is one sentence
// - pure C, no HAL and no hardware, so this can be compiled and tested on a PC
//   by feeding it recorded sentences
// - only RMC and GGA are handled, plus the PMTK001 answer the module gives a
//   command; everything else is ignored
// - also builds the other direction: a PMTK command with its checksum, ready
//   to send to the module
//

#ifndef NMEA_H
#define NMEA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    NMEA_UNKNOWN = 0,
    NMEA_RMC,
    NMEA_GGA,
    NMEA_PMTK_ACK,   // $PMTK001 - the module answering one of our commands
} nmea_type_t;

// RMC - position, speed, course, UTC date and time
typedef struct {
    bool valid;          // status field was A
    bool time_valid;     // date and time fields were filled in
    double lat_deg;      // + north, - south
    double lon_deg;      // + east, - west
    float speed_kmh;
    float course_deg;
    uint8_t hour, minute, second;
    uint8_t day, month, year;  // year is 2 digits, 26 means 2026
} nmea_rmc_t;

// GGA - fix quality, satellite count, altitude
typedef struct {
    uint8_t fix_quality;  // 0 none, 1 GPS, 2 differential/SBAS, 6 dead reckoning
    uint8_t sats_used;
    float hdop;           // lower is better, under 2 is good
    float altitude_m;     // above mean sea level
} nmea_gga_t;

// true when the two hex digits after * match the XOR of everything between
// $ and *; nmea_identify does this too, this is for counting failures
bool nmea_checksum_ok(const char *sentence);

// verify the checksum and say what kind of sentence this is
nmea_type_t nmea_identify(const char *sentence);

bool nmea_parse_rmc(const char *sentence, nmea_rmc_t *out);
bool nmea_parse_gga(const char *sentence, nmea_gga_t *out);

// $PMTK001,<cmd>,<flag> - which command it answers and how it went; flag 3 is
// "valid packet, and action succeeded", 0-2 are the ways it can fail
// (protocol spec p34)
bool nmea_parse_ack(const char *sentence, uint16_t *cmd, uint8_t *flag);

// wrap a command body like "PMTK220,200" into "$PMTK220,200*2F\r\n" - the $,
// the checksum over the body, the line ending. returns the length written, 0
// if it does not fit in out
size_t nmea_build_command(char *out, size_t size, const char *body);

#endif // NMEA_H