//
// GPS module - public interface
//
// - pulls bytes from uart_bus, builds them into whole sentences, hands them to
//   the NMEA parser and keeps the newest fix
// - the module talks on its own so nothing here asks it for anything; gps_poll
//   just picks up whatever has arrived
//

#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    GPS_OK = 0,
    GPS_ERR_ARG,
    GPS_ERR_UART,
} gps_status_t;

typedef struct {
    bool fix_valid;       // there is a usable position
    bool time_valid;      // date and time are real UTC, arrives before the fix;
                          // false while the module still counts from 1980
    double lat_deg;
    double lon_deg;
    float speed_kmh;
    float course_deg;     // heading, 0 is north
    float altitude_m;
    float hdop;
    uint8_t sats_used;
    uint8_t fix_quality;
    uint8_t hour, minute, second;   // UTC
    uint8_t day, month, year;       // year is 2 digits
    uint32_t updated_tick;          // HAL tick of the last RMC heard, fix or no fix
} gps_fix_t;

// sentence counters - bring-up first, line-noise watch in the car later;
// bytes arriving is uart_bus_rx_count, this starts one level up at whole
// sentences
typedef struct {
    uint32_t sentences;   // complete $...* sentences seen
    uint32_t rmc;         // RMC that parsed
    uint32_t gga;         // GGA that parsed
    uint32_t other;       // good checksum, a type not handled - GSV, GSA, VTG, GLL
    uint32_t bad;         // failed the checksum - noise on the line

    // a position fix whose date failed the year check - session 5 had a fix
    // for minutes and never a clock, and nothing kept what the module sent
    uint32_t fix_no_date;           // RMCs like that
    bool     rejected_have;         // false if the date field was empty
    uint8_t  rejected_day, rejected_month, rejected_year;  // the last one, as sent
} gps_stats_t;

// clear the state and start the UART; call after MX_USART1_UART_Init
gps_status_t gps_init(void);

// take whatever bytes have arrived and parse any complete sentences;
// does not block, call it regularly
void gps_poll(void);

// copy out the newest fix
gps_status_t gps_get_fix(gps_fix_t *out);

// copy out the counters
gps_status_t gps_get_stats(gps_stats_t *out);

// ms since the GPS last produced an RMC sentence, fix or no fix - so this says
// whether the module is alive, not whether the position is fresh; in a tunnel
// it stays near 0 while fix_valid goes false, and a big number means the
// module or the wiring has died. check fix_valid for the position
uint32_t gps_fix_age_ms(void);

#endif // GPS_H