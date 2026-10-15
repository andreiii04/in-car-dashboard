//
// console module - public interface
//
// - every printf that used to live in main.c: the banner, the init results,
//   the once-a-second status block. main hands over the few things only it
//   knows and console fetches the rest from the modules' own getters
// - gated on usb_serial_ready, so in the car, where nothing is listening,
//   the whole report is one function call that returns straight away
// - main-loop use only, like every printf in this firmware
//

#ifndef CONSOLE_H
#define CONSOLE_H

#include <stdbool.h>
#include <stdint.h>
#include "imu.h"
#include "env.h"
#include "gps.h"
#include "lcd.h"
#include "session_log.h"

// the bits of state that belong to main and not to any module
typedef struct {
    imu_status_t imu_s;
    imu_status_t int1_s;        // imu_int1_enable, only meaningful when imu_s is OK
    env_status_t env_s;
    gps_status_t gps_s;
    lcd_status_t lcd_s;
    log_status_t log_s;
    lcd_probe_t  lcd_probe;

    bool         env_have;      // an env sample has arrived since boot
    env_sample_t env;           // the newest one

    uint32_t reads;             // IMU samples read since boot
    uint32_t dt_max_ms;         // worst gap between samples since the last report
    uint32_t att_err;           // IMU reads that failed
    uint32_t env_err;           // BME680 measurements that failed or never finished
    uint32_t dt_max_session_ms; // worst gap between samples since boot, for the record
    uint32_t pass_max_us;       // longest loop pass since the last report
    uint32_t pass_count;        // loop passes since the last report
} console_status_t;

// build, clock, float check - first thing after the USB port is up
void console_banner(void);

// walk the whole 7-bit I2C range and print whatever answers; one ACK proves
// the pull-ups, the two pins and that part's solder joints in one go
void console_i2c_scan(void);

// what init found: sensors, the panel gate, the card
void console_report_init(const console_status_t *s);

// the once-a-second block; the caller resets its own per-second maxima after
void console_report(const console_status_t *s);

#endif // CONSOLE_H
