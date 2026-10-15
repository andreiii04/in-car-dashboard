//
// console module - public interface
//
// - every printf in the firmware: the banner, the init results, the
//   once-a-second status block
// - the per-second block is built from the shared snapshot (app_state), not
//   from the modules' own getters - each module now belongs to one task, and
//   only the snapshot is safe to read from another
// - gated on usb_serial_ready, so in the car, where nothing is listening,
//   the whole report is one function call that returns straight away
// - the boot task prints the start, the con task everything after; never
//   from two tasks at once, never from an ISR
//

#ifndef CONSOLE_H
#define CONSOLE_H

#include "app_state.h"
#include "app_tasks.h"
#include "session_log.h"

// build, clock, float check - first thing after the USB port is up
void console_banner(void);

// walk the whole 7-bit I2C range and print whatever answers; one ACK proves
// the pull-ups, the two pins and that part's solder joints in one go
void console_i2c_scan(void);

// what init found: reset cause, sensors, the panel gate, the card
void console_report_init(const app_boot_t *b, log_status_t log_s);

// the once-a-second block; s comes from app_state_take, r from the kernel
void console_report(const app_state_t *s, const app_rtos_t *r);

#endif // CONSOLE_H
