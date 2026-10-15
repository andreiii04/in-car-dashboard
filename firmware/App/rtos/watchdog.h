//
// watchdog - public interface
//
// - the IWDG is a counter on its own ~32 kHz clock (the LSI) that resets the
//   chip unless it is reloaded in time; once started nothing can stop it
// - set to ~2 s: prescaler 32, reload 1999 in CubeMX, which is 1.36-3.76 s
//   across the LSI's 17-47 kHz spread (DS10693 table 42)
// - every task checks in on each pass; the wdg task reloads the IWDG only if
//   all of them checked in within their own deadline. one stuck task is
//   enough to stop the reloads, and ~2 s later the chip resets
// - a late task only skips reloads, so a one-off slow pass costs nothing;
//   it has to stay stuck for the whole ~2 s
// - a watchdog reset is a new power-on to the rest of the firmware, so it
//   starts a new session record; the console says when that happened
//

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <stdbool.h>
#include "app_tasks.h"

// how often the wdg task looks
#define WATCHDOG_PERIOD_MS 250u

// note that a task is alive; one store, call it every pass
void watchdog_checkin(app_task_t t);

// read and clear the reset flags; true if the last reset came from the
// IWDG. call once, early in boot - the flags survive a reset until cleared
bool watchdog_was_reset(void);

// freeze the IWDG while the debugger has the core halted, then start it
void watchdog_start(void);

// reload the IWDG if every task checked in on time; false, and no reload,
// when one is late
bool watchdog_feed(void);

#endif // WATCHDOG_H
