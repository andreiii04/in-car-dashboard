//
// watchdog - implementation
//
// - the check-ins are plain HAL ticks, one per task; a 32-bit store cannot
//   tear, and the wdg task outranks every task it checks, so no lock
// - MX_IWDG_Init is not called from main (CubeMX "do not generate call") -
//   started from there it would run out during the 3 s USB wait at boot
//

#include "watchdog.h"
#include "iwdg.h"
#include "stm32f4xx_hal.h"

// the longest each task may go without checking in, well past its own
// period so one slow pass is not a fault; 0 means not watched - the wdg
// task cannot watch itself
static const uint32_t deadline_ms[APP_TASK_COUNT] = {
    [APP_TASK_IMU]  = 100u,    // wakes at least every 20 ms
    [APP_TASK_GPS]  = 100u,    // every 10 ms
    [APP_TASK_ENV]  = 5000u,   // every 3 s
    [APP_TASK_DISP] = 250u,    // every 50 ms
    [APP_TASK_CON]  = 3000u,   // every 1 s
    [APP_TASK_LOG]  = 5000u,   // every 1 s; a record write is ~1.3 s worst case
    [APP_TASK_WDG]  = 0u,
};

static volatile uint32_t last_checkin[APP_TASK_COUNT];

// reloads skipped because a task was late - for the debugger
static uint32_t late_count;

void watchdog_checkin(app_task_t t) {
    last_checkin[t] = HAL_GetTick();
}

bool watchdog_was_reset(void) {
    bool iwdg = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != 0u);
    __HAL_RCC_CLEAR_RESET_FLAGS();
    return iwdg;
}

void watchdog_start(void) {
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0; i < (uint32_t)APP_TASK_COUNT; i++) {
        last_checkin[i] = now;
    }
    // stop the count while the core sits at a breakpoint, or every debug
    // session would end in a reset (DBGMCU_APB1_FZ_DBG_IWDG_STOP)
    __HAL_DBGMCU_FREEZE_IWDG();
    MX_IWDG_Init();
}

bool watchdog_feed(void) {
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0; i < (uint32_t)APP_TASK_COUNT; i++) {
        if ((deadline_ms[i] != 0u) && ((now - last_checkin[i]) > deadline_ms[i])) {
            late_count++;
            return false;
        }
    }
    HAL_IWDG_Refresh(&hiwdg);
    return true;
}
