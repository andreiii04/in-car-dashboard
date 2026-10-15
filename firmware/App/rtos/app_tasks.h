//
// RTOS tasks - public interface
//
// - the boot sequence and every task loop live in app_tasks.c; main.c only
//   starts the scheduler, freertos.c only calls app_boot
// - one task per job, and each one is the only owner of its hardware:
//     wdg   Realtime      every 250 ms   the IWDG
//     imu   High          INT1 edge      LSM6DSO, attitude, trip, elevation
//     gps   AboveNormal   every 10 ms    USART1 ring and the parser
//     env   Normal        every 3 s      the BME680
//     disp  BelowNormal   every 50 ms    SPI1 and the panel
//     con   Low1          every 1 s      every printf
//     log   Low           every 1 s      SPI2, FatFs and the record
// - the IMU and the BME680 share I2C1 through a mutex; everything else
//   reaches the other tasks through app_state
//

#ifndef APP_TASKS_H
#define APP_TASKS_H

#include <stdint.h>

// the tasks by name; also the index for the watchdog check-ins and the
// per-task console figures
typedef enum {
    APP_TASK_IMU = 0,
    APP_TASK_GPS,
    APP_TASK_ENV,
    APP_TASK_DISP,
    APP_TASK_CON,
    APP_TASK_LOG,
    APP_TASK_WDG,
    APP_TASK_COUNT,
} app_task_t;

// what the con task reads off the kernel itself, for the console
typedef struct {
    uint32_t stack_free[APP_TASK_COUNT];  // bytes of stack never touched since boot
    uint32_t heap_free;                   // RTOS heap free right now
    uint32_t heap_min;                    // the least it has ever been
} app_rtos_t;

// the boot task's whole job: every init in the order the superloop had it,
// then the other tasks. returns once they exist; the boot task then ends
void app_boot(void);

// short name of a task, for the console
const char *app_task_name(app_task_t t);

#endif // APP_TASKS_H
