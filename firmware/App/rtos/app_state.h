//
// shared state - public interface
//
// - one snapshot of everything the panel, the console and the record show.
//   each task writes only its own section; readers copy the whole thing out
//   and let go before doing any I/O, so the lock is only ever held for a
//   struct copy, a few microseconds
// - the imu task is the only owner of attitude, trip and elevation, so GPS
//   fixes and BME680 samples reach it through two queues and not a call
// - the I2C mutex lives here too: the IMU and the BME680 sit on one bus
// - the mutexes and queues are used straight through CMSIS-RTOS2 calls, no
//   wrappers; only the snapshot has functions, because of the copy
//

#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "cmsis_os.h"
#include "app_tasks.h"
#include "imu.h"
#include "env.h"
#include "gps.h"
#include "lcd.h"
#include "attitude.h"
#include "trip.h"
#include "elevation.h"
#include "sd_spi.h"
#include "session_log.h"

#define APP_FIX_QUEUE_LEN  8u   // ~1.6 s of fixes at 5 Hz before one is dropped
#define APP_ENV_QUEUE_LEN  2u   // one every 3 s, so 2 is plenty

// init results; boot writes them once, before the other tasks exist
typedef struct {
    imu_status_t imu_s;
    imu_status_t int1_s;     // imu_int1_enable, only meaningful when imu_s is OK
    env_status_t env_s;
    gps_status_t gps_s;
    lcd_status_t lcd_s;
    lcd_probe_t  lcd_probe;
    bool         mount_ok;   // attitude_set_mount took the vector in vehicle_axes.h
    bool         wdg_reset;  // this boot came from the watchdog, not a power-on
} app_boot_t;

// imu task, after every wake
typedef struct {
    attitude_t   att;
    trip_stats_t trip;
    elevation_t  elev;
    float        push_long_g;   // trip_push_long_g, the gauge dot
    float        push_lat_g;    // trip_push_lat_g
    float        speed_kmh;     // trip_speed_kmh, GPS or coasted
    uint32_t     reads;         // samples read since boot
    uint32_t     read_errors;
    uint32_t     recoveries;    // reads forced because INT1 went quiet
    uint32_t     gap_max_ms;    // worst gap between two samples since boot
} app_imu_t;

// gps task, every 10 ms
typedef struct {
    gps_fix_t   fix;
    gps_stats_t stats;
    gps_link_t  link;
    uint32_t    rx_bytes;       // uart_bus_rx_count
    uint32_t    ring_overruns;  // uart_bus_overruns
    uint32_t    fixes_dropped;  // the fix queue was full
} app_gps_t;

// env task, after every measurement
typedef struct {
    bool         have;          // a sample has arrived since boot
    env_sample_t sample;        // the newest one
    uint32_t     errors;        // measurements that failed or never finished
    uint32_t     dropped;       // the env queue was full
} app_env_t;

// disp task, every frame
typedef struct {
    uint32_t bands;
    uint32_t bytes;
    uint32_t errors;
} app_disp_t;

// log task, every second
typedef struct {
    log_status_t status;
    log_stats_t  stats;
    sd_stats_t   sd;
} app_log_t;

// worst cases since the console last looked; every note keeps the larger
// value, and the con task starts them over after it prints
typedef struct {
    uint32_t imu_gap_ms;                 // dtmax
    uint32_t wake_us[APP_TASK_COUNT];    // longest wake-to-done per task
} app_second_t;

typedef struct {
    app_boot_t   boot;
    app_imu_t    imu;
    app_gps_t    gps;
    app_env_t    env;
    app_disp_t   disp;
    app_log_t    log;
    app_second_t second;
} app_state_t;

// one item on the env queue; the tick lets the imu task work out the time
// between two samples
typedef struct {
    env_sample_t sample;
    uint32_t     tick;
} app_env_msg_t;

// made by app_state_init
extern osMutexId_t        app_i2c_mutex;   // held around every I2C transfer once the tasks run
extern osMessageQueueId_t app_fix_queue;   // gps_fix_t, gps -> imu
extern osMessageQueueId_t app_env_queue;   // app_env_msg_t, env -> imu

// make the mutexes and the queues; call before the scheduler starts
void app_state_init(void);

// each task writes its own section and nothing else
void app_state_put_boot(const app_boot_t *v);
void app_state_put_imu(const app_imu_t *v);
void app_state_put_gps(const app_gps_t *v);
void app_state_put_env(const app_env_t *v);
void app_state_put_disp(const app_disp_t *v);
void app_state_put_log(const app_log_t *v);

// keep the larger of what is stored and what is new
void app_state_note_gap(uint32_t ms);
void app_state_note_wake(app_task_t t, uint32_t us);

// copy everything out
void app_state_get(app_state_t *out);

// copy everything out and start the worst cases over; the con task only
void app_state_take(app_state_t *out);

// just the newest fix - for get_fattime, which FatFs calls from the log task
void app_state_get_fix(gps_fix_t *out);

#endif // APP_STATE_H
