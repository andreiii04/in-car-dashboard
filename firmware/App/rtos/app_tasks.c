//
// RTOS tasks - implementation
//
// - app_boot runs the init the superloop used to run before its loop, in the
//   same order, then creates the tasks and lets the boot task end
// - every task is a plain loop: wait, do its one job, publish what it has,
//   check in with the watchdog. the waits are what hand the CPU over -
//   nothing in here spins
// - the imu task is the one that matters: highest after the watchdog, woken
//   by the LSM6DSO's own data-ready line, and the only task that touches
//   attitude, trip and elevation. the card is the lowest, so a 250 ms record
//   write no longer costs IMU samples
// - each task times its own pass, wake to done, with the DWT cycle counter;
//   the con task prints the worst one per second
//

#include "app_tasks.h"
#include <math.h>
#include <string.h>
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"   // xPortGetFreeHeapSize - CMSIS-RTOS2 has no heap call
#include "app_state.h"
#include "watchdog.h"
#include "usb_serial.h"
#include "uart_bus.h"
#include "imu.h"
#include "env.h"
#include "gps.h"
#include "vehicle_axes.h"
#include "attitude.h"
#include "trip.h"
#include "elevation.h"
#include "lcd.h"
#include "gfx.h"
#include "screen.h"
#include "sd_spi.h"
#include "session_log.h"
#include "session_record.h"
#include "console.h"
#include "bench_cal.h"

// task timing, in ms
#define IMU_WAIT_MS      20u      // longest the imu task sleeps without an edge
#define IMU_QUIET_MS     50u      // no edge this long - read once to drop the latched line
#define GPS_PERIOD_MS    10u      // ~12 bytes at the 5 Hz load; the ring holds ~1.7 s
#define ENV_PERIOD_MS    3000u    // one BME680 measurement every 3 s
#define ENV_GIVE_UP_MS   1000u    // a measurement that never finishes is dropped after this
#define ENV_RETRY_MS     5u       // asked too early - look again this much later
#define DISP_PERIOD_MS   50u      // the gauge dot at 20 Hz; the numbers change slower anyway
#define CON_PERIOD_MS    1000u
#define LOG_TICK_MS      1000u    // how often the log task looks
#define LOG_PERIOD_MS    10000u   // rewrite the session record every 10 s
#define LOG_RETRY_MS     5000u    // look for a card again every 5 s while there is none
#define BOOT_SCREEN_MS   2000u    // the boot screen stays up this long
#define USB_WAIT_MS      3000u    // how long boot waits for a terminal to attach
#define USB_POLL_MS      10u
#define LCD_BRIGHTNESS   1000u    // permille; nothing to dim it from yet
#define SYSCLK_MHZ       168u     // DWT cycle counter -> microseconds

// the thread flag the INT1 interrupt sets on the imu task
#define IMU_FLAG_EDGE    0x0001u

// stacks in bytes; trimmed later from the high-water marks on the console
#if BENCH_CAL
#define IMU_STACK        3072u    // bench_cal's average and its printf run on this stack
#else
#define IMU_STACK        2048u
#endif
#define GPS_STACK        1536u
#define ENV_STACK        1536u    // 1024 left only 236 bytes on the board (15 Oct)
#define DISP_STACK       2048u
#define CON_STACK        2560u
#define LOG_STACK        3072u
#define WDG_STACK        512u

static const osThreadAttr_t imu_attr = {
    .name = "imu", .stack_size = IMU_STACK, .priority = osPriorityHigh,
};
static const osThreadAttr_t gps_attr = {
    .name = "gps", .stack_size = GPS_STACK, .priority = osPriorityAboveNormal,
};
static const osThreadAttr_t env_attr = {
    .name = "env", .stack_size = ENV_STACK, .priority = osPriorityNormal,
};
static const osThreadAttr_t disp_attr = {
    .name = "disp", .stack_size = DISP_STACK, .priority = osPriorityBelowNormal,
};
static const osThreadAttr_t con_attr = {
    .name = "con", .stack_size = CON_STACK, .priority = osPriorityLow1,
};
static const osThreadAttr_t log_attr = {
    .name = "log", .stack_size = LOG_STACK, .priority = osPriorityLow,
};
static const osThreadAttr_t wdg_attr = {
    .name = "wdg", .stack_size = WDG_STACK, .priority = osPriorityRealtime,
};

static const char *const task_names[APP_TASK_COUNT] = {
    "imu", "gps", "env", "disp", "con", "log", "wdg",
};

static osThreadId_t task_id[APP_TASK_COUNT];

// file scope so the debugger can live-watch them; each one belongs to the
// task named in front of it, and only that task touches it after boot
static app_boot_t boot;            // boot, read-only afterwards
static bool       display_ok;      // boot, read-only afterwards
static uint32_t   boot_tick;       // boot, read-only afterwards
static app_imu_t  imu_out;         // imu
static app_gps_t  gps_out;         // gps
static app_env_t  env_out;         // env
static app_disp_t disp_out;        // disp
static app_log_t  log_out;         // log, first written by boot
static app_state_t disp_view;      // disp
static app_state_t con_view;       // con
static app_state_t log_view;       // log
static app_rtos_t  con_rtos;       // con
static screen_data_t    screen;    // disp - what the panel is being shown
static session_record_t record;    // log - what the card is being sent

// ---- helpers ----------------------------------------------------------------

// the DWT cycle counter: 168 counts per microsecond, free-running, the only
// clock fine enough to time one task pass
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// microseconds since a DWT stamp; wraps every ~25 s, far past any pass
static uint32_t us_since(uint32_t start) {
    return (DWT->CYCCNT - start) / SYSCLK_MHZ;
}

// sleep until the next period boundary. after a long stall, count again
// from now instead of running the missed periods back to back
static void sleep_until(uint32_t *next, uint32_t period_ms) {
    *next += period_ms;
    if ((int32_t)(*next - osKernelGetTickCount()) <= 0) {
        *next = osKernelGetTickCount() + period_ms;
    }
    osDelayUntil(*next);
}

// called from the EXTI interrupt on every INT1 edge, at priority 5 - the
// highest one allowed to call the kernel. wakes the imu task, nothing else
static void imu_edge_isr(void) {
    osThreadFlagsSet(task_id[APP_TASK_IMU], IMU_FLAG_EDGE);
}

const char *app_task_name(app_task_t t) {
    return (t < APP_TASK_COUNT) ? task_names[t] : "?";
}

// ---- imu task ---------------------------------------------------------------

// one IMU sample: read, map into car axes, step the attitude filter, feed
// the trip stats. about 105 times a second
static void imu_tick(uint32_t dt_ms) {
    imu_sample_t raw;
    imu_status_t s;
    float sensor[3];
    float accel[3];
    float gyro[3];
    attitude_t att;

    // the BME680 is on the same bus; at worst this waits one of its
    // transfers - ~2.4 ms measured at 100 kHz, inside the 9.5 ms between samples
    osMutexAcquire(app_i2c_mutex, osWaitForever);
    s = imu_read(&raw);
    osMutexRelease(app_i2c_mutex);
    if (s != IMU_OK) {
        imu_out.read_errors++;   // counted, not printed
        return;
    }
    imu_out.reads++;             // the read is also what drops INT1 again

    // sensor axes -> car axes; nothing above imu.c ever sees sensor axes
    sensor[0] = raw.accel_x_g;
    sensor[1] = raw.accel_y_g;
    sensor[2] = raw.accel_z_g;
    vehicle_map(sensor, accel);
    sensor[0] = raw.gyro_x_dps;
    sensor[1] = raw.gyro_y_dps;
    sensor[2] = raw.gyro_z_dps;
    vehicle_map(sensor, gyro);

    float dt_s = (float)dt_ms / 1000.0f;
    attitude_update(accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2], dt_s);
    attitude_get(&att);

    // trip gets the sample after the mount rotation and the gyro bias, the
    // same numbers the filter just used. handing it the board's own axes left
    // 0.69 g of gravity on the forward axis at the 43.4 deg dash mount
    attitude_get_car_axes(accel, gyro);
    trip_update_motion(accel, gyro, att.pitch_deg, att.roll_deg, dt_s);

    // tell the filter how hard the car is pushing, every sample - trip works
    // the sideways part out from speed x yaw rate. handed over only once per
    // GPS fix, a bend went up to 1 s uncorrected, which read as ~9 deg of
    // roll in session 17. used from the next sample on
    attitude_set_linear_accel(trip_long_accel_g(), trip_lat_accel_g());
}

static void imu_task(void *arg) {
    uint32_t last_sample = HAL_GetTick();   // HAL tick of the last sample
    uint32_t last_edge = imu_int1_count();  // last INT1 count acted on
    gps_fix_t fix;
    gps_fix_t latest;                       // the newest fix off the queue
    app_env_msg_t env;
    uint32_t last_env_tick = 0;
    bool env_have = false;

    (void)arg;
    memset(&latest, 0, sizeof(latest));

    for (;;) {
        // sleep until INT1 fires, or 20 ms, whichever comes first. the flag
        // only wakes the task; the edge counter decides whether there is a
        // sample, same as the superloop did
        (void)osThreadFlagsWait(IMU_FLAG_EDGE, osFlagsWaitAny, IMU_WAIT_MS);
        uint32_t t0 = DWT->CYCCNT;
        uint32_t now = HAL_GetTick();

        if (boot.imu_s == IMU_OK) {
            uint32_t edge = imu_int1_count();
            // INT1 is latched: if a sample is ever left unread the line stays
            // high and no edge comes again. no edge for 50 ms means exactly
            // that, so read once anyway - that drops the line. it also covers
            // the first sample, which arrived before this task existed
            bool quiet = ((now - last_sample) >= IMU_QUIET_MS);
            if ((edge != last_edge) || quiet) {
                uint32_t dt_ms = now - last_sample;
                if (edge == last_edge) {
                    imu_out.recoveries++;
                }
                last_edge = edge;
                last_sample = now;
                if (dt_ms > imu_out.gap_max_ms) {
                    imu_out.gap_max_ms = dt_ms;   // goes in the record's faults
                }
                app_state_note_gap(dt_ms);
                imu_tick(dt_ms);
            }
        }

        // every new fix, once each - every call restarts trip's between-fixes
        // clock, so the gps task only sends a fix whose updated_tick moved
        while (osMessageQueueGet(app_fix_queue, &fix, NULL, 0u) == osOK) {
            trip_update_gps(&fix);
            latest = fix;
        }

        // every BME680 sample; the time between two comes from their own ticks
        while (osMessageQueueGet(app_env_queue, &env, NULL, 0u) == osOK) {
            float dt_s = env_have ? (float)(env.tick - last_env_tick) / 1000.0f
                                  : (float)ENV_PERIOD_MS / 1000.0f;
            last_env_tick = env.tick;
            env_have = true;
            trip_update_env(env.sample.temperature_c, env.sample.pressure_hpa,
                            env.sample.humidity_pct);
            elev_update(env.sample.pressure_hpa, latest.fix_valid, latest.altitude_m, dt_s);
        }

#if BENCH_CAL
        // blocks ~4.8 s every 20 s and reads the IMU itself, so it holds the
        // bus for the whole average; no watchdog in this build
        osMutexAcquire(app_i2c_mutex, osWaitForever);
        bench_cal_tick(boot.imu_s == IMU_OK);
        osMutexRelease(app_i2c_mutex);
#endif

        attitude_get(&imu_out.att);
        trip_get(&imu_out.trip);
        elev_get(&imu_out.elev);
        imu_out.push_long_g = trip_push_long_g();
        imu_out.push_lat_g = trip_push_lat_g();
        imu_out.speed_kmh = trip_speed_kmh();
        app_state_put_imu(&imu_out);

        watchdog_checkin(APP_TASK_IMU);
        app_state_note_wake(APP_TASK_IMU, us_since(t0));
    }
}

// ---- gps task ---------------------------------------------------------------

// polls the ring instead of being woken by the UART: that keeps the USART1
// interrupt out of the kernel, so it stays at priority 0, above everything
static void gps_task(void *arg) {
    uint32_t next = osKernelGetTickCount();
    uint32_t last_fix_tick = 0;   // updated_tick of the last fix sent on

    (void)arg;

    for (;;) {
        uint32_t t0 = DWT->CYCCNT;

        if (boot.gps_s == GPS_OK) {
            gps_poll();
            gps_get_fix(&gps_out.fix);
            if (gps_out.fix.updated_tick != last_fix_tick) {
                last_fix_tick = gps_out.fix.updated_tick;
                // never wait for room - a full queue means the imu task is
                // stuck, and that is the watchdog's business
                if (osMessageQueuePut(app_fix_queue, &gps_out.fix, 0u, 0u) != osOK) {
                    gps_out.fixes_dropped++;
                }
            }
            gps_get_stats(&gps_out.stats);
            gps_out.link = gps_link();
            gps_out.rx_bytes = uart_bus_rx_count();
            gps_out.ring_overruns = uart_bus_overruns();
            app_state_put_gps(&gps_out);
        }

        watchdog_checkin(APP_TASK_GPS);
        app_state_note_wake(APP_TASK_GPS, us_since(t0));
        sleep_until(&next, GPS_PERIOD_MS);
    }
}

// ---- env task ---------------------------------------------------------------

// one BME680 measurement: trigger, sleep while it runs (~143 ms), collect.
// the same two halves the superloop used, with a sleep in the gap. each half
// is timed on its own, so the console shows the work and not the sleep
static void env_measure(void) {
    uint32_t start = HAL_GetTick();
    uint32_t wait_ms = 0;
    uint32_t t0 = DWT->CYCCNT;
    env_sample_t sample;
    app_env_msg_t msg;
    env_status_t s;

    osMutexAcquire(app_i2c_mutex, osWaitForever);
    s = env_start(&wait_ms);
    osMutexRelease(app_i2c_mutex);
    app_state_note_wake(APP_TASK_ENV, us_since(t0));
    if (s != ENV_OK) {
        env_out.errors++;
        return;
    }

    osDelay(wait_ms);
    for (;;) {
        t0 = DWT->CYCCNT;
        osMutexAcquire(app_i2c_mutex, osWaitForever);
        s = env_read(&sample);
        osMutexRelease(app_i2c_mutex);
        app_state_note_wake(APP_TASK_ENV, us_since(t0));
        if (s != ENV_ERR_NO_DATA) {
            break;
        }
        if ((HAL_GetTick() - start) > ENV_GIVE_UP_MS) {
            env_out.errors++;   // stuck; the next period triggers again
            return;
        }
        osDelay(ENV_RETRY_MS);  // not done yet, look again soon
    }
    if (s != ENV_OK) {
        env_out.errors++;       // bus error; try again next period
        return;
    }

    env_out.have = true;
    env_out.sample = sample;
    msg.sample = sample;
    msg.tick = HAL_GetTick();
    if (osMessageQueuePut(app_env_queue, &msg, 0u, 0u) != osOK) {
        env_out.dropped++;
    }
}

static void env_task(void *arg) {
    uint32_t next = osKernelGetTickCount();

    (void)arg;

    for (;;) {
        if (boot.env_s == ENV_OK) {
            env_measure();
            app_state_put_env(&env_out);
        }
        watchdog_checkin(APP_TASK_ENV);
        sleep_until(&next, ENV_PERIOD_MS);
    }
}

// ---- disp task --------------------------------------------------------------

// everything the panel shows, out of the snapshot
static void gather_screen(screen_data_t *d, const app_state_t *s) {
    const trip_stats_t *t = &s->imu.trip;
    const gps_fix_t *f = &s->gps.fix;
    const elevation_t *e = &s->imu.elev;

    d->long_g = s->imu.push_long_g;
    d->lat_g = s->imu.push_lat_g;
    d->peak_accel_g = t->accel_peak_pos_g;
    // stored negative, shown as a size; fabsf and not a minus, because minus
    // zero prints as -0.0 on the panel
    d->peak_brake_g = fabsf(t->accel_peak_neg_g);
    d->peak_left_g = t->accel_peak_left_g;
    d->peak_right_g = fabsf(t->accel_peak_right_g);

    d->speed_kmh = s->imu.speed_kmh;
    d->speed_max_kmh = t->speed_max_kmh;
    d->distance_km = t->distance_km;
    d->coasting = t->coasting;

    d->env_valid = s->env.have;
    d->temp_c = s->env.sample.temperature_c;
    d->hum_pct = s->env.sample.humidity_pct;
    d->press_hpa = s->env.sample.pressure_hpa;
    d->gas_valid = s->env.sample.gas_valid;
    d->gas_kohm = s->env.sample.gas_resistance_ohm / 1000.0f;

    d->duration_s = t->duration_s;
    d->sd_ok = s->log.stats.mounted;

    d->fix_valid = f->fix_valid;
    d->time_valid = f->time_valid;
    d->sats = f->sats_used;
    d->lat_deg = f->lat_deg;
    d->lon_deg = f->lon_deg;
    // the fused altitude, not the raw fix: GPS height wanders by tens of
    // metres on a poor fix and the barometer is what steadies it
    d->alt_m = e->valid ? e->altitude_m : f->altitude_m;
    d->alt_settled = e->valid && e->settled;
    d->hour = f->hour;
    d->minute = f->minute;
    d->second = f->second;
}

static void disp_task(void *arg) {
    uint32_t next = osKernelGetTickCount();
    bool boot_screen_up = display_ok;

    (void)arg;

    for (;;) {
        uint32_t t0 = DWT->CYCCNT;
        uint32_t now = HAL_GetTick();

        if (display_ok) {
            if (boot_screen_up && ((now - boot_tick) >= BOOT_SCREEN_MS)) {
                boot_screen_up = false;
                screen_show_main();
            }
            if (!boot_screen_up) {
                app_state_get(&disp_view);
                gather_screen(&screen, &disp_view);
                screen_update(&screen);
            }
            // every dirty band of the frame in one go - the imu task preempts
            // this whenever INT1 fires, so the length of the frame no longer
            // matters. a dot move is 2 bands, a full frame 15
            while (gfx_flush_band()) {
            }
            // one ramp step per call at most, so the fade-in takes ~1.5 s at
            // 20 Hz, inside the 2 s boot screen
            lcd_backlight_tick();
        }
        disp_out.bands = gfx_bands_sent();
        disp_out.bytes = lcd_bytes_sent();
        disp_out.errors = lcd_errors();
        app_state_put_disp(&disp_out);

        watchdog_checkin(APP_TASK_DISP);
        app_state_note_wake(APP_TASK_DISP, us_since(t0));
        sleep_until(&next, DISP_PERIOD_MS);
    }
}

// ---- con task ---------------------------------------------------------------

// what only the kernel knows: stack left per task and the RTOS heap
static void rtos_figures(app_rtos_t *out) {
    for (uint32_t i = 0; i < (uint32_t)APP_TASK_COUNT; i++) {
        out->stack_free[i] = osThreadGetStackSpace(task_id[i]);
    }
    out->heap_free = (uint32_t)xPortGetFreeHeapSize();
    out->heap_min = (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

static void con_task(void *arg) {
    uint32_t next = osKernelGetTickCount();

    (void)arg;

    for (;;) {
        sleep_until(&next, CON_PERIOD_MS);
        uint32_t t0 = DWT->CYCCNT;

        // take, not get: the worst cases start over for the next second
        app_state_take(&con_view);
        rtos_figures(&con_rtos);
        console_report(&con_view, &con_rtos);

        watchdog_checkin(APP_TASK_CON);
        app_state_note_wake(APP_TASK_CON, us_since(t0));
    }
}

// ---- log task ---------------------------------------------------------------

// every counter a failure would have turned up in, collected once per record.
// the card's own numbers come straight from this task's modules, the rest
// out of the snapshot
static void gather_faults(session_faults_t *f, const app_state_t *s) {
    sd_stats_t sd;
    log_stats_t log;

    memset(f, 0, sizeof(*f));
    sd_spi_get_stats(&sd);
    session_log_get_stats(&log);

    f->imu_init = s->boot.imu_s;
    f->env_init = s->boot.env_s;
    f->gps_init = s->boot.gps_s;
    f->display_init = s->boot.lcd_s;
    f->card_mounted = log.mounted;
    f->fatfs_error = log.fatfs_err;

    f->imu_read_errors = s->imu.read_errors;
    f->env_read_errors = s->env.errors;
    f->gps_bad_checksums = s->gps.stats.bad;
    f->gps_ring_overruns = s->gps.ring_overruns;
    f->display_errors = s->disp.errors;
    f->card_crc_errors = sd.crc_errors;
    f->card_timeouts = sd.timeouts;
    f->card_rejected = sd.rejected;
    f->log_write_errors = log.errors;

    f->longest_imu_gap_ms = s->imu.gap_max_ms;
    f->slowest_log_write_ms = log.max_write_ms;

    f->gps_date_refused_have = (s->gps.stats.fix_no_date > 0u);
    f->gps_date_refused_empty = !s->gps.stats.rejected_have;
    f->gps_date_refused_day = s->gps.stats.rejected_day;
    f->gps_date_refused_month = s->gps.stats.rejected_month;
    f->gps_date_refused_year = s->gps.stats.rejected_year;
}

static void log_task(void *arg) {
    uint32_t next = osKernelGetTickCount();
    uint32_t last_log = HAL_GetTick();
    uint32_t last_retry = last_log;

    (void)arg;

    for (;;) {
        sleep_until(&next, LOG_TICK_MS);
        uint32_t t0 = DWT->CYCCNT;
        uint32_t now = HAL_GetTick();

        // rewrite this drive's record every 10 s, or keep looking for a card
        // every 5 s until one turns up
        if (session_log_ready()) {
            if ((now - last_log) >= LOG_PERIOD_MS) {
                session_faults_t faults;
                last_log = now;
                app_state_get(&log_view);
                gather_faults(&faults, &log_view);
                session_record_build(&record, session_log_session_n(),
                                     &log_view.imu.trip, &log_view.imu.elev, &faults);
                log_out.status = session_log_write(&record);
            }
        } else if ((now - last_retry) >= LOG_RETRY_MS) {
            last_retry = now;
            log_out.status = session_log_init();
            last_log = now;
        }

        session_log_get_stats(&log_out.stats);
        sd_spi_get_stats(&log_out.sd);
        app_state_put_log(&log_out);

        watchdog_checkin(APP_TASK_LOG);
        app_state_note_wake(APP_TASK_LOG, us_since(t0));
    }
}

// ---- wdg task ---------------------------------------------------------------

static void wdg_task(void *arg) {
    uint32_t next = osKernelGetTickCount();

    (void)arg;

    // a BENCH_CAL build blocks the imu task ~4.8 s at a time, so it runs
    // with no watchdog at all
    if (BENCH_CAL == 0) {
        watchdog_start();
    }
    for (;;) {
        if (BENCH_CAL == 0) {
            (void)watchdog_feed();
        }
        sleep_until(&next, WATCHDOG_PERIOD_MS);
    }
}

// ---- boot -------------------------------------------------------------------

// make one task; nothing works without all of them, so a failure stops here
static void start_task(app_task_t t, osThreadFunc_t fn, const osThreadAttr_t *attr) {
    task_id[t] = osThreadNew(fn, NULL, attr);
    if (task_id[t] == NULL) {
        Error_Handler();
    }
}

void app_boot(void) {
    boot.wdg_reset = watchdog_was_reset();
    dwt_init();
    usb_serial_init();

    // the host needs a moment to enumerate the port and nothing printed
    // before that can arrive; wait for it, but not forever - the board still
    // has to run in the car with no host attached
    uint32_t wait = HAL_GetTick();
    while (!usb_serial_ready() && ((HAL_GetTick() - wait) < USB_WAIT_MS)) {
        osDelay(USB_POLL_MS);
    }
    console_banner();
    console_i2c_scan();

    // no I2C mutex yet - nothing else is running
    boot.imu_s = imu_init();
    boot.env_s = env_init();

    // hand the filter the two measured constants out of vehicle_axes.h; both
    // were recorded after vehicle_map, which is the frame attitude.c works in
    attitude_init();
    boot.mount_ok = attitude_set_mount(VEHICLE_REST_ACCEL_X, VEHICLE_REST_ACCEL_Y,
                                       VEHICLE_REST_ACCEL_Z);
    attitude_set_gyro_bias(VEHICLE_GYRO_BIAS_X, VEHICLE_GYRO_BIAS_Y, VEHICLE_GYRO_BIAS_Z);
    // hand the sensor its own interrupt line; from here the imu task is
    // paced by the LSM6DSO and not by a timer
    if (boot.imu_s == IMU_OK) {
        boot.int1_s = imu_int1_enable();
    }
    trip_init();
    elev_init();

    // the display: the ID gate first, then the full init, then the boot
    // screen while the backlight fades up. the gate is advisory - this module
    // never drives SDO, so every read answers FF
    (void)lcd_probe(&boot.lcd_probe);
    boot.lcd_s = lcd_init();
    display_ok = (boot.lcd_s == LCD_OK);
    gfx_init(screen_paint, lcd_write_band);
    screen_init();
    if (display_ok) {
        screen_show_boot();
        while (gfx_flush_band()) {
        }
        lcd_backlight_set(LCD_BRIGHTNESS);
    }

    // the card: mount, open the file, claim this drive's record. no card
    // detect pin, so a missing card is just a failed init, retried by log
    log_out.status = session_log_init();

    // starts the USART1 receive interrupt; last on purpose - the panel init
    // and the card mount above stall for longer than the ring would last
    boot.gps_s = gps_init();

    app_state_put_boot(&boot);
    session_log_get_stats(&log_out.stats);
    sd_spi_get_stats(&log_out.sd);
    app_state_put_log(&log_out);
    console_report_init(&boot, log_out.status);
    boot_tick = HAL_GetTick();

    // every print above is done, so con is the only printer from here on.
    // boot runs at Realtime, so none of these starts before boot ends
    start_task(APP_TASK_WDG, wdg_task, &wdg_attr);
    start_task(APP_TASK_IMU, imu_task, &imu_attr);
    start_task(APP_TASK_GPS, gps_task, &gps_attr);
    start_task(APP_TASK_ENV, env_task, &env_attr);
    start_task(APP_TASK_DISP, disp_task, &disp_attr);
    start_task(APP_TASK_CON, con_task, &con_attr);
    start_task(APP_TASK_LOG, log_task, &log_attr);

    // only now is there a task to wake; until here the edges were counted
    // and nothing else, and the imu task's first quiet read picks them up
    imu_set_edge_callback(imu_edge_isr);
}
