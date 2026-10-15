//
// console module - implementation
//
// - the status-to-text helpers are here so a failure reads as a reason and
//   not a number
// - the per-second block is the bring-up evidence for every step at once:
//   INT1 edges against reads, dtmax, the GPS ladder, the panel byte count,
//   the card's write times. print less often rather than shorter, USB CDC
//   does not care about bytes, only about lines
//

#include "console.h"
#include <stdio.h>
#include "usb_serial.h"
#include "i2c_bus.h"
#include "sd_spi.h"
#include "imu.h"
#include "stm32f4xx_hal.h"

static const char *imu_status_str(imu_status_t s) {
    switch (s) {
    case IMU_OK:          return "OK";
    case IMU_ERR_ARG:     return "ERR_ARG - null pointer";
    case IMU_ERR_BUS:     return "ERR_BUS - no ACK or transfer failed";
    case IMU_ERR_ID:      return "ERR_ID - WHO_AM_I mismatch";
    case IMU_ERR_TIMEOUT: return "ERR_TIMEOUT - reset never finished";
    default:              return "unknown";
    }
}

static const char *env_status_str(env_status_t s) {
    switch (s) {
    case ENV_OK:          return "OK";
    case ENV_ERR_ARG:     return "ERR_ARG - null pointer";
    case ENV_ERR_BUS:     return "ERR_BUS - no ACK or transfer failed";
    case ENV_ERR_ID:      return "ERR_ID - chip ID mismatch";
    case ENV_ERR_NO_DATA: return "ERR_NO_DATA - measurement not finished";
    default:              return "unknown";
    }
}

static const char *gps_status_str(gps_status_t s) {
    switch (s) {
    case GPS_OK:       return "OK";
    case GPS_ERR_ARG:  return "ERR_ARG - null pointer";
    case GPS_ERR_UART: return "ERR_UART - HAL refused to start RX";
    default:           return "unknown";
    }
}

static const char *lcd_status_str(lcd_status_t s) {
    switch (s) {
    case LCD_OK:      return "OK";
    case LCD_ERR_ARG: return "ERR_ARG - null pointer";
    case LCD_ERR_SPI: return "ERR_SPI - a transfer failed";
    case LCD_ERR_ID:  return "ERR_ID - panel did not answer like an ILI9341";
    default:          return "unknown";
    }
}

static const char *log_status_str(log_status_t s) {
    switch (s) {
    case LOG_OK:            return "OK";
    case LOG_ERR_NOT_READY: return "ERR_NOT_READY - no card mounted";
    case LOG_ERR_CARD:      return "ERR_CARD - nothing answered CMD0";
    case LOG_ERR_MOUNT:     return "ERR_MOUNT - no usable filesystem";
    case LOG_ERR_FILE:      return "ERR_FILE - SESSIONS.JSON open or read failed";
    case LOG_ERR_WRITE:     return "ERR_WRITE - write or sync failed";
    default:                return "unknown";
    }
}

void console_banner(void) {
    printf("\n=== dashboard firmware ===\n");
    printf("build %s %s\n", __DATE__, __TIME__);
    printf("sysclk %lu Hz\n", (unsigned long)HAL_RCC_GetSysClockFreq());
    printf("float check: %.2f (want 1.25)\n", 1.25f);
}

void console_i2c_scan(void) {
    uint8_t found = 0;

    printf("i2c scan:");
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_bus_probe(addr) == I2C_BUS_OK) {
            printf(" 0x%02X", addr);
            found++;
        }
    }
    if (found == 0u) {
        printf(" nothing");
    }
    printf("   (%u found, want 0x6A + 0x76)\n", found);
}

void console_report_init(const app_boot_t *b, log_status_t log_s) {
    const sd_info_t *sd = sd_spi_info();
    log_stats_t log;

    // a watchdog reset looks like a power-on to everything else - it even
    // starts a new session record - so this line is the only trace of it
    printf("reset: %s\n", b->wdg_reset ? "WATCHDOG" : "power-on, pin or debugger");

    printf("imu_init: %s\n", imu_status_str(b->imu_s));
    if (b->imu_s == IMU_OK) {
        printf("imu_int1_enable: %s\n", imu_status_str(b->int1_s));
    }
    if (!b->mount_ok) {
        printf("attitude: mount vector too short, staying level\n");
    }
    printf("env_init: %s\n", env_status_str(b->env_s));
    printf("gps_init: %s\n", gps_status_str(b->gps_s));

    // the ID gate, raw and decoded; either decode landing on 009341 passes,
    // and 06 -> 05 on the pixel format is the write-then-read proof
    printf("lcd probe: raw D3h %02X %02X %02X %02X  dummy-bit %06lX  dummy-byte %06lX  %s\n",
        b->lcd_probe.raw_id4[0], b->lcd_probe.raw_id4[1],
        b->lcd_probe.raw_id4[2], b->lcd_probe.raw_id4[3],
        (unsigned long)b->lcd_probe.id4_dummy_bit,
        (unsigned long)b->lcd_probe.id4_dummy_byte,
        b->lcd_probe.id_ok ? "(want 009341: yes)" : "(want 009341: NO)");
    printf("lcd probe: power mode %02X (want 08)  pixfmt %02X -> %02X (want 06 -> 05) %s\n",
        b->lcd_probe.power_mode, b->lcd_probe.pixfmt_reset, b->lcd_probe.pixfmt_set,
        b->lcd_probe.pixfmt_ok ? "ok" : "NO");
    printf("lcd_init: %s\n", lcd_status_str(b->lcd_s));

    printf("sd card: v2 %c  high-capacity %c  r1 cmd0 %02X cmd8 %02X  r7 %08lX  ocr %08lX  acmd41 x%lu  %lu ms  %lu blocks (%lu MB)\n",
        sd->v2 ? 'Y' : 'N', sd->high_capacity ? 'Y' : 'N',
        sd->r1_cmd0, sd->r1_cmd8, (unsigned long)sd->r7, (unsigned long)sd->ocr,
        (unsigned long)sd->acmd41_tries, (unsigned long)sd->init_ms,
        (unsigned long)sd->blocks, (unsigned long)(sd->blocks / 2048u));
    session_log_get_stats(&log);
    printf("session_log: %s  fatfs err %u  session %lu\n",
        log_status_str(log_s), (unsigned)log.fatfs_err, (unsigned long)log.session_n);
}

void console_report(const app_state_t *s, const app_rtos_t *r) {
    static uint32_t tick_n;
    static uint32_t last_edges;
    static uint32_t last_reads;
    static uint32_t last_rmc;
    const app_boot_t *b = &s->boot;
    const gps_fix_t *fix = &s->gps.fix;
    const gps_stats_t *gst = &s->gps.stats;

    tick_n++;

    // the counters still have to move on while nobody is listening, or the
    // first line after a terminal opens would report a huge second
    uint32_t edges_now = imu_int1_count();
    uint32_t edges = edges_now - last_edges;
    uint32_t reads = s->imu.reads - last_reads;
    uint32_t rmc = gst->rmc - last_rmc;
    last_edges = edges_now;
    last_reads = s->imu.reads;
    last_rmc = gst->rmc;

    if (!usb_serial_ready()) {
        return;
    }

    printf("--- %lu   up %lu s   drops %lu\n",
        (unsigned long)tick_n,
        (unsigned long)(HAL_GetTick() / 1000u),
        (unsigned long)usb_serial_drops());

    // stack each task has never touched, in bytes - what is left to trim -
    // and the RTOS heap now and at its lowest
    printf("stack");
    for (uint32_t i = 0; i < (uint32_t)APP_TASK_COUNT; i++) {
        printf("  %s %lu", app_task_name((app_task_t)i), (unsigned long)r->stack_free[i]);
    }
    printf("   heap %lu min %lu\n", (unsigned long)r->heap_free, (unsigned long)r->heap_min);

    // the longest pass of each task this second, wake to done, preemption
    // included - this is what pass max used to be
    printf("wake us");
    for (uint32_t i = 0; i < (uint32_t)APP_TASK_COUNT; i++) {
        if ((app_task_t)i != APP_TASK_WDG) {
            printf("  %s %lu", app_task_name((app_task_t)i), (unsigned long)s->second.wake_us[i]);
        }
    }
    printf("\n");

    if (b->imu_s == IMU_OK) {
        printf("int1  edges/s %lu  reads/s %lu  dtmax %lu ms  err %lu  rec %lu\n",
            (unsigned long)edges, (unsigned long)reads,
            (unsigned long)s->second.imu_gap_ms, (unsigned long)s->imu.read_errors,
            (unsigned long)s->imu.recoveries);
        printf("att   pitch %+7.2f  roll %+7.2f  acc %c  set %c   push %+.2f %+.2f g\n",
            s->imu.att.pitch_deg, s->imu.att.roll_deg,
            s->imu.att.accel_used ? 'Y' : 'N', s->imu.att.settled ? 'Y' : 'N',
            s->imu.push_long_g, s->imu.push_lat_g);
    } else {
        printf("imu   init failed: %s\n", imu_status_str(b->imu_s));
    }

    const trip_stats_t *trip = &s->imu.trip;
    printf("trip  %.2f km  vmax %.0f  mov %lu s  peaks %+.2f %+.2f %+.2f %+.2f g%s\n",
        trip->distance_km, trip->speed_max_kmh, (unsigned long)trip->moving_s,
        trip->accel_peak_pos_g, trip->accel_peak_neg_g,
        trip->accel_peak_left_g, trip->accel_peak_right_g,
        trip->coasting ? "  coasting" : "");

    if (b->gps_s == GPS_OK) {
        // the ladder: bytes rising proves the wire, sent rising proves the
        // baud, rmc rising proves the parser; bad is line noise, qdrop is
        // fixes the imu task never took
        printf("gps   bytes %lu  sent %lu  rmc %lu  gga %lu  other %lu  bad %lu  ovr %lu  qdrop %lu\n",
            (unsigned long)s->gps.rx_bytes, (unsigned long)gst->sentences,
            (unsigned long)gst->rmc, (unsigned long)gst->gga, (unsigned long)gst->other,
            (unsigned long)gst->bad, (unsigned long)s->gps.ring_overruns,
            (unsigned long)s->gps.fixes_dropped);
        // the faster link: rmc/s is the fix rate actually arriving, 5 once it
        // took; acks should reach 2 (sentence set, fix rate) - the baud
        // command never gets one
        printf("gps   link %s  rmc/s %lu  acks ok %lu failed %lu  tries %lu\n",
            (s->gps.link == GPS_LINK_FAST) ? "115200 5 Hz" :
            (s->gps.link == GPS_LINK_TRYING) ? "trying 115200" : "9600 1 Hz",
            (unsigned long)rmc, (unsigned long)gst->acks_ok,
            (unsigned long)gst->acks_failed, (unsigned long)gst->switch_tries);
        if (gst->rmc == 0u) {
            printf("gps   no RMC yet\n");
        } else {
            printf("gps   age %lu ms  ", (unsigned long)(HAL_GetTick() - fix->updated_tick));
            if (fix->time_valid) {
                printf("time %02u:%02u:%02u %02u/%02u/%02u  ",
                    fix->hour, fix->minute, fix->second, fix->day, fix->month, fix->year);
            } else if (gst->fix_no_date > 0u) {
                // a fix came with a date the year check refused; show it as sent
                if (gst->rejected_have) {
                    printf("time -- (fix date %02u%02u%02u refused x%lu)  ",
                        gst->rejected_day, gst->rejected_month, gst->rejected_year,
                        (unsigned long)gst->fix_no_date);
                } else {
                    printf("time -- (fix with empty date x%lu)  ",
                        (unsigned long)gst->fix_no_date);
                }
            } else {
                printf("time --  ");
            }
            printf("fix %s  sats %u  q %u  hdop %.1f\n",
                fix->fix_valid ? "YES" : "no", fix->sats_used, fix->fix_quality, fix->hdop);
        }
        if (fix->fix_valid) {
            const elevation_t *elev = &s->imu.elev;
            printf("gps   lat %+.6f  lon %+.6f  spd %.1f km/h  crs %.1f  alt %.1f m  ",
                fix->lat_deg, fix->lon_deg, fix->speed_kmh, fix->course_deg, fix->altitude_m);
            // the fused figure only exists once the first fixes are averaged;
            // start is where it began, so a cold-start walk shows up here
            if (elev->valid) {
                printf("fused %.1f m %s  start %.1f m\n", elev->altitude_m,
                    elev->settled ? "settled" : "settling", elev->seed_m);
            } else {
                printf("fused -- (averaging)\n");
            }
        }
    } else {
        printf("gps   init failed: %s\n", gps_status_str(b->gps_s));
    }

    if (b->env_s == ENV_OK) {
        if (s->env.have) {
            printf("env   %.2f C   %.2f hPa   %.2f %%RH   gas %.0f ohm %s  err %lu  qdrop %lu\n",
                s->env.sample.temperature_c, s->env.sample.pressure_hpa,
                s->env.sample.humidity_pct, s->env.sample.gas_resistance_ohm,
                s->env.sample.gas_valid ? "(valid)" : "(heater warming)",
                (unsigned long)s->env.errors, (unsigned long)s->env.dropped);
        }
    } else {
        printf("env   init failed: %s\n", env_status_str(b->env_s));
    }

    const sd_stats_t *sd = &s->log.sd;
    const log_stats_t *log = &s->log.stats;
    printf("lcd   bands %lu  bytes %lu  err %lu   sd  rd %lu wr %lu crc %lu to %lu rej %lu busy max %lu ms   log %s n %lu  writes %lu err %lu  last %lu ms max %lu ms\n",
        (unsigned long)s->disp.bands, (unsigned long)s->disp.bytes,
        (unsigned long)s->disp.errors,
        (unsigned long)sd->reads, (unsigned long)sd->writes, (unsigned long)sd->crc_errors,
        (unsigned long)sd->timeouts, (unsigned long)sd->rejected, (unsigned long)sd->busy_max_ms,
        log->mounted ? "up" : "DOWN", (unsigned long)log->session_n,
        (unsigned long)log->writes, (unsigned long)log->errors,
        (unsigned long)log->last_write_ms, (unsigned long)log->max_write_ms);
}
