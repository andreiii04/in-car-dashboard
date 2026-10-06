/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "fatfs.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "usb_serial.h"
#include "uart_bus.h"
#include "sd_spi.h"
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
#include "session_log.h"
#include "session_record.h"
#include "console.h"
#include "bench_cal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// loop cadences, in ms. the IMU has no cadence of its own here - it runs on
// its data-ready interrupt, everything else is paced off HAL_GetTick
#define ENV_PERIOD_MS      3000u   // one BME680 measurement every 3 s
#define ENV_GIVE_UP_MS     1000u   // a measurement that never finishes is dropped after this
#define SCREEN_PERIOD_MS   50u     // the gauge dot at 20 Hz; the numbers change slower anyway
#define LOG_PERIOD_MS      10000u  // rewrite the session record every 10 s
#define LOG_RETRY_MS       5000u   // look for a card again every 5 s while there is none
#define CONSOLE_PERIOD_MS  1000u
#define BOOT_SCREEN_MS     2000u   // the boot screen stays up this long
#define USB_WAIT_MS        3000u   // how long boot waits for a terminal to attach
#define LCD_BRIGHTNESS     1000u   // permille; nothing to dim it from yet
#define SYSCLK_MHZ         168u    // DWT cycle counter -> microseconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// file scope so the debugger can live-watch them; a local on the stack has no
// fixed address
static console_status_t st;      // init results and per-second counters
static screen_data_t    screen;  // what the panel is being shown
static session_record_t record;  // what the card is being sent
static bool display_ok;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// the DWT cycle counter: 168 counts per microsecond, free-running, the only
// clock fine enough to time one loop pass
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

// one IMU sample: read, map into car axes, step the attitude filter, feed
// the trip stats. called once per INT1 edge, about 105 times a second
static void imu_tick(uint32_t dt_ms) {
    imu_sample_t raw;
    float sensor[3];
    float accel[3];
    float gyro[3];
    attitude_t att;

    if (imu_read(&raw) != IMU_OK) {
        st.att_err++;       // counted, not printed - a failing read at 104 Hz
        return;             // would bury the console
    }
    st.reads++;             // the read is also what drops INT1 again

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
    trip_update_motion(accel, gyro, att.pitch_deg, att.roll_deg, dt_s);

    // tell the filter how hard the car is pushing, every sample - trip works
    // the sideways part out from speed x yaw rate 105 times a second. handed
    // over only once per GPS fix, a bend went up to 1 s uncorrected, which
    // read as ~9 deg of roll in session 17. used from the next sample on
    attitude_set_linear_accel(trip_long_accel_g(), trip_lat_accel_g());
}

// every counter a failure would have turned up in, collected once per record.
// the console shouts these at a terminal nobody is watching in the car; the
// card keeps them with the drive they belong to
static void gather_faults(session_faults_t *f) {
    gps_stats_t gst;
    sd_stats_t sd;
    log_stats_t log;

    memset(f, 0, sizeof(*f));
    gps_get_stats(&gst);
    sd_spi_get_stats(&sd);
    session_log_get_stats(&log);

    f->imu_init = st.imu_s;
    f->env_init = st.env_s;
    f->gps_init = st.gps_s;
    f->display_init = st.lcd_s;
    f->card_mounted = log.mounted;
    f->fatfs_error = log.fatfs_err;

    f->imu_read_errors = st.att_err;
    f->env_read_errors = st.env_err;
    f->gps_bad_checksums = gst.bad;
    f->gps_ring_overruns = uart_bus_overruns();
    f->display_errors = lcd_errors();
    f->card_crc_errors = sd.crc_errors;
    f->card_timeouts = sd.timeouts;
    f->card_rejected = sd.rejected;
    f->log_write_errors = log.errors;

    f->longest_imu_gap_ms = st.dt_max_session_ms;
    f->slowest_log_write_ms = log.max_write_ms;

    f->gps_date_refused_have = (gst.fix_no_date > 0u);
    f->gps_date_refused_empty = !gst.rejected_have;
    f->gps_date_refused_day = gst.rejected_day;
    f->gps_date_refused_month = gst.rejected_month;
    f->gps_date_refused_year = gst.rejected_year;
}

// everything the panel shows, pulled from the modules' getters
static void gather_screen(screen_data_t *d) {
    trip_stats_t t;
    gps_fix_t f;
    elevation_t e;

    trip_get(&t);
    gps_get_fix(&f);
    elev_get(&e);

    d->long_g = trip_push_long_g();
    d->lat_g = trip_push_lat_g();
    d->peak_accel_g = t.accel_peak_pos_g;
    // stored negative, shown as a size; fabsf and not a minus, because minus
    // zero prints as -0.0 on the panel
    d->peak_brake_g = fabsf(t.accel_peak_neg_g);
    d->peak_left_g = t.accel_peak_left_g;
    d->peak_right_g = fabsf(t.accel_peak_right_g);

    d->speed_kmh = trip_speed_kmh();
    d->speed_max_kmh = t.speed_max_kmh;
    d->distance_km = t.distance_km;
    d->coasting = t.coasting;

    d->env_valid = st.env_have;
    d->temp_c = st.env.temperature_c;
    d->hum_pct = st.env.humidity_pct;
    d->press_hpa = st.env.pressure_hpa;
    d->gas_valid = st.env.gas_valid;
    d->gas_kohm = st.env.gas_resistance_ohm / 1000.0f;

    d->duration_s = t.duration_s;
    d->sd_ok = session_log_ready();

    d->fix_valid = f.fix_valid;
    d->time_valid = f.time_valid;
    d->sats = f.sats_used;
    d->lat_deg = f.lat_deg;
    d->lon_deg = f.lon_deg;
    // the fused altitude, not the raw fix: GPS height wanders by tens of
    // metres on a poor fix and the barometer is what steadies it
    d->alt_m = e.valid ? e.altitude_m : f.altitude_m;
    d->alt_settled = e.valid && e.settled;
    d->hour = f.hour;
    d->minute = f.minute;
    d->second = f.second;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

    uint32_t last_att_tick = 0;      // HAL tick of the last IMU sample
    uint32_t att_last_edge = 0;      // last INT1 count this loop acted on
    uint32_t last_gps_tick = 0;      // updated_tick of the last fix handed to trip
    uint32_t env_last_start = 0;
    uint32_t env_ready_at = 0;
    uint32_t env_last_sample = 0;
    bool     env_pending = false;
    uint32_t last_screen = 0;
    uint32_t last_log = 0;
    uint32_t last_log_retry = 0;
    uint32_t last_console = 0;
    uint32_t boot_tick = 0;
    bool     boot_screen_up = false;

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN1_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

    dwt_init();
    usb_serial_init();

    // the host needs a moment to enumerate the port and nothing printed
    // before that can arrive; wait for it, but not forever - the board still
    // has to run in the car with no host attached
    uint32_t wait = HAL_GetTick();
    while (!usb_serial_ready() && (HAL_GetTick() - wait) < USB_WAIT_MS) {
    }
    console_banner();
    console_i2c_scan();

    st.imu_s = imu_init();
    st.env_s = env_init();

    // hand the filter the two measured constants out of vehicle_axes.h; both
    // were recorded after vehicle_map, which is the frame attitude.c works in
    attitude_init();
    if (!attitude_set_mount(VEHICLE_REST_ACCEL_X, VEHICLE_REST_ACCEL_Y, VEHICLE_REST_ACCEL_Z)) {
        printf("attitude: mount vector too short, staying level\n");
    }
    attitude_set_gyro_bias(VEHICLE_GYRO_BIAS_X, VEHICLE_GYRO_BIAS_Y, VEHICLE_GYRO_BIAS_Z);
    // hand the sensor its own interrupt line; from here the loop is paced by
    // the LSM6DSO and not by a timer
    if (st.imu_s == IMU_OK) {
        st.int1_s = imu_int1_enable();
    }
    trip_init();
    elev_init();

    // the display: the ID gate first, then the full init, then the colour
    // bars while the backlight fades up. the gate is advisory - this module
    // never drives SDO, so every read answers FF; the numbers still go on the
    // console and the colour bars are what proves the write path
    (void)lcd_probe(&st.lcd_probe);
    st.lcd_s = lcd_init();
    display_ok = (st.lcd_s == LCD_OK);
    gfx_init(screen_paint, lcd_write_band);
    screen_init();
    if (display_ok) {
        screen_show_boot();
        while (gfx_flush_band()) {
        }
        lcd_backlight_set(LCD_BRIGHTNESS);
        boot_screen_up = true;
    }

    // the card: mount, open the file, claim this drive's record. no card
    // detect pin, so a missing card is just a failed init, retried in the loop
    st.log_s = session_log_init();

    // starts the USART1 receive interrupt; the module has been talking since
    // power-up, from here on its bytes land in the ring instead of being lost.
    // last on purpose: the panel init and the card mount above stall for well
    // over the ~265 ms the old 256-byte ring lasted at 9600 baud, which lost bytes
    // on every boot and hid the real stalls in gps_ring_overruns
    st.gps_s = gps_init();

    console_report_init(&st);

    // start the clocks here; left at zero the first dt would be the whole USB
    // wait above, which prints as a startling dtmax on the first line
    boot_tick = HAL_GetTick();
    last_att_tick = boot_tick;
    last_console = boot_tick;
    last_log = boot_tick;
    last_log_retry = boot_tick;
    env_last_start = boot_tick - ENV_PERIOD_MS;   // first measurement straight away

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      uint32_t pass_start = DWT->CYCCNT;
      uint32_t now = HAL_GetTick();

      // drain the ring every pass - the module sends up to ~900 bytes a second
      // and the ring holds 2048, ~2 s; a slow card write is the worst wait
      if (st.gps_s == GPS_OK) {
          gps_poll();
      }

      // attitude tick - one filter step per INT1 edge, about 105 a second. the
      // count only ever goes up in the ISR, so reading it here cannot race
      uint32_t att_edge = imu_int1_count();
      if ((st.imu_s == IMU_OK) && (att_edge != att_last_edge)) {
          uint32_t dt_ms = now - last_att_tick;
          att_last_edge = att_edge;
          last_att_tick = now;
          if (dt_ms > st.dt_max_ms) {
              st.dt_max_ms = dt_ms;
          }
          if (dt_ms > st.dt_max_session_ms) {
              st.dt_max_session_ms = dt_ms;     // goes in the record's faults
          }
          imu_tick(dt_ms);
      }

      // a fresh RMC: hand it to the trip stats once. once only - every call
      // restarts trip's between-fixes clock
      if (st.gps_s == GPS_OK) {
          gps_fix_t fix;
          gps_get_fix(&fix);
          if (fix.updated_tick != last_gps_tick) {
              last_gps_tick = fix.updated_tick;
              trip_update_gps(&fix);
          }
      }

      // the BME680, in two halves so the loop never waits on it: trigger,
      // come back ~143 ms later for the result
      if (st.env_s == ENV_OK) {
          if (!env_pending) {
              if ((now - env_last_start) >= ENV_PERIOD_MS) {
                  uint32_t wait_ms;
                  env_last_start = now;
                  if (env_start(&wait_ms) == ENV_OK) {
                      env_pending = true;
                      env_ready_at = now + wait_ms;
                  } else {
                      st.env_err++;
                  }
              }
          } else if ((int32_t)(now - env_ready_at) >= 0) {
              env_status_t s = env_read(&st.env);
              if (s == ENV_OK) {
                  env_pending = false;
                  gps_fix_t fix;
                  gps_get_fix(&fix);
                  float dt_s = st.env_have ? (float)(now - env_last_sample) / 1000.0f
                                           : (float)ENV_PERIOD_MS / 1000.0f;
                  env_last_sample = now;
                  st.env_have = true;
                  trip_update_env(st.env.temperature_c, st.env.pressure_hpa, st.env.humidity_pct);
                  elev_update(st.env.pressure_hpa, fix.fix_valid, fix.altitude_m, dt_s);
              } else if (s == ENV_ERR_NO_DATA) {
                  env_ready_at = now + 5u;                  // not done yet, look again soon
                  if ((now - env_last_start) > ENV_GIVE_UP_MS) {
                      env_pending = false;                 // stuck; the next period retriggers
                      st.env_err++;
                  }
              } else {
                  env_pending = false;                     // bus error; try again next period
                  st.env_err++;
              }
          }
      }

      // the panel: the boot screen gives way to the drive view, then one band
      // per pass at most - a full frame is 15 bands, a dot move is 2
      if (display_ok) {
          if (boot_screen_up && ((now - boot_tick) >= BOOT_SCREEN_MS)) {
              boot_screen_up = false;
              screen_show_main();
          }
          if (!boot_screen_up && ((now - last_screen) >= SCREEN_PERIOD_MS)) {
              last_screen = now;
              gather_screen(&screen);
              screen_update(&screen);
          }
          gfx_flush_band();
          lcd_backlight_tick();
      }

      // the card: rewrite this drive's record every 10 s, or keep looking for
      // a card every 5 s until one turns up
      if (session_log_ready()) {
          if ((now - last_log) >= LOG_PERIOD_MS) {
              trip_stats_t t;
              elevation_t e;
              session_faults_t faults;
              last_log = now;
              trip_get(&t);
              elev_get(&e);
              gather_faults(&faults);
              session_record_build(&record, session_log_session_n(), &t, &e, &faults);
              st.log_s = session_log_write(&record);
          }
      } else if ((now - last_log_retry) >= LOG_RETRY_MS) {
          last_log_retry = now;
          st.log_s = session_log_init();
          last_log = now;
      }

      // the console, once a second, and the per-second maxima start over
      if ((now - last_console) >= CONSOLE_PERIOD_MS) {
          last_console = now;
          console_report(&st);
          st.dt_max_ms = 0;
          st.pass_max_us = 0;
          st.pass_count = 0;
      }

      bench_cal_tick(st.imu_s == IMU_OK);

      // how long this pass took; the budget is the ~9.5 ms between IMU samples
      uint32_t pass_us = (DWT->CYCCNT - pass_start) / SYSCLK_MHZ;
      if (pass_us > st.pass_max_us) {
          st.pass_max_us = pass_us;
      }
      st.pass_count++;
  }

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
