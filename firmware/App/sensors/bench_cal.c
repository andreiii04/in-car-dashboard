//
// bench calibration - implementation
//
// - averages a batch of still samples and prints the resting accel and the
//   gyro offset in car axes, plus the spread inside the batch; the between-run
//   spread is what says whether the board is warm yet
// - re-runs every 20 s rather than once at boot, because a startup print
//   lands in the dropped bytes before screen is attached
//

#include "bench_cal.h"

#if BENCH_CAL

#include <math.h>
#include <stdio.h>
#include "imu.h"
#include "vehicle_axes.h"
#include "stm32f4xx_hal.h"

#define CAL_SAMPLES    500u
#define CAL_WAIT_MS    2000u        // give up if the IMU stops producing data
#define CAL_PERIOD_MS  20000u
#define CAL_RAD_TO_DEG 57.29577951f

static uint32_t last_cal_tick;

static void imu_calibrate(void) {
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_min[3] = {0.0f, 0.0f, 0.0f};
    float accel_max[3] = {0.0f, 0.0f, 0.0f};
    float gyro_min[3] = {0.0f, 0.0f, 0.0f};
    float gyro_max[3] = {0.0f, 0.0f, 0.0f};
    float accel_avg[3];
    float gyro_avg[3];
    float mag;
    uint32_t taken = 0;
    uint32_t last_new = HAL_GetTick();

    printf("\ncalibrating - %lu samples, keep the board still\n",
        (unsigned long)CAL_SAMPLES);

    while (taken < CAL_SAMPLES) {
        imu_sample_t raw;
        float sensor[3];
        float mapped[3];

        // only count a sample the sensor says is new; polling faster than the
        // 104 Hz output rate would add the same one in twice
        if (!imu_data_ready()) {
            if ((HAL_GetTick() - last_new) > CAL_WAIT_MS) {
                printf("calibrate: no data-ready, gave up after %lu\n",
                    (unsigned long)taken);
                return;
            }
            continue;
        }

        if (imu_read(&raw) != IMU_OK) {
            printf("calibrate: read failed after %lu\n", (unsigned long)taken);
            return;
        }
        last_new = HAL_GetTick();

        // map into car axes here - that is the frame the header stores both
        // constants in, so what gets printed can be pasted straight in
        sensor[0] = raw.accel_x_g;
        sensor[1] = raw.accel_y_g;
        sensor[2] = raw.accel_z_g;
        vehicle_map(sensor, mapped);
        for (uint8_t i = 0; i < 3u; i++) {
            accel_sum[i] += mapped[i];
            if ((taken == 0u) || (mapped[i] < accel_min[i])) {
                accel_min[i] = mapped[i];
            }
            if ((taken == 0u) || (mapped[i] > accel_max[i])) {
                accel_max[i] = mapped[i];
            }
        }

        sensor[0] = raw.gyro_x_dps;
        sensor[1] = raw.gyro_y_dps;
        sensor[2] = raw.gyro_z_dps;
        vehicle_map(sensor, mapped);
        for (uint8_t i = 0; i < 3u; i++) {
            gyro_sum[i] += mapped[i];
            if ((taken == 0u) || (mapped[i] < gyro_min[i])) {
                gyro_min[i] = mapped[i];
            }
            if ((taken == 0u) || (mapped[i] > gyro_max[i])) {
                gyro_max[i] = mapped[i];
            }
        }

        taken++;
    }

    for (uint8_t i = 0; i < 3u; i++) {
        accel_avg[i] = accel_sum[i] / (float)CAL_SAMPLES;
        gyro_avg[i] = gyro_sum[i] / (float)CAL_SAMPLES;
    }

    mag = sqrtf((accel_avg[0] * accel_avg[0]) +
                (accel_avg[1] * accel_avg[1]) +
                (accel_avg[2] * accel_avg[2]));

    printf("--- calibration, car axes, %lu samples ---\n",
        (unsigned long)CAL_SAMPLES);
    printf("accel avg  %+.5f %+.5f %+.5f g   |a| %.5f\n",
        accel_avg[0], accel_avg[1], accel_avg[2], mag);
    printf("accel p-p   %.5f  %.5f  %.5f g\n",
        accel_max[0] - accel_min[0],
        accel_max[1] - accel_min[1],
        accel_max[2] - accel_min[2]);
    printf("gyro  avg  %+.4f %+.4f %+.4f dps\n",
        gyro_avg[0], gyro_avg[1], gyro_avg[2]);
    printf("gyro  p-p   %.4f  %.4f  %.4f dps\n",
        gyro_max[0] - gyro_min[0],
        gyro_max[1] - gyro_min[1],
        gyro_max[2] - gyro_min[2]);

    // the tilt this average implies; only worth comparing against another
    // surface, the accelerometer's own zero error is mixed into it
    printf("tilt       pitch %+.2f  roll %+.2f deg\n",
        atan2f(accel_avg[0], sqrtf((accel_avg[1] * accel_avg[1]) +
                                   (accel_avg[2] * accel_avg[2]))) * CAL_RAD_TO_DEG,
        atan2f(accel_avg[1], accel_avg[2]) * CAL_RAD_TO_DEG);

    printf("paste into vehicle_axes.h:\n");
    printf("#define VEHICLE_REST_ACCEL_X   %+.5ff\n", accel_avg[0]);
    printf("#define VEHICLE_REST_ACCEL_Y   %+.5ff\n", accel_avg[1]);
    printf("#define VEHICLE_REST_ACCEL_Z   %+.5ff\n", accel_avg[2]);
    printf("#define VEHICLE_GYRO_BIAS_X    %+.4ff\n", gyro_avg[0]);
    printf("#define VEHICLE_GYRO_BIAS_Y    %+.4ff\n", gyro_avg[1]);
    printf("#define VEHICLE_GYRO_BIAS_Z    %+.4ff\n", gyro_avg[2]);
}

void bench_cal_tick(bool imu_ok) {
    if ((HAL_GetTick() - last_cal_tick) < CAL_PERIOD_MS) {
        return;
    }
    last_cal_tick = HAL_GetTick();
    if (imu_ok) {
        imu_calibrate();
    }
}

#else

void bench_cal_tick(bool imu_ok) {
    (void)imu_ok;
}

#endif // BENCH_CAL
