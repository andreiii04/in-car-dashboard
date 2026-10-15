//
// IMU module - public interface
//
// - IMU means inertial measurement unit; here it is the LSM6DSO, which holds
//   an accelerometer and a gyroscope in one package
// - this header stays free of HAL and vendor types, so code above it does not
//   need to know which part is fitted or which bus it sits on
// - prefix is imu_ and not lsm6dso_ because the vendor driver already owns
//   that name; lsm6dso_status_t in particular already exists in lsm6dso_reg.h
//

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    IMU_OK = 0,
    IMU_ERR_ARG,      // caller passed a NULL pointer
    IMU_ERR_BUS,      // the I2C transfer failed
    IMU_ERR_ID,       // WHO_AM_I did not match - wrong part, wiring or address
    IMU_ERR_TIMEOUT,  // the software reset never finished
} imu_status_t;

// axes as marked on the LSM6DSO package, not car axes; which one is
// forward/sideways/up depends on how the board ends up mounted, so that
// mapping is done one layer higher
typedef struct {
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;
    float gyro_x_dps;
    float gyro_y_dps;
    float gyro_z_dps;
} imu_sample_t;

// check the part is there, reset it and configure it; call after MX_I2C1_Init
imu_status_t imu_init(void);

// read one accel + gyro sample, already converted to g and deg/s
imu_status_t imu_read(imu_sample_t *out);

// true when both accel and gyro have a fresh sample waiting
bool imu_data_ready(void);

// route the accelerometer data-ready signal on the INT1 pin; call after
// imu_init. the line is latched - it goes high on a new sample and only drops
// once that sample has been read, so if nothing reads it the edges stop
imu_status_t imu_int1_enable(void);

// how many INT1 edges the handler has counted; compare it against your own
// last value rather than keeping a flag - only the ISR writes it, so reading
// it cannot race, and a jump bigger than 1 means samples were missed
uint32_t imu_int1_count(void);

#endif // IMU_H
