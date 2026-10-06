//
// env module - public interface
//
// - env is the environmental sensor, here the BME680 - temperature, pressure,
//   humidity and gas resistance in one package
// - gas resistance is a raw number in ohms; turning it into an air quality
//   index needs Bosch's BSEC library, which is a later step
// - a measurement is two calls, not one: env_start kicks it off and says how
//   long it takes, env_read collects it once that time has passed. the old
//   single blocking call sat in a busy wait for ~143 ms, longer than one IMU
//   sample gap, which is not allowed in the main loop any more
// - stays free of HAL and vendor types, same idea as imu.h
//

#ifndef ENV_H
#define ENV_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ENV_OK = 0,
    ENV_ERR_ARG,        // caller passed a NULL pointer
    ENV_ERR_BUS,        // the I2C transfer failed
    ENV_ERR_ID,         // chip ID did not match - wrong part, wiring or address
    ENV_ERR_NO_DATA,    // the measurement has not finished yet, ask again later
} env_status_t;

typedef struct {
    float temperature_c;
    float pressure_hpa;
    float humidity_pct;
    float gas_resistance_ohm;   // only means anything when gas_valid is true
    bool gas_valid;             // false if the hot plate had not settled yet
} env_sample_t;

// reset the part, read its calibration and configure it; call after MX_I2C1_Init
env_status_t env_init(void);

// trigger one measurement; the sensor goes back to sleep by itself when it is
// done. wait_ms comes back with how long that takes, about 143 ms - call
// env_read after that, not before
env_status_t env_start(uint32_t *wait_ms);

// collect the finished measurement; ENV_ERR_NO_DATA means the sensor is still
// busy, so try again on a later pass
env_status_t env_read(env_sample_t *out);

#endif // ENV_H
