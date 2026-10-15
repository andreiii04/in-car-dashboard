//
// env module - public interface
//
// - env is the environmental sensor, here the BME680 - temperature, pressure,
//   humidity and gas resistance in one package
// - gas resistance is a raw number in ohms; air.c turns it into an air
//   quality index through Bosch's BSEC library
// - a measurement is two calls, not one: env_start kicks it off and says how
//   long it takes, env_read collects it once that time has passed, so the env
//   task sleeps through the ~150-250 ms instead of busy-waiting
// - the settings for each measurement come from the caller - BSEC decides the
//   oversampling and the heater before every one
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

// how to take one measurement; the oversampling codes are the sensor's own,
// 0 none, 1 1x, 2 2x, 3 4x, 4 8x, 5 16x
typedef struct {
    uint8_t  os_temp;
    uint8_t  os_pres;
    uint8_t  os_hum;
    bool     gas_on;            // heat the plate and measure gas this time
    uint16_t heater_temp_c;
    uint16_t heater_ms;
} env_settings_t;

// reset the part, read its calibration, set the IIR filter; call after
// MX_I2C1_Init. oversampling and heater come with every env_start
env_status_t env_init(void);

// trigger one measurement with these settings; the sensor goes back to sleep
// by itself when it is done. wait_ms comes back with how long that takes -
// call env_read after that, not before
env_status_t env_start(const env_settings_t *s, uint32_t *wait_ms);

// collect the finished measurement; ENV_ERR_NO_DATA means the sensor is still
// busy, so try again a bit later
env_status_t env_read(env_sample_t *out);

#endif // ENV_H
