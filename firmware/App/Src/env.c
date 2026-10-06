//
// env module - implementation
//
// - sits on top of Bosch's vendor driver, which sits on top of i2c_bus
// - the BME680 only has sleep and forced mode; forced means one measurement on
//   demand and the sensor puts itself back to sleep afterwards
// - so a read is trigger -> wait -> read back, unlike the IMU which free-runs;
//   the wait is the caller's, so the loop keeps running through it
// - 2x temperature, 16x pressure, 1x humidity, IIR coefficient 3, gas heater
//   at 300 C for 100 ms; the reasoning behind those is in docs/notes.md
//

#include "env.h"
#include "bme68x.h"
#include <stddef.h>

#define ENV_HEATR_TEMP_C 300
#define ENV_HEATR_DUR_MS 100

// defined in bme68x_platform.c - holds the callbacks and the address
extern struct bme68x_dev bme68x_ctx;

// both kept at file scope because env_start needs them again to work out how
// long one measurement takes
static struct bme68x_conf env_conf;
static struct bme68x_heatr_conf env_heatr_conf;

env_status_t env_init(void) {
    int8_t rslt;

    // bme68x_init does the soft reset, checks the chip ID and reads the factory
    // calibration numbers out of the sensor - no separate WHO_AM_I step needed
    rslt = bme68x_init(&bme68x_ctx);
    if (rslt == BME68X_E_DEV_NOT_FOUND) {
        return ENV_ERR_ID;
    }
    if (rslt != BME68X_OK) {
        return ENV_ERR_BUS;
    }

    // oversampling = how many readings the sensor averages internally per sample
    env_conf.os_temp = BME68X_OS_2X;    // accuracy is capped by calibration, not by noise
    env_conf.os_pres = BME68X_OS_16X;   // pressure is the noisy one and the extra time is free here
    env_conf.os_hum = BME68X_OS_1X;     // capped by the sensing material, avg cannot help

    // IIR filter, applies to temperature and pressure only; kills door slams
    // and vent blasts without hiding real changes
    env_conf.filter = BME68X_FILTER_SIZE_3;

    // standby time between measurements, only used in sequential mode
    env_conf.odr = BME68X_ODR_NONE;

    rslt = bme68x_set_conf(&env_conf, &bme68x_ctx);
    if (rslt != BME68X_OK) {
        return ENV_ERR_BUS;
    }

    // gas heater on; the hot plate heats the sensing layer,
    // whose resistance changes with the gases around it
    env_heatr_conf.enable = BME68X_ENABLE;
    env_heatr_conf.heatr_temp = ENV_HEATR_TEMP_C;
    env_heatr_conf.heatr_dur = ENV_HEATR_DUR_MS;

    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &env_heatr_conf, &bme68x_ctx);
    if (rslt != BME68X_OK) {
        return ENV_ERR_BUS;
    }

    return ENV_OK;
}

env_status_t env_start(uint32_t *wait_ms) {
    uint32_t meas_dur_us;

    if (wait_ms == NULL) {
        return ENV_ERR_ARG;
    }

    // kick off one measurement; the sensor drops back to sleep on its own.
    // the driver first makes sure the part is asleep, polling with a 10 ms
    // delay if not - it always is by now, the previous measurement finished
    // long ago, so that loop runs zero times
    if (bme68x_set_op_mode(BME68X_FORCED_MODE, &bme68x_ctx) != BME68X_OK) {
        return ENV_ERR_BUS;
    }

    // bme68x_get_meas_dur only covers the temp/pressure/humidity part, so the
    // heating time goes on top; us -> ms, rounded up, plus one for luck
    meas_dur_us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &env_conf, &bme68x_ctx);
    *wait_ms = ((meas_dur_us + 999u) / 1000u) + (uint32_t)ENV_HEATR_DUR_MS + 1u;
    return ENV_OK;
}

env_status_t env_read(env_sample_t *out) {
    struct bme68x_data data;
    uint8_t n_data = 0;
    int8_t rslt;

    if (out == NULL) {
        return ENV_ERR_ARG;
    }

    // negative means a real failure; positive is only a warning, and the one
    // warning that matters here is "nothing new yet"
    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_data, &bme68x_ctx);
    if (rslt < BME68X_OK) {
        return ENV_ERR_BUS;
    }
    if ((rslt == BME68X_W_NO_NEW_DATA) || (n_data == 0u)) {
        return ENV_ERR_NO_DATA;
    }

    out->temperature_c = data.temperature;
    out->pressure_hpa = data.pressure / 100.0f; // the driver gives Pascal
    out->humidity_pct = data.humidity;
    out->gas_resistance_ohm = data.gas_resistance;

    // the gas number only counts if the reading completed
    // and the hot plate actually reached its target
    out->gas_valid = ((data.status & BME68X_GASM_VALID_MSK) != 0) &&
        ((data.status & BME68X_HEAT_STAB_MSK) != 0);

    return ENV_OK;
}
