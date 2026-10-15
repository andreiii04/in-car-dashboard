//
// env module - implementation
//
// - sits on top of Bosch's vendor driver, which sits on top of i2c_bus
// - the BME680 only has sleep and forced mode; forced means one measurement on
//   demand and the sensor puts itself back to sleep afterwards
// - so a read is trigger -> wait -> read back, unlike the IMU which free-runs;
//   the wait is the caller's, so the task sleeps through it
// - IIR coefficient 3 is set once here; oversampling and the gas heater come
//   in with every env_start, from BSEC. they are only written to the sensor
//   when they change - in LP mode they never do, so a measurement costs the
//   same few I2C transfers as before. the reasoning is in docs/notes.md
//

#include "env.h"
#include "bme68x.h"
#include <stddef.h>

// defined in bme68x_platform.c - holds the callbacks and the address
extern struct bme68x_dev bme68x_ctx;

// what the sensor holds right now; env_start needs it again to work out how
// long one measurement takes
static struct bme68x_conf env_conf;
static struct bme68x_heatr_conf env_heatr_conf;
static env_settings_t env_applied;
static bool env_have_applied;

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

    // IIR filter, applies to temperature and pressure only; kills door slams
    // and vent blasts without hiding real changes
    env_conf.filter = BME68X_FILTER_SIZE_3;

    // standby time between measurements, only used in sequential mode
    env_conf.odr = BME68X_ODR_NONE;

    // placeholders until the first env_start brings the real ones
    env_conf.os_temp = BME68X_OS_2X;
    env_conf.os_pres = BME68X_OS_16X;
    env_conf.os_hum = BME68X_OS_1X;

    rslt = bme68x_set_conf(&env_conf, &bme68x_ctx);
    if (rslt != BME68X_OK) {
        return ENV_ERR_BUS;
    }
    env_have_applied = false;
    return ENV_OK;
}

// write oversampling and heater only if they differ from what the sensor has
static env_status_t env_apply(const env_settings_t *s) {
    if (env_have_applied &&
        (s->os_temp == env_applied.os_temp) && (s->os_pres == env_applied.os_pres) &&
        (s->os_hum == env_applied.os_hum) && (s->gas_on == env_applied.gas_on) &&
        (s->heater_temp_c == env_applied.heater_temp_c) &&
        (s->heater_ms == env_applied.heater_ms)) {
        return ENV_OK;
    }

    env_conf.os_temp = s->os_temp;
    env_conf.os_pres = s->os_pres;
    env_conf.os_hum = s->os_hum;
    if (bme68x_set_conf(&env_conf, &bme68x_ctx) != BME68X_OK) {
        return ENV_ERR_BUS;
    }

    // the hot plate heats the sensing layer, whose resistance changes with
    // the gases around it
    env_heatr_conf.enable = s->gas_on ? BME68X_ENABLE : BME68X_DISABLE;
    env_heatr_conf.heatr_temp = s->heater_temp_c;
    env_heatr_conf.heatr_dur = s->heater_ms;
    if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &env_heatr_conf, &bme68x_ctx) != BME68X_OK) {
        return ENV_ERR_BUS;
    }

    env_applied = *s;
    env_have_applied = true;
    return ENV_OK;
}

env_status_t env_start(const env_settings_t *s, uint32_t *wait_ms) {
    uint32_t meas_dur_us;
    env_status_t st;

    if ((s == NULL) || (wait_ms == NULL)) {
        return ENV_ERR_ARG;
    }

    st = env_apply(s);
    if (st != ENV_OK) {
        env_have_applied = false;   // half written - write it all next time
        return st;
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
    *wait_ms = ((meas_dur_us + 999u) / 1000u) + 1u;
    if (s->gas_on) {
        *wait_ms += s->heater_ms;
    }
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
