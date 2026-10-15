//
// air module - implementation
//
// - follows the loop in Bosch's integration guide (AN011): bsec_sensor_control
//   says what to do, the sensor measures, bsec_do_steps gets the result, then
//   sleep until the next_call it gave. the call times may be off by 6.25% at
//   most, ~187 ms at 3 s, before BSEC warns
// - LP mode, one measurement every 3 s, with Bosch's BME680 config for a 3.3 V
//   supply, 3 s and 4 days of history (bme680_iaq_33v_3s_4d)
// - the temperature offset goes in as an input on every call, and BSEC takes
//   it off the temperature and works the humidity out again from that
// - one 4 KB work buffer, static, used only while loading or saving the state
//   - too big for the env task's stack
// - HAVE_BSEC comes from CMakeLists.txt, set only when the library is there
//

#include "air.h"
#include <stddef.h>
#include <string.h>

#if HAVE_BSEC

#include "bsec_interface.h"
#include "bsec_iaq.h"

_Static_assert(AIR_STATE_MAX == BSEC_MAX_STATE_BLOB_SIZE, "AIR_STATE_MAX out of step with BSEC");

#define AIR_N_OUTPUTS 9u   // the outputs asked for in air_init

// the bit in process_data that says BSEC wants this input; the same shift
// Bosch's example uses
#define AIR_WANTS(bits, input) (((bits) & (1u << ((input) - 1u))) != 0u)

static uint8_t air_work[BSEC_MAX_WORKBUFFER_SIZE];
static uint32_t air_process_bits;   // which inputs the last air_control asked for
static float air_offset_c;
static air_sample_t air_last;

air_status_t air_init(const uint8_t *state, uint16_t state_len, float temp_offset_c,
                      air_restore_t *restored) {
    bsec_sensor_configuration_t want[AIR_N_OUTPUTS];
    bsec_sensor_configuration_t required[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required = BSEC_MAX_PHYSICAL_SENSOR;
    // raw pressure is on the list only so BSEC measures it - BSEC asks the
    // sensor for what its outputs need, and without it the pressure is
    // skipped and reads as the 0x80000 placeholder (673 hPa on the board,
    // 15 Oct; BME680 datasheet p21)
    static const uint8_t outputs[AIR_N_OUTPUTS] = {
        BSEC_OUTPUT_RAW_PRESSURE,
        BSEC_OUTPUT_IAQ,
        BSEC_OUTPUT_STATIC_IAQ,
        BSEC_OUTPUT_CO2_EQUIVALENT,
        BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
        BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
        BSEC_OUTPUT_STABILIZATION_STATUS,
        BSEC_OUTPUT_RUN_IN_STATUS,
    };

    *restored = AIR_STATE_FRESH;
    air_offset_c = temp_offset_c;
    memset(&air_last, 0, sizeof(air_last));

    if (bsec_init() != BSEC_OK) {
        return AIR_ERR_INIT;
    }

    // the config is a blob from Bosch's package; it picks the supply voltage,
    // the sample rate and the history length
    if (bsec_set_configuration(bsec_config_iaq, sizeof(bsec_config_iaq),
                               air_work, sizeof(air_work)) != BSEC_OK) {
        return AIR_ERR_CONFIG;
    }

    // a refused state is not fatal - BSEC carries on from scratch, the same as
    // a card that never had one
    if ((state != NULL) && (state_len > 0u)) {
        if (bsec_set_state(state, state_len, air_work, sizeof(air_work)) == BSEC_OK) {
            *restored = AIR_STATE_RESTORED;
        } else {
            *restored = AIR_STATE_REFUSED;
        }
    }

    // every output at the LP rate, one sample every 3 s
    for (uint32_t i = 0; i < AIR_N_OUTPUTS; i++) {
        want[i].sensor_id = outputs[i];
        want[i].sample_rate = BSEC_SAMPLE_RATE_LP;
    }
    if (bsec_update_subscription(want, AIR_N_OUTPUTS, required, &n_required) != BSEC_OK) {
        return AIR_ERR_SUBSCRIBE;
    }
    return AIR_OK;
}

int32_t air_control(int64_t now_ns, env_settings_t *s, bool *measure, int64_t *next_ns) {
    bsec_bme_settings_t set;
    bsec_library_return_t r;

    memset(&set, 0, sizeof(set));
    r = bsec_sensor_control(now_ns, &set);

    *next_ns = set.next_call;
    *measure = (r >= BSEC_OK) && (set.trigger_measurement != 0u) &&
               (set.op_mode == 1u);   // 1 is forced mode, the only one the BME680 has
    air_process_bits = set.process_data;

    s->os_temp = set.temperature_oversampling;
    s->os_pres = set.pressure_oversampling;
    s->os_hum = set.humidity_oversampling;
    s->gas_on = (set.run_gas != 0u);
    s->heater_temp_c = set.heater_temperature;
    s->heater_ms = set.heater_duration;
    return (int32_t)r;
}

// add one input to the list BSEC gets
static void air_input(bsec_input_t *in, uint8_t *n, uint8_t id, float value, int64_t ts_ns) {
    in[*n].sensor_id = id;
    in[*n].signal = value;
    in[*n].signal_dimensions = 1u;
    in[*n].time_stamp = ts_ns;
    (*n)++;
}

int32_t air_process(int64_t ts_ns, const env_sample_t *raw, air_sample_t *out) {
    bsec_input_t in[BSEC_MAX_PHYSICAL_SENSOR];
    bsec_output_t res[BSEC_NUMBER_OUTPUTS];
    uint8_t n_in = 0;
    uint8_t n_res = BSEC_NUMBER_OUTPUTS;
    bsec_library_return_t r;
    uint32_t bits = air_process_bits;

    // whatever BSEC leaves out this time keeps its last value
    if (!air_last.valid) {
        air_last.temperature_c = raw->temperature_c;
        air_last.humidity_pct = raw->humidity_pct;
    }

    if (AIR_WANTS(bits, BSEC_INPUT_HEATSOURCE)) {
        air_input(in, &n_in, BSEC_INPUT_HEATSOURCE, air_offset_c, ts_ns);
    }
    if (AIR_WANTS(bits, BSEC_INPUT_TEMPERATURE)) {
        air_input(in, &n_in, BSEC_INPUT_TEMPERATURE, raw->temperature_c, ts_ns);
    }
    if (AIR_WANTS(bits, BSEC_INPUT_HUMIDITY)) {
        air_input(in, &n_in, BSEC_INPUT_HUMIDITY, raw->humidity_pct, ts_ns);
    }
    if (AIR_WANTS(bits, BSEC_INPUT_PRESSURE)) {
        // BSEC wants Pascal
        air_input(in, &n_in, BSEC_INPUT_PRESSURE, raw->pressure_hpa * 100.0f, ts_ns);
    }
    // a gas reading from a plate that never got hot is no reading at all
    if (AIR_WANTS(bits, BSEC_INPUT_GASRESISTOR) && raw->gas_valid) {
        air_input(in, &n_in, BSEC_INPUT_GASRESISTOR, raw->gas_resistance_ohm, ts_ns);
    }
    // forced mode only ever has profile step 0
    if (AIR_WANTS(bits, BSEC_INPUT_PROFILE_PART) && raw->gas_valid) {
        air_input(in, &n_in, BSEC_INPUT_PROFILE_PART, 0.0f, ts_ns);
    }

    if (n_in == 0u) {
        *out = air_last;
        return BSEC_OK;
    }

    r = bsec_do_steps(in, n_in, res, &n_res);
    if (r == BSEC_OK) {
        for (uint8_t i = 0; i < n_res; i++) {
            float v = res[i].signal;
            switch (res[i].sensor_id) {
            case BSEC_OUTPUT_IAQ:
                air_last.iaq = v;
                air_last.iaq_accuracy = res[i].accuracy;
                air_last.valid = true;
                break;
            case BSEC_OUTPUT_CO2_EQUIVALENT:
                air_last.eco2_ppm = v;
                break;
            case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
                air_last.bvoc_ppm = v;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                air_last.temperature_c = v;
                break;
            case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                air_last.humidity_pct = v;
                break;
            case BSEC_OUTPUT_STABILIZATION_STATUS:
                air_last.stabilised = (v > 0.5f);
                break;
            case BSEC_OUTPUT_RUN_IN_STATUS:
                air_last.run_in = (v > 0.5f);
                break;
            default:
                break;   // static IAQ and raw pressure are asked for, not shown
            }
        }
    }
    *out = air_last;
    return (int32_t)r;
}

bool air_get_state(uint8_t *buf, uint16_t *len) {
    uint32_t n = 0;

    // 0 is "every part of the state"
    if (bsec_get_state(0u, buf, AIR_STATE_MAX, air_work, sizeof(air_work), &n) != BSEC_OK) {
        return false;
    }
    if ((n == 0u) || (n > AIR_STATE_MAX)) {
        return false;
    }
    *len = (uint16_t)n;
    return true;
}

bool air_version(uint8_t v[4]) {
    bsec_version_t ver;

    if (bsec_get_version(&ver) != BSEC_OK) {
        return false;
    }
    v[0] = ver.major;
    v[1] = ver.minor;
    v[2] = ver.major_bugfix;
    v[3] = ver.minor_bugfix;
    return true;
}

#else // no BSEC - the old fixed settings, raw readings straight through

#define AIR_PERIOD_NS 3000000000LL   // one measurement every 3 s, as before

air_status_t air_init(const uint8_t *state, uint16_t state_len, float temp_offset_c,
                      air_restore_t *restored) {
    (void)state;
    (void)state_len;
    (void)temp_offset_c;
    *restored = AIR_STATE_FRESH;
    return AIR_NO_LIB;
}

int32_t air_control(int64_t now_ns, env_settings_t *s, bool *measure, int64_t *next_ns) {
    // 2x temperature, 16x pressure, 1x humidity, heater 300 C for 100 ms -
    // the settings env.c had before BSEC; why is in docs/notes.md
    s->os_temp = 2u;
    s->os_pres = 5u;
    s->os_hum = 1u;
    s->gas_on = true;
    s->heater_temp_c = 300u;
    s->heater_ms = 100u;
    *measure = true;
    *next_ns = now_ns + AIR_PERIOD_NS;
    return 0;
}

int32_t air_process(int64_t ts_ns, const env_sample_t *raw, air_sample_t *out) {
    (void)ts_ns;
    memset(out, 0, sizeof(*out));
    out->temperature_c = raw->temperature_c;
    out->humidity_pct = raw->humidity_pct;
    return 0;
}

bool air_get_state(uint8_t *buf, uint16_t *len) {
    (void)buf;
    (void)len;
    return false;
}

bool air_version(uint8_t v[4]) {
    (void)v;
    return false;
}

#endif // HAVE_BSEC
