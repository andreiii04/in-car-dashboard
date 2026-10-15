//
// air module - public interface
//
// - air quality from the BME680 through Bosch's BSEC library: IAQ, eCO2 and
//   breath VOC, plus temperature and humidity with the board's own heat taken
//   off
// - BSEC is closed source and not in the repo. without it this builds as a
//   stand-in that keeps the old fixed settings and hands the raw readings
//   straight through, so a clone without the library still runs
// - BSEC runs the sensor: before every measurement air_control says whether to
//   measure, with which oversampling and heater, and when to come back. env.c
//   does the I2C, this file only does the maths - it never touches the bus
// - BSEC learns the air over hours and keeps that in RAM; air_get_state hands
//   it out to be saved on the card, air_init takes it back at boot
// - no BSEC types in here, same idea as env.h
//

#ifndef AIR_H
#define AIR_H

#include <stdbool.h>
#include <stdint.h>
#include "env.h"

// the largest state BSEC hands out, in bytes; air.c checks it against the
// library's own number
#define AIR_STATE_MAX 180u

typedef enum {
    AIR_OK = 0,
    AIR_NO_LIB,         // built without BSEC - raw readings only
    AIR_ERR_INIT,       // bsec_init failed
    AIR_ERR_CONFIG,     // the config blob was refused
    AIR_ERR_SUBSCRIBE,  // the list of outputs was refused
} air_status_t;

typedef enum {
    AIR_STATE_FRESH = 0,    // nothing handed in - BSEC starts from scratch
    AIR_STATE_RESTORED,     // the saved state was taken back
    AIR_STATE_REFUSED,      // a state was handed in but BSEC said no - scratch
} air_restore_t;

typedef struct {
    bool    valid;          // BSEC has produced an IAQ at least once
    float   iaq;            // 0-500, 50 is good air, relative to what it has seen
    uint8_t iaq_accuracy;   // 0 stabilising, 1 low, 2 medium, 3 calibrated
    float   eco2_ppm;       // CO2 equivalent, worked out from the VOCs - not measured CO2
    float   bvoc_ppm;       // breath VOC equivalent
    float   temperature_c;  // heat-compensated; the raw one without BSEC
    float   humidity_pct;   // follows the temperature; the raw one without BSEC
    bool    stabilised;     // the sensor's readings have settled after power-on
    bool    run_in;         // the first-use burn-in is done
} air_sample_t;

// start BSEC, load its config, take back a saved state if there is one
// (state NULL or len 0 for none) and ask for the outputs above. temp_offset_c
// is how much warmer the sensor runs than the air, from vehicle_axes.h
air_status_t air_init(const uint8_t *state, uint16_t state_len, float temp_offset_c,
                      air_restore_t *restored);

// what to do now: measure true means trigger with these settings; next_ns is
// when to call again. now_ns must keep counting up across the whole drive.
// returns BSEC's code - 0 fine, above 0 a warning (100 is a late call), below
// 0 an error
int32_t air_control(int64_t now_ns, env_settings_t *s, bool *measure, int64_t *next_ns);

// one finished measurement in, the outputs out; ts_ns is the now_ns of the
// air_control call that asked for it. out keeps the last values for anything
// BSEC did not send this time. same return codes as air_control
int32_t air_process(int64_t ts_ns, const env_sample_t *raw, air_sample_t *out);

// copy BSEC's learned state into buf, which holds AIR_STATE_MAX bytes; false
// without the library or if BSEC refused
bool air_get_state(uint8_t *buf, uint16_t *len);

// the library version, four numbers; false without the library
bool air_version(uint8_t v[4]);

#endif // AIR_H
