//
// elevation module - implementation
//
// - the barometer's absolute height is never right, because working it out
//   needs today's sea level pressure and that changes with the weather; but
//   its *changes* are right, and changes are all the filter needs from it
// - so: step the height along with the barometer's change, then lean it a
//   little towards whatever the GPS says. exactly the shape of attitude.c,
//   with a much longer time constant because the drift here takes hours
// - climb is added up from the barometer alone, with a deadband, so noise
//   wobbling either side of the same height does not slowly count as a
//   mountain, and the GPS settling after a cold start does not either
// - the start is the average of the first few fixes, taken as an offset from
//   the barometer so the car may already be moving while it averages
//

#include "elevation.h"
#include <math.h>
#include <stddef.h>
#include <stdint.h>

// how the two are weighed against each other, in seconds. much longer than the
// attitude filter's because what it is fighting is slow: air pressure moves
// about 1 hPa an hour in changeable weather, which is roughly 8 metres an hour
// of false height, while GPS altitude noise is tens of metres but averages out
//
// how to change it later:
//  - too small and the GPS noise comes straight through, so the height jitters
//    and the climb total inflates
//  - too big and a real climb reads low until the filter catches up
//  - 60 s is the starting point; the way to settle it is the same as the
//    attitude one - log GPS altitude, pressure and this output on a drive over
//    a hill of known height, then replay it on the PC with different values
#define ELEV_TAU_S 60.0f

// how far the height has to move before it counts as climbing or descending;
// under this it is treated as the same height
#define ELEV_DEADBAND_M 3.0f

// when the height counts as settled: the gap between the GPS and the combined
// figure, smoothed, has to stay under ELEV_SETTLE_M for ELEV_SETTLE_HOLD_S.
// session 7 walked from about -170 m to 65 m over 5 minutes; while the GPS is
// still walking like that, the gap stays tens of metres
#define ELEV_SETTLE_TAU_S  20.0f
#define ELEV_SETTLE_M      10.0f
#define ELEV_SETTLE_HOLD_S 60.0f

// standard atmosphere - the pressure at sea level on an average day and the
// exponent that links pressure to height. the sea level figure is wrong most
// days, which is exactly why only the changes get used
#define ELEV_SEA_LEVEL_HPA 1013.25f
#define ELEV_EXPONENT      0.190294957f
#define ELEV_SCALE_M       44330.0f

static float fused_m;
static float baro_prev_m;
static bool  have_baro_prev;
static bool  seeded;
static float seed_offset_sum_m;   // GPS height minus barometer height, added up
static uint32_t seed_count;
static float seed_m;
static float reference_m;    // the barometer height the climb count is measured from
static float gain_m;
static float loss_m;
static float gap_filt_m;     // GPS minus the combined figure, smoothed
static float settle_s;       // how long the gap has stayed small
static bool  settled;

// pressure to height, standard atmosphere; the answer is off by however far
// today's weather is from average, but that error is the same from one sample
// to the next so it cancels when the difference is taken
static float elev_from_pressure(float pressure_hpa) {
    return ELEV_SCALE_M *
           (1.0f - powf(pressure_hpa / ELEV_SEA_LEVEL_HPA, ELEV_EXPONENT));
}

void elev_init(void) {
    fused_m = 0.0f;
    baro_prev_m = 0.0f;
    have_baro_prev = false;
    seeded = false;
    seed_offset_sum_m = 0.0f;
    seed_count = 0;
    seed_m = 0.0f;
    reference_m = 0.0f;
    gain_m = 0.0f;
    loss_m = 0.0f;
    gap_filt_m = 0.0f;
    settle_s = 0.0f;
    settled = false;
}

void elev_update(float pressure_hpa, bool gps_valid, float gps_altitude_m,
                 float dt_s) {
    if ((pressure_hpa <= 0.0f) || (dt_s <= 0.0f)) {
        return;
    }

    float baro_m = elev_from_pressure(pressure_hpa);

    // add up the climbing, barometer only. the reference only moves once the
    // height has really gone somewhere, so small wobbles around one level add
    // nothing
    if (!have_baro_prev) {
        reference_m = baro_m;
    } else if (baro_m > (reference_m + ELEV_DEADBAND_M)) {
        gain_m += baro_m - (reference_m + ELEV_DEADBAND_M);
        reference_m = baro_m - ELEV_DEADBAND_M;
    } else if (baro_m < (reference_m - ELEV_DEADBAND_M)) {
        loss_m += (reference_m - ELEV_DEADBAND_M) - baro_m;
        reference_m = baro_m + ELEV_DEADBAND_M;
    }

    // the barometer's change since last time, which is the part of it worth
    // trusting
    float step_m = 0.0f;
    if (have_baro_prev) {
        step_m = baro_m - baro_prev_m;
    }
    baro_prev_m = baro_m;
    have_baro_prev = true;

    // nothing absolute until the GPS says where we are; average the first
    // fixes as an offset from the barometer, so climbing while it averages
    // does not drag the start down to the middle of the climb
    if (!seeded) {
        if (gps_valid) {
            seed_offset_sum_m += gps_altitude_m - baro_m;
            seed_count++;
            if (seed_count >= ELEV_SEED_FIXES) {
                fused_m = baro_m + (seed_offset_sum_m / (float)seed_count);
                seed_m = fused_m;
                seeded = true;
            }
        }
        return;
    }

    float predicted_m = fused_m + step_m;

    if (gps_valid) {
        float gap_m = gps_altitude_m - predicted_m;
        float alpha = ELEV_TAU_S / (ELEV_TAU_S + dt_s);
        fused_m = predicted_m + ((1.0f - alpha) * gap_m);

        // settled once the GPS stops pulling the figure somewhere else; latched,
        // after that the 60 s filter takes care of a bad fix on its own
        if (!settled) {
            float settle_alpha = dt_s / (ELEV_SETTLE_TAU_S + dt_s);
            gap_filt_m += settle_alpha * (gap_m - gap_filt_m);
            if (fabsf(gap_filt_m) < ELEV_SETTLE_M) {
                settle_s += dt_s;
                if (settle_s >= ELEV_SETTLE_HOLD_S) {
                    settled = true;
                }
            } else {
                settle_s = 0.0f;
            }
        }
    } else {
        // no fix, so coast on the barometer alone; it drifts slowly enough
        // that a tunnel or a canyon does no real harm
        fused_m = predicted_m;
    }
}

void elev_get(elevation_t *out) {
    if (out == NULL) {
        return;
    }
    out->altitude_m = fused_m;
    out->seed_m = seed_m;
    out->gain_m = gain_m;
    out->loss_m = loss_m;
    out->valid = seeded;
    out->settled = settled;
    out->counting = have_baro_prev;
}
