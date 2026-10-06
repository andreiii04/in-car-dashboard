//
// trip stats module - implementation
//
// - the accelerometer reading still has gravity in it, so the first thing that
//   happens to every sample is taking gravity back out using pitch and roll;
//   without that, parking on a slope reads as permanent braking
// - what is left gets smoothed at 5 Hz before any peak is recorded, because a
//   pothole is a single 3 g sample and would otherwise become the hardest
//   braking of the whole trip
// - distance is speed times time and nothing cleverer; GPS speed comes from
//   doppler shift and is far steadier than subtracting one noisy position from
//   the next
// - when the GPS drops out the speed carries on from the accelerometer for a
//   while; that only works because gravity has already been removed
// - duration and distance accumulate as whole units in integers plus a small
//   float fraction; a single float that keeps growing starts rounding its own
//   additions once a drive passes a couple of hours
//

#include "trip.h"
#include "vehicle_axes.h"
#include <math.h>
#include <stddef.h>

// below this the GPS speed is treated as zero, everywhere - the speedometer,
// the maximum, the odometer and the acceleration estimate all read the same
// number, so one gate covers them all. measured on a balcony at hdop 2.5: a
// still board reported up to 8 km/h, which walked into vmax and the odometer
// because only the odometer was gated. a real drive settles the value
#define TRIP_SPEED_MIN_KMH 10.0f

// below this the odometer stops counting, so it does not creep upward while
// the car sits at a red light
#define TRIP_DISTANCE_MIN_KMH 3.0f

// below this no acceleration peak is recorded - a door slam or picking the
// case up is not cornering
#define TRIP_PEAK_MIN_KMH 5.0f

// smoothing corner for the peaks; body movement is under 5 Hz and impacts are
// well above it, so this keeps one and drops the other
#define TRIP_PEAK_LPF_HZ 5.0f

// a fix older than this means the sky is gone - a tunnel, a car park
#define TRIP_GPS_STALE_S 2.0f

// how long the accelerometer is allowed to carry the speed on its own; the
// error grows with time, so past this the distance is frozen rather than
// invented
#define TRIP_COAST_MAX_S 120.0f

// what counts as pulling away again after coasting down to a standstill
#define TRIP_RESTART_G 0.05f
#define TRIP_RESTART_S 0.5f

#define TRIP_G_TO_MPS2 9.80665f
#define TRIP_DEG_TO_RAD 0.017453293f
#define TRIP_TWO_PI 6.283185307f

static trip_stats_t stats;

// whole seconds and whole metres live in integers, which never round; only
// the part below 1.0 lives in a float, which never grows
static uint32_t elapsed_whole_s;
static float    elapsed_frac_s;
static uint32_t moving_whole_s;
static float    moving_frac_s;
static uint32_t dist_whole_m;
static float    dist_frac_m;

static float speed_mps;           // best guess right now
static float gps_speed_mps;       // last one the GPS actually gave
static bool  have_gps_speed;
static float since_fix_s;

static bool  coasting;
static float coast_s;
static bool  stopped;             // coasted all the way down to standing still
static float restart_s;

static float accel_long_filt_g;   // smoothed, gravity already out
static float accel_lat_filt_g;

static float long_accel_est_g;    // from the GPS speed changing
static float lat_accel_est_g;     // from speed and how fast it is turning

static uint32_t temp_n, press_n, hum_n;

static float trip_abs(float v) {
    return (v < 0.0f) ? -v : v;
}

// carry the whole units out of a growing fraction into the integer next to
// it, like a clock carrying seconds into minutes
static void trip_carry(uint32_t *whole, float *frac) {
    if (*frac >= 1.0f) {
        float w = floorf(*frac);
        *whole += (uint32_t)w;
        *frac -= w;
    }
}

void trip_init(void) {
    trip_stats_t blank = { 0 };
    stats = blank;

    elapsed_whole_s = 0;
    elapsed_frac_s = 0.0f;
    moving_whole_s = 0;
    moving_frac_s = 0.0f;
    dist_whole_m = 0;
    dist_frac_m = 0.0f;
    speed_mps = 0.0f;
    gps_speed_mps = 0.0f;
    have_gps_speed = false;
    since_fix_s = 0.0f;
    coasting = false;
    coast_s = 0.0f;
    stopped = false;
    restart_s = 0.0f;
    accel_long_filt_g = 0.0f;
    accel_lat_filt_g = 0.0f;
    long_accel_est_g = 0.0f;
    lat_accel_est_g = 0.0f;
    temp_n = 0;
    press_n = 0;
    hum_n = 0;
}

// one running average that does not lose precision on a long drive; adding
// every sample into a growing total and dividing at the end does
static void trip_mean(float *mean, uint32_t *count, float value) {
    (*count)++;
    *mean += (value - *mean) / (float)(*count);
}

void trip_update_motion(const float accel_g[3], const float gyro_dps[3],
                        float pitch_deg, float roll_deg, float dt_s) {
    if ((accel_g == NULL) || (gyro_dps == NULL) || (dt_s <= 0.0f)) {
        return;
    }

    elapsed_frac_s += dt_s;
    trip_carry(&elapsed_whole_s, &elapsed_frac_s);
    since_fix_s += dt_s;

    // where gravity is pointing in car axes right now, from the two angles the
    // attitude filter worked out; subtracting it leaves only what the car is
    // actually doing
    float pitch_rad = pitch_deg * TRIP_DEG_TO_RAD;
    float roll_rad = roll_deg * TRIP_DEG_TO_RAD;
    float gravity_x = sinf(pitch_rad);
    float gravity_y = cosf(pitch_rad) * sinf(roll_rad);

    float long_g = accel_g[0] - gravity_x;
    float lat_g = accel_g[1] - gravity_y;

    // single pole low pass; alpha works out from the corner frequency and the
    // step size, so changing the sample rate does not change the filtering
    float lpf_tau = 1.0f / (TRIP_TWO_PI * TRIP_PEAK_LPF_HZ);
    float lpf_alpha = dt_s / (lpf_tau + dt_s);
    accel_long_filt_g += lpf_alpha * (long_g - accel_long_filt_g);
    accel_lat_filt_g += lpf_alpha * (lat_g - accel_lat_filt_g);

    // the GPS has gone quiet - carry the speed on with the accelerometer. only
    // once a real GPS speed has been seen: before the first fix there is no
    // speed to carry on from, and guessing one from the board being picked up
    // gave phantom distance and lean angles on the bench and in the car
    if (have_gps_speed && (since_fix_s > TRIP_GPS_STALE_S)) {
        coasting = true;
        coast_s += dt_s;
        long_accel_est_g = 0.0f;   // no GPS means no independent measurement

        if (coast_s > TRIP_COAST_MAX_S) {
            // too long to keep guessing; freeze rather than invent distance
            speed_mps = 0.0f;
        } else if (stopped) {
            // held at a standstill until something clearly pushes again -
            // stopping also wipes the error that had built up, so a tunnel
            // where the car actually stops comes out better than one where it
            // crawls
            if (accel_long_filt_g > TRIP_RESTART_G) {
                restart_s += dt_s;
                if (restart_s >= TRIP_RESTART_S) {
                    stopped = false;
                }
            } else {
                restart_s = 0.0f;
            }
        } else {
            speed_mps += accel_long_filt_g * TRIP_G_TO_MPS2 * dt_s;
            if (speed_mps <= 0.0f) {
                speed_mps = 0.0f;
                stopped = true;
                restart_s = 0.0f;
            }
        }
    }
    stats.coasting = coasting;

    float speed_kmh = speed_mps * 3.6f;

    // sideways acceleration worked out without the accelerometer: how fast the
    // car is turning, times how fast it is going. clamped to what the car can
    // physically do at this speed so one bad gyro sample cannot poison it
    float yaw_dps = gyro_dps[2];
    float yaw_limit = vehicle_max_yaw_rate_dps(speed_kmh);
    if (yaw_dps > yaw_limit) {
        yaw_dps = yaw_limit;
    }
    if (yaw_dps < -yaw_limit) {
        yaw_dps = -yaw_limit;
    }
    lat_accel_est_g = (speed_mps * yaw_dps * TRIP_DEG_TO_RAD) / TRIP_G_TO_MPS2;

    if (speed_kmh >= TRIP_DISTANCE_MIN_KMH) {
        dist_frac_m += speed_mps * dt_s;
        trip_carry(&dist_whole_m, &dist_frac_m);
        moving_frac_s += dt_s;
        trip_carry(&moving_whole_s, &moving_frac_s);

        // lean angles count from walking pace up, because crawling over a rock
        // at 4 km/h is exactly when the tilt matters most
        if (pitch_deg > stats.pitch_max_up_deg) {
            stats.pitch_max_up_deg = pitch_deg;
        }
        if (pitch_deg < stats.pitch_max_down_deg) {
            stats.pitch_max_down_deg = pitch_deg;
        }
        if (roll_deg > stats.roll_max_right_deg) {
            stats.roll_max_right_deg = roll_deg;
        }
        if (roll_deg < stats.roll_max_left_deg) {
            stats.roll_max_left_deg = roll_deg;
        }

        float roll_load = trip_abs(roll_deg) / vehicle_critical_roll_deg();
        if (roll_load > stats.roll_load_worst) {
            stats.roll_load_worst = roll_load;
        }
    }

    // push peaks need a bit more speed under them than lean angles do
    if (speed_kmh >= TRIP_PEAK_MIN_KMH) {
        if (accel_long_filt_g > stats.accel_peak_pos_g) {
            stats.accel_peak_pos_g = accel_long_filt_g;
        }
        if (accel_long_filt_g < stats.accel_peak_neg_g) {
            stats.accel_peak_neg_g = accel_long_filt_g;
        }
        if (accel_lat_filt_g > stats.accel_peak_left_g) {
            stats.accel_peak_left_g = accel_lat_filt_g;
        }
        if (accel_lat_filt_g < stats.accel_peak_right_g) {
            stats.accel_peak_right_g = accel_lat_filt_g;
        }

        float lat_load = trip_abs(accel_lat_filt_g) / VEHICLE_ROLLOVER_LIMIT_G;
        if (lat_load > stats.lateral_load_worst) {
            stats.lateral_load_worst = lat_load;
        }
    }

    stats.duration_s = elapsed_whole_s;
    stats.moving_s = moving_whole_s;
    // whole metres are exact in a float up to 16 million, far past one session
    stats.distance_km = ((float)dist_whole_m + dist_frac_m) / 1000.0f;
}

void trip_update_gps(const gps_fix_t *fix) {
    if (fix == NULL) {
        return;
    }

    // the clock and the date turn up before the position does, because the
    // module keeps its own time on the backup cell
    if (fix->time_valid && !stats.start_utc_valid) {
        stats.start_hour = fix->hour;
        stats.start_minute = fix->minute;
        stats.start_second = fix->second;
        stats.start_day = fix->day;
        stats.start_month = fix->month;
        stats.start_year = fix->year;
        stats.start_utc_valid = true;
    }

    if (!fix->fix_valid) {
        return;
    }

    // GPS speed noise grows as the fix gets worse and it is indistinguishable
    // from real creeping, so anything under the floor is simply not movement
    float fix_speed_kmh = (fix->speed_kmh < TRIP_SPEED_MIN_KMH) ? 0.0f : fix->speed_kmh;
    float new_speed_mps = fix_speed_kmh / 3.6f;

    // how hard the car is really accelerating, straight from how its speed
    // changed; nothing to do with the accelerometer, which is the point
    if (have_gps_speed && (since_fix_s > 0.0f)) {
        long_accel_est_g = ((new_speed_mps - gps_speed_mps) / since_fix_s) /
                           TRIP_G_TO_MPS2;
    }

    gps_speed_mps = new_speed_mps;
    have_gps_speed = true;
    speed_mps = new_speed_mps;
    since_fix_s = 0.0f;
    coasting = false;
    coast_s = 0.0f;
    stopped = (new_speed_mps <= 0.0f);
    restart_s = 0.0f;

    if (fix_speed_kmh > stats.speed_max_kmh) {
        stats.speed_max_kmh = fix_speed_kmh;
    }
}

void trip_update_env(float temp_c, float press_hpa, float hum_pct) {
    // the first sample sets both ends, otherwise every minimum would stay at
    // zero and every maximum would look right by accident
    if (temp_n == 0) {
        stats.temp_min_c = temp_c;
        stats.temp_max_c = temp_c;
        stats.press_min_hpa = press_hpa;
        stats.press_max_hpa = press_hpa;
        stats.hum_min_pct = hum_pct;
        stats.hum_max_pct = hum_pct;
    }
    if (temp_c < stats.temp_min_c) {
        stats.temp_min_c = temp_c;
    }
    if (temp_c > stats.temp_max_c) {
        stats.temp_max_c = temp_c;
    }
    if (press_hpa < stats.press_min_hpa) {
        stats.press_min_hpa = press_hpa;
    }
    if (press_hpa > stats.press_max_hpa) {
        stats.press_max_hpa = press_hpa;
    }
    if (hum_pct < stats.hum_min_pct) {
        stats.hum_min_pct = hum_pct;
    }
    if (hum_pct > stats.hum_max_pct) {
        stats.hum_max_pct = hum_pct;
    }

    trip_mean(&stats.temp_avg_c, &temp_n, temp_c);
    trip_mean(&stats.press_avg_hpa, &press_n, press_hpa);
    trip_mean(&stats.hum_avg_pct, &hum_n, hum_pct);
}

void trip_get(trip_stats_t *out) {
    if (out == NULL) {
        return;
    }
    *out = stats;
}

float trip_speed_kmh(void) {
    return speed_mps * 3.6f;
}

float trip_push_long_g(void) {
    return accel_long_filt_g;
}

float trip_push_lat_g(void) {
    return accel_lat_filt_g;
}

float trip_long_accel_g(void) {
    return long_accel_est_g;
}

float trip_lat_accel_g(void) {
    return lat_accel_est_g;
}
