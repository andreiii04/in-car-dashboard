//
// trip stats module - public interface
//
// - everything that only makes sense across a whole drive: how far, how long,
//   how hard it was pushed, how far it leaned, and the weather while it drove
// - one session per power-on; the car cuts power when the ignition goes off,
//   so there is no such thing as resuming
// - no HAL and no hardware in here either, so it runs on the mac; time comes
//   in as a dt on every motion update and there is no clock of its own
// - the accelerations it is given are in car axes and still have gravity in
//   them; taking gravity out needs the pitch and roll, which is why those are
//   passed in too
//

#ifndef TRIP_H
#define TRIP_H

#include <stdbool.h>
#include <stdint.h>
#include "gps.h"

typedef struct {
    // time
    uint32_t duration_s;        // since the ignition came on
    uint32_t moving_s;          // only the part spent actually moving
    bool     start_utc_valid;   // false if the GPS never got a fix
    uint8_t  start_hour, start_minute, start_second;
    uint8_t  start_day, start_month, start_year;

    // distance and speed
    float    distance_km;
    float    speed_max_kmh;

    // acceleration peaks, car axes, gravity taken out, in g
    // sign note: the accelerometer reads the push, not the feel - in a left
    // bend the tyres push the car leftward, so a left turn is the positive
    // number here; the "thrown to the right" feeling is the fictitious force
    // and points the other way
    float    accel_peak_pos_g;  // speeding up
    float    accel_peak_neg_g;  // braking, negative number
    float    accel_peak_left_g; // hardest left cornering
    float    accel_peak_right_g;// hardest right cornering, negative number

    // how far it leaned, in degrees
    float    pitch_max_up_deg;
    float    pitch_max_down_deg;   // negative number
    float    roll_max_right_deg;
    float    roll_max_left_deg;    // negative number

    // how close it came to tipping over; 1.0 means at the limit worked out
    // from the car's own track width and centre of gravity height
    float    lateral_load_worst;
    float    roll_load_worst;

    // the weather while it drove
    float    temp_min_c, temp_max_c, temp_avg_c;
    float    press_min_hpa, press_max_hpa, press_avg_hpa;
    float    hum_min_pct, hum_max_pct, hum_avg_pct;

    // true while the GPS is missing and the speed is being guessed from the
    // accelerometer instead; the numbers are still usable, just less certain
    bool     coasting;
} trip_stats_t;

// wipe everything and start a fresh session
void trip_init(void);

// one IMU sample; accel in g and gyro in deg/s, both already in car axes,
// pitch and roll from the attitude filter, dt in seconds
//
// this is also the module's clock - everything that counts seconds counts them
// from the dt given here
void trip_update_motion(const float accel_g[3], const float gyro_dps[3],
                        float pitch_deg, float roll_deg, float dt_s);

// one GPS fix, whenever a fresh one turns up; works at any update rate,
// nothing in here assumes one per second
//
// only call this when the fix is actually new - updated_tick changed - because
// every call restarts the between-fixes clock, and repeating the same fix
// grinds trip_long_accel_g down to zero without any error showing
void trip_update_gps(const gps_fix_t *fix);

// one BME680 sample, roughly every 3 seconds
void trip_update_env(float temp_c, float press_hpa, float hum_pct);

// copy out everything
void trip_get(trip_stats_t *out);

// the best guess at how fast the car is going right now, GPS if there is one
// and the coasted estimate if there is not
float trip_speed_kmh(void);

// the smoothed pushes the peaks are taken from - car axes, gravity already
// out, 5 Hz low pass, in g; this is what the gauge dot shows
float trip_push_long_g(void);
float trip_push_lat_g(void);

// how hard the car itself is accelerating, worked out without the
// accelerometer - forward from how the GPS speed is changing, sideways from
// speed times how fast it is turning
//
// these two are what gets handed to attitude_set_linear_accel; the point is
// that they come from somewhere else entirely, so subtracting them from the
// accelerometer actually tells you something
float trip_long_accel_g(void);
float trip_lat_accel_g(void);

#endif // TRIP_H
