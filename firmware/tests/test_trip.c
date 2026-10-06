//
// host test for the trip stats module
//
// - trip.c has no HAL in it, so it runs on the mac like the other two
// - the awkward things to get right in here are gravity being removed before
//   anything is measured, and the speed carrying on when the GPS drops out;
//   both get their own section below
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/test_trip.c App/Src/trip.c -lm
//  -o /tmp/test_trip && /tmp/test_trip

#include <stdio.h>
#include <math.h>
#include "trip.h"
#include "vehicle_axes.h"

static int fails = 0;

static void check(const char *what, int ok) {
    printf("  %-46s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        fails++;
    }
}

static int near(float a, float b, float tol) {
    float d = a - b;
    if (d < 0) {
        d = -d;
    }
    return d < tol;
}

#define DEG (float)(M_PI / 180.0)
#define DT  (1.0f / 104.0f)
#define HZ  104

// what the accelerometer reads: gravity for the tilt, plus whatever the car is
// doing on top of it
static void reading(float pitch, float roll, float extra_x, float extra_y,
                    float out[3]) {
    out[0] = sinf(pitch * DEG) + extra_x;
    out[1] = (cosf(pitch * DEG) * sinf(roll * DEG)) + extra_y;
    out[2] = cosf(pitch * DEG) * cosf(roll * DEG);
}

static gps_fix_t fix_at(float speed_kmh) {
    gps_fix_t f = { 0 };
    f.fix_valid = true;
    f.time_valid = true;
    f.speed_kmh = speed_kmh;
    f.hour = 9;
    f.minute = 30;
    f.second = 15;
    f.day = 23;
    f.month = 8;
    f.year = 26;
    return f;
}

// drive for a while with the GPS reporting in once a second, the way it will
// on the car
static void drive(float speed_kmh, float pitch, float roll,
                  float extra_x, float extra_y, float yaw_dps, float seconds) {
    float accel[3];
    float gyro[3] = { 0.0f, 0.0f, 0.0f };
    gyro[2] = yaw_dps;
    reading(pitch, roll, extra_x, extra_y, accel);

    int samples = (int)(seconds * (float)HZ);
    for (int i = 0; i < samples; i++) {
        if ((i % HZ) == 0) {
            gps_fix_t f = fix_at(speed_kmh);
            trip_update_gps(&f);
        }
        trip_update_motion(accel, gyro, pitch, roll, DT);
    }
}

// same, but the GPS has gone - a tunnel
static void coast(float pitch, float extra_x, float seconds) {
    float accel[3];
    float gyro[3] = { 0.0f, 0.0f, 0.0f };
    reading(pitch, 0.0f, extra_x, 0.0f, accel);

    int samples = (int)(seconds * (float)HZ);
    for (int i = 0; i < samples; i++) {
        trip_update_motion(accel, gyro, pitch, 0.0f, DT);
    }
}

int main(void) {
    trip_stats_t t;

    printf("distance is speed times time, with a gate at the bottom\n");
    trip_init();
    drive(36.0f, 0, 0, 0, 0, 0, 10.0f);        // 10 m/s for 10 s
    trip_get(&t);
    check("36 km/h for 10 s is 0.1 km", near(t.distance_km, 0.1f, 0.002f));
    check("max speed recorded", near(t.speed_max_kmh, 36.0f, 0.01f));
    check("moving time counted", t.moving_s >= 9 && t.moving_s <= 11);

    trip_init();
    drive(2.0f, 0, 0, 0, 0, 0, 30.0f);         // creeping at a red light
    trip_get(&t);
    check("2 km/h adds no distance at all", near(t.distance_km, 0.0f, 1e-6f));
    check("and no moving time", t.moving_s == 0);

    printf("the clock comes off the GPS, before the fix does\n");
    trip_init();
    gps_fix_t timeonly = { 0 };
    timeonly.time_valid = true;                 // no position yet
    timeonly.hour = 7;
    timeonly.minute = 5;
    timeonly.day = 1;
    timeonly.month = 12;
    timeonly.year = 25;
    trip_update_gps(&timeonly);
    trip_get(&t);
    check("start time taken without a fix", t.start_utc_valid &&
                                            t.start_hour == 7 &&
                                            t.start_month == 12);

    printf("gravity comes out before anything is measured\n");
    trip_init();
    drive(50.0f, 10.0f, 0, 0, 0, 0, 5.0f);     // steady up a 10 degree climb
    trip_get(&t);
    check("a climb is not braking", near(t.accel_peak_pos_g, 0.0f, 0.01f) &&
                                    near(t.accel_peak_neg_g, 0.0f, 0.01f));

    trip_init();
    drive(50.0f, 0, 20.0f, 0, 0, 0, 5.0f);     // steady across a 20 degree slope
    trip_get(&t);
    check("a side slope is not cornering", near(t.accel_peak_left_g, 0.0f, 0.01f) &&
                                           near(t.accel_peak_right_g, 0.0f, 0.01f));
    check("but the lean itself is recorded", near(t.roll_max_right_deg, 20.0f, 0.01f));

    printf("peaks\n");
    trip_init();
    drive(50.0f, 0, 0, -0.3f, 0, 0, 3.0f);     // 0.3 g of braking
    trip_get(&t);
    check("0.3 g braking recorded", near(t.accel_peak_neg_g, -0.3f, 0.01f));
    check("nothing recorded the other way", near(t.accel_peak_pos_g, 0.0f, 0.01f));

    trip_init();
    drive(50.0f, 0, 0, 0, 0, 0, 2.0f);
    {   // one single 3 g sample, the way a pothole arrives
        float accel[3];
        float gyro[3] = { 0.0f, 0.0f, 0.0f };
        reading(0, 0, -3.0f, 0, accel);
        trip_update_motion(accel, gyro, 0.0f, 0.0f, DT);
    }
    trip_get(&t);
    check("a one sample pothole is smoothed away", t.accel_peak_neg_g > -0.8f);

    trip_init();
    drive(2.0f, 0, 0, -0.5f, 0, 0, 3.0f);      // hard push, but barely moving
    trip_get(&t);
    check("nothing counts below walking pace", near(t.accel_peak_neg_g, 0.0f, 0.01f));

    printf("how close it came to tipping over\n");
    trip_init();
    drive(50.0f, 0, 0, 0, 0.4f, 0, 3.0f);      // 0.4 g sideways
    trip_get(&t);
    check("sideways load against the car's own limit",
          near(t.lateral_load_worst, 0.4f / VEHICLE_ROLLOVER_LIMIT_G, 0.02f));
    trip_init();
    drive(50.0f, 0, 25.0f, 0, 0, 0, 3.0f);
    trip_get(&t);
    check("lean angle against the tipping angle",
          near(t.roll_load_worst, 25.0f / vehicle_critical_roll_deg(), 0.01f));

    printf("what gets handed to the attitude filter\n");
    trip_init();
    {   // 0 to 10 m/s in one second is 10 m/s2, which is 1.02 g
        gps_fix_t a = fix_at(0.0f);
        gps_fix_t b = fix_at(36.0f);
        trip_update_gps(&a);
        coast(0, 0, 1.0f);                      // one second of motion updates
        trip_update_gps(&b);
    }
    check("forward push from the GPS speed changing",
          near(trip_long_accel_g(), 10.0f / 9.80665f, 0.02f));

    trip_init();
    drive(72.0f, 0, 0, 0, 0, 10.0f, 3.0f);      // 20 m/s while turning 10 deg/s
    check("sideways push from speed and turn rate",
          near(trip_lat_accel_g(), (20.0f * 10.0f * (float)(M_PI / 180.0)) / 9.80665f,
               0.01f));

    trip_init();
    drive(72.0f, 0, 0, 0, 0, 500.0f, 3.0f);     // a gyro glitch, not a manoeuvre
    check("an impossible turn rate is clamped",
          trip_lat_accel_g() < ((20.0f * 500.0f * (float)(M_PI / 180.0)) / 9.80665f));

    printf("the GPS drops out - a tunnel\n");
    trip_init();
    drive(36.0f, 0, 0, 0, 0, 0, 3.0f);          // 10 m/s, then the sky is gone
    coast(0, 0.0f, 10.0f);                      // 10 s, no push either way
    trip_get(&t);
    check("it knows it is guessing", t.coasting);
    check("speed carries on unchanged", near(trip_speed_kmh(), 36.0f, 0.5f));
    check("and distance keeps counting", t.distance_km > 0.12f);

    trip_init();
    drive(36.0f, 0, 0, 0, 0, 0, 3.0f);
    coast(0, -0.2f, 10.0f);                     // braking to a stop in the tunnel
    check("braking in the tunnel stops the car", near(trip_speed_kmh(), 0.0f, 0.01f));
    coast(0, 0.0f, 10.0f);                      // sitting still, engine idling
    check("and it stays stopped", near(trip_speed_kmh(), 0.0f, 0.01f));
    coast(0, 0.2f, 2.0f);                       // pulling away again
    check("a real push starts it moving again", trip_speed_kmh() > 1.0f);

    trip_init();
    drive(36.0f, 0, 0, 0, 0, 0, 3.0f);
    coast(0, 0.0f, 130.0f);                     // far too long to keep guessing
    trip_get(&t);
    float frozen = t.distance_km;
    coast(0, 0.0f, 10.0f);
    trip_get(&t);
    check("after two minutes it gives up guessing", near(trip_speed_kmh(), 0.0f, 0.01f));
    check("and the distance is frozen, not invented",
          near(t.distance_km, frozen, 1e-6f));

    // a still car on a poor fix reports a few km/h of noise; nothing downstream
    // may believe it, so the gate is checked on the speedometer, the maximum
    // and the odometer at once
    printf("GPS speed noise at a standstill\n");
    trip_init();
    for (int i = 0; i < 20; i++) {
        gps_fix_t noise = fix_at(8.0f);          // under the floor, plausible noise
        trip_update_gps(&noise);
        float a[3] = { 0.0f, 0.0f, 1.0f };
        float g[3] = { 0.0f, 0.0f, 0.0f };
        trip_update_motion(a, g, 0.0f, 0.0f, 1.0f);
    }
    trip_get(&t);
    check("noise under the floor reads as stopped", near(trip_speed_kmh(), 0.0f, 0.01f));
    check("noise never becomes a maximum speed", near(t.speed_max_kmh, 0.0f, 0.01f));
    check("noise never becomes distance", near(t.distance_km, 0.0f, 1e-6f));
    check("noise never becomes moving time", t.moving_s == 0u);

    trip_init();
    for (int i = 0; i < 20; i++) {
        gps_fix_t real = fix_at(50.0f);          // well over the floor
        trip_update_gps(&real);
        float a[3] = { 0.0f, 0.0f, 1.0f };
        float g[3] = { 0.0f, 0.0f, 0.0f };
        trip_update_motion(a, g, 0.0f, 0.0f, 1.0f);
    }
    trip_get(&t);
    check("a real speed still gets through", near(trip_speed_kmh(), 50.0f, 0.5f));
    check("a real speed still sets the maximum", near(t.speed_max_kmh, 50.0f, 0.5f));
    check("a real speed still counts distance", t.distance_km > 0.2f);

    // no fix yet and the board handled - pushed, tilted and put down again;
    // with nothing to carry on from, none of it may count as driving
    printf("no GPS since power-on\n");
    trip_init();
    coast(0, 0.2f, 3.0f);                        // a shove along the car
    coast(40.0f, 0.1f, 3.0f);                    // tipped nose up while pushed
    coast(0, 0.0f, 3.0f);
    trip_get(&t);
    check("no speed before the first fix", near(trip_speed_kmh(), 0.0f, 0.01f));
    check("no coasting before the first fix", !t.coasting);
    check("no distance before the first fix", near(t.distance_km, 0.0f, 1e-6f));
    check("no moving time before the first fix", t.moving_s == 0u);
    check("no lean angle before the first fix", near(t.pitch_max_up_deg, 0.0f, 0.01f));
    check("no push peak before the first fix", near(t.accel_peak_pos_g, 0.0f, 0.01f));
    check("but the clock still runs", t.duration_s >= 8u);

    printf("the weather while it drove\n");
    trip_init();
    trip_update_env(20.0f, 1000.0f, 40.0f);
    trip_update_env(30.0f, 1010.0f, 60.0f);
    trip_update_env(25.0f, 1005.0f, 50.0f);
    trip_get(&t);
    check("lowest temperature kept", near(t.temp_min_c, 20.0f, 0.01f));
    check("highest temperature kept", near(t.temp_max_c, 30.0f, 0.01f));
    check("average temperature kept", near(t.temp_avg_c, 25.0f, 0.01f));
    check("pressure average", near(t.press_avg_hpa, 1005.0f, 0.01f));
    check("humidity average", near(t.hum_avg_pct, 50.0f, 0.01f));

    printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
    return fails != 0;
}
