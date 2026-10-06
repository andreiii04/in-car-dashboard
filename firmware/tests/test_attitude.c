//
// host test for the attitude filter
//
// - attitude.c has no HAL in it either, so it runs on the mac like the NMEA one
// - every expected number here is worked out by hand from the angle fed in,
//   so a wrong sign or a mixed up axis shows up as a failed line, not as a
//   filter that merely looks a bit sluggish on the car
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/test_attitude.c App/Src/attitude.c -lm -o
//  /tmp/test_attitude && /tmp/test_attitude

#include <stdio.h>
#include <math.h>
#include "attitude.h"

static int fails = 0;

static void check(const char *what, int ok) {
    printf("  %-44s %s\n", what, ok ? "ok" : "FAIL");
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
#define DT  (1.0f / 104.0f)   // one sample at the IMU's 104 Hz

// hold a steady reading for a while so the filter has time to settle
static void feed(float ax, float ay, float az,
                 float gx, float gy, float gz, int samples) {
    for (int i = 0; i < samples; i++) {
        attitude_update(ax, ay, az, gx, gy, gz, DT);
    }
}

// what the accelerometer reads when the car is tilted by these two angles
static void gravity_at(float pitch, float roll, float out[3]) {
    out[0] = sinf(pitch * DEG);
    out[1] = cosf(pitch * DEG) * sinf(roll * DEG);
    out[2] = cosf(pitch * DEG) * cosf(roll * DEG);
}

int main(void) {
    attitude_t a;
    float g[3];

    printf("flat mount, angles come straight out of gravity\n");
    attitude_init();
    check("set_mount accepts a level reading", attitude_set_mount(0.0f, 0.0f, 1.0f));
    gravity_at(10.0f, 0.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 3000);
    attitude_get(&a);
    check("nose up 10 gives pitch +10", near(a.pitch_deg, 10.0f, 0.01f));
    check("and roll stays 0", near(a.roll_deg, 0.0f, 0.01f));

    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    gravity_at(0.0f, 10.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 3000);
    attitude_get(&a);
    check("right side down 10 gives roll +10", near(a.roll_deg, 10.0f, 0.01f));
    check("and pitch stays 0", near(a.pitch_deg, 0.0f, 0.01f));

    printf("the mount rotation cancels a 45 degree case\n");
    // the reading taken once with the car parked, board tipped 45 deg back
    float rest[3];
    gravity_at(45.0f, 0.0f, rest);
    attitude_init();
    check("set_mount accepts it", attitude_set_mount(rest[0], rest[1], rest[2]));
    feed(rest[0], rest[1], rest[2], 0, 0, 0, 3000);
    attitude_get(&a);
    check("parked car reads level", near(a.pitch_deg, 0.0f, 0.01f) &&
                                    near(a.roll_deg, 0.0f, 0.01f));
    // now the car itself noses up 10, so the board sits at 55
    gravity_at(55.0f, 0.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 3000);
    attitude_get(&a);
    check("car nose up 10 reads 10, not 55", near(a.pitch_deg, 10.0f, 0.02f));
    check("nothing leaks into roll", near(a.roll_deg, 0.0f, 0.02f));

    attitude_init();
    check("upside down mount accepted", attitude_set_mount(0.0f, 0.0f, -1.0f));
    feed(0.0f, 0.0f, -1.0f, 0, 0, 0, 3000);
    attitude_get(&a);
    check("upside down parked car reads level", near(a.pitch_deg, 0.0f, 0.01f) &&
                                                near(a.roll_deg, 0.0f, 0.01f));
    check("a zero vector is refused", !attitude_set_mount(0.0f, 0.0f, 0.0f));

    printf("gyro signs - the easiest thing in here to get backwards\n");
    // accel is doubled so it reads 2 g and the gate throws it away; that
    // leaves the gyro on its own, which is the point of these two
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 1);            // seed at zero
    feed(0.0f, 0.0f, 2.0f, 0, -10, 0, 104);        // 1 s of -10 dps on Y
    attitude_get(&a);
    check("gyro Y -10 dps for 1 s is pitch +10", near(a.pitch_deg, 10.0f, 0.05f));
    check("accel_used says it was gyro only", !a.accel_used);

    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 1);
    feed(0.0f, 0.0f, 2.0f, 10, 0, 0, 104);         // 1 s of +10 dps on X
    attitude_get(&a);
    check("gyro X +10 dps for 1 s is roll +10", near(a.roll_deg, 10.0f, 0.05f));

    printf("the gyro bias gets subtracted\n");
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    attitude_set_gyro_bias(1.0f, 1.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 1);
    feed(0.0f, 0.0f, 2.0f, 1.0f, 1.0f, 1.0f, 1040);  // 10 s of pure bias
    attitude_get(&a);
    check("10 s of bias only moves nothing", near(a.pitch_deg, 0.0f, 0.01f) &&
                                             near(a.roll_deg, 0.0f, 0.01f));

    printf("the gate - what it catches and what it does not\n");
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 300);
    feed(-0.5f, 0.0f, 1.0f, 0, 0, 0, 312);   // 3 s of braking at 0.5 g
    attitude_get(&a);
    check("0.5 g braking is thrown away", near(a.pitch_deg, 0.0f, 0.01f));
    check("accel_used goes false while it lasts", !a.accel_used);

    // 0.2 g sits inside the gate, so it does leak in; the false angle is
    // atan(0.2) = 11.3 deg and after 3 s at tau = 2 s about 78% of it is there
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 300);
    feed(-0.2f, 0.0f, 1.0f, 0, 0, 0, 312);
    attitude_get(&a);
    check("0.2 g slips through, about -8.8 deg", near(a.pitch_deg, -8.8f, 0.3f));
    // and it comes back on its own once the car stops pushing
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 2000);
    attitude_get(&a);
    check("and decays back to level after", near(a.pitch_deg, 0.0f, 0.05f));

    printf("startup - nothing until the first believable accel sample\n");
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 2.0f, 50, 50, 0, 500);   // driving already, gate shut
    attitude_get(&a);
    check("stays unsettled", !a.settled);
    check("and the angles stay at zero", near(a.pitch_deg, 0.0f, 0.001f) &&
                                         near(a.roll_deg, 0.0f, 0.001f));
    gravity_at(10.0f, 0.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 1);       // one good sample
    attitude_get(&a);
    check("first good sample settles it", a.settled);
    check("and jumps straight to 10, no crawl", near(a.pitch_deg, 10.0f, 0.01f));

    printf("a dt of zero or less is ignored\n");
    attitude_get(&a);
    float before = a.pitch_deg;
    attitude_update(0.0f, 0.0f, 2.0f, 0, -1000, 0, 0.0f);
    attitude_update(0.0f, 0.0f, 2.0f, 0, -1000, 0, -1.0f);
    attitude_get(&a);
    check("nothing moved", near(a.pitch_deg, before, 0.001f));

    printf("roll wraps at the far end instead of spinning the wrong way\n");
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    gravity_at(0.0f, 179.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 1);              // seeded at +179
    attitude_get(&a);
    check("seeds at +179", near(a.roll_deg, 179.0f, 0.01f));
    feed(0.0f, 0.0f, 2.0f, 10, 0, 0, 31);            // gyro pushes it past 180
    attitude_get(&a);
    check("rolls past 180 into -178", near(a.roll_deg, -178.0f, 0.2f));
    // and the blend takes the short way round too, not backwards through zero
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    gravity_at(0.0f, 179.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 1);
    gravity_at(0.0f, -179.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 3000);
    attitude_get(&a);
    check("blends 179 to -179 the short way", near(a.roll_deg, -179.0f, 0.05f));

    printf("big tilts - the small angle shortcut is not good enough here\n");
    // rolled right over onto its side, so the car's own Z axis now points
    // sideways and turning about it pitches the nose; the old shortcut would
    // have reported nothing at all
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 1.0f, 0.0f, 0, 0, 0, 1);              // seeds at roll +90
    attitude_get(&a);
    check("seeds at roll +90", near(a.roll_deg, 90.0f, 0.01f));
    feed(0.0f, 2.0f, 0.0f, 0, 0, 10, 104);           // 1 s of yaw rate, gated
    attitude_get(&a);
    check("on its side, yaw rate becomes pitch", near(a.pitch_deg, 10.0f, 0.05f));
    check("and roll stays at 90", near(a.roll_deg, 90.0f, 0.05f));

    // nose up 30, then turning; part of the yaw rate has to land in roll
    // because the car's axes are tilted. tan(30) x 10 = 5.77 deg/s
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    gravity_at(30.0f, 0.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 1);
    feed(g[0] * 2.0f, g[1] * 2.0f, g[2] * 2.0f, 0, 0, 10, 104);
    attitude_get(&a);
    check("nose up 30, yaw leaks into roll", near(a.roll_deg, -5.7251f, 0.05f));
    check("and pitch drops a little with it", near(a.pitch_deg, 29.4987f, 0.05f));

    // all three axes at once, which exercises every term in the maths; the
    // two expected numbers come from integrating the rotation directly, with
    // no angle formulas involved at all, so they are a real cross check
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    gravity_at(30.0f, 20.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 1);
    feed(g[0] * 2.0f, g[1] * 2.0f, g[2] * 2.0f, 5, 5, 5, 104);
    attitude_get(&a);
    check("pitch 30 roll 20, 5 dps on all three", near(a.pitch_deg, 27.0920f, 0.05f));
    check("roll matches the direct integration", near(a.roll_deg, 21.4953f, 0.05f));

    // straight up would divide by zero without the clamp
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    gravity_at(89.0f, 0.0f, g);
    feed(g[0], g[1], g[2], 0, 0, 0, 1);
    feed(g[0] * 2.0f, g[1] * 2.0f, g[2] * 2.0f, 0, 0, 10, 104);
    attitude_get(&a);
    check("nearly vertical stays a real number", a.roll_deg == a.roll_deg &&
                                                 a.roll_deg < 1e6f);

    printf("telling it what the car is doing lets the accel back in\n");
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 300);
    attitude_set_linear_accel(-0.5f, 0.0f);          // 0.5 g of braking, known
    feed(-0.5f, 0.0f, 1.0f, 0, 0, 0, 312);           // 3 s of it
    attitude_get(&a);
    check("compensated braking is believed again", a.accel_used);
    check("and the angle does not move", near(a.pitch_deg, 0.0f, 0.01f));

    // and without being told, the same 3 seconds is thrown away instead
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 300);
    feed(-0.5f, 0.0f, 1.0f, 0, 0, 0, 312);
    attitude_get(&a);
    check("init clears what it was last told", !a.accel_used);

    // session 17: a 0.4 g left bend on a flat road. 0.4 g sideways is only
    // 1.077 g in total, inside the gate, so if nobody says the car is turning
    // the filter takes it for a lean - atan(0.4) = 21.8 deg, pulled in at
    // tau 2 s, is 21.8 x (1 - e^-0.5) = 8.6 deg after 1 s. that second is how
    // long main.c used to wait for the next GPS fix before telling it
    printf("a bend nobody mentions for a second\n");
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 300);
    feed(0.0f, 0.4f, 1.0f, 0, 0, 0, 104);            // 1 s, not told
    attitude_get(&a);
    check("the gate lets 0.4 g sideways through", a.accel_used);
    check("and a second of it reads as ~8.6 deg of roll", near(a.roll_deg, 8.6f, 1.0f));

    // told on every sample, the same second leaves the road flat
    attitude_init();
    attitude_set_mount(0.0f, 0.0f, 1.0f);
    feed(0.0f, 0.0f, 1.0f, 0, 0, 0, 300);
    attitude_set_linear_accel(0.0f, 0.4f);
    feed(0.0f, 0.4f, 1.0f, 0, 0, 0, 104);
    attitude_get(&a);
    check("told at once, the road stays flat", near(a.roll_deg, 0.0f, 0.01f));

    printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
    return fails != 0;
}
