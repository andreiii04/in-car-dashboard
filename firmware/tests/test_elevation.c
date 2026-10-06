//
// host test for the elevation module
//
// - elevation.c has no HAL in it, so it runs on the mac like the others
// - the pressures below were worked out backwards from the standard atmosphere
//   formula, so 1001.29 hPa is exactly 100 m above 1013.25 hPa
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/test_elevation.c
//  App/Src/elevation.c -lm -o /tmp/test_elevation && /tmp/test_elevation

#include <stdio.h>
#include "elevation.h"

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

#define SEA   1013.25f   // sea level on an average day
#define UP100 1001.29f   // the same air 100 m higher up
#define DOWN50 1019.26f  // and 50 m lower down
#define STEP_S 3.0f      // the BME680 gives a sample about this often

// hold one pressure and one GPS altitude for a while
static void hold(float pressure, int gps_valid, float gps_alt, int steps) {
    for (int i = 0; i < steps; i++) {
        elev_update(pressure, gps_valid ? true : false, gps_alt, STEP_S);
    }
}

int main(void) {
    elevation_t e;

    printf("nothing is known until the GPS says where we are\n");
    elev_init();
    hold(SEA, 0, 0.0f, 10);
    elev_get(&e);
    check("stays invalid without a fix", !e.valid);
    hold(SEA, 1, 200.0f, (int)ELEV_SEED_FIXES - 1);
    elev_get(&e);
    check("one fix short of the average is not enough", !e.valid);
    hold(SEA, 1, 200.0f, 1);
    elev_get(&e);
    check("the last fix of the average seeds it", e.valid);
    check("and it starts at the GPS altitude", near(e.altitude_m, 200.0f, 0.01f));

    printf("the barometer supplies the movement\n");
    elev_init();
    hold(SEA, 1, 0.0f, (int)ELEV_SEED_FIXES);          // seeded at sea level
    hold(UP100, 0, 0.0f, 1);        // one step, 100 m of climb, no GPS
    elev_get(&e);
    check("a 12 hPa drop is 100 m of climb", near(e.altitude_m, 100.0f, 1.0f));
    hold(SEA, 0, 0.0f, 1);
    elev_get(&e);
    check("and coming back down returns it", near(e.altitude_m, 0.0f, 1.0f));

    printf("the GPS supplies the truth, slowly\n");
    elev_init();
    hold(SEA, 1, 0.0f, (int)ELEV_SEED_FIXES);          // seeded at 0
    hold(SEA, 1, 500.0f, 400);      // pressure says nothing moved, GPS says 500
    elev_get(&e);
    check("it is pulled onto the GPS altitude", near(e.altitude_m, 500.0f, 1.0f));

    // and it gets there slowly, not in one jump - that is the whole point
    elev_init();
    hold(SEA, 1, 0.0f, (int)ELEV_SEED_FIXES);
    hold(SEA, 1, 500.0f, 1);
    elev_get(&e);
    check("one noisy fix barely moves it", e.altitude_m < 50.0f);

    printf("climb adds up, noise does not\n");
    elev_init();
    hold(SEA, 1, 0.0f, (int)ELEV_SEED_FIXES);
    hold(UP100, 0, 0.0f, 1);        // up 100
    elev_get(&e);
    check("100 m of climb, less the deadband", near(e.gain_m, 97.0f, 1.5f));
    check("and nothing counted as descent", near(e.loss_m, 0.0f, 0.01f));
    hold(SEA, 0, 0.0f, 1);          // back down again
    elev_get(&e);
    check("coming back down counts as descent", e.loss_m > 90.0f);

    elev_init();
    hold(SEA, 1, 0.0f, (int)ELEV_SEED_FIXES);
    // wobbling either side of the same height; 0.15 hPa is about 1.25 m, well
    // inside the deadband - the earlier 1.5 hPa i tried is 12 m, which is a
    // real hill and should count
    for (int i = 0; i < 20; i++) {
        hold(1013.40f, 0, 0.0f, 1);
        hold(1013.10f, 0, 0.0f, 1);
    }
    elev_get(&e);
    check("small wobble is not a mountain", near(e.gain_m, 0.0f, 0.5f));
    check("nor a valley", near(e.loss_m, 0.0f, 0.5f));

    printf("a descent below the start still works\n");
    elev_init();
    hold(SEA, 1, 0.0f, (int)ELEV_SEED_FIXES);
    hold(DOWN50, 0, 0.0f, 1);
    elev_get(&e);
    check("50 m down is negative altitude", near(e.altitude_m, -50.0f, 1.0f));
    check("and counted as descent", near(e.loss_m, 47.0f, 1.5f));

    // the start is the plain average of the first fixes - 50 and 150 in turn
    // average to 100 by arithmetic, nothing to do with the filter
    printf("the start is averaged, not the first fix\n");
    elev_init();
    for (int i = 0; i < (int)ELEV_SEED_FIXES / 2; i++) {
        hold(SEA, 1, 50.0f, 1);
        hold(SEA, 1, 150.0f, 1);
    }
    elev_get(&e);
    check("50 and 150 in turn start it at 100", near(e.altitude_m, 100.0f, 0.01f));
    check("and the start is kept", near(e.seed_m, 100.0f, 0.01f));

    // climbing 100 m while it averages, the GPS right all the way: the start
    // must be where the car is now, 100 m, not the middle of the climb
    printf("climbing while it averages\n");
    elev_init();
    hold(SEA, 1, 0.0f, (int)ELEV_SEED_FIXES / 2);
    hold(UP100, 1, 100.0f, (int)ELEV_SEED_FIXES / 2);
    elev_get(&e);
    check("it starts where the car is now", near(e.altitude_m, 100.0f, 1.0f));

    // climb needs no GPS at all - the pressure fell by 100 m worth
    printf("climb counts before any fix\n");
    elev_init();
    hold(SEA, 0, 0.0f, 1);
    hold(UP100, 0, 0.0f, 1);
    elev_get(&e);
    check("counting from the first pressure sample", e.counting);
    check("but no altitude yet", !e.valid);
    check("100 m of climb, less the deadband, without a fix",
          near(e.gain_m, 97.0f, 1.5f));

    // sessions 6 and 7: the board never moved and the pressure never changed,
    // while the GPS height walked from -280 m up to 65 m over 5 minutes after a
    // cold start. still air means no climb, whatever the GPS says
    printf("a still board on a walking GPS\n");
    elev_init();
    elev_get(&e);
    check("not settled before anything arrived", !e.settled);
    for (int i = 0; i < 100; i++) {
        float gps_alt = -280.0f + (345.0f * (float)i / 99.0f);
        hold(SEA, 1, gps_alt, 1);
        if (i == 60) {
            elev_get(&e);
            check("not settled while the GPS is still walking", e.valid && !e.settled);
        }
    }
    hold(SEA, 1, 65.0f, 200);       // 10 minutes of a steady fix
    elev_get(&e);
    check("no climb on a still board", near(e.gain_m, 0.0f, 0.01f));
    check("no descent on a still board", near(e.loss_m, 0.0f, 0.01f));
    check("ends on the steady GPS height", near(e.altitude_m, 65.0f, 1.0f));
    check("settled once the GPS stopped walking", e.settled);
    check("the start shows where it began", e.seed_m < -200.0f);

    printf("bad inputs are ignored\n");
    elev_init();
    hold(SEA, 1, 100.0f, (int)ELEV_SEED_FIXES);
    elev_update(0.0f, true, 900.0f, STEP_S);     // no pressure
    elev_update(SEA, true, 900.0f, 0.0f);        // no time passed
    elev_get(&e);
    check("nothing moved", near(e.altitude_m, 100.0f, 0.01f));

    printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
    return fails != 0;
}
