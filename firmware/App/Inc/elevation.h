//
// elevation module - public interface
//
// - two ways to know how high the car is, both flawed, so they get combined
//   the same way pitch and roll do:
//     GPS altitude   right on average, but jumps around by tens of metres
//     air pressure   very smooth, but slides with the weather over hours
// - so the GPS supplies the truth and the barometer supplies the detail; that
//   is the same complementary filter as attitude.c, only much slower
// - climb and descent are added up from the barometer alone. the combined
//   figure walks for minutes after a cold start while the GPS height settles,
//   and every metre of that walk counted as climb - 170 m and 234 m on a still
//   board in sessions 6 and 7. a weather front is ~8 m an hour, far less
// - no HAL and no hardware, runs on the mac
//

#ifndef ELEVATION_H
#define ELEVATION_H

#include <stdbool.h>

// how many fixes the starting height is averaged over; one per pressure
// sample, so about 30 s
#define ELEV_SEED_FIXES 10u

typedef struct {
    float altitude_m;   // the combined figure, height above sea level
    float seed_m;       // the starting height the GPS gave, averaged
    float gain_m;       // everything climbed this session, added up
    float loss_m;       // everything descended, as a positive number
    bool  valid;        // false until ELEV_SEED_FIXES GPS altitudes arrived
    bool  settled;      // the GPS and the combined figure have agreed for a
                        // while; false while it still walks off a bad start
    bool  counting;     // gain and loss mean something - from the first
                        // pressure sample, no GPS needed
} elevation_t;

// clear the session; nothing is known until the GPS altitudes turn up
void elev_init(void);

// one step; pressure in hPa straight from the BME680, the newest GPS altitude
// with a flag saying whether it is usable, and the time since the last call
//
// call it whenever a fresh pressure sample exists, which is roughly every 3
// seconds - the GPS is faster than that, so it just hands over its latest
void elev_update(float pressure_hpa, bool gps_valid, float gps_altitude_m,
                 float dt_s);

// copy out the current numbers
void elev_get(elevation_t *out);

#endif // ELEVATION_H
