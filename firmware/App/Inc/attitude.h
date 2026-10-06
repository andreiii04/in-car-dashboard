//
// attitude module - public interface
//
// - attitude is the word for which way a body is pointing; here it is only
//   pitch and roll, the two angles the car can be tilted at
// - neither sensor can give them alone - the accelerometer is fooled by the
//   car's own braking and cornering, the gyroscope drifts away over time;
//   this fuses the two, called a complementary filter
// - no HAL and no vendor types in here, only plain floats, so this compiles
//   and runs on a PC the same way nmea.c does - keep it that way
// - everything coming in is already in vehicle axes: X forward, Y left, Z up;
//   turning the LSM6DSO's own axes into those is the caller's job, because it
//   depends on how the board ends up mounted
//

#ifndef ATTITUDE_H
#define ATTITUDE_H

#include <stdbool.h>

typedef struct {
    float pitch_deg;   // + is nose up,          range ±90
    float roll_deg;    // + is right side down,  range ±180
    bool  accel_used;  // false when the last step ran on the gyro alone
    bool  settled;     // false until the first usable accelerometer sample
} attitude_t;

// clear the angles; mount rotation goes back to "board sits level" and the
// gyro bias back to zero, so call the two setters below after this
void attitude_init(void);

// teach it how the board sits in the car - pass one accelerometer reading
// taken with the car parked on flat ground and nothing moving; every later
// sample gets rotated by this, so the maths always sees a level board
// returns false if the vector is too short to point anywhere useful
bool attitude_set_mount(float ax_g, float ay_g, float az_g);

// constant gyro offset to take off every reading, deg/s, measured by averaging
// a few hundred samples while standing still
void attitude_set_gyro_bias(float gx_dps, float gy_dps, float gz_dps);

// tell the filter how much of what the accelerometer feels is the car pushing
// rather than gravity, in g, car axes - forward and sideways only, because
// those are the two we can work out from other sensors
//
// it is a separate call and not extra arguments on attitude_update because the
// two numbers turn up at completely different rates: the filter runs at the
// IMU's 104 Hz, while this comes off the GPS speed and the yaw rate, once a
// second. different rates means different places in the loop
//
// whatever was last set keeps being used; it starts at zero, which is the same
// as no compensation at all
void attitude_set_linear_accel(float ax_g, float ay_g);

// one filter step; accel in g, gyro in deg/s, dt is the time since the last
// call in seconds
void attitude_update(float ax_g, float ay_g, float az_g,
                     float gx_dps, float gy_dps, float gz_dps,
                     float dt_s);

// copy out the angles as they stand now
void attitude_get(attitude_t *out);

#endif // ATTITUDE_H
