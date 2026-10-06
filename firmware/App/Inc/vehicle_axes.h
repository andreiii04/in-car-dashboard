//
// vehicle axes and vehicle specs - one place for everything that changes when
// the board moves to a different car or a different mount
//
// three separate things live here, and they get measured at different times:
//   1. the axis map    - which sensor axis points which way in the car;
//                        decided by how the case is built, known before fitting
//   2. the calibration - the resting gravity reading and the gyro offset;
//                        measured once with the board fitted and the car parked
//   3. the car specs   - wheelbase, track, centre of gravity height and so on;
//                        looked up, not measured
//
// nothing in here is HAL or vendor code, so files that include it stay
// host-testable; docs/vehicle_info.md explains every number and where it is
// used, in plain language
//

#ifndef VEHICLE_AXES_H
#define VEHICLE_AXES_H

#include <math.h>

// ---------------------------------------------------------------------------
// 1. axis map
// ---------------------------------------------------------------------------
//
// the car frame is ISO 8855, the one the car industry uses:
//   X forward, Y left, Z up
//
// the LSM6DSO's own X/Y/Z are whatever the board's layout and the case make
// them. this map is the swap-and-flip between the two - no maths, just picking
// which sensor number goes where and whether it needs a minus.
//
// measured on the bench, board flat and warm, step 2. the LSM6DSO is on the
// underside of the board, which is a 180 degree turn about the fore-aft axis -
// so X survives and Y and Z both need a minus. front edge is the one opposite
// the USB-C connector, and that direction is sensor +X
//
// determinant is (+1)(-1)(-1) = +1, so this is a real rotation and not a
// mirror; the frame stays right-handed and X cross Y still gives Z
//
// provisional until the case fixes the real orientation in the car
#define VEHICLE_X_FROM   0
#define VEHICLE_X_SIGN   1.0f
#define VEHICLE_Y_FROM   1
#define VEHICLE_Y_SIGN  (-1.0f)
#define VEHICLE_Z_FROM   2
#define VEHICLE_Z_SIGN  (-1.0f)

// turn one sensor-frame vector into a car-frame one; works for the
// accelerometer and the gyroscope alike, they are both plain vectors
static inline void vehicle_map(const float sensor[3], float out[3]) {
    out[0] = VEHICLE_X_SIGN * sensor[VEHICLE_X_FROM];
    out[1] = VEHICLE_Y_SIGN * sensor[VEHICLE_Y_FROM];
    out[2] = VEHICLE_Z_SIGN * sensor[VEHICLE_Z_FROM];
}

// ---------------------------------------------------------------------------
// 2. calibration - the rest reading is the car set, taken with the board on
//    the dash; the flat set from the bench tray is kept below to paste back.
//    the gyro bias is the bench one and stays
// ---------------------------------------------------------------------------
//
// the accelerometer reading at rest, already run through vehicle_map above.
// it measures the mount's tilt, and attitude_set_mount turns it into the
// rotation that cancels that tilt.
//
// FLAT SET - keep these, paste them back to make the tray-on-a-table read
// flat again after the car calibration has replaced them:
//   board in the printed bottom tray (no screws, USB-C in), tray on the same
//   wooden table as step 2, ~17-19 min warm, 3 runs of 500 samples averaged
//   (agreed to 0.0001 g), 3 Oct 2026; raw tilt pitch +0.84, roll -0.65 deg
//     #define VEHICLE_REST_ACCEL_X  +0.01448f
//     #define VEHICLE_REST_ACCEL_Y  -0.01117f
//     #define VEHICLE_REST_ACCEL_Z  +0.99091f
// the step 2 set (-0.00002, 0.00530, 0.98941) was the bare board resting on
// its solder tails, not parallel to the table - it read +0.9 / -1.3 in the tray
//
// CAR SET - in use. board on the dash, front edge forward, 4 Oct 2026. parked
// on a slight slope, so measured both ways round on the same spot and the two
// directions averaged - the slope flips sign, the mount does not:
//   front up the slope    +0.71401 +0.02540 +0.69531   pitch 45.74 roll 2.09
//   turned round          +0.65235 +0.03786 +0.75008   pitch 40.98 roll 2.89
//   2 + 3 runs of 500, agreed to 0.0004 g; slope came out 2.4 deg pitch, 0.4
//   roll. mount: 43.4 deg nose up, 2.5 deg roll. the axis map above does not
//   change with that tilt; only these three numbers do
#define VEHICLE_REST_ACCEL_X   0.68318f
#define VEHICLE_REST_ACCEL_Y   0.03163f
#define VEHICLE_REST_ACCEL_Z   0.72270f

// what the gyroscope reads while standing perfectly still, in deg/s, again
// after vehicle_map. this one does not care how the board sits, so the bench
// value is the real one - keep it after the car calibration, and if the map
// changes later these three just re-map, no new measurement
//
// same 3 runs as the flat set above, ~17-19 min warm, board at ~28 C; they
// agreed to 0.004 dps. the step 2 values (X 0.6096, Y -0.4463, Z 0.2017, 21
// min soak) left X 0.16 dps high and Y 0.06 low - at tau 2 s that was the
// -0.33 deg of roll and +0.12 of pitch left over in the tray. X moves with
// temperature (+0.42 cold to +0.61 warm in step 2), so it still carries
// ~0.2 dps of doubt, about 0.4 deg of standing error
#define VEHICLE_GYRO_BIAS_X    0.4460f
#define VEHICLE_GYRO_BIAS_Y   -0.5069f
#define VEHICLE_GYRO_BIAS_Z    0.1804f

// ---------------------------------------------------------------------------
// 3. car specs - an Audi A4 B6 Avant 1.8T 190hp quattro, 2004, 6-speed
//    manual. everything except the two centre-of-gravity numbers comes off
//    the car's own data sheet. the estimates and the worked limits are in
//    docs/vehicle_info.md
// ---------------------------------------------------------------------------
//
// used in the maths:
#define VEHICLE_TRACK_M          1.527f  // 1528 mm front, 1526 mm rear - averaged
#define VEHICLE_COG_HEIGHT_M     0.53f   // estimate, typical for a low passenger car
#define VEHICLE_WHEELBASE_M      2.65f   // 2650 mm
#define VEHICLE_COG_TO_REAR_M    1.54f   // estimate, 0.58 x wheelbase from the rear
#define VEHICLE_TURN_RADIUS_M    5.55f   // 11.1 m turning circle, so half of it

// written into the log for context, not used in any calculation:
#define VEHICLE_MASS_KG          1650.0f // kerb is 1530; this is with driver + passenger
#define VEHICLE_LENGTH_M         4.548f
#define VEHICLE_WIDTH_M          1.772f
#define VEHICLE_HEIGHT_M         1.428f

// ---------------------------------------------------------------------------
// limits worked out from the specs above
// ---------------------------------------------------------------------------

// how much sideways g it takes to lift the inside wheels; this is the static
// stability factor, track over twice the centre of gravity height. a normal
// car lands near 1.4, a tall SUV near 1.1 - lower means it tips sooner
#define VEHICLE_ROLLOVER_LIMIT_G  (VEHICLE_TRACK_M / (2.0f * VEHICLE_COG_HEIGHT_M))

#define VEHICLE_RAD_TO_DEG 57.29577951f

// the side slope the car would tip over on, standing still. same geometry as
// above, written as an angle instead of a g figure
static inline float vehicle_critical_roll_deg(void) {
    return atanf(VEHICLE_ROLLOVER_LIMIT_G) * VEHICLE_RAD_TO_DEG;
}

// the climb it would tip backwards on, standing still - the centre of gravity
// passes over the rear axle
static inline float vehicle_critical_pitch_deg(void) {
    return atanf(VEHICLE_COG_TO_REAR_M / VEHICLE_COG_HEIGHT_M) * VEHICLE_RAD_TO_DEG;
}

// the fastest the car can possibly be turning at this speed; anything past
// this is a sensor glitch, not a manoeuvre
//
// two separate limits, and the tighter one wins:
//   slow  - the steering only goes so far, so the circle cannot get tighter
//           than the handbook's turning radius
//   fast  - turning harder than the tipping limit would put it on its roof,
//           so it physically cannot be happening
static inline float vehicle_max_yaw_rate_dps(float speed_kmh) {
    float speed_mps = speed_kmh / 3.6f;
    if (speed_mps < 0.1f) {
        return 0.0f;
    }
    float steering_limit = speed_mps / VEHICLE_TURN_RADIUS_M;
    float tipping_limit = (VEHICLE_ROLLOVER_LIMIT_G * 9.80665f) / speed_mps;
    float limit = (steering_limit < tipping_limit) ? steering_limit : tipping_limit;
    return limit * VEHICLE_RAD_TO_DEG;
}

// ---- local time ----------------------------------------------------------------
//
// the GPS gives UTC and the firmware keeps UTC everywhere; this offset is
// applied only where a human reads the time - the clock on the panel and the
// start_local field in the session record
//
// Romania is UTC+2 in winter and UTC+3 in summer, so this gets edited twice a
// year. no DST rule in firmware on purpose - it would be a calendar to
// maintain for a number i can change in one line
#define VEHICLE_UTC_OFFSET_MIN 180   // +3 h, summer time

#endif // VEHICLE_AXES_H
