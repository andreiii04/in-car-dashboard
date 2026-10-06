//
// attitude module - implementation
//
// - four things happen to every sample, in this order:
//     1. rotate it, so the fixed tilt of the mount is taken out
//     2. take off whatever the car's own pushing adds, if anyone told us
//     3. work out pitch and roll from gravity, if the accel can be trusted
//     4. blend that with the gyro spun forward from the last angle
// - step 4 is the complementary filter; the gyro is right over short times and
//   the accel is right over long ones, so each one is used where it is good
//

#include "attitude.h"
#include <math.h>
#include <stddef.h>

// how the two sensors are weighed against each other, in seconds - below this
// time the answer comes from the gyro, above it from the accelerometer
//
// how to change it later, once there is hardware:
//  - too small and the gyro's leftover zero offset shows up as a standing
//    error; the error in degrees is roughly (offset in dps) x tau
//  - too big and the bumps the gate below does not catch pull the angle away
//    for longer before the accel drags it back
//  - 2 s is the starting point, not a measured value; the way to settle it is
//    to log raw accel, raw gyro and this output to the SD card on one drive,
//    then replay that log on the PC with different values here and keep the
//    one that tracks a known manoeuvre best
// see docs/notes.md for the full working
#define ATTITUDE_TAU_S 2.0f

// how far off 1 g the accelerometer may read and still be believed; outside
// this the car is braking, cornering or on a bump, and what it measures is not
// gravity any more, so that sample's accel half is skipped
#define ATTITUDE_GATE_G 0.10f

// a mount vector shorter than this points nowhere useful
#define ATTITUDE_MIN_NORM_G 0.10f

#define ATTITUDE_RAD_TO_DEG 57.29577951f
#define ATTITUDE_DEG_TO_RAD 0.017453293f

// tan() runs away to infinity at 90 degrees, and the roll rate below uses
// one; 80 degrees of nose-up is already a rollover, so cap it there and the
// maths can never produce a silly number
#define ATTITUDE_TAN_LIMIT_DEG 80.0f

// rotation that turns the board's own axes into the car's; identity means the
// board is already level, which is what init leaves behind
static float mount_r[3][3];

static float gyro_bias_dps[3];

// the car's own acceleration, forward and sideways, as told to us from
// outside; subtracted from the reading so what is left is closer to gravity
static float linear_accel_g[2];

static float pitch_deg;
static float roll_deg;
static bool  accel_used;
static bool  settled;

static void attitude_identity(void) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mount_r[i][j] = (i == j) ? 1.0f : 0.0f;
        }
    }
}

// keep an angle inside ±180, so a wrap at the far end does not turn a tiny
// difference into a 350 degree one
static float attitude_wrap180(float deg) {
    while (deg > 180.0f) {
        deg -= 360.0f;
    }
    while (deg < -180.0f) {
        deg += 360.0f;
    }
    return deg;
}

// multiply a vector by the mount rotation; out and in must not be the same
static void attitude_rotate(const float in[3], float out[3]) {
    for (int i = 0; i < 3; i++) {
        out[i] = (mount_r[i][0] * in[0]) +
                 (mount_r[i][1] * in[1]) +
                 (mount_r[i][2] * in[2]);
    }
}

void attitude_init(void) {
    attitude_identity();
    for (int i = 0; i < 3; i++) {
        gyro_bias_dps[i] = 0.0f;
    }
    linear_accel_g[0] = 0.0f;
    linear_accel_g[1] = 0.0f;
    pitch_deg = 0.0f;
    roll_deg = 0.0f;
    accel_used = false;
    settled = false;
}

bool attitude_set_mount(float ax_g, float ay_g, float az_g) {
    float norm = sqrtf((ax_g * ax_g) + (ay_g * ay_g) + (az_g * az_g));

    if (norm < ATTITUDE_MIN_NORM_G) {
        return false;
    }

    // unit vector pointing the way the board thinks "up" is
    float gx = ax_g / norm;
    float gy = ay_g / norm;
    float gz = az_g / norm;

    // smallest rotation that swings that vector onto the car's up axis, which
    // is (0,0,1); this is Rodrigues' formula written out
    //   v is the axis to turn about, c is how much the two already agree
    float v1 = gy;
    float v2 = -gx;
    float v3 = 0.0f;
    float c = gz;
    float s2 = (v1 * v1) + (v2 * v2) + (v3 * v3);

    if (s2 < 1e-12f) {
        // the two vectors are already on the same line - either nothing to do,
        // or the board is mounted upside down and needs a half turn
        attitude_identity();
        if (c < 0.0f) {
            mount_r[1][1] = -1.0f;
            mount_r[2][2] = -1.0f;
        }
        return true;
    }

    float k = (1.0f - c) / s2;

    mount_r[0][0] = c + (k * v1 * v1);
    mount_r[0][1] = -v3 + (k * v1 * v2);
    mount_r[0][2] = v2 + (k * v1 * v3);
    mount_r[1][0] = v3 + (k * v2 * v1);
    mount_r[1][1] = c + (k * v2 * v2);
    mount_r[1][2] = -v1 + (k * v2 * v3);
    mount_r[2][0] = -v2 + (k * v3 * v1);
    mount_r[2][1] = v1 + (k * v3 * v2);
    mount_r[2][2] = c + (k * v3 * v3);
    return true;
}

void attitude_set_gyro_bias(float gx_dps, float gy_dps, float gz_dps) {
    gyro_bias_dps[0] = gx_dps;
    gyro_bias_dps[1] = gy_dps;
    gyro_bias_dps[2] = gz_dps;
}

void attitude_set_linear_accel(float ax_g, float ay_g) {
    linear_accel_g[0] = ax_g;
    linear_accel_g[1] = ay_g;
}

void attitude_update(float ax_g, float ay_g, float az_g,
                     float gx_dps, float gy_dps, float gz_dps,
                     float dt_s) {
    if (dt_s <= 0.0f) {
        return;
    }

    // the bias belongs to the sensor's own axes, so take it off before turning
    // the reading into car axes
    const float accel_in[3] = { ax_g, ay_g, az_g };
    const float gyro_in[3] = {
        gx_dps - gyro_bias_dps[0],
        gy_dps - gyro_bias_dps[1],
        gz_dps - gyro_bias_dps[2],
    };
    float accel[3];
    float gyro[3];

    attitude_rotate(accel_in, accel);
    attitude_rotate(gyro_in, gyro);

    // take the car's own pushing back out; if nobody set it this is zero and
    // the line does nothing. it goes here, after the rotation, because the two
    // numbers are given in car axes
    accel[0] -= linear_accel_g[0];
    accel[1] -= linear_accel_g[1];

    // whatever is left should be gravity alone, so its length should be 1 g;
    // anything else means the car is still pushing in a way we did not account
    // for - braking harder than the GPS noticed, a pothole, a rough track
    float norm = sqrtf((accel[0] * accel[0]) +
                       (accel[1] * accel[1]) +
                       (accel[2] * accel[2]));
    accel_used = (norm > (1.0f - ATTITUDE_GATE_G)) &&
                 (norm < (1.0f + ATTITUDE_GATE_G));

    // gravity tilts the same way the car does, so the angles fall out of the
    // three accel numbers with two arctangents
    float pitch_acc = 0.0f;
    float roll_acc = 0.0f;
    if (accel_used) {
        // an accelerometer at rest reads +1 g on whichever axis points up, so
        // what it gives back is the "up" direction seen from the car. nose up
        // tilts the forward axis towards up, so X goes positive - no minus
        // sign here, and putting one in makes the accel half and the gyro half
        // pull against each other
        pitch_acc = atan2f(accel[0],
                           sqrtf((accel[1] * accel[1]) +
                                 (accel[2] * accel[2]))) * ATTITUDE_RAD_TO_DEG;
        roll_acc = atan2f(accel[1], accel[2]) * ATTITUDE_RAD_TO_DEG;
    }

    // nothing to add the gyro onto until the first trustworthy accel sample
    // has set a starting point
    if (!settled) {
        if (accel_used) {
            pitch_deg = pitch_acc;
            roll_deg = roll_acc;
            settled = true;
        }
        return;
    }

    // the gyro measures turning about the car's own axes, which is not the
    // same thing as pitch and roll changing - once the car is properly tilted
    // its axes are tilted too, and the two mix into each other
    //
    // at small angles this all collapses to pitch_rate = -gyro_y and
    // roll_rate = gyro_x, and that shortcut is what most code uses; it is
    // fine on tarmac (1.5% out at 10 degrees) and wrong off-road (13% out at
    // 30 degrees, and it stays wrong for as long as the tilt lasts)
    //
    // nose up being a negative turn about the left-pointing Y axis is where
    // the minus signs come from; getting one wrong does not break anything
    // loudly, the filter just fights itself and looks slow
    float roll_rad = roll_deg * ATTITUDE_DEG_TO_RAD;
    float sin_roll = sinf(roll_rad);
    float cos_roll = cosf(roll_rad);

    float pitch_clamped = pitch_deg;
    if (pitch_clamped > ATTITUDE_TAN_LIMIT_DEG) {
        pitch_clamped = ATTITUDE_TAN_LIMIT_DEG;
    }
    if (pitch_clamped < -ATTITUDE_TAN_LIMIT_DEG) {
        pitch_clamped = -ATTITUDE_TAN_LIMIT_DEG;
    }
    float tan_pitch = tanf(pitch_clamped * ATTITUDE_DEG_TO_RAD);

    float pitch_rate = (-cos_roll * gyro[1]) + (sin_roll * gyro[2]);
    float roll_rate = gyro[0] -
                      (tan_pitch * ((sin_roll * gyro[1]) + (cos_roll * gyro[2])));

    // where the gyro alone says the car is now
    float pitch_gyro = pitch_deg + (pitch_rate * dt_s);
    float roll_gyro = roll_deg + (roll_rate * dt_s);

    if (!accel_used) {
        pitch_deg = pitch_gyro;
        roll_deg = attitude_wrap180(roll_gyro);
        return;
    }

    // the blend; alpha is how much of the gyro answer is kept, and writing it
    // as a step towards the accel answer keeps roll safe across the ±180 edge
    float alpha = ATTITUDE_TAU_S / (ATTITUDE_TAU_S + dt_s);

    pitch_deg = pitch_gyro + ((1.0f - alpha) * (pitch_acc - pitch_gyro));
    roll_deg = attitude_wrap180(
        roll_gyro + ((1.0f - alpha) * attitude_wrap180(roll_acc - roll_gyro)));
}

void attitude_get(attitude_t *out) {
    if (out == NULL) {
        return;
    }
    out->pitch_deg = pitch_deg;
    out->roll_deg = roll_deg;
    out->accel_used = accel_used;
    out->settled = settled;
}
