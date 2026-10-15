//
// screen module - public interface
//
// - screen 1, the main drive view: a round G-meter in the middle, speed and
//   distance down the left, air down the right, the wall clock and the time
//   since power-up across the top, and the GPS across the bottom; landscape,
//   320 x 240
// - takes plain numbers in, works out what changed, and tells gfx which
//   rectangles to repaint; gfx then calls screen_paint for each band
// - no HAL and no sensor types in here, so the whole layout renders on the
//   mac - tests/render_screen.c writes it out as an image
// - the dot shows what the driver feels, not what the tyres push: braking
//   throws you forward, so the dot goes up, towards the front of the car; a
//   left bend throws you right, so the dot goes right. trip.c stores the push,
//   which is the opposite sign - the flip happens in here and nowhere else
//

#ifndef SCREEN_H
#define SCREEN_H

#include <stdbool.h>
#include <stdint.h>
#include "gfx.h"

typedef struct {
    // the gauge - car axes, gravity already taken out, in g
    float long_g;           // + speeding up, - braking
    float lat_g;            // + the tyres pushing the car left, a left bend
    float peak_accel_g;     // hardest forward push this session, positive
    float peak_brake_g;     // hardest braking, as a positive number
    float peak_left_g;      // hardest left bend, positive
    float peak_right_g;     // hardest right bend, positive

    // left column
    float    speed_kmh;
    float    speed_max_kmh;
    float    distance_km;
    bool     coasting;      // speed is being guessed, no GPS

    // right column
    bool  env_valid;
    float temp_c;
    float hum_pct;
    float press_hpa;
    bool  gas_valid;
    float gas_kohm;

    // top strip
    uint32_t duration_s;    // since power-up
    bool     sd_ok;

    // bottom strip
    bool    fix_valid;
    bool    time_valid;
    uint8_t sats;
    double  lat_deg;
    double  lon_deg;
    float   alt_m;          // the baro-fused altitude, not the raw fix
    bool    alt_settled;    // false while it still walks off a cold start - drawn grey
    uint8_t hour, minute, second;   // UTC; the local offset is applied in here
} screen_data_t;

// wipe the shown state; the next flush paints everything
void screen_init(void);

// the boot screen: black with the build on it, up for a couple of seconds
// while the backlight fades in
void screen_show_boot(void);

// switch to the drive view; everything gets repainted
void screen_show_main(void);

// hand over fresh numbers; only what changed gets marked for repaint
void screen_update(const screen_data_t *d);

// gfx calls this for every band; draws whatever the current view is
void screen_paint(const gfx_rect_t *band);

#endif // SCREEN_H
