//
// host preview of the display - renders screen 1 and the boot screen into
// image files, no board needed
//
// - gfx.c and screen.c have no HAL in them, so the same code that will drive
//   the panel runs here; the only thing swapped is where the bands go: into a
//   framebuffer in RAM instead of over SPI
// - writes PPM files, the simplest image format there is; convert with
//   `magick screen_main.ppm screen_main.png` to look at them
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/render_screen.c App/Src/gfx.c
//  App/Src/screen.c App/Fonts/font_small.c App/Fonts/font_medium.c
//  App/Fonts/font_large.c -lm -o /tmp/render_screen && /tmp/render_screen /tmp

#include <stdio.h>
#include <string.h>
#include "gfx.h"
#include "screen.h"

static uint16_t fb[GFX_H][GFX_W];   // native RGB565

static void out(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *px) {
    for (int16_t r = 0; r < h; r++) {
        for (int16_t c = 0; c < w; c++) {
            fb[y + r][x + c] = gfx_swap(px[(size_t)r * (size_t)w + (size_t)c]);
        }
    }
}

static void write_ppm(const char *dir, const char *name) {
    char path[512];
    snprintf(path, sizeof(path), "%s/%s.ppm", dir, name);
    FILE *f = fopen(path, "wb");
    if (f == NULL) {
        printf("cannot write %s\n", path);
        return;
    }
    fprintf(f, "P6\n%d %d\n255\n", GFX_W, GFX_H);
    for (int y = 0; y < GFX_H; y++) {
        for (int x = 0; x < GFX_W; x++) {
            uint16_t c = fb[y][x];
            uint8_t rgb[3];
            rgb[0] = (uint8_t)(((c >> 11) & 0x1Fu) * 255u / 31u);
            rgb[1] = (uint8_t)(((c >> 5) & 0x3Fu) * 255u / 63u);
            rgb[2] = (uint8_t)((c & 0x1Fu) * 255u / 31u);
            fwrite(rgb, 1, 3, f);
        }
    }
    fclose(f);
    printf("wrote %s\n", path);
}

static uint32_t flush_all(void) {
    uint32_t before = gfx_bands_sent();
    while (gfx_flush_band()) {
    }
    return gfx_bands_sent() - before;
}

int main(int argc, char **argv) {
    const char *dir = (argc > 1) ? argv[1] : ".";

    gfx_init(screen_paint, out);
    screen_init();

    screen_show_boot();
    printf("boot screen: %lu bands\n", (unsigned long)flush_all());
    write_ppm(dir, "screen_boot");

    screen_show_main();
    screen_data_t d;
    memset(&d, 0, sizeof(d));
    d.long_g = -0.35f;      // braking a little
    d.lat_g = 0.42f;        // in a left bend
    d.peak_accel_g = 0.7f;
    d.peak_brake_g = 1.1f;
    d.peak_left_g = 0.8f;
    d.peak_right_g = 0.6f;
    d.speed_kmh = 87.0f;
    d.speed_max_kmh = 132.0f;
    d.distance_km = 45.7f;
    d.env_valid = true;
    d.temp_c = 24.6f;
    d.hum_pct = 43.0f;
    d.press_hpa = 1006.0f;
    d.gas_valid = true;
    d.gas_kohm = 51.0f;
    d.duration_s = 3737u;
    d.sd_ok = true;
    d.fix_valid = true;
    d.time_valid = true;
    d.sats = 9;
    d.lat_deg = 44.42681;
    d.lon_deg = 26.10253;
    d.alt_m = 87.0f;
    d.alt_settled = true;
    d.hour = 20;
    d.minute = 42;
    d.second = 17;
    screen_update(&d);
    printf("main, first paint: %lu bands\n", (unsigned long)flush_all());
    write_ppm(dir, "screen_main");

    // one tick later: the dot moved, one number changed
    d.long_g = -0.6f;
    d.lat_g = 0.1f;
    d.second = 18;
    screen_update(&d);
    printf("main, dot moved + clock tick: %lu bands\n", (unsigned long)flush_all());
    write_ppm(dir, "screen_main2");

    // nothing changed: nothing should go out
    screen_update(&d);
    printf("main, no change: %lu bands\n", (unsigned long)flush_all());

    // the drive view with nothing known yet, before any sensor or the card has
    // said anything - not the same thing as the boot screen above
    screen_show_main();
    memset(&d, 0, sizeof(d));
    screen_update(&d);
    flush_all();
    write_ppm(dir, "screen_empty");
    return 0;
}
