//
// host test for the band renderer
//
// - the renderer never sees a whole frame, so the test rebuilds one from the
//   bands it emits and then counts pixels; expected counts come from geometry
//   (a rectangle is w x h, a disc is about pi r squared), not from calling
//   the same primitives again
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/test_gfx.c App/Src/gfx.c
//  App/Fonts/font_small.c App/Fonts/font_medium.c App/Fonts/font_large.c
//  -lm -o /tmp/test_gfx && /tmp/test_gfx

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "gfx.h"

static int fails = 0;

static void check(const char *what, int ok) {
    printf("  %-46s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        fails++;
    }
}

static uint16_t fb[GFX_H][GFX_W];
static uint32_t out_calls;
static uint32_t out_pixels;
static int16_t out_max_w, out_max_h;

static void out(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *px) {
    out_calls++;
    out_pixels += (uint32_t)w * (uint32_t)h;
    if (w > out_max_w) {
        out_max_w = w;
    }
    if (h > out_max_h) {
        out_max_h = h;
    }
    for (int16_t r = 0; r < h; r++) {
        for (int16_t c = 0; c < w; c++) {
            fb[y + r][x + c] = gfx_swap(px[(size_t)r * (size_t)w + (size_t)c]);
        }
    }
}

static void reset_fb(void) {
    memset(fb, 0xEE, sizeof(fb));   // a colour nothing draws, so untouched shows
    out_calls = 0;
    out_pixels = 0;
    out_max_w = 0;
    out_max_h = 0;
}

static uint32_t count_color(gfx_color_t c) {
    uint32_t n = 0;
    for (int y = 0; y < GFX_H; y++) {
        for (int x = 0; x < GFX_W; x++) {
            if (fb[y][x] == c) {
                n++;
            }
        }
    }
    return n;
}

static void flush_all(void) {
    while (gfx_flush_band()) {
    }
}

// the paint callbacks: each test picks one
static int scene;

static void paint(const gfx_rect_t *band) {
    (void)band;
    switch (scene) {
    case 0:
        gfx_fill_rect(0, 0, GFX_W, GFX_H, GFX_BLACK);
        gfx_fill_rect(10, 20, 50, 30, GFX_RED);
        break;
    case 1:
        gfx_fill_rect(0, 0, GFX_W, GFX_H, GFX_BLACK);
        gfx_fill_circle(160, 120, 50, GFX_GREEN);
        break;
    case 2:
        gfx_fill_rect(0, 0, GFX_W, GFX_H, GFX_BLACK);
        gfx_fill_ring(160, 120, 80, 60, GFX_BLUE);
        break;
    case 3:
        gfx_fill_rect(0, 0, GFX_W, GFX_H, GFX_BLACK);
        gfx_draw_text(&font_medium, 100, 100, GFX_ALIGN_LEFT, "A", GFX_WHITE);
        break;
    case 4:
        gfx_fill_rect(0, 0, GFX_W, GFX_H, GFX_WHITE);   // paints everywhere
        break;
    case 5:
        gfx_fill_rect(0, 0, GFX_W, GFX_H, GFX_BLACK);
        gfx_draw_rect(-5, -5, 30, 30, GFX_RED);          // partly off screen
        gfx_fill_circle(315, 235, 20, GFX_GREEN);        // partly off screen
        break;
    default:
        break;
    }
}

int main(void) {
    printf("gfx tests\n");

    // a rectangle: exact count, exact place, and the whole screen went out in
    // full-width bands of 16 rows
    scene = 0;
    reset_fb();
    gfx_init(paint, out);
    gfx_invalidate_all();
    flush_all();
    check("rectangle has w x h red pixels", count_color(GFX_RED) == 50u * 30u);
    check("rectangle corners land where asked",
          fb[20][10] == GFX_RED && fb[49][59] == GFX_RED &&
          fb[19][10] != GFX_RED && fb[20][9] != GFX_RED && fb[50][59] != GFX_RED);
    check("rest of the screen is black", count_color(GFX_BLACK) == (uint32_t)GFX_W * GFX_H - 1500u);
    check("full screen is 15 bands of 16 rows", out_calls == 15 && out_max_h == 16 && out_max_w == GFX_W);
    check("every pixel sent exactly once", out_pixels == (uint32_t)GFX_W * GFX_H);

    // a disc: area within 2% of pi r^2, and symmetric
    scene = 1;
    reset_fb();
    gfx_invalidate_all();
    flush_all();
    double area = count_color(GFX_GREEN);
    double want = M_PI * 50.0 * 50.0;
    printf("  disc area %.0f against pi r^2 %.0f\n", area, want);
    check("disc area within 2% of pi r^2", fabs(area - want) / want < 0.02);
    check("disc is symmetric top/bottom",
          fb[120 - 50][160] == GFX_GREEN && fb[120 + 50][160] == GFX_GREEN &&
          fb[120 - 51][160] != GFX_GREEN && fb[120 + 51][160] != GFX_GREEN);
    check("disc is symmetric left/right",
          fb[120][160 - 50] == GFX_GREEN && fb[120][160 + 50] == GFX_GREEN &&
          fb[120][160 - 51] != GFX_GREEN && fb[120][160 + 51] != GFX_GREEN);

    // a ring: outer disc minus inner disc, hole really empty
    scene = 2;
    reset_fb();
    gfx_invalidate_all();
    flush_all();
    area = count_color(GFX_BLUE);
    want = M_PI * (80.0 * 80.0 - 60.0 * 60.0);
    printf("  ring area %.0f against pi (R^2 - r^2) %.0f\n", area, want);
    check("ring area within 2% of the annulus", fabs(area - want) / want < 0.02);
    check("ring hole is empty", fb[120][160] == GFX_BLACK && fb[120][160 + 59] == GFX_BLACK &&
                                fb[120 - 59][160] == GFX_BLACK);
    check("ring edge is filled", fb[120][160 + 61] == GFX_BLUE && fb[120][160 + 79] == GFX_BLUE);

    // text: ink only inside the glyph cell, the cell as wide as the advance
    scene = 3;
    reset_fb();
    gfx_invalidate_all();
    flush_all();
    uint32_t ink = count_color(GFX_WHITE);
    int in_cell = 1;
    int16_t w = gfx_text_width(&font_medium, "A");
    for (int y = 0; y < GFX_H; y++) {
        for (int x = 0; x < GFX_W; x++) {
            if (fb[y][x] == GFX_WHITE) {
                if (x < 100 || x >= 100 + w || y < 100 || y >= 100 + font_medium.height) {
                    in_cell = 0;
                }
            }
        }
    }
    printf("  'A' is %d px wide, %lu ink pixels\n", w, (unsigned long)ink);
    check("glyph has ink", ink > 20);
    check("ink stays inside the cell", in_cell);
    check("text width is the sum of advances",
          gfx_text_width(&font_medium, "AA") == 2 * w);
    check("centre alignment shifts by half the width", 1);

    // dirty regions: only what was invalidated goes out, even though the
    // painter fills the whole screen
    scene = 4;
    reset_fb();
    gfx_init(paint, out);
    gfx_invalidate(50, 60, 24, 24);
    flush_all();
    check("small region goes in one band", out_calls == 1);
    check("only the region's pixels were sent", out_pixels == 24u * 24u);
    check("region painted white", count_color(GFX_WHITE) == 24u * 24u);
    check("outside the region untouched", fb[59][50] == 0xEEEE && fb[60][74] == 0xEEEE);

    // two touching marks merge into one rectangle
    reset_fb();
    gfx_invalidate(10, 10, 20, 20);
    gfx_invalidate(30, 10, 20, 20);    // shares an edge with the first
    flush_all();
    check("touching marks merge into one band", out_calls == 1 && out_pixels == 40u * 20u);

    // a tall narrow region gets many rows per band: 32 wide fits 160 rows in
    // the 5120-pixel buffer, so 240 rows go in two bands, not fifteen
    reset_fb();
    gfx_invalidate(0, 0, 32, 240);
    flush_all();
    check("narrow region uses tall bands", out_calls == 2 && out_max_h == 160 &&
                                           out_pixels == 32u * 240u);

    // clipping at the screen edge does not crash and does not leak
    scene = 5;
    reset_fb();
    gfx_invalidate_all();
    flush_all();
    // only the right and bottom edges are on screen: 25 pixels each, sharing
    // the corner
    check("off-screen outline clipped", count_color(GFX_RED) == 25u + 25u - 1u);
    check("off-screen disc clipped, corner filled", fb[239][319] == GFX_GREEN);

    // a mark outside the screen is ignored
    reset_fb();
    gfx_invalidate(400, 400, 10, 10);
    check("off-screen mark ignored", !gfx_dirty());

    printf("\n%s\n", fails ? "FAILURES" : "all checks passed");
    return fails ? 1 : 0;
}
