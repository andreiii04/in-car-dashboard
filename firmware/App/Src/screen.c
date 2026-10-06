//
// screen module - implementation
//
// - every piece of text is an element with a fixed box; screen_update formats
//   the new text, compares it with what is on the panel, and invalidates the
//   box only when it differs. the numbers around the edge change once a
//   second, the dot up to twenty times, so a frame is usually two small squares
// - screen_paint draws the whole view every time; gfx clips everything to the
//   band and rejects what falls outside in a few compares, so painting the
//   full scene into a small band costs about nothing
//

#include "screen.h"
#include "vehicle_axes.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

// ---- geometry ----------------------------------------------------------------

#define SCR_CX          160
#define SCR_CY          120
#define SCR_R_RING      84     // outer edge of the grey ring
#define SCR_R_DISC      60     // the black disc the dot lives on
#define SCR_R_NUMBERS   72     // the four peak numbers sit on this radius
#define SCR_PX_PER_G    40.0f  // 1.5 g reaches the edge of the disc
#define SCR_DOT_R       7
#define SCR_DOT_MAX     (SCR_R_DISC - SCR_DOT_R - 1)

#define SCR_TOP_H       32
#define SCR_BOTTOM_Y    200
#define SCR_COL_W       76
#define SCR_LCOL_X      58     // left values end here - the ring's side numbers
                               // start at 68 and one row is level with them
#define SCR_ROW_Y0      36     // right column, four rows
#define SCR_ROW_H       43
#define SCR_LROW_Y0     40     // left column, three rows, so they get more room
#define SCR_LROW_H      57
#define SCR_LABEL_DY    0      // label at the top of the row
#define SCR_VALUE_DY    13     // value sits below the label
#define SCR_LVALUE_DY   15     // the speed value is the large font, so it sits lower

// ---- colours -----------------------------------------------------------------

#define SCR_BG      GFX_BLACK
#define SCR_RING    GFX_RGB(44, 44, 44)
#define SCR_TICK    GFX_RGB(110, 110, 110)
#define SCR_LABEL   GFX_RGB(140, 140, 140)
#define SCR_VALUE   GFX_WHITE
#define SCR_DOT     GFX_RGB(30, 110, 255)
#define SCR_NUMBER  GFX_RGB(120, 170, 255)
#define SCR_OK      GFX_RGB(60, 200, 80)
#define SCR_BAD     GFX_RGB(230, 60, 60)
#define SCR_DIM     GFX_RGB(90, 90, 90)
#define SCR_LINE    GFX_RGB(30, 30, 30)

// ---- text elements -------------------------------------------------------------

typedef enum {
    EL_CLOCK = 0,      // wall clock, local time
    EL_DURATION,       // since power-up
    EL_SD,
    EL_GPS,
    EL_RING_TOP,
    EL_RING_BOTTOM,
    EL_RING_LEFT,
    EL_RING_RIGHT,
    EL_CENTRE,
    EL_L0, EL_L1, EL_L2,
    EL_R0, EL_R1, EL_R2, EL_R3,
    EL_BOT_SATS,
    EL_BOT_POS,
    EL_BOT_ALT,
    EL_COUNT
} screen_el_id_t;

typedef struct {
    const gfx_font_t *font;
    int16_t     ax;        // anchor x the alignment works from
    int16_t     y;         // top of the text cell
    gfx_align_t align;
    gfx_rect_t  box;       // the most the element may ever cover
    gfx_color_t color;
    char        text[40];
} screen_el_t;

static screen_el_t els[EL_COUNT];

static const char *left_labels[3]  = { "SPEED km/h", "MAX km/h", "DIST km" };
static const char *right_labels[4] = { "TEMP C", "HUMID %", "PRESS hPa", "GAS kohm" };

typedef enum {
    MODE_MAIN = 0,
    MODE_BOOT,
} screen_mode_t;

static screen_mode_t mode;
static int16_t dot_x, dot_y;     // where the dot is on the panel right now
static bool have_dot;

// an element's box, given its anchor and the widest text it can hold
static void el_setup(screen_el_id_t id, const gfx_font_t *font, int16_t ax, int16_t y,
                     gfx_align_t align, int16_t max_w, gfx_color_t color) {
    screen_el_t *e = &els[id];
    e->font = font;
    e->ax = ax;
    e->y = y;
    e->align = align;
    e->color = color;
    e->text[0] = '\0';
    e->box.y = y;
    e->box.h = font->height;
    e->box.w = max_w;
    if (align == GFX_ALIGN_LEFT) {
        e->box.x = ax;
    } else if (align == GFX_ALIGN_CENTER) {
        e->box.x = (int16_t)(ax - max_w / 2);
    } else {
        e->box.x = (int16_t)(ax - max_w);
    }
}

// new text or colour for an element; marks its box when something differs
static void el_set(screen_el_id_t id, gfx_color_t color, const char *text) {
    screen_el_t *e = &els[id];
    if ((e->color == color) && (strcmp(e->text, text) == 0)) {
        return;
    }
    e->color = color;
    strncpy(e->text, text, sizeof(e->text) - 1u);
    e->text[sizeof(e->text) - 1u] = '\0';
    gfx_invalidate(e->box.x, e->box.y, e->box.w, e->box.h);
}

static void el_paint(const screen_el_t *e) {
    if (e->text[0] != '\0') {
        gfx_draw_text(e->font, e->ax, e->y, e->align, e->text, e->color);
    }
}

static void screen_layout(void) {
    // top strip: the wall clock on the left, time since power-up in the middle
    el_setup(EL_CLOCK, &font_medium, 4, 7, GFX_ALIGN_LEFT, 96, SCR_VALUE);
    el_setup(EL_DURATION, &font_medium, SCR_CX, 7, GFX_ALIGN_CENTER, 110, SCR_VALUE);
    el_setup(EL_SD, &font_small, 316, 10, GFX_ALIGN_RIGHT, 24, SCR_DIM);
    el_setup(EL_GPS, &font_small, 286, 10, GFX_ALIGN_RIGHT, 30, SCR_DIM);

    // the four peaks, on the ring
    int16_t half = (int16_t)(font_medium.height / 2);
    el_setup(EL_RING_TOP, &font_medium, SCR_CX, (int16_t)(SCR_CY - SCR_R_NUMBERS - half),
             GFX_ALIGN_CENTER, 40, SCR_VALUE);
    el_setup(EL_RING_BOTTOM, &font_medium, SCR_CX, (int16_t)(SCR_CY + SCR_R_NUMBERS - half),
             GFX_ALIGN_CENTER, 40, SCR_VALUE);
    el_setup(EL_RING_LEFT, &font_medium, (int16_t)(SCR_CX - SCR_R_NUMBERS), (int16_t)(SCR_CY - half),
             GFX_ALIGN_CENTER, 40, SCR_VALUE);
    el_setup(EL_RING_RIGHT, &font_medium, (int16_t)(SCR_CX + SCR_R_NUMBERS), (int16_t)(SCR_CY - half),
             GFX_ALIGN_CENTER, 40, SCR_VALUE);

    // the live magnitude, left of centre; its bottom sits level with the top
    // of the side peak numbers, which keeps it clear of the dot at rest
    el_setup(EL_CENTRE, &font_large, (int16_t)(SCR_CX - 30),
             (int16_t)(SCR_CY - half - font_large.height),
             GFX_ALIGN_CENTER, 72, SCR_NUMBER);

    // left column: speed is the headline number, so it gets the large font
    el_setup(EL_L0, &font_large, SCR_LCOL_X,
             (int16_t)(SCR_LROW_Y0 + SCR_LVALUE_DY), GFX_ALIGN_RIGHT, SCR_LCOL_X - 4, SCR_VALUE);
    for (int i = 1; i < 3; i++) {
        int16_t y = (int16_t)(SCR_LROW_Y0 + i * SCR_LROW_H + SCR_VALUE_DY);
        el_setup((screen_el_id_t)(EL_L0 + i), &font_medium, SCR_LCOL_X, y,
                 GFX_ALIGN_RIGHT, SCR_LCOL_X - 4, SCR_VALUE);
    }

    // right column, four rows, values right-aligned under their labels
    for (int i = 0; i < 4; i++) {
        int16_t y = (int16_t)(SCR_ROW_Y0 + i * SCR_ROW_H + SCR_VALUE_DY);
        el_setup((screen_el_id_t)(EL_R0 + i), &font_medium, (int16_t)(GFX_W - 4), y,
                 GFX_ALIGN_RIGHT, SCR_COL_W - 6, SCR_VALUE);
    }

    // bottom strip: satellites then position down the left, altitude on its
    // own on the right in a bigger font - it is the number worth reading
    el_setup(EL_BOT_SATS, &font_small, 4, (int16_t)(SCR_BOTTOM_Y + 4), GFX_ALIGN_LEFT, 200, SCR_LABEL);
    el_setup(EL_BOT_POS, &font_small, 4, (int16_t)(SCR_BOTTOM_Y + 22), GFX_ALIGN_LEFT, 200, SCR_VALUE);
    el_setup(EL_BOT_ALT, &font_medium, (int16_t)(GFX_W - 4), (int16_t)(SCR_BOTTOM_Y + 10),
             GFX_ALIGN_RIGHT, 116, SCR_VALUE);
}

void screen_init(void) {
    mode = MODE_MAIN;
    have_dot = false;
    dot_x = SCR_CX;
    dot_y = SCR_CY;
    screen_layout();
    gfx_invalidate_all();
}

void screen_show_boot(void) {
    mode = MODE_BOOT;
    gfx_invalidate_all();
}

void screen_show_main(void) {
    mode = MODE_MAIN;
    have_dot = false;
    for (int i = 0; i < EL_COUNT; i++) {
        els[i].text[0] = '\0';
    }
    gfx_invalidate_all();
}

// ---- update ------------------------------------------------------------------

static void fmt_clock(char *buf, size_t n, uint32_t s, bool with_seconds) {
    uint32_t h = s / 3600u;
    uint32_t m = (s / 60u) % 60u;
    if (with_seconds) {
        snprintf(buf, n, "%lu:%02lu:%02lu", (unsigned long)h, (unsigned long)m,
                 (unsigned long)(s % 60u));
    } else {
        snprintf(buf, n, "%lu:%02lu", (unsigned long)h, (unsigned long)m);
    }
}

static void screen_update_dot(const screen_data_t *d) {
    // feel, not push: braking is a negative long_g and must move the dot up
    // (negative screen y), a left bend is a positive lat_g and must move it
    // right (positive screen x); both come out of the plain sums below
    float dx = d->lat_g * SCR_PX_PER_G;
    float dy = d->long_g * SCR_PX_PER_G;
    float len = sqrtf((dx * dx) + (dy * dy));
    if (len > (float)SCR_DOT_MAX) {
        dx *= (float)SCR_DOT_MAX / len;
        dy *= (float)SCR_DOT_MAX / len;
    }
    int16_t nx = (int16_t)(SCR_CX + lroundf(dx));
    int16_t ny = (int16_t)(SCR_CY + lroundf(dy));

    if (have_dot && (nx == dot_x) && (ny == dot_y)) {
        return;
    }
    if (have_dot) {
        gfx_invalidate((int16_t)(dot_x - SCR_DOT_R - 1), (int16_t)(dot_y - SCR_DOT_R - 1),
                       2 * SCR_DOT_R + 3, 2 * SCR_DOT_R + 3);
    }
    dot_x = nx;
    dot_y = ny;
    have_dot = true;
    gfx_invalidate((int16_t)(dot_x - SCR_DOT_R - 1), (int16_t)(dot_y - SCR_DOT_R - 1),
                   2 * SCR_DOT_R + 3, 2 * SCR_DOT_R + 3);
}

void screen_update(const screen_data_t *d) {
    char buf[40];

    if ((d == NULL) || (mode != MODE_MAIN)) {
        return;
    }

    // top: the wall clock in local time, then the time since power-up
    if (d->time_valid) {
        // UTC in, wall clock out; the date is not shown so only the hours wrap
        uint32_t mins = ((uint32_t)d->hour * 60u) + d->minute +
                        (uint32_t)(VEHICLE_UTC_OFFSET_MIN + (24 * 60));
        mins %= 1440u;
        snprintf(buf, sizeof(buf), "%02lu:%02lu:%02u",
                 (unsigned long)(mins / 60u), (unsigned long)(mins % 60u),
                 (unsigned)d->second);
        el_set(EL_CLOCK, SCR_VALUE, buf);
    } else {
        el_set(EL_CLOCK, SCR_DIM, "--:--:--");
    }
    fmt_clock(buf, sizeof(buf), d->duration_s, true);
    el_set(EL_DURATION, SCR_VALUE, buf);
    el_set(EL_SD, d->sd_ok ? SCR_OK : SCR_BAD, "SD");
    el_set(EL_GPS, d->fix_valid ? SCR_OK : SCR_DIM, "GPS");

    // the gauge
    snprintf(buf, sizeof(buf), "%.1f", (double)d->peak_brake_g);
    el_set(EL_RING_TOP, SCR_VALUE, buf);
    snprintf(buf, sizeof(buf), "%.1f", (double)d->peak_accel_g);
    el_set(EL_RING_BOTTOM, SCR_VALUE, buf);
    snprintf(buf, sizeof(buf), "%.1f", (double)d->peak_right_g);   // thrown left in a right bend
    el_set(EL_RING_LEFT, SCR_VALUE, buf);
    snprintf(buf, sizeof(buf), "%.1f", (double)d->peak_left_g);    // thrown right in a left bend
    el_set(EL_RING_RIGHT, SCR_VALUE, buf);
    float mag = sqrtf((d->long_g * d->long_g) + (d->lat_g * d->lat_g));
    snprintf(buf, sizeof(buf), "%.1f", (double)mag);
    el_set(EL_CENTRE, SCR_NUMBER, buf);
    screen_update_dot(d);

    // left column
    snprintf(buf, sizeof(buf), "%.0f", (double)d->speed_kmh);
    el_set(EL_L0, d->coasting ? SCR_LABEL : SCR_VALUE, buf);
    snprintf(buf, sizeof(buf), "%.0f", (double)d->speed_max_kmh);
    el_set(EL_L1, SCR_VALUE, buf);
    if (d->distance_km < 100.0f) {
        snprintf(buf, sizeof(buf), "%.1f", (double)d->distance_km);
    } else {
        snprintf(buf, sizeof(buf), "%.0f", (double)d->distance_km);
    }
    el_set(EL_L2, SCR_VALUE, buf);

    // right column
    if (d->env_valid) {
        snprintf(buf, sizeof(buf), "%.1f", (double)d->temp_c);
        el_set(EL_R0, SCR_VALUE, buf);
        snprintf(buf, sizeof(buf), "%.0f", (double)d->hum_pct);
        el_set(EL_R1, SCR_VALUE, buf);
        snprintf(buf, sizeof(buf), "%.0f", (double)d->press_hpa);
        el_set(EL_R2, SCR_VALUE, buf);
    } else {
        el_set(EL_R0, SCR_DIM, "--");
        el_set(EL_R1, SCR_DIM, "--");
        el_set(EL_R2, SCR_DIM, "--");
    }
    if (d->env_valid && d->gas_valid) {
        snprintf(buf, sizeof(buf), "%.0f", (double)d->gas_kohm);
        el_set(EL_R3, SCR_VALUE, buf);
    } else {
        el_set(EL_R3, SCR_DIM, "--");
    }

    // bottom: how many satellites, where it thinks it is, and the altitude on
    // its own because that is the one worth reading
    if (d->fix_valid) {
        snprintf(buf, sizeof(buf), "%u sats", (unsigned)d->sats);
    } else {
        snprintf(buf, sizeof(buf), "no fix  %u sats", (unsigned)d->sats);
    }
    el_set(EL_BOT_SATS, SCR_LABEL, buf);

    if (d->fix_valid) {
        snprintf(buf, sizeof(buf), "%.5f %c  %.5f %c",
                 fabs(d->lat_deg), (d->lat_deg < 0.0) ? 'S' : 'N',
                 fabs(d->lon_deg), (d->lon_deg < 0.0) ? 'W' : 'E');
        el_set(EL_BOT_POS, SCR_VALUE, buf);
        // grey until it settles - after a cold start the GPS height walks for
        // minutes and the number is not worth reading yet
        snprintf(buf, sizeof(buf), "ALT %.0f m", (double)d->alt_m);
        el_set(EL_BOT_ALT, d->alt_settled ? SCR_VALUE : SCR_DIM, buf);
    } else {
        el_set(EL_BOT_POS, SCR_DIM, "--");
        el_set(EL_BOT_ALT, SCR_DIM, "ALT --");
    }
}

// ---- paint -------------------------------------------------------------------

static void paint_gauge_chrome(void) {
    gfx_fill_ring(SCR_CX, SCR_CY, SCR_R_RING, SCR_R_DISC, SCR_RING);
    gfx_fill_circle(SCR_CX, SCR_CY, SCR_R_DISC, SCR_BG);

    // ticks every 0.5 g on the four arms, a dot in the middle
    for (int i = 1; i <= 2; i++) {
        int16_t d = (int16_t)(i * 20);
        gfx_hline((int16_t)(SCR_CX - d), (int16_t)(SCR_CY - 2), 1, SCR_TICK);
        gfx_vline((int16_t)(SCR_CX - d), (int16_t)(SCR_CY - 2), 5, SCR_TICK);
        gfx_vline((int16_t)(SCR_CX + d), (int16_t)(SCR_CY - 2), 5, SCR_TICK);
        gfx_hline((int16_t)(SCR_CX - 2), (int16_t)(SCR_CY - d), 5, SCR_TICK);
        gfx_hline((int16_t)(SCR_CX - 2), (int16_t)(SCR_CY + d), 5, SCR_TICK);
    }
    gfx_fill_rect(SCR_CX - 1, SCR_CY - 1, 3, 3, SCR_TICK);
}

static void paint_main(void) {
    gfx_fill_rect(0, 0, GFX_W, GFX_H, SCR_BG);

    // strip separators
    gfx_hline(0, SCR_TOP_H, GFX_W, SCR_LINE);
    gfx_hline(0, SCR_BOTTOM_Y - 1, GFX_W, SCR_LINE);

    paint_gauge_chrome();
    el_paint(&els[EL_CENTRE]);
    if (have_dot) {
        gfx_fill_circle(dot_x, dot_y, SCR_DOT_R, SCR_DOT);
    }
    for (int i = EL_RING_TOP; i <= EL_RING_RIGHT; i++) {
        el_paint(&els[i]);
    }

    // column labels are fixed text, so they are not elements
    for (int i = 0; i < 3; i++) {
        int16_t y = (int16_t)(SCR_LROW_Y0 + i * SCR_LROW_H + SCR_LABEL_DY);
        gfx_draw_text(&font_small, 4, y, GFX_ALIGN_LEFT, left_labels[i], SCR_LABEL);
        el_paint(&els[EL_L0 + i]);
    }
    for (int i = 0; i < 4; i++) {
        int16_t y = (int16_t)(SCR_ROW_Y0 + i * SCR_ROW_H + SCR_LABEL_DY);
        gfx_draw_text(&font_small, (int16_t)(GFX_W - SCR_COL_W + 4), y, GFX_ALIGN_LEFT,
                      right_labels[i], SCR_LABEL);
        el_paint(&els[EL_R0 + i]);
    }

    el_paint(&els[EL_CLOCK]);
    el_paint(&els[EL_DURATION]);
    el_paint(&els[EL_SD]);
    el_paint(&els[EL_GPS]);
    el_paint(&els[EL_BOT_SATS]);
    el_paint(&els[EL_BOT_POS]);
    el_paint(&els[EL_BOT_ALT]);
}

static void paint_boot(void) {
    // black and quiet; the colour bars did their job settling MADCTL and the
    // byte order, and there is nothing left to read off them
    // font_large starts at character 45, so it holds digits, a dot and a minus
    // and no letters at all - any text has to use one of the other two
    gfx_fill_rect(0, 0, GFX_W, GFX_H, SCR_BG);
    gfx_draw_text(&font_medium, SCR_CX, 104, GFX_ALIGN_CENTER, "DASHBOARD", SCR_VALUE);
    gfx_draw_text(&font_small, SCR_CX, 130, GFX_ALIGN_CENTER, "build " __DATE__, SCR_LABEL);
}

void screen_paint(const gfx_rect_t *band) {
    (void)band;   // every primitive clips to it on its own
    if (mode == MODE_BOOT) {
        paint_boot();
    } else {
        paint_main();
    }
}
