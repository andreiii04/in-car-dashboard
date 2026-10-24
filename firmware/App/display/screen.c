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
#define SCR_CY          116    // the ring fills the gap between the two divider
                               // lines exactly, y 32 to 200
#define SCR_R_RING      84     // outer edge of the grey ring
#define SCR_R_DISC      60     // the black disc the dot lives on
#define SCR_R_NUMBERS   72     // the four peak numbers sit on this radius
#define SCR_PX_PER_G    40.0f  // 1.5 g reaches the edge of the disc
#define SCR_DOT_R       7
#define SCR_DOT_MAX     (SCR_R_DISC - SCR_DOT_R - 1)

#define SCR_EDGE        8      // no text closer than this to the left or right edge
#define SCR_TOP_H       32
#define SCR_BOTTOM_Y    200
#define SCR_COL_W       76
#define SCR_LCOL_X      58     // left values end here - the ring's side numbers
                               // start at 68 and one row is level with them
#define SCR_SPEED_X     74     // the speed ends further right: "100" is 3 x 22 px
                               // in the large font, so it starts at x 8; the ring
                               // comes no closer than x 84 in that row
#define SCR_ROW_Y0      36     // right column, four rows
#define SCR_ROW_H       43
#define SCR_LROW_Y0     40     // left column, three rows, so they get more room
#define SCR_LROW_H      57
#define SCR_LABEL_DY    0      // label at the top of the row
#define SCR_VALUE_DY    13     // value sits below the label
#define SCR_LVALUE_DY   15     // the speed value is the large font, so it sits lower
#define SCR_MAG_X       118    // the live g number, centred here - up and left,
#define SCR_MAG_Y       63     // half on the ring between two peak numbers; past
                               // ~1.2 g up-left the dot covers its first digit

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

// the IAQ bands and their colours, from the BME680 datasheet Table 4 (p9);
// the last two lightened a little so they still read on black
#define SCR_IAQ_EXCELLENT GFX_RGB(0, 230, 0)       // 0-50
#define SCR_IAQ_GOOD      GFX_RGB(146, 208, 80)    // 51-100
#define SCR_IAQ_LIGHT     GFX_RGB(255, 255, 0)     // 101-150, lightly polluted
#define SCR_IAQ_MODERATE  GFX_RGB(255, 128, 0)     // 151-200
#define SCR_IAQ_HEAVY     GFX_RGB(255, 0, 0)       // 201-250
#define SCR_IAQ_SEVERE    GFX_RGB(200, 0, 110)     // 251-350
#define SCR_IAQ_EXTREME   GFX_RGB(170, 90, 30)     // above 350

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
    EL_R3_LABEL,       // "IAQ", with the accuracy until BSEC is calibrated
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
// PRESS has no unit: right-aligned, "PRESS hPa" touched the gauge's right
// peak number, and 1013 reads as hPa anyway
static const char *right_labels[3] = { "TEMP C", "HUMID %", "PRESS" };

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
    el_setup(EL_CLOCK, &font_medium, SCR_EDGE, 7, GFX_ALIGN_LEFT, 96, SCR_VALUE);
    el_setup(EL_DURATION, &font_medium, SCR_CX, 7, GFX_ALIGN_CENTER, 110, SCR_VALUE);
    el_setup(EL_SD, &font_small, (int16_t)(GFX_W - SCR_EDGE), 10, GFX_ALIGN_RIGHT, 24, SCR_DIM);
    el_setup(EL_GPS, &font_small, (int16_t)(GFX_W - SCR_EDGE - 30), 10, GFX_ALIGN_RIGHT, 30, SCR_DIM);

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

    // the live magnitude, up and left on the ring, clear of the dot and of the
    // left and top peak numbers
    el_setup(EL_CENTRE, &font_gauge, SCR_MAG_X, SCR_MAG_Y, GFX_ALIGN_CENTER, 60, SCR_NUMBER);

    // left column: speed is the headline number, so it gets the large font
    el_setup(EL_L0, &font_large, SCR_SPEED_X,
             (int16_t)(SCR_LROW_Y0 + SCR_LVALUE_DY), GFX_ALIGN_RIGHT, SCR_SPEED_X - SCR_EDGE, SCR_VALUE);
    for (int i = 1; i < 3; i++) {
        int16_t y = (int16_t)(SCR_LROW_Y0 + i * SCR_LROW_H + SCR_VALUE_DY);
        el_setup((screen_el_id_t)(EL_L0 + i), &font_medium, SCR_LCOL_X, y,
                 GFX_ALIGN_RIGHT, SCR_LCOL_X - SCR_EDGE, SCR_VALUE);
    }

    // right column, four rows, labels and values on one right edge
    for (int i = 0; i < 4; i++) {
        int16_t y = (int16_t)(SCR_ROW_Y0 + i * SCR_ROW_H + SCR_VALUE_DY);
        el_setup((screen_el_id_t)(EL_R0 + i), &font_medium, (int16_t)(GFX_W - SCR_EDGE), y,
                 GFX_ALIGN_RIGHT, SCR_COL_W - SCR_EDGE, SCR_VALUE);
    }
    // the IAQ label changes with the accuracy, so it is an element too
    el_setup(EL_R3_LABEL, &font_small, (int16_t)(GFX_W - SCR_EDGE),
             (int16_t)(SCR_ROW_Y0 + 3 * SCR_ROW_H + SCR_LABEL_DY),
             GFX_ALIGN_RIGHT, SCR_COL_W - SCR_EDGE, SCR_LABEL);

    // bottom strip: satellites then position down the left, altitude on its
    // own on the right in a bigger font - it is the number worth reading
    el_setup(EL_BOT_SATS, &font_small, SCR_EDGE, (int16_t)(SCR_BOTTOM_Y + 4), GFX_ALIGN_LEFT, 196, SCR_LABEL);
    el_setup(EL_BOT_POS, &font_small, SCR_EDGE, (int16_t)(SCR_BOTTOM_Y + 22), GFX_ALIGN_LEFT, 196, SCR_VALUE);
    el_setup(EL_BOT_ALT, &font_medium, (int16_t)(GFX_W - SCR_EDGE), (int16_t)(SCR_BOTTOM_Y + 10),
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

// which band of the datasheet's table an IAQ falls in
static gfx_color_t iaq_color(float iaq) {
    if (iaq <= 50.0f) {
        return SCR_IAQ_EXCELLENT;
    }
    if (iaq <= 100.0f) {
        return SCR_IAQ_GOOD;
    }
    if (iaq <= 150.0f) {
        return SCR_IAQ_LIGHT;
    }
    if (iaq <= 200.0f) {
        return SCR_IAQ_MODERATE;
    }
    if (iaq <= 250.0f) {
        return SCR_IAQ_HEAVY;
    }
    if (iaq <= 350.0f) {
        return SCR_IAQ_SEVERE;
    }
    return SCR_IAQ_EXTREME;
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
    // no fix and nothing to coast on means no speed at all, not a 0
    if (!d->fix_valid && !d->coasting) {
        el_set(EL_L0, SCR_DIM, "--");
    } else {
        snprintf(buf, sizeof(buf), "%.0f", (double)d->speed_kmh);
        el_set(EL_L0, d->coasting ? SCR_LABEL : SCR_VALUE, buf);
    }
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
    // the IAQ in its band's colour; grey at accuracy 0, where BSEC's 50 is
    // only a starting value. the label carries the accuracy until it is 3
    if (d->env_valid && d->iaq_valid) {
        snprintf(buf, sizeof(buf), "%.0f", (double)d->iaq);
        el_set(EL_R3, (d->iaq_accuracy == 0u) ? SCR_DIM : iaq_color(d->iaq), buf);
        if (d->iaq_accuracy < 3u) {
            snprintf(buf, sizeof(buf), "IAQ %u/3", (unsigned)d->iaq_accuracy);
            el_set(EL_R3_LABEL, SCR_LABEL, buf);
        } else {
            el_set(EL_R3_LABEL, SCR_LABEL, "IAQ");
        }
    } else {
        el_set(EL_R3, SCR_DIM, "--");
        el_set(EL_R3_LABEL, SCR_LABEL, "IAQ");
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
        gfx_draw_text(&font_small, SCR_EDGE, y, GFX_ALIGN_LEFT, left_labels[i], SCR_LABEL);
        el_paint(&els[EL_L0 + i]);
    }
    for (int i = 0; i < 4; i++) {
        if (i < 3) {
            int16_t y = (int16_t)(SCR_ROW_Y0 + i * SCR_ROW_H + SCR_LABEL_DY);
            gfx_draw_text(&font_small, (int16_t)(GFX_W - SCR_EDGE), y, GFX_ALIGN_RIGHT,
                          right_labels[i], SCR_LABEL);
        }
        el_paint(&els[EL_R0 + i]);
    }
    el_paint(&els[EL_R3_LABEL]);

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
