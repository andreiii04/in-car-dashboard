//
// gfx module - implementation
//
// - one band buffer, 320 x 16 pixels; a dirty rectangle is painted in slices
//   that fit in it, narrow rectangles get more rows per slice
// - the dirty list is a handful of rectangles; a new one that touches an old
//   one is merged into it, so the list never grows past its size
// - every primitive starts by intersecting itself with the current band and
//   gives up straight away when there is no overlap, so painting the whole
//   screen into a small band costs about nothing outside that band
//

#include "gfx.h"
#include <stddef.h>
#include <string.h>

#define GFX_DIRTY_MAX 16

static uint16_t band_buf[GFX_BAND_PIXELS];   // wire order
static gfx_rect_t band;                      // what band_buf covers right now
static bool band_active;

static gfx_rect_t dirty[GFX_DIRTY_MAX];
static uint8_t dirty_n;
static int16_t dirty_row;                    // rows of dirty[0] already sent

static gfx_paint_fn paint_cb;
static gfx_output_fn output_cb;
static uint32_t bands_sent;

static int16_t gfx_min(int16_t a, int16_t b) {
    return (a < b) ? a : b;
}

static int16_t gfx_max(int16_t a, int16_t b) {
    return (a > b) ? a : b;
}

// clip a rectangle to the screen; false if nothing is left
static bool gfx_clip_screen(gfx_rect_t *r) {
    int16_t x1 = (int16_t)(r->x + r->w);
    int16_t y1 = (int16_t)(r->y + r->h);
    r->x = gfx_max(r->x, 0);
    r->y = gfx_max(r->y, 0);
    x1 = gfx_min(x1, GFX_W);
    y1 = gfx_min(y1, GFX_H);
    r->w = (int16_t)(x1 - r->x);
    r->h = (int16_t)(y1 - r->y);
    return (r->w > 0) && (r->h > 0);
}

// true when the two touch or overlap - touching is enough to merge, since a
// seam between two bands costs a second window setup on the panel
static bool gfx_touches(const gfx_rect_t *a, const gfx_rect_t *b) {
    return (a->x <= b->x + b->w) && (b->x <= a->x + a->w) &&
           (a->y <= b->y + b->h) && (b->y <= a->y + a->h);
}

static void gfx_union(gfx_rect_t *a, const gfx_rect_t *b) {
    int16_t x1 = gfx_max((int16_t)(a->x + a->w), (int16_t)(b->x + b->w));
    int16_t y1 = gfx_max((int16_t)(a->y + a->h), (int16_t)(b->y + b->h));
    a->x = gfx_min(a->x, b->x);
    a->y = gfx_min(a->y, b->y);
    a->w = (int16_t)(x1 - a->x);
    a->h = (int16_t)(y1 - a->y);
}

void gfx_init(gfx_paint_fn paint, gfx_output_fn output) {
    paint_cb = paint;
    output_cb = output;
    dirty_n = 0;
    dirty_row = 0;
    band_active = false;
    bands_sent = 0;
}

void gfx_invalidate(int16_t x, int16_t y, int16_t w, int16_t h) {
    gfx_rect_t r = { x, y, w, h };
    if (!gfx_clip_screen(&r)) {
        return;
    }

    // the one being sent right now must not grow under the cursor - merging
    // into it would leave its already-sent rows stale. skip it in the search
    for (uint8_t i = (dirty_row > 0) ? 1u : 0u; i < dirty_n; i++) {
        if (gfx_touches(&dirty[i], &r)) {
            gfx_union(&dirty[i], &r);
            return;
        }
    }
    if (dirty_n < GFX_DIRTY_MAX) {
        dirty[dirty_n++] = r;
    } else {
        gfx_union(&dirty[GFX_DIRTY_MAX - 1], &r);   // full: widen the last one
    }
}

void gfx_invalidate_all(void) {
    // everything at once; whatever was partly sent gets repainted with it
    dirty_n = 1;
    dirty_row = 0;
    dirty[0].x = 0;
    dirty[0].y = 0;
    dirty[0].w = GFX_W;
    dirty[0].h = GFX_H;
}

bool gfx_dirty(void) {
    return dirty_n > 0;
}

bool gfx_flush_band(void) {
    if ((dirty_n == 0) || (paint_cb == NULL)) {
        return false;
    }

    gfx_rect_t *r = &dirty[0];
    int16_t rows_fit = (int16_t)(GFX_BAND_PIXELS / r->w);
    int16_t rows_left = (int16_t)(r->h - dirty_row);
    int16_t rows = gfx_min(rows_fit, rows_left);

    band.x = r->x;
    band.y = (int16_t)(r->y + dirty_row);
    band.w = r->w;
    band.h = rows;
    band_active = true;

    // start from black so a painter that skips a spot leaves black, not the
    // previous band's leftovers
    memset(band_buf, 0, (size_t)band.w * (size_t)band.h * sizeof(uint16_t));
    paint_cb(&band);
    band_active = false;

    if (output_cb != NULL) {
        output_cb(band.x, band.y, band.w, band.h, band_buf);
    }
    bands_sent++;

    dirty_row = (int16_t)(dirty_row + rows);
    if (dirty_row >= r->h) {
        // this rectangle is done; shift the rest down
        for (uint8_t i = 1; i < dirty_n; i++) {
            dirty[i - 1] = dirty[i];
        }
        dirty_n--;
        dirty_row = 0;
    }
    return dirty_n > 0;
}

uint32_t gfx_bands_sent(void) {
    return bands_sent;
}

// ---- primitives --------------------------------------------------------------

// clip a span on one row to the band and write it; the workhorse for
// everything below
static void gfx_span(int16_t x, int16_t y, int16_t w, uint16_t wire) {
    if (!band_active || (y < band.y) || (y >= band.y + band.h)) {
        return;
    }
    int16_t x0 = gfx_max(x, band.x);
    int16_t x1 = gfx_min((int16_t)(x + w), (int16_t)(band.x + band.w));
    if (x1 <= x0) {
        return;
    }
    uint16_t *p = &band_buf[(size_t)(y - band.y) * (size_t)band.w + (size_t)(x0 - band.x)];
    for (int16_t i = x0; i < x1; i++) {
        *p++ = wire;
    }
}

void gfx_fill_rect(int16_t x, int16_t y, int16_t w, int16_t h, gfx_color_t c) {
    if (!band_active || (w <= 0) || (h <= 0)) {
        return;
    }
    int16_t y0 = gfx_max(y, band.y);
    int16_t y1 = gfx_min((int16_t)(y + h), (int16_t)(band.y + band.h));
    uint16_t wire = gfx_swap(c);
    for (int16_t row = y0; row < y1; row++) {
        gfx_span(x, row, w, wire);
    }
}

void gfx_hline(int16_t x, int16_t y, int16_t w, gfx_color_t c) {
    gfx_span(x, y, w, gfx_swap(c));
}

void gfx_vline(int16_t x, int16_t y, int16_t h, gfx_color_t c) {
    gfx_fill_rect(x, y, 1, h, c);
}

void gfx_draw_rect(int16_t x, int16_t y, int16_t w, int16_t h, gfx_color_t c) {
    if ((w <= 0) || (h <= 0)) {
        return;
    }
    gfx_hline(x, y, w, c);
    gfx_hline(x, (int16_t)(y + h - 1), w, c);
    gfx_vline(x, y, h, c);
    gfx_vline((int16_t)(x + w - 1), y, h, c);
}

// integer square root, rounded down; enough for circle edges and it keeps
// the host preview and the board bit-identical
static int16_t gfx_isqrt(int32_t n) {
    int32_t r = 0;
    while ((r + 1) * (r + 1) <= n) {
        r++;
    }
    return (int16_t)r;
}

void gfx_fill_circle(int16_t cx, int16_t cy, int16_t r, gfx_color_t c) {
    if (!band_active || (r <= 0)) {
        return;
    }
    // only the rows this band holds; each row is one span whose half-width
    // comes from the circle equation
    int16_t y0 = gfx_max((int16_t)(cy - r), band.y);
    int16_t y1 = gfx_min((int16_t)(cy + r), (int16_t)(band.y + band.h - 1));
    uint16_t wire = gfx_swap(c);
    for (int16_t y = y0; y <= y1; y++) {
        int32_t dy = y - cy;
        int16_t dx = gfx_isqrt((int32_t)r * r - dy * dy);
        gfx_span((int16_t)(cx - dx), y, (int16_t)(2 * dx + 1), wire);
    }
}

void gfx_fill_ring(int16_t cx, int16_t cy, int16_t r_outer, int16_t r_inner, gfx_color_t c) {
    if (!band_active || (r_outer <= 0) || (r_inner >= r_outer)) {
        return;
    }
    int16_t y0 = gfx_max((int16_t)(cy - r_outer), band.y);
    int16_t y1 = gfx_min((int16_t)(cy + r_outer), (int16_t)(band.y + band.h - 1));
    uint16_t wire = gfx_swap(c);
    for (int16_t y = y0; y <= y1; y++) {
        int32_t dy = y - cy;
        int16_t dxo = gfx_isqrt((int32_t)r_outer * r_outer - dy * dy);
        if ((r_inner <= 0) || (dy * dy > (int32_t)r_inner * r_inner)) {
            // above or below the hole: one solid span
            gfx_span((int16_t)(cx - dxo), y, (int16_t)(2 * dxo + 1), wire);
        } else {
            // through the hole: a span each side of it
            int16_t dxi = gfx_isqrt((int32_t)r_inner * r_inner - dy * dy);
            gfx_span((int16_t)(cx - dxo), y, (int16_t)(dxo - dxi), wire);
            gfx_span((int16_t)(cx + dxi + 1), y, (int16_t)(dxo - dxi), wire);
        }
    }
}

// look a character up, falling back to '?' for anything the font lacks
static const gfx_glyph_t *gfx_glyph(const gfx_font_t *font, char ch) {
    uint8_t code = (uint8_t)ch;
    if ((code < font->first) || (code >= font->first + font->count)) {
        code = (uint8_t)'?';
        if ((code < font->first) || (code >= font->first + font->count)) {
            return NULL;
        }
    }
    return &font->glyphs[code - font->first];
}

int16_t gfx_text_width(const gfx_font_t *font, const char *s) {
    int16_t w = 0;
    if ((font == NULL) || (s == NULL)) {
        return 0;
    }
    for (; *s != '\0'; s++) {
        const gfx_glyph_t *g = gfx_glyph(font, *s);
        if (g != NULL) {
            w = (int16_t)(w + g->advance);
        }
    }
    return w;
}

static void gfx_draw_glyph(const gfx_font_t *font, const gfx_glyph_t *g,
                           int16_t x, int16_t y, uint16_t wire) {
    // skip cells entirely outside the band before touching any bits
    if ((x >= band.x + band.w) || (x + g->width <= band.x) ||
        (y >= band.y + band.h) || (y + font->height <= band.y)) {
        return;
    }
    uint16_t row_bytes = (uint16_t)((g->width + 7u) / 8u);
    int16_t r0 = gfx_max((int16_t)(band.y - y), 0);
    int16_t r1 = gfx_min((int16_t)(band.y + band.h - y), (int16_t)font->height);
    int16_t c0 = gfx_max((int16_t)(band.x - x), 0);
    int16_t c1 = gfx_min((int16_t)(band.x + band.w - x), (int16_t)g->width);

    for (int16_t r = r0; r < r1; r++) {
        const uint8_t *row = &font->bits[g->offset + (size_t)r * row_bytes];
        uint16_t *p = &band_buf[(size_t)(y + r - band.y) * (size_t)band.w +
                                (size_t)(x + c0 - band.x)];
        for (int16_t col = c0; col < c1; col++) {
            if ((row[col >> 3] & (0x80u >> (col & 7))) != 0u) {
                *p = wire;
            }
            p++;
        }
    }
}

void gfx_draw_text(const gfx_font_t *font, int16_t x, int16_t y, gfx_align_t align,
                   const char *s, gfx_color_t c) {
    if (!band_active || (font == NULL) || (s == NULL)) {
        return;
    }
    int16_t w = gfx_text_width(font, s);
    if (align == GFX_ALIGN_CENTER) {
        x = (int16_t)(x - w / 2);
    } else if (align == GFX_ALIGN_RIGHT) {
        x = (int16_t)(x - w);
    }
    // the whole line outside the band: nothing to do
    if ((y >= band.y + band.h) || (y + font->height <= band.y) ||
        (x >= band.x + band.w) || (x + w <= band.x)) {
        return;
    }
    uint16_t wire = gfx_swap(c);
    for (; *s != '\0'; s++) {
        const gfx_glyph_t *g = gfx_glyph(font, *s);
        if (g == NULL) {
            continue;
        }
        gfx_draw_glyph(font, g, x, y, wire);
        x = (int16_t)(x + g->advance);
    }
}
