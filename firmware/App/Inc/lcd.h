//
// LCD driver - public interface
//
// - talks to the ILI9341 on the display module over SPI1: reset, the init
//   sequence, the window and pixel stream, register read-back, and the
//   backlight PWM on TIM1
// - this is the bottom of the display stack and the one file in it that
//   includes HAL; gfx.c above it only ever sees lcd_write_band, and even that
//   through a function pointer
// - every register number and wait in here is checked against
//   datasheets/Adafruit_ILI9341.pdf, the ILI9341 V1.13 datasheet; the page
//   is noted next to each one
//

#ifndef LCD_H
#define LCD_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    LCD_OK = 0,
    LCD_ERR_ARG,
    LCD_ERR_SPI,     // a HAL transfer failed
    LCD_ERR_ID,      // the panel did not answer like an ILI9341
} lcd_status_t;

// what the ID gate reads back, raw and decoded, so the console can show all
// of it whatever the panel does
typedef struct {
    uint8_t  raw_id4[4];     // the four bytes clocked out after D3h, untouched
    uint32_t id4_dummy_bit;  // decoded as the datasheet says: one dummy clock, then 24 bits
    uint32_t id4_dummy_byte; // decoded as if the dummy were a whole byte
    uint8_t  power_mode;     // 0Ah after reset - datasheet says 0x08
    uint8_t  pixfmt_reset;   // 0Ch after reset - datasheet says 0x06
    uint8_t  pixfmt_set;     // 0Ch after COLMOD 0x55 - must read 0x05
    bool     id_ok;          // either decode is 0x009341
    bool     pixfmt_ok;      // 0x06 became 0x05 - a write went in and came back
} lcd_probe_t;

// hardware reset, then the read-back gate: ID4, pixel format before and after
// writing it, power mode. leaves the panel asleep with the backlight off, so
// it is safe to call before deciding whether to go on
lcd_status_t lcd_probe(lcd_probe_t *out);

// reset, sleep out, 16-bit colour, landscape, clear to black, display on;
// the backlight stays off until lcd_backlight_set asks for light
lcd_status_t lcd_init(void);

// one band of pixels to a window on the panel; pixels are already in wire
// order (high byte first). this is the gfx output callback
void lcd_write_band(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pixels);

// brightness target in permille; the actual level ramps towards it in
// lcd_backlight_tick so the panel fades up instead of snapping on
void lcd_backlight_set(uint16_t permille);
void lcd_backlight_tick(void);

// for the console
uint32_t lcd_bytes_sent(void);
uint32_t lcd_errors(void);

#endif // LCD_H
