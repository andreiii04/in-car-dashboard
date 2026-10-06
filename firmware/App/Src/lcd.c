//
// LCD driver - implementation
//
// - 4-wire serial: CS frames a transfer, DC low means the byte is a command
//   and high means data; datasheet ch7.1.8 p33
// - writes run at 42 MBit/s, four times over the 100 ns cycle the datasheet
//   guarantees (p243), which every driver for these modules does; if pixels
//   ever go wrong, LCD_WRITE_PRESCALER is the knob
// - reads are different: the read cycle minimum is 150 ns (p243), so every
//   read drops SPI1 to 5.25 MHz and puts it back after
// - the 24/32-bit reads (04h, 09h, D3h) start with ONE dummy clock, not a
//   dummy byte (p38-39), so four bytes read back hold the value shifted right
//   by seven bits; lcd_probe decodes it both ways and the console shows both
//

#include "lcd.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include <string.h>

// ---- commands, ch8.1 p83-87 ------------------------------------------------------
#define LCD_CMD_SLPOUT    0x11u   // sleep out, p101
#define LCD_CMD_DISPON    0x29u   // display on, p109
#define LCD_CMD_CASET     0x2Au   // column address set, p110
#define LCD_CMD_PASET     0x2Bu   // page address set, p112
#define LCD_CMD_RAMWR     0x2Cu   // memory write, p114
#define LCD_CMD_MADCTL    0x36u   // memory access control, p127
#define LCD_CMD_COLMOD    0x3Au   // pixel format, p134
#define LCD_CMD_RDDPM     0x0Au   // read display power mode, p94
#define LCD_CMD_RDPIXFMT  0x0Cu   // read display pixel format, p96
#define LCD_CMD_RDID4     0xD3u   // read ID4, p187

// COLMOD 0x55: 16 bits per pixel on both the RGB and the MCU interface, p134
#define LCD_COLMOD_16BIT  0x55u

// MADCTL, p127: bit7 MY, bit6 MX, bit5 MV, bit4 ML, bit3 BGR, bit2 MH.
// landscape is MV=1 (rows and columns swapped); which way is up and whether
// red and blue swap are settled on the test pattern - if it comes out upside
// down use the _FLIP value, if red shows as blue clear bit 3
#define LCD_MADCTL_LANDSCAPE       0x28u   // MV | BGR
#define LCD_MADCTL_LANDSCAPE_FLIP  0xE8u   // MY | MX | MV | BGR
#define LCD_MADCTL                 LCD_MADCTL_LANDSCAPE_FLIP

#define LCD_ID4_EXPECTED  0x009341u  // p187: 00h, 93h, 41h
#define LCD_PIXFMT_RESET  0x06u      // p96: DPI 110, DBI 110 after reset
#define LCD_PIXFMT_16BIT  0x05u      // after COLMOD 0x55
#define LCD_RDDPM_RESET   0x08u      // p94: display normal mode on, everything else off

// SPI1 on APB2 at 84 MHz: prescaler 2 is 42 MBit/s for pixels, prescaler 16
// is 5.25 MHz for reads, under the 6.67 MHz the 150 ns read cycle allows
#define LCD_WRITE_PRESCALER  SPI_BAUDRATEPRESCALER_2
#define LCD_READ_PRESCALER   SPI_BAUDRATEPRESCALER_16

#define LCD_WIDTH  320
#define LCD_HEIGHT 240

#define LCD_SPI_TIMEOUT_MS 100u

// backlight ramp: one step every LCD_BL_STEP_MS, LCD_BL_STEP permille at a
// time - 0 to full in about 300 ms
#define LCD_BL_STEP_MS  10u
#define LCD_BL_STEP     33u

static uint32_t bytes_sent;
static uint32_t errors;
static uint16_t bl_target;
static uint16_t bl_now;
static uint32_t bl_last_tick;

// whether the panel answers reads at all - lcd_probe sets it, lcd_init uses it
// to decide whether reading a register back proves anything
static bool reads_work;

// ---- pins and SPI --------------------------------------------------------------

static void cs_low(void) {
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void cs_high(void) {
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static void dc_command(void) {
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
}

static void dc_data(void) {
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
}

// change the clock divider; the BR bits may only move while the peripheral
// is disabled, and HAL re-enables it on the next transfer
static void spi_prescaler(uint32_t prescaler) {
    __HAL_SPI_DISABLE(&hspi1);
    MODIFY_REG(hspi1.Instance->CR1, SPI_CR1_BR, prescaler);
    hspi1.Init.BaudRatePrescaler = prescaler;
    __HAL_SPI_ENABLE(&hspi1);
}

static bool spi_send(const uint8_t *data, uint16_t len) {
    if (HAL_SPI_Transmit(&hspi1, data, len, LCD_SPI_TIMEOUT_MS) != HAL_OK) {
        errors++;
        return false;
    }
    bytes_sent += len;
    return true;
}

// one command byte, then optional parameter bytes, all inside one CS frame
static bool lcd_command(uint8_t cmd, const uint8_t *params, uint16_t n) {
    bool ok;
    cs_low();
    dc_command();
    ok = spi_send(&cmd, 1);
    dc_data();
    if (ok && (n > 0u)) {
        ok = spi_send(params, n);
    }
    cs_high();
    return ok;
}

// a read: command out, then len bytes clocked in at the slow read speed; the
// panel ignores what is on SDI meanwhile, 0xFF keeps the line quiet
static bool lcd_read(uint8_t cmd, uint8_t *out, uint16_t len) {
    static const uint8_t idle[4] = { 0xFF, 0xFF, 0xFF, 0xFF };
    bool ok;

    spi_prescaler(LCD_READ_PRESCALER);
    cs_low();
    dc_command();
    ok = spi_send(&cmd, 1);
    dc_data();
    if (ok) {
        if (HAL_SPI_TransmitReceive(&hspi1, idle, out, len, LCD_SPI_TIMEOUT_MS) != HAL_OK) {
            errors++;
            ok = false;
        }
    }
    cs_high();
    spi_prescaler(LCD_WRITE_PRESCALER);
    return ok;
}

// RESX low for at least 10 us, then 120 ms before anything is sent: p230
// note 7 says 5 ms before a command and 120 ms before sleep out, so waiting
// the longer one once covers both
static void lcd_reset(void) {
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(120);
}

// ---- gate --------------------------------------------------------------------

lcd_status_t lcd_probe(lcd_probe_t *out) {
    uint8_t v = 0;
    uint8_t colmod = LCD_COLMOD_16BIT;
    uint32_t w;

    if (out == NULL) {
        return LCD_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));
    reads_work = false;
    cs_high();
    spi_prescaler(LCD_WRITE_PRESCALER);
    lcd_reset();

    if (!lcd_read(LCD_CMD_RDDPM, &v, 1)) {
        return LCD_ERR_SPI;
    }
    out->power_mode = v;

    if (!lcd_read(LCD_CMD_RDPIXFMT, &v, 1)) {
        return LCD_ERR_SPI;
    }
    out->pixfmt_reset = v;

    if (!lcd_read(LCD_CMD_RDID4, out->raw_id4, 4)) {
        return LCD_ERR_SPI;
    }
    // one dummy clock then 24 bits: the value sits 7 bits down in the 32
    // that came back. the other reading is for a panel that pads a whole byte
    w = ((uint32_t)out->raw_id4[0] << 24) | ((uint32_t)out->raw_id4[1] << 16) |
        ((uint32_t)out->raw_id4[2] << 8) | (uint32_t)out->raw_id4[3];
    out->id4_dummy_bit = (w >> 7) & 0xFFFFFFu;
    out->id4_dummy_byte = w & 0xFFFFFFu;
    out->id_ok = (out->id4_dummy_bit == LCD_ID4_EXPECTED) ||
                 (out->id4_dummy_byte == LCD_ID4_EXPECTED);

    // write-then-read: proves the outbound path even on a panel whose ID4 is
    // blanked out (p87 note 2 lets the vendor do that)
    if (!lcd_command(LCD_CMD_COLMOD, &colmod, 1)) {
        return LCD_ERR_SPI;
    }
    if (!lcd_read(LCD_CMD_RDPIXFMT, &v, 1)) {
        return LCD_ERR_SPI;
    }
    out->pixfmt_set = v;
    out->pixfmt_ok = (out->pixfmt_reset == LCD_PIXFMT_RESET) &&
                     (out->pixfmt_set == LCD_PIXFMT_16BIT);

    // a module that never drives SDO answers FF to everything; then neither
    // gate can pass and every later read-back is just as blind
    reads_work = out->id_ok || out->pixfmt_ok;

    return reads_work ? LCD_OK : LCD_ERR_ID;
}

// ---- init --------------------------------------------------------------------

static void lcd_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t p[4];
    p[0] = (uint8_t)(x0 >> 8);
    p[1] = (uint8_t)x0;
    p[2] = (uint8_t)(x1 >> 8);
    p[3] = (uint8_t)x1;
    lcd_command(LCD_CMD_CASET, p, 4);
    p[0] = (uint8_t)(y0 >> 8);
    p[1] = (uint8_t)y0;
    p[2] = (uint8_t)(y1 >> 8);
    p[3] = (uint8_t)y1;
    lcd_command(LCD_CMD_PASET, p, 4);
}

lcd_status_t lcd_init(void) {
    uint8_t v;
    static const uint8_t zeros[1024];

    cs_high();
    spi_prescaler(LCD_WRITE_PRESCALER);
    lcd_backlight_set(0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    lcd_reset();

    // sleep out, then 120 ms: p101 says 5 ms before the next command and
    // 120 ms to actually be awake
    if (!lcd_command(LCD_CMD_SLPOUT, NULL, 0)) {
        return LCD_ERR_SPI;
    }
    HAL_Delay(120);

    v = LCD_COLMOD_16BIT;
    if (!lcd_command(LCD_CMD_COLMOD, &v, 1)) {
        return LCD_ERR_SPI;
    }
    v = LCD_MADCTL;
    if (!lcd_command(LCD_CMD_MADCTL, &v, 1)) {
        return LCD_ERR_SPI;
    }

    // frame memory is random at power-on (p114), so paint it black before the
    // panel is switched on; 150 KB at 42 MBit/s is about 30 ms, once
    lcd_window(0, 0, LCD_WIDTH - 1u, LCD_HEIGHT - 1u);
    cs_low();
    dc_command();
    v = LCD_CMD_RAMWR;
    if (!spi_send(&v, 1)) {
        cs_high();
        return LCD_ERR_SPI;
    }
    dc_data();
    for (uint32_t sent = 0; sent < (uint32_t)LCD_WIDTH * LCD_HEIGHT * 2u; sent += sizeof(zeros)) {
        if (!spi_send(zeros, sizeof(zeros))) {
            cs_high();
            return LCD_ERR_SPI;
        }
    }
    cs_high();

    if (!lcd_command(LCD_CMD_DISPON, NULL, 0)) {
        return LCD_ERR_SPI;
    }

    // one read to confirm the settings landed, but only where reads work at
    // all - on a panel that cannot be read the colour bars are the proof
    if (reads_work) {
        if (!lcd_read(LCD_CMD_RDPIXFMT, &v, 1)) {
            return LCD_ERR_SPI;
        }
        if (v != LCD_PIXFMT_16BIT) {
            return LCD_ERR_ID;
        }
    }
    return LCD_OK;
}

// ---- pixels ------------------------------------------------------------------

void lcd_write_band(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pixels) {
    uint8_t cmd = LCD_CMD_RAMWR;
    uint32_t n = (uint32_t)w * (uint32_t)h * 2u;

    if ((pixels == NULL) || (w <= 0) || (h <= 0) || (n > 65535u)) {
        errors++;
        return;
    }
    lcd_window((uint16_t)x, (uint16_t)y, (uint16_t)(x + w - 1), (uint16_t)(y + h - 1));
    cs_low();
    dc_command();
    if (spi_send(&cmd, 1)) {
        dc_data();
        spi_send((const uint8_t *)pixels, (uint16_t)n);
    }
    cs_high();
}

// ---- backlight ---------------------------------------------------------------

void lcd_backlight_set(uint16_t permille) {
    bl_target = (permille > 1000u) ? 1000u : permille;
}

void lcd_backlight_tick(void) {
    uint32_t now = HAL_GetTick();
    if ((now - bl_last_tick) < LCD_BL_STEP_MS) {
        return;
    }
    bl_last_tick = now;
    if (bl_now == bl_target) {
        return;
    }
    if (bl_now < bl_target) {
        bl_now = (uint16_t)(((bl_target - bl_now) > LCD_BL_STEP) ? (bl_now + LCD_BL_STEP) : bl_target);
    } else {
        bl_now = (uint16_t)(((bl_now - bl_target) > LCD_BL_STEP) ? (bl_now - LCD_BL_STEP) : bl_target);
    }
    // TIM1 counts 0..999, so permille maps straight onto the compare value
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, (bl_now > 999u) ? 999u : bl_now);
}

uint32_t lcd_bytes_sent(void) {
    return bytes_sent;
}

uint32_t lcd_errors(void) {
    return errors;
}
