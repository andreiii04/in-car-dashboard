//
// SD card CRCs - public interface
//
// - every SPI command to the card carries a 7-bit CRC and every data block a
//   16-bit one; the card can be told to ignore them, but checking them is the
//   only way to see a marginal 21 MHz link, so they get computed and checked
// - polynomials from the SD Physical Layer spec ch4.5: CRC7 is x^7 + x^3 + 1,
//   CRC16 is the CCITT x^16 + x^12 + x^5 + 1, both starting from zero
// - plain integer maths, no HAL, so test_sd_crc.c runs on the mac against the
//   worked examples printed in the spec
//

#ifndef SD_CRC_H
#define SD_CRC_H

#include <stdint.h>

// 7-bit result in the low bits; the card wants it shifted left once with the
// end bit set, which is the caller's job
uint8_t sd_crc7(const uint8_t *data, uint32_t len);

uint16_t sd_crc16(const uint8_t *data, uint32_t len);

#endif // SD_CRC_H
