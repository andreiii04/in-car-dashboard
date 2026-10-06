//
// SD card CRCs - implementation
//
// - bit-serial, no lookup tables; 512 bytes of CRC16 is about 4000 shift steps,
//   well under 50 us at 168 MHz, once per block
//

#include "sd_crc.h"

// the polynomial is kept left-aligned in an 8-bit register, so x^7 lands on
// bit 7 and the generator x^3 + 1 becomes 0x12 once shifted up by one; the
// answer is read back down one bit at the end
uint8_t sd_crc7(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0;
    while (len-- > 0u) {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8u; i++) {
            crc = (uint8_t)((crc & 0x80u) ? ((crc << 1) ^ 0x12u) : (crc << 1));
        }
    }
    return (uint8_t)(crc >> 1);
}

uint16_t sd_crc16(const uint8_t *data, uint32_t len) {
    uint16_t crc = 0;
    while (len-- > 0u) {
        crc ^= (uint16_t)((uint16_t)*data++ << 8);
        for (uint8_t i = 0; i < 8u; i++) {
            crc = (uint16_t)((crc & 0x8000u) ? ((crc << 1) ^ 0x1021u) : (crc << 1));
        }
    }
    return crc;
}
