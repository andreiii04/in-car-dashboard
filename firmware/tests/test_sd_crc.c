//
// host test for the SD card CRCs
//
// - the expected values come from two places that share nothing with
//   sd_crc.c: the worked examples printed in the SD spec ch4.5, and a
//   textbook long division over a bit array written here, the way the spec
//   describes the maths, not the way the shift-register code does it
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/Inc tests/test_sd_crc.c App/Src/sd_crc.c
//  -o /tmp/test_sd_crc && /tmp/test_sd_crc

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "sd_crc.h"

static int fails = 0;

static void check(const char *what, int ok) {
    printf("  %-46s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        fails++;
    }
}

// remainder of M(x) * x^deg divided by G(x), done literally on bits: lay the
// message out MSB first, append deg zero bits, and wherever a 1 is left at the
// front subtract the generator. this is the definition in the spec, nothing
// cleverer
static uint32_t long_division(const uint8_t *msg, uint32_t len, uint32_t gen, int deg) {
    static uint8_t bits[600 * 8];
    uint32_t n = len * 8;
    for (uint32_t i = 0; i < n; i++) {
        bits[i] = (uint8_t)((msg[i / 8] >> (7 - (i % 8))) & 1u);
    }
    for (int i = 0; i < deg; i++) {
        bits[n + i] = 0;
    }
    n += (uint32_t)deg;
    for (uint32_t i = 0; i + (uint32_t)deg < n; i++) {
        if (bits[i]) {
            for (int j = 0; j <= deg; j++) {
                bits[i + (uint32_t)j] ^= (uint8_t)((gen >> (deg - j)) & 1u);
            }
        }
    }
    uint32_t r = 0;
    for (int i = 0; i < deg; i++) {
        r = (r << 1) | bits[n - (uint32_t)deg + (uint32_t)i];
    }
    return r;
}

#define GEN7  0x89u     // x^7 + x^3 + 1, as bits 1000 1001
#define GEN16 0x11021u  // x^16 + x^12 + x^5 + 1

int main(void) {
    printf("sd_crc tests\n");

    // spec ch4.5 examples: CMD0 with argument 0 has CRC 1001010b, CMD17 with
    // argument 0 has 0101010b, and the CMD17 response 00 11 00 00 09 00 has
    // 0110011b
    const uint8_t cmd0[5]  = { 0x40, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t cmd17[5] = { 0x51, 0x00, 0x00, 0x00, 0x00 };
    const uint8_t resp17[5] = { 0x11, 0x00, 0x00, 0x09, 0x00 };
    check("CRC7 of CMD0 is 0x4A, spec example", sd_crc7(cmd0, 5) == 0x4A);
    check("CRC7 of CMD17 is 0x2A, spec example", sd_crc7(cmd17, 5) == 0x2A);
    check("CRC7 of the CMD17 response is 0x33", sd_crc7(resp17, 5) == 0x33);
    // the byte that actually goes on the wire for CMD0 is the famous 0x95
    check("CMD0 CRC byte on the wire is 0x95", ((sd_crc7(cmd0, 5) << 1) | 1u) == 0x95);
    // CMD8 with 0x1AA is the other constant every SPI init carries: 0x87
    const uint8_t cmd8[5] = { 0x48, 0x00, 0x00, 0x01, 0xAA };
    check("CMD8 CRC byte on the wire is 0x87", ((sd_crc7(cmd8, 5) << 1) | 1u) == 0x87);

    // against long division for a spread of messages
    int agree7 = 1;
    int agree16 = 1;
    uint8_t msg[64];
    uint32_t seed = 12345u;
    for (int t = 0; t < 200; t++) {
        uint32_t len = (uint32_t)(1 + (t % 60));
        for (uint32_t i = 0; i < len; i++) {
            seed = seed * 1103515245u + 12345u;
            msg[i] = (uint8_t)(seed >> 16);
        }
        if (sd_crc7(msg, len) != long_division(msg, len, GEN7, 7)) {
            agree7 = 0;
        }
        if (sd_crc16(msg, len) != long_division(msg, len, GEN16, 16)) {
            agree16 = 0;
        }
    }
    check("CRC7 agrees with long division, 200 messages", agree7);
    check("CRC16 agrees with long division, 200 messages", agree16);

    // a block of 0xFF is what an erased card hands back, and its CRC16 is a
    // number every SD implementation ends up knowing: 0x7FA1
    static uint8_t block[512];
    memset(block, 0xFF, sizeof(block));
    check("CRC16 of 512 x 0xFF is 0x7FA1", sd_crc16(block, 512) == 0x7FA1);
    check("CRC16 of 512 x 0xFF matches long division",
          sd_crc16(block, 512) == long_division(block, 512, GEN16, 16));

    static uint8_t zeros[512];
    memset(zeros, 0, sizeof(zeros));
    check("CRC16 of 512 x 0x00 is 0", sd_crc16(zeros, 512) == 0);

    printf("\n%s\n", fails ? "FAILURES" : "all checks passed");
    return fails ? 1 : 0;
}
