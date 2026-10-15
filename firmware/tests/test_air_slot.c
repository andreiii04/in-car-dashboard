//
// host test for the BSEC state slot
//
// - the expected header bytes are written out by hand below, and the CRC is
//   worked out here by a plain bit-by-bit loop over the CRC16 the SD spec
//   uses (x^16 + x^12 + x^5 + 1, start 0) - not by calling sd_crc16 again
// - the torn-write cases are the reason the slot exists: half a slot on the
//   card has to read as bad, never as a state
// - not part of the firmware build, CMakeLists never sees this file
//

// how to run
//  cc -std=c11 -Wall -Wextra -I App/storage tests/test_air_slot.c
//  App/storage/air_slot.c App/storage/sd_crc.c -o /tmp/test_air_slot
//  && /tmp/test_air_slot

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "air_slot.h"

static int fails = 0;

static void check(const char *what, int ok) {
    printf("  %-46s %s\n", what, ok ? "ok" : "FAIL");
    if (!ok) {
        fails++;
    }
}

// CRC16, one bit at a time, MSB first - the textbook shift-and-xor
static uint16_t crc16_bits(const uint8_t *p, uint32_t n) {
    uint16_t crc = 0;
    for (uint32_t i = 0; i < n; i++) {
        for (int b = 7; b >= 0; b--) {
            int in = (p[i] >> b) & 1;
            int top = (crc >> 15) & 1;
            crc = (uint16_t)(crc << 1);
            if (in ^ top) {
                crc ^= 0x1021;
            }
        }
    }
    return crc;
}

int main(void) {
    static uint8_t slot[AIR_SLOT_LEN];
    static uint8_t other[AIR_SLOT_LEN];
    uint8_t state[180];
    uint8_t back[180];
    uint32_t seq = 0;
    uint16_t len = 0;

    printf("air_slot tests\n");

    for (int i = 0; i < 180; i++) {
        state[i] = (uint8_t)(i * 7 + 3);
    }

    // ---- the layout, byte by byte ------------------------------------------
    check("a full-size state packs", air_slot_pack(slot, 0x12345678u, state, 180));
    check("magic is AIRS", memcmp(slot, "AIRS", 4) == 0);
    check("format byte is 1", slot[4] == 1);
    check("sequence is little-endian at 8",
          slot[8] == 0x78 && slot[9] == 0x56 && slot[10] == 0x34 && slot[11] == 0x12);
    check("length is little-endian at 12", slot[12] == 180 && slot[13] == 0);
    check("state starts at 16", memcmp(slot + 16, state, 180) == 0);
    int pad_zero = 1;
    for (uint32_t i = 16 + 180; i < AIR_SLOT_LEN; i++) {
        pad_zero &= (slot[i] == 0);
    }
    check("the rest is zeros", pad_zero);

    // the CRC covers bytes 0-13 then the state
    uint8_t covered[14 + 180];
    memcpy(covered, slot, 14);
    memcpy(covered + 14, state, 180);
    uint16_t want = crc16_bits(covered, sizeof(covered));
    check("CRC matches a bit-by-bit CRC16",
          slot[14] == (uint8_t)want && slot[15] == (uint8_t)(want >> 8));
    uint8_t spec[9] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };
    check("the reference CRC gives 31C3 for \"123456789\"", crc16_bits(spec, 9) == 0x31C3);

    // ---- round trip ---------------------------------------------------------
    memset(back, 0, sizeof(back));
    check("a good slot unpacks", air_slot_unpack(slot, &seq, back, &len, 180));
    check("sequence comes back", seq == 0x12345678u);
    check("length comes back", len == 180);
    check("state comes back", memcmp(back, state, 180) == 0);
    check("a short state packs", air_slot_pack(other, 3, state, 1));
    check("a short state unpacks", air_slot_unpack(other, &seq, back, &len, 180) && len == 1);

    // ---- what must be refused -----------------------------------------------
    check("an empty state does not pack", !air_slot_pack(other, 1, state, 0));
    check("a state past the slot does not pack", !air_slot_pack(other, 1, state, AIR_SLOT_DATA + 1u));
    memset(other, 0, sizeof(other));
    check("a zeroed slot is refused", !air_slot_unpack(other, &seq, back, &len, 180));

    memcpy(other, slot, AIR_SLOT_LEN);
    other[16 + 90] ^= 0x01;
    check("one flipped state bit is refused", !air_slot_unpack(other, &seq, back, &len, 180));

    memcpy(other, slot, AIR_SLOT_LEN);
    other[9] ^= 0x80;
    check("a flipped sequence bit is refused", !air_slot_unpack(other, &seq, back, &len, 180));

    memcpy(other, slot, AIR_SLOT_LEN);
    memset(other + 100, 0, AIR_SLOT_LEN - 100);
    check("a write torn halfway is refused", !air_slot_unpack(other, &seq, back, &len, 180));

    memcpy(other, slot, AIR_SLOT_LEN);
    other[4] = 2;
    check("another format is refused", !air_slot_unpack(other, &seq, back, &len, 180));

    check("a state bigger than the buffer is refused", !air_slot_unpack(slot, &seq, back, &len, 179));

    memcpy(other, slot, AIR_SLOT_LEN);
    other[AIR_SLOT_LEN - 1] = 0xFF;
    check("padding is not covered - still good", air_slot_unpack(other, &seq, back, &len, 180));

    // ---- which slot is newer ------------------------------------------------
    check("both good, 1 later", air_slot_newest(true, 5, true, 6) == 1);
    check("both good, 0 later", air_slot_newest(true, 6, true, 5) == 0);
    check("the counter wrapping still counts as later",
          air_slot_newest(true, 0xFFFFFFFFu, true, 0) == 1);
    check("only 0 good", air_slot_newest(true, 9, false, 10) == 0);
    check("only 1 good", air_slot_newest(false, 10, true, 9) == 1);
    check("neither good", air_slot_newest(false, 0, false, 0) == -1);

    printf("\n%s\n", fails ? "SOME CHECKS FAILED" : "all checks passed");
    return fails != 0;
}
