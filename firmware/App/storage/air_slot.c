//
// BSEC state slot - implementation
//
// - numbers are stored little-endian byte by byte, so the file reads the same
//   whatever packs it - the mac in the tests, the STM32 on the card
// - the CRC is sd_crc16, the same one every card block already carries; here
//   it catches a torn write, which the card's own CRC cannot - the card
//   received exactly what was sent, just not all of it
//

#include "air_slot.h"
#include "sd_crc.h"
#include <string.h>

#define AIR_SLOT_FORMAT 1u

// byte positions inside a slot
#define AT_MAGIC  0u
#define AT_FORMAT 4u
#define AT_SEQ    8u
#define AT_LEN    12u
#define AT_CRC    14u

static const uint8_t air_magic[4] = { 'A', 'I', 'R', 'S' };

static void put32(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

// the CRC covers the header up to itself, then the state - not the padding
static uint16_t slot_crc(const uint8_t *slot, uint16_t len) {
    uint8_t buf[AIR_SLOT_HEADER + AIR_SLOT_DATA];
    memcpy(buf, slot, AT_CRC);
    memcpy(buf + AT_CRC, slot + AIR_SLOT_HEADER, len);
    return sd_crc16(buf, (uint32_t)AT_CRC + len);
}

bool air_slot_pack(uint8_t slot[AIR_SLOT_LEN], uint32_t seq,
                   const uint8_t *state, uint16_t len) {
    if ((state == NULL) || (len == 0u) || (len > AIR_SLOT_DATA)) {
        return false;
    }
    memset(slot, 0, AIR_SLOT_LEN);
    memcpy(slot + AT_MAGIC, air_magic, sizeof(air_magic));
    slot[AT_FORMAT] = AIR_SLOT_FORMAT;
    put32(slot + AT_SEQ, seq);
    slot[AT_LEN] = (uint8_t)len;
    slot[AT_LEN + 1u] = (uint8_t)(len >> 8);
    memcpy(slot + AIR_SLOT_HEADER, state, len);

    uint16_t crc = slot_crc(slot, len);
    slot[AT_CRC] = (uint8_t)crc;
    slot[AT_CRC + 1u] = (uint8_t)(crc >> 8);
    return true;
}

bool air_slot_unpack(const uint8_t slot[AIR_SLOT_LEN], uint32_t *seq,
                     uint8_t *state, uint16_t *len, uint16_t max) {
    if (memcmp(slot + AT_MAGIC, air_magic, sizeof(air_magic)) != 0) {
        return false;
    }
    if (slot[AT_FORMAT] != AIR_SLOT_FORMAT) {
        return false;
    }
    uint16_t n = (uint16_t)(slot[AT_LEN] | (slot[AT_LEN + 1u] << 8));
    if ((n == 0u) || (n > AIR_SLOT_DATA) || (n > max)) {
        return false;
    }
    uint16_t crc = (uint16_t)(slot[AT_CRC] | (slot[AT_CRC + 1u] << 8));
    if (crc != slot_crc(slot, n)) {
        return false;
    }
    *seq = get32(slot + AT_SEQ);
    *len = n;
    memcpy(state, slot + AIR_SLOT_HEADER, n);
    return true;
}

int air_slot_newest(bool ok0, uint32_t seq0, bool ok1, uint32_t seq1) {
    if (ok0 && ok1) {
        // the difference as a signed number: positive means 1 came later,
        // even across 0xFFFFFFFF -> 0
        return ((int32_t)(seq1 - seq0) > 0) ? 1 : 0;
    }
    if (ok0) {
        return 0;
    }
    if (ok1) {
        return 1;
    }
    return -1;
}
