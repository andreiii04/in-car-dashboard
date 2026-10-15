//
// BSEC state slot - public interface
//
// - BSEC.BIN on the card holds two 512-byte slots, written turn about, so a
//   power cut mid-write can only tear the one being written and the other
//   still holds the last good state
// - one slot: "AIRS", a format byte, a sequence number, the length, a CRC16
//   over all of that and the state, then the state, then zeros
// - turning a state into a slot and back, nothing else - no HAL and no FatFs,
//   so tests/test_air_slot.c checks it on the mac. air_store.c does the card
//

#ifndef AIR_SLOT_H
#define AIR_SLOT_H

#include <stdbool.h>
#include <stdint.h>

#define AIR_SLOT_LEN     512u   // one sector
#define AIR_SLOT_HEADER  16u    // magic, format, sequence, length, CRC
#define AIR_SLOT_DATA    (AIR_SLOT_LEN - AIR_SLOT_HEADER)

// fill one slot; false if the state does not fit
bool air_slot_pack(uint8_t slot[AIR_SLOT_LEN], uint32_t seq,
                   const uint8_t *state, uint16_t len);

// read one slot back; false if it is empty, not ours, or the CRC is off.
// state needs room for max bytes
bool air_slot_unpack(const uint8_t slot[AIR_SLOT_LEN], uint32_t *seq,
                     uint8_t *state, uint16_t *len, uint16_t max);

// which slot to read: 0 or 1 for the newer good one, -1 if neither is good.
// the sequence comparison survives the counter wrapping
int air_slot_newest(bool ok0, uint32_t seq0, bool ok1, uint32_t seq1);

#endif // AIR_SLOT_H
