//
// BSEC state on the card - public interface
//
// - BSEC.BIN, 1 KB, two slots (air_slot.h). made once at boot if it is
//   missing, so the FAT is only touched with the car parked; after that every
//   save rewrites one 512-byte sector in place
// - loaded at boot, saved by the log task - the only task that touches FatFs
// - only a card that was there at boot gets saved to: a card that turns up
//   mid-drive may hold a good state this boot never loaded, and a fresh state
//   written over it would throw days of learning away
//

#ifndef AIR_STORE_H
#define AIR_STORE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    AIR_STORE_NO_CARD = 0,  // no card at boot - nothing loaded, nothing saved this drive
    AIR_STORE_NEW,          // there was no file - made one, nothing to load
    AIR_STORE_EMPTY,        // the file is there but neither slot is good
    AIR_STORE_LOADED,       // the newer good slot was read
    AIR_STORE_ERR,          // the file could not be made or read - no saves this drive
} air_store_boot_t;

typedef struct {
    air_store_boot_t boot;
    bool     armed;           // saves allowed this drive
    uint32_t saves;
    uint32_t errors;
    uint32_t last_save_tick;  // HAL tick of the last good save; 0 = none yet
    uint8_t  fatfs_err;       // the last FRESULT that went wrong
} air_store_stats_t;

// boot only, after session_log_init has mounted the card. state needs
// AIR_STATE_MAX bytes; len is 0 unless the result is AIR_STORE_LOADED
air_store_boot_t air_store_load(uint8_t *state, uint16_t *len);

// write the state into the older slot; false if the card is not there, the
// boot check did not allow it, or the write failed
bool air_store_save(const uint8_t *state, uint16_t len);

void air_store_get_stats(air_store_stats_t *out);

#endif // AIR_STORE_H
