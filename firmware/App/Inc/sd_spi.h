//
// SD card over SPI - public interface
//
// - the full-size SD socket on the display module is on SPI2, on its own bus so the
//   display's SDO can never corrupt it
// - this is the card protocol only: init, one block in, one block out. FatFs
//   sits above it through FATFS/Target/user_diskio.c and knows nothing about
//   SPI; nothing above FatFs knows the card exists
// - every command, response and token in here is checked against
//   datasheets/SD_Physical_Layer_Spec.pdf (Simplified, version 2.00), chapter
//   7 for SPI mode; the section is noted next to each one. two numbers the
//   simplified spec leaves blank - how many bytes to wait for a response, and
//   how long ACMD41 may take - are marked as host choices
//

#ifndef SD_SPI_H
#define SD_SPI_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SD_OK = 0,
    SD_ERR_ARG,
    SD_ERR_NOT_READY,   // init has not succeeded yet
    SD_ERR_NO_CARD,     // nothing answered CMD0
    SD_ERR_UNUSABLE,    // answered, but not a card this host can use
    SD_ERR_TIMEOUT,     // the card stopped answering mid-transaction
    SD_ERR_SPI,         // a HAL transfer failed
    SD_ERR_CRC,         // a data block failed its CRC16
    SD_ERR_REJECTED,    // the card refused a command or a block
} sd_status_t;

// what init learned, for the console
typedef struct {
    bool     v2;              // answered CMD8, so a version 2.00 or later card
    bool     high_capacity;   // CCS set in the OCR: addresses are blocks, not bytes
    uint8_t  r1_cmd0;
    uint8_t  r1_cmd8;
    uint32_t r7;              // CMD8 echo, low 12 bits should be 0x1AA
    uint32_t ocr;
    uint32_t acmd41_tries;
    uint32_t init_ms;
    uint32_t blocks;          // capacity in 512-byte blocks from the CSD, 0 if unread
} sd_info_t;

typedef struct {
    uint32_t reads;
    uint32_t writes;
    uint32_t crc_errors;      // blocks that failed CRC16, read or write
    uint32_t timeouts;
    uint32_t rejected;
    uint32_t busy_max_ms;     // the longest the card ever held the line after a write
} sd_stats_t;

// bring the card into SPI mode at 164 kHz, run the init handshake, then go
// to 21 MBit/s; call after MX_SPI2_Init. safe to call again after a failure
sd_status_t sd_spi_init(void);

bool sd_spi_ready(void);

// one 512-byte block; lba is the block number whatever the card type
sd_status_t sd_spi_read_block(uint32_t lba, uint8_t *buf);
sd_status_t sd_spi_write_block(uint32_t lba, const uint8_t *buf);

// wait until the card has finished whatever it was programming
sd_status_t sd_spi_wait_ready(void);

const sd_info_t *sd_spi_info(void);
void sd_spi_get_stats(sd_stats_t *out);

#endif // SD_SPI_H
