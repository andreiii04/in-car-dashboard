//
// SD card over SPI - implementation
//
// - a command is 6 bytes: 01 + index, 32-bit argument, CRC7 + end bit (spec
//   7.3.1.1). the card answers with R1, one byte with the top bit clear, after
//   a few idle bytes; R3 and R7 add four more
// - init flow is figure 7-2 in the spec: CMD0 with CS low puts the card in SPI
//   mode, CMD8 asks whether it takes 3.3 V and whether it is a 2.00 card,
//   ACMD41 is repeated until the idle bit clears, CMD58 says whether addresses
//   are bytes or blocks
// - the clock has to be 100-400 kHz until init is done (spec 6.4.1) and may
//   go to 25 MHz after; SPI2 sits on APB1 at 42 MHz, so prescaler 256 gives
//   164 kHz and prescaler 2 gives 21 MHz
// - the card only ever drives MISO while CS is low; with no card at all the
//   pull-up on PB14 makes every byte read 0xFF, which the code treats as
//   silence
//

#include "sd_spi.h"
#include "sd_crc.h"
#include "main.h"
#include "spi.h"
#include <stddef.h>
#include <string.h>

// ---- protocol constants, all from the spec ---------------------------------------

#define SD_CMD0   0u    // GO_IDLE_STATE, 7.3.1.3
#define SD_CMD1   1u    // SEND_OP_COND, for cards older than ACMD41
#define SD_CMD8   8u    // SEND_IF_COND
#define SD_CMD9   9u    // SEND_CSD
#define SD_CMD16  16u   // SET_BLOCKLEN
#define SD_CMD17  17u   // READ_SINGLE_BLOCK
#define SD_CMD24  24u   // WRITE_BLOCK
#define SD_CMD55  55u   // APP_CMD, goes before every ACMD
#define SD_CMD58  58u   // READ_OCR
#define SD_CMD59  59u   // CRC_ON_OFF
#define SD_ACMD41 41u   // SD_SEND_OP_COND

// CMD8 argument, table 4-15: VHS 0001b is 2.7-3.6 V, check pattern 0xAA as the
// spec recommends; the card echoes both back in R7
#define SD_CMD8_ARG       0x000001AAu
#define SD_CMD8_ECHO_MASK 0x00000FFFu

#define SD_ACMD41_HCS     0x40000000u   // bit 30: this host handles high capacity
#define SD_OCR_BUSY       0x80000000u   // bit 31 set once power-up is done, table 5-1
#define SD_OCR_CCS        0x40000000u   // bit 30: high capacity card

// R1 bits, 7.3.2.1, listed from bit 0 up
#define SD_R1_IDLE        0x01u
#define SD_R1_ILLEGAL     0x04u
#define SD_R1_CRC_ERR     0x08u
#define SD_R1_NONE        0xFFu   // nothing answered

#define SD_TOKEN_START    0xFEu   // start block, single read or write, 7.3.3.2
#define SD_DATA_RESP_MASK 0x1Fu   // data response token, 7.3.3.1: xxx0sss1
#define SD_DATA_ACCEPTED  0x05u   // status 010
#define SD_DATA_CRC_ERR   0x0Bu   // status 101
#define SD_DATA_WR_ERR    0x0Du   // status 110

// timeouts: 100 ms read and 250 ms write are the spec's own (4.6.2); the
// other two are host choices the simplified spec does not give
#define SD_READ_TIMEOUT_MS   100u
#define SD_WRITE_TIMEOUT_MS  250u
#define SD_INIT_TIMEOUT_MS   1000u  // host choice: how long ACMD41 may keep saying idle
#define SD_NCR_BYTES         8u     // host choice: idle bytes to wait for a response
#define SD_CMD0_TRIES        10u    // host choice: some cards want CMD0 more than once

// SPI2 on APB1 at 42 MHz
#define SD_INIT_PRESCALER  SPI_BAUDRATEPRESCALER_256   // 164 kHz
#define SD_RUN_PRESCALER   SPI_BAUDRATEPRESCALER_2     // 21 MBit/s; drop to _4 if CRC errors appear

// tell the card to check CRCs on commands and data (CMD59); every command
// carries a real CRC7 anyway, so this costs nothing and catches a bad link
#define SD_CRC_ON 1

#define SD_SPI_TIMEOUT_MS 100u

static bool ready;
static sd_info_t info;
static sd_stats_t stats;

// 0xFF on MOSI while receiving: the card ignores MOSI during a data block, but
// a stray byte that looks like a command could still be taken as one (spec
// 7.2.8), so the line stays high. const, so it lives in flash
static const uint8_t ff_block[512] = { [0 ... 511] = 0xFFu };

// ---- SPI plumbing --------------------------------------------------------------

static void cs_low(void) {
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);
}

// release the card and give it eight clocks to let go of MISO - not in the
// simplified spec, but a card that has just finished a transaction needs the
// clocks to finish its own bookkeeping, and every SPI host does this
static void cs_high(void) {
    uint8_t rx;
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    (void)HAL_SPI_TransmitReceive(&hspi2, ff_block, &rx, 1, SD_SPI_TIMEOUT_MS);
}

static void spi_prescaler(uint32_t prescaler) {
    __HAL_SPI_DISABLE(&hspi2);
    MODIFY_REG(hspi2.Instance->CR1, SPI_CR1_BR, prescaler);
    hspi2.Init.BaudRatePrescaler = prescaler;
    __HAL_SPI_ENABLE(&hspi2);
}

// one byte each way; a failed transfer reads as silence
static uint8_t xfer(uint8_t out) {
    uint8_t in = 0xFF;
    if (HAL_SPI_TransmitReceive(&hspi2, &out, &in, 1, SD_SPI_TIMEOUT_MS) != HAL_OK) {
        return 0xFF;
    }
    return in;
}

// clock idle bytes until the card stops holding the line low
static bool wait_not_busy(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    for (;;) {
        if (xfer(0xFF) == 0xFF) {
            return true;
        }
        if ((HAL_GetTick() - start) > timeout_ms) {
            stats.timeouts++;
            return false;
        }
    }
}

// send one command and return its R1; CS must already be low. any R3/R7
// bytes that follow are the caller's to read
static uint8_t command(uint8_t index, uint32_t arg) {
    uint8_t frame[6];
    frame[0] = (uint8_t)(0x40u | index);
    frame[1] = (uint8_t)(arg >> 24);
    frame[2] = (uint8_t)(arg >> 16);
    frame[3] = (uint8_t)(arg >> 8);
    frame[4] = (uint8_t)arg;
    frame[5] = (uint8_t)((sd_crc7(frame, 5) << 1) | 1u);   // CRC7 then the end bit

    if (HAL_SPI_Transmit(&hspi2, frame, 6, SD_SPI_TIMEOUT_MS) != HAL_OK) {
        return SD_R1_NONE;
    }
    // the response starts with a 0 bit; up to NCR idle bytes may come first
    for (uint8_t i = 0; i < SD_NCR_BYTES; i++) {
        uint8_t r = xfer(0xFF);
        if ((r & 0x80u) == 0u) {
            return r;
        }
    }
    return SD_R1_NONE;
}

static uint32_t read_u32(void) {
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) {
        v = (v << 8) | xfer(0xFF);
    }
    return v;
}

// CMD55 then the application command, 7.3.1.3 table 7-4
static uint8_t app_command(uint8_t index, uint32_t arg) {
    uint8_t r = command(SD_CMD55, 0);
    if ((r & ~SD_R1_IDLE) != 0u) {
        return r;
    }
    return command(index, arg);
}

// wait for the start token of a data block, then take the block and its CRC
static sd_status_t read_data_block(uint8_t *buf, uint16_t len) {
    uint32_t start = HAL_GetTick();
    uint8_t token;

    for (;;) {
        token = xfer(0xFF);
        if (token != 0xFFu) {
            break;
        }
        if ((HAL_GetTick() - start) > SD_READ_TIMEOUT_MS) {
            stats.timeouts++;
            return SD_ERR_TIMEOUT;
        }
    }
    if (token != SD_TOKEN_START) {
        // a data error token instead, 7.3.3.3
        stats.rejected++;
        return SD_ERR_REJECTED;
    }
    if (HAL_SPI_TransmitReceive(&hspi2, ff_block, buf, len, SD_SPI_TIMEOUT_MS) != HAL_OK) {
        return SD_ERR_SPI;
    }
    uint16_t crc = (uint16_t)((uint16_t)xfer(0xFF) << 8);
    crc |= xfer(0xFF);
    if (crc != sd_crc16(buf, len)) {
        stats.crc_errors++;
        return SD_ERR_CRC;
    }
    return SD_OK;
}

// capacity out of the CSD, spec 5.3: version 1.0 stores C_SIZE and a
// multiplier, version 2.0 stores blocks of 512 KB
static uint32_t csd_blocks(const uint8_t *csd) {
    if ((csd[0] >> 6) == 1u) {
        uint32_t c_size = (((uint32_t)csd[7] & 0x3Fu) << 16) | ((uint32_t)csd[8] << 8) | csd[9];
        return (c_size + 1u) * 1024u;
    }
    uint32_t read_bl_len = csd[5] & 0x0Fu;
    uint32_t c_size = (((uint32_t)csd[6] & 0x03u) << 10) | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
    uint32_t c_size_mult = (((uint32_t)csd[9] & 0x03u) << 1) | (csd[10] >> 7);
    uint32_t block_len = 1u << read_bl_len;
    uint32_t blocks = (c_size + 1u) * (1u << (c_size_mult + 2u));
    return blocks * (block_len / 512u);
}

// ---- init ------------------------------------------------------------------------

sd_status_t sd_spi_init(void) {
    uint32_t start = HAL_GetTick();
    uint8_t r1 = SD_R1_NONE;
    uint8_t csd[16];

    ready = false;
    memset(&info, 0, sizeof(info));

    // 6.4.1: at least 74 clocks with CS high before the first command, at the
    // init speed
    spi_prescaler(SD_INIT_PRESCALER);
    HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_SET);
    if (HAL_SPI_Transmit(&hspi2, ff_block, 10, SD_SPI_TIMEOUT_MS) != HAL_OK) {
        return SD_ERR_SPI;
    }

    // CMD0 with CS low is what selects SPI mode (7.2.1); the answer is R1
    // with only the idle bit set
    cs_low();
    for (uint8_t i = 0; i < SD_CMD0_TRIES; i++) {
        r1 = command(SD_CMD0, 0);
        if (r1 == SD_R1_IDLE) {
            break;
        }
    }
    info.r1_cmd0 = r1;
    if (r1 != SD_R1_IDLE) {
        cs_high();
        return SD_ERR_NO_CARD;
    }

    // CMD8: a 2.00 card echoes the argument in R7; an older card says
    // illegal command and goes down the other branch of figure 7-2
    r1 = command(SD_CMD8, SD_CMD8_ARG);
    info.r1_cmd8 = r1;
    if (r1 == SD_R1_IDLE) {
        info.r7 = read_u32();
        if ((info.r7 & SD_CMD8_ECHO_MASK) != SD_CMD8_ARG) {
            cs_high();
            return SD_ERR_UNUSABLE;   // wrong voltage or a garbled echo
        }
        info.v2 = true;
    } else if ((r1 & SD_R1_ILLEGAL) != 0u) {
        info.v2 = false;
    } else {
        cs_high();
        return SD_ERR_UNUSABLE;
    }

    // ACMD41 until the idle bit clears; HCS only means something to a card
    // that took CMD8. a card that calls ACMD41 illegal is an old MMC and gets
    // CMD1 instead
    for (;;) {
        uint32_t arg = info.v2 ? SD_ACMD41_HCS : 0u;
        r1 = app_command(SD_ACMD41, arg);
        info.acmd41_tries++;
        if ((r1 & SD_R1_ILLEGAL) != 0u) {
            r1 = command(SD_CMD1, 0);
        }
        if (r1 == 0x00u) {
            break;
        }
        if ((r1 & ~SD_R1_IDLE) != 0u) {
            cs_high();
            return SD_ERR_UNUSABLE;
        }
        if ((HAL_GetTick() - start) > SD_INIT_TIMEOUT_MS) {
            cs_high();
            stats.timeouts++;
            return SD_ERR_TIMEOUT;
        }
    }

    // CMD58: the OCR, where bit 30 says block or byte addressing (5.1)
    r1 = command(SD_CMD58, 0);
    if (r1 != 0x00u) {
        cs_high();
        return SD_ERR_UNUSABLE;
    }
    info.ocr = read_u32();
    info.high_capacity = info.v2 && ((info.ocr & SD_OCR_BUSY) != 0u) &&
                         ((info.ocr & SD_OCR_CCS) != 0u);

    // a standard capacity card wants the block length set; high capacity is
    // fixed at 512 whatever is asked (table 7-3, CMD16)
    if (!info.high_capacity) {
        if (command(SD_CMD16, 512) != 0x00u) {
            cs_high();
            return SD_ERR_UNUSABLE;
        }
    }

#if SD_CRC_ON
    (void)command(SD_CMD59, 1);   // CRC option bit 0 = on; a card that ignores it just does
#endif

    // the CSD comes as a 16-byte data block (7.2.6); optional, capacity only
    if (command(SD_CMD9, 0) == 0x00u) {
        if (read_data_block(csd, 16) == SD_OK) {
            info.blocks = csd_blocks(csd);
        }
    }
    cs_high();

    spi_prescaler(SD_RUN_PRESCALER);
    info.init_ms = HAL_GetTick() - start;
    ready = true;
    return SD_OK;
}

bool sd_spi_ready(void) {
    return ready;
}

// ---- blocks ----------------------------------------------------------------------

// byte address for standard capacity, block number for high capacity (table
// 7-3 note 10)
static uint32_t block_address(uint32_t lba) {
    return info.high_capacity ? lba : (lba * 512u);
}

sd_status_t sd_spi_read_block(uint32_t lba, uint8_t *buf) {
    sd_status_t s;
    if (buf == NULL) {
        return SD_ERR_ARG;
    }
    if (!ready) {
        return SD_ERR_NOT_READY;
    }
    cs_low();
    if (!wait_not_busy(SD_WRITE_TIMEOUT_MS)) {
        cs_high();
        return SD_ERR_TIMEOUT;
    }
    if (command(SD_CMD17, block_address(lba)) != 0x00u) {
        cs_high();
        stats.rejected++;
        return SD_ERR_REJECTED;
    }
    s = read_data_block(buf, 512);
    cs_high();
    if (s == SD_OK) {
        stats.reads++;
    }
    return s;
}

sd_status_t sd_spi_write_block(uint32_t lba, const uint8_t *buf) {
    uint8_t crc[2];
    uint8_t resp;
    uint32_t t0;

    if (buf == NULL) {
        return SD_ERR_ARG;
    }
    if (!ready) {
        return SD_ERR_NOT_READY;
    }
    cs_low();
    if (!wait_not_busy(SD_WRITE_TIMEOUT_MS)) {
        cs_high();
        return SD_ERR_TIMEOUT;
    }
    if (command(SD_CMD24, block_address(lba)) != 0x00u) {
        cs_high();
        stats.rejected++;
        return SD_ERR_REJECTED;
    }

    // one idle byte, the start token, the block, its CRC16 (figure 7-6)
    (void)xfer(0xFF);
    (void)xfer(SD_TOKEN_START);
    if (HAL_SPI_Transmit(&hspi2, buf, 512, SD_SPI_TIMEOUT_MS) != HAL_OK) {
        cs_high();
        return SD_ERR_SPI;
    }
    uint16_t c = sd_crc16(buf, 512);
    crc[0] = (uint8_t)(c >> 8);
    crc[1] = (uint8_t)c;
    (void)HAL_SPI_Transmit(&hspi2, crc, 2, SD_SPI_TIMEOUT_MS);

    // the data response token says whether the block was taken (7.3.3.1)
    resp = xfer(0xFF) & SD_DATA_RESP_MASK;
    if (resp != SD_DATA_ACCEPTED) {
        cs_high();
        if (resp == SD_DATA_CRC_ERR) {
            stats.crc_errors++;
            return SD_ERR_CRC;
        }
        stats.rejected++;
        return SD_ERR_REJECTED;
    }

    // the card holds MISO low while it programs the block, up to 250 ms by
    // the spec; this is the one wait in the whole loop that is the card's
    // to decide, so its worst case is kept for the console
    t0 = HAL_GetTick();
    bool done = wait_not_busy(SD_WRITE_TIMEOUT_MS);
    uint32_t busy = HAL_GetTick() - t0;
    if (busy > stats.busy_max_ms) {
        stats.busy_max_ms = busy;
    }
    cs_high();
    if (!done) {
        return SD_ERR_TIMEOUT;
    }
    stats.writes++;
    return SD_OK;
}

sd_status_t sd_spi_wait_ready(void) {
    if (!ready) {
        return SD_ERR_NOT_READY;
    }
    cs_low();
    bool ok = wait_not_busy(SD_WRITE_TIMEOUT_MS);
    cs_high();
    return ok ? SD_OK : SD_ERR_TIMEOUT;
}

const sd_info_t *sd_spi_info(void) {
    return &info;
}

void sd_spi_get_stats(sd_stats_t *out) {
    if (out != NULL) {
        *out = stats;
    }
}
