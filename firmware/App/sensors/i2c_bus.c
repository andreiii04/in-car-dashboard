//
// I2C bus wrapper - implementation
//
// - every sensor read/write on I2C1 goes through these two functions
// - HAL_I2C_Mem_Read/Write already do the send-address-then-data sequence both
//   sensors need, so no point building it by hand
// - HAL error codes get translated here; nothing above this file sees them
//

#include "i2c_bus.h"
#include "stm32f4xx_hal_i2c.h"

// the I2C1 handle CubeMX generates in Core/Src/i2c.c
extern I2C_HandleTypeDef hi2c1;

// how long one transfer may take before giving up
#define I2C_BUS_TIMEOUT_MS 100
// a probe is one address byte, so it needs far less time than a transfer
#define I2C_BUS_PROBE_TRIALS 2
#define I2C_BUS_PROBE_TIMEOUT_MS 5

i2c_bus_status_t i2c_bus_read_reg(uint16_t dev_addr, uint8_t reg_addr,
    uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Read(
        &hi2c1,
        (uint16_t)(dev_addr << 1),  // HAL wants the address shifted; bit 0 is the read/write flag
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,       // both sensors use 8-bit register addresses
        data,
        len,
        I2C_BUS_TIMEOUT_MS
    );

    switch (hal_status) {
        case HAL_OK:      return I2C_BUS_OK;
        case HAL_BUSY:    return I2C_BUS_ERR_BUSY;
        case HAL_TIMEOUT: return I2C_BUS_ERR_TIMEOUT;
        default:          return I2C_BUS_ERR_HAL;
    }
}

i2c_bus_status_t i2c_bus_write_reg(uint16_t dev_addr, uint8_t reg_addr,
    const uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef hal_status = HAL_I2C_Mem_Write(
        &hi2c1,
        (uint16_t)(dev_addr << 1),
        reg_addr,
        I2C_MEMADD_SIZE_8BIT,
        (uint8_t *)data,            // HAL wants a non-const pointer even though it only reads it
        len,
        I2C_BUS_TIMEOUT_MS
    );

    switch (hal_status) {
        case HAL_OK:      return I2C_BUS_OK;
        case HAL_BUSY:    return I2C_BUS_ERR_BUSY;
        case HAL_TIMEOUT: return I2C_BUS_ERR_TIMEOUT;
        default:          return I2C_BUS_ERR_HAL;
    }
}

i2c_bus_status_t i2c_bus_probe(uint16_t dev_addr) {
    // start, address, look at the ACK bit, stop; no register and no data
    HAL_StatusTypeDef hal_status = HAL_I2C_IsDeviceReady(
        &hi2c1,
        (uint16_t)(dev_addr << 1),
        I2C_BUS_PROBE_TRIALS,
        I2C_BUS_PROBE_TIMEOUT_MS
    );

    switch (hal_status) {
    case HAL_OK:      return I2C_BUS_OK;
    case HAL_BUSY:    return I2C_BUS_ERR_BUSY;
    case HAL_TIMEOUT: return I2C_BUS_ERR_TIMEOUT;
    default:          return I2C_BUS_ERR_HAL;
    }
}