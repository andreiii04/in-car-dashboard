//
// I2C bus wrapper - public interface
//
// - thin layer over the STM32 HAL, used by every sensor on I2C1
// - takes 7-bit device addresses; the shift that HAL wants happens inside
// - returns i2c_bus_status_t, so callers never see HAL types
//

#ifndef I2C_BUS_H
#define I2C_BUS_H

#include "stm32f4xx_hal.h"

typedef enum {
    I2C_BUS_OK = 0,
    I2C_BUS_ERR_HAL,
    I2C_BUS_ERR_TIMEOUT,
    I2C_BUS_ERR_BUSY,
} i2c_bus_status_t;

// read len bytes starting at reg_addr; dev_addr is 7-bit
i2c_bus_status_t i2c_bus_read_reg(uint16_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

// write len bytes starting at reg_addr; dev_addr is 7-bit
i2c_bus_status_t i2c_bus_write_reg(uint16_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);

// send just the address and see if anything ACKs; used for the startup scan
i2c_bus_status_t i2c_bus_probe(uint16_t dev_addr);

#endif // I2C_BUS_H
