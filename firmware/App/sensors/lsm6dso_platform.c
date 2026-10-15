//
// platform glue for the LSM6DSO vendor driver
//
// - ST's driver knows nothing about STM32 or I2C - it calls function pointers
//   it is given, which is how the same driver works on any board
// - the three callbacks below match the shape ST expects and just forward to
//   i2c_bus
// - lsm6dso_ctx bundles those callbacks plus the device address; every vendor
//   call takes it as its first argument
//

#include "lsm6dso_reg.h"
#include "i2c_bus.h"

// 7-bit address; CS tied to +3.3V puts the part in I2C mode
#define LSM6DSO_I2C_ADDR 0x6A

static int32_t lsm6dso_platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    // handle is whatever was stored in ctx.handle - here a pointer to the address
    uint16_t dev_addr = *(uint16_t *)handle;
    i2c_bus_status_t status = i2c_bus_write_reg(dev_addr, reg, bufp, len);
    // ST's driver expects 0 for success, anything else for failure
    return (status == I2C_BUS_OK) ? 0 : -1;
}

static int32_t lsm6dso_platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    uint16_t dev_addr = *(uint16_t *)handle;
    i2c_bus_status_t status = i2c_bus_read_reg(dev_addr, reg, bufp, len);
    return (status == I2C_BUS_OK) ? 0 : -1;
}

static void lsm6dso_platform_delay(uint32_t ms) {
    HAL_Delay(ms);
}

// lives in RAM so a pointer to it can be handed to the driver
static uint16_t lsm6dso_i2c_addr = LSM6DSO_I2C_ADDR;

stmdev_ctx_t lsm6dso_ctx = {
    .write_reg = lsm6dso_platform_write,
    .read_reg  = lsm6dso_platform_read,
    .mdelay    = lsm6dso_platform_delay,
    .handle    = &lsm6dso_i2c_addr,
};
