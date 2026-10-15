//
// platform glue for the BME68x vendor driver
//
// - same idea as lsm6dso_platform.c, but Bosch's driver uses its own
//   callback signatures, so these are not interchangeable with ST's
// - bme68x_ctx bundles the callbacks, the device address and a few fields
//   the driver fills in itself during init
//

#include "bme68x.h"
#include "i2c_bus.h"

// 0x76 - the low address; would be 0x77 if SDO were pulled high
#define BME68X_I2C_ADDR BME68X_I2C_ADDR_LOW

static int8_t bme68x_platform_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    // intf_ptr is Bosch's version of ST's handle - same trick, different name
    uint16_t dev_addr = *(uint16_t *)intf_ptr;
    i2c_bus_status_t status = i2c_bus_read_reg(dev_addr, reg_addr, reg_data, (uint16_t)length);
    return (status == I2C_BUS_OK) ? BME68X_OK : -1;
}

static int8_t bme68x_platform_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr) {
    uint16_t dev_addr = *(uint16_t *)intf_ptr;
    i2c_bus_status_t status = i2c_bus_write_reg(dev_addr, reg_addr, reg_data, (uint16_t)length);
    return (status == I2C_BUS_OK) ? BME68X_OK : -1;
}

static void bme68x_platform_delay_us(uint32_t period, void *intf_ptr) {
    (void)intf_ptr;  // not used, but the signature requires it
    // HAL_Delay only does whole milliseconds, so round up
    HAL_Delay((period + 999) / 1000);
}

static uint16_t bme68x_i2c_addr = BME68X_I2C_ADDR;

struct bme68x_dev bme68x_ctx = {
    .intf     = BME68X_I2C_INTF,
    .amb_temp = 25,  // rough room temperature; feeds the gas heater calculation
    .read     = bme68x_platform_read,
    .write    = bme68x_platform_write,
    .delay_us = bme68x_platform_delay_us,
    .intf_ptr = &bme68x_i2c_addr,
};
