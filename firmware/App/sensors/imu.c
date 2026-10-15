//
// IMU module - implementation
//
// - sits on top of ST's vendor driver, which sits on top of i2c_bus
// - imu_init sets the part up once at startup; imu_read pulls one sample
// - configured at 104 Hz, ±4 g accel, ±500 dps gyro; the reasoning behind
//   those three numbers is written up in docs/notes.md
//

#include "imu.h"
#include "lsm6dso_reg.h"
#include "stm32f4xx_hal.h"
#include "main.h"   // ACC_INT_Pin, CubeMX generated
#include <stddef.h>

// defined in lsm6dso_platform.c - holds the read/write callbacks and the address
extern stmdev_ctx_t lsm6dso_ctx;
// only the EXTI handler writes this; main compares it against its own copy
static volatile uint32_t int1_edges;

#define IMU_WHO_AM_I_EXPECTED 0x6CU  // fixed value, datasheet ch9.11
#define IMU_RESET_TIMEOUT_MS  100U

// full scale = the biggest value the sensor can report
// ODR = output data rate, how many samples per second it produces
// keep the full scale defines matching the conversion helpers used in
// imu_read, or every reading comes out silently scaled wrong
#define IMU_XL_FS  LSM6DSO_4g            // ±4 g,     0.122 mg per count
#define IMU_GY_FS  LSM6DSO_500dps        // ±500 dps, 17.50 mdps per count
#define IMU_XL_ODR LSM6DSO_XL_ODR_104Hz
#define IMU_GY_ODR LSM6DSO_GY_ODR_104Hz

imu_status_t imu_init(void) {
    uint8_t who_am_i = 0;
    uint8_t reset_pending = 0;
    uint32_t start_tick;

    // WHO_AM_I first - one read that proves the bus, the address and the part
    // number at once; if it fails nothing below is worth trying
    if (lsm6dso_device_id_get(&lsm6dso_ctx, &who_am_i) != 0) {
        return IMU_ERR_BUS;
    }
    if (who_am_i != IMU_WHO_AM_I_EXPECTED) {
        return IMU_ERR_ID;
    }

    // software reset puts every setting back to its power-on default, so a
    // restart from the debugger behaves the same as a real power cycle
    if (lsm6dso_reset_set(&lsm6dso_ctx, PROPERTY_ENABLE) != 0) {
        return IMU_ERR_BUS;
    }
    start_tick = HAL_GetTick();
    do {
        // the reset bit clears itself once the part is done
        if (lsm6dso_reset_get(&lsm6dso_ctx, &reset_pending) != 0) {
            return IMU_ERR_BUS;
        }
        if ((HAL_GetTick() - start_tick) > IMU_RESET_TIMEOUT_MS) {
            return IMU_ERR_TIMEOUT;
        }
    } while (reset_pending);

    // I3C shares the same pins as I2C; left on, a glitch can knock the part
    // off the bus until it is power cycled
    if (lsm6dso_i3c_disable_set(&lsm6dso_ctx, LSM6DSO_I3C_DISABLE) != 0) {
        return IMU_ERR_BUS;
    }

    // block data update - hold the output registers still between the low and
    // high byte read, so two different samples never get mixed into one value
    if (lsm6dso_block_data_update_set(&lsm6dso_ctx, PROPERTY_ENABLE) != 0) {
        return IMU_ERR_BUS;
    }

    // step the register address automatically during a multi-byte read;
    // the 6-byte reads in imu_read need this
    if (lsm6dso_auto_increment_set(&lsm6dso_ctx, PROPERTY_ENABLE) != 0) {
        return IMU_ERR_BUS;
    }

    if (lsm6dso_xl_full_scale_set(&lsm6dso_ctx, IMU_XL_FS) != 0) {
        return IMU_ERR_BUS;
    }
    if (lsm6dso_gy_full_scale_set(&lsm6dso_ctx, IMU_GY_FS) != 0) {
        return IMU_ERR_BUS;
    }

    // ODR goes last - setting it is what wakes the sensor up and starts
    // measuring, so everything else is already in place by then
    if (lsm6dso_xl_data_rate_set(&lsm6dso_ctx, IMU_XL_ODR) != 0) {
        return IMU_ERR_BUS;
    }
    if (lsm6dso_gy_data_rate_set(&lsm6dso_ctx, IMU_GY_ODR) != 0) {
        return IMU_ERR_BUS;
    }

    return IMU_OK;
}

imu_status_t imu_read(imu_sample_t *out) {
    int16_t raw_accel[3];
    int16_t raw_gyro[3];

    if (out == NULL) {
        return IMU_ERR_ARG;
    }

    // raw counts, not real units yet
    if (lsm6dso_acceleration_raw_get(&lsm6dso_ctx, raw_accel) != 0) {
        return IMU_ERR_BUS;
    }
    if (lsm6dso_angular_rate_raw_get(&lsm6dso_ctx, raw_gyro) != 0) {
        return IMU_ERR_BUS;
    }

    // the vendor helpers do counts -> mg and counts -> mdps for the full scale
    // set above; /1000 gets to g and deg/s
    out->accel_x_g = lsm6dso_from_fs4_to_mg(raw_accel[0]) / 1000.0f;
    out->accel_y_g = lsm6dso_from_fs4_to_mg(raw_accel[1]) / 1000.0f;
    out->accel_z_g = lsm6dso_from_fs4_to_mg(raw_accel[2]) / 1000.0f;

    out->gyro_x_dps = lsm6dso_from_fs500_to_mdps(raw_gyro[0]) / 1000.0f;
    out->gyro_y_dps = lsm6dso_from_fs500_to_mdps(raw_gyro[1]) / 1000.0f;
    out->gyro_z_dps = lsm6dso_from_fs500_to_mdps(raw_gyro[2]) / 1000.0f;

    return IMU_OK;
}

bool imu_data_ready(void) {
    uint8_t xl_ready = 0;
    uint8_t gy_ready = 0;

    if (lsm6dso_xl_flag_data_ready_get(&lsm6dso_ctx, &xl_ready) != 0) {
        return false;
    }
    if (lsm6dso_gy_flag_data_ready_get(&lsm6dso_ctx, &gy_ready) != 0) {
        return false;
    }

    return (xl_ready != 0) && (gy_ready != 0);
}

imu_status_t imu_int1_enable(void) {
    lsm6dso_int1_ctrl_t int1_ctrl;

    // INT1_CTRL, 0x0D, datasheet ch9.9 - read then write back, so any other bit
    // in there stays as it was. the driver's pin_int1_route helper does the same
    // job but writes nine registers, INT2 and the FSM pages included
    if (lsm6dso_read_reg(&lsm6dso_ctx, LSM6DSO_INT1_CTRL,
            (uint8_t *)&int1_ctrl, 1) != 0) {
        return IMU_ERR_BUS;
    }

    // accel data-ready only - the pin is the OR of everything enabled on it, so
    // switching the gyro on as well would give two edges per sample
    int1_ctrl.int1_drdy_xl = 1;

    if (lsm6dso_write_reg(&lsm6dso_ctx, LSM6DSO_INT1_CTRL,
            (uint8_t *)&int1_ctrl, 1) != 0) {
        return IMU_ERR_BUS;
    }

    return IMU_OK;
}

uint32_t imu_int1_count(void) {
    return int1_edges;
}

// HAL calls this out of EXTI15_10_IRQHandler once it has cleared the pending
// bit; keep it to a few instructions - USART1 feeds the GPS ring one byte at a
// time, and no printf is allowed in an ISR
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ACC_INT_Pin) {
        int1_edges++;
    }
}
