#include "mpu6050_raw.h"

#include <stddef.h>

#include "mpu6050_port.h"

#define MPU6050_ADDRESS             0x68U
#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_FIFO_EN         0x23U
#define MPU6050_REG_INT_PIN_CFG     0x37U
#define MPU6050_REG_INT_ENABLE      0x38U
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_USER_CTRL       0x6AU
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_PWR_MGMT_2      0x6CU
#define MPU6050_REG_WHO_AM_I        0x75U

#define MPU6050_DEVICE_ID           0x68U
#define MPU6050_RESET               0x80U
#define MPU6050_CLOCK_PLL_XGYRO     0x01U
#define MPU6050_DLPF_42HZ           0x03U
#define MPU6050_GYRO_FS_1000DPS     0x10U
#define MPU6050_ACCEL_FS_4G         0x08U
#define MPU6050_INT_ACTIVE_LOW_READ_CLEAR 0x90U
#define MPU6050_INT_DATA_READY      0x01U
#define MPU6050_SAMPLE_DIVIDER      4U
#define MPU6050_BURST_LENGTH        14U

#define MPU6050_ACCEL_LSB_PER_G     8192.0f
#define MPU6050_GYRO_LSB_PER_DPS    32.8f
#define MPU6050_TEMP_LSB_PER_C      340.0f
#define MPU6050_TEMP_OFFSET_C       36.53f

static int write_register(uint8_t reg, uint8_t value)
{
    return mpu6050_port_write(MPU6050_ADDRESS, reg, 1U, &value);
}

static int16_t read_be_i16(const uint8_t *data)
{
    return (int16_t) (((uint16_t) data[0] << 8U) | data[1]);
}

int mpu6050_raw_init(void)
{
    uint8_t who_am_i;

    if (mpu6050_port_init() != 0) {
        return -1;
    }

    if (write_register(MPU6050_REG_PWR_MGMT_1, MPU6050_RESET) != 0) {
        return -1;
    }
    mpu6050_port_delay_ms(100U);

    if ((write_register(MPU6050_REG_PWR_MGMT_1,
             MPU6050_CLOCK_PLL_XGYRO) != 0) ||
        (write_register(MPU6050_REG_PWR_MGMT_2, 0U) != 0)) {
        return -1;
    }
    mpu6050_port_delay_ms(10U);

    if ((mpu6050_port_read(MPU6050_ADDRESS, MPU6050_REG_WHO_AM_I,
             1U, &who_am_i) != 0) ||
        (who_am_i != MPU6050_DEVICE_ID)) {
        return -2;
    }

    if ((write_register(MPU6050_REG_INT_ENABLE, 0U) != 0) ||
        (write_register(MPU6050_REG_FIFO_EN, 0U) != 0) ||
        (write_register(MPU6050_REG_USER_CTRL, 0U) != 0) ||
        (write_register(MPU6050_REG_CONFIG, MPU6050_DLPF_42HZ) != 0) ||
        (write_register(MPU6050_REG_SMPLRT_DIV,
             MPU6050_SAMPLE_DIVIDER) != 0) ||
        (write_register(MPU6050_REG_GYRO_CONFIG,
             MPU6050_GYRO_FS_1000DPS) != 0) ||
        (write_register(MPU6050_REG_ACCEL_CONFIG,
             MPU6050_ACCEL_FS_4G) != 0) ||
        (write_register(MPU6050_REG_INT_PIN_CFG,
             MPU6050_INT_ACTIVE_LOW_READ_CLEAR) != 0) ||
        (write_register(MPU6050_REG_INT_ENABLE,
             MPU6050_INT_DATA_READY) != 0)) {
        return -1;
    }

    return 0;
}

int mpu6050_raw_read(mpu6050_raw_sample_t *sample)
{
    uint8_t data[MPU6050_BURST_LENGTH];

    if (sample == NULL) {
        return -1;
    }

    if (mpu6050_port_read(MPU6050_ADDRESS, MPU6050_REG_ACCEL_XOUT_H,
            MPU6050_BURST_LENGTH, data) != 0) {
        return -1;
    }

    sample->accel[0] = (int16_t) -read_be_i16(&data[0]);
    sample->accel[1] = (int16_t) -read_be_i16(&data[2]);
    sample->accel[2] = read_be_i16(&data[4]);
    sample->temperature = read_be_i16(&data[6]);
    sample->gyro[0] = (int16_t) -read_be_i16(&data[8]);
    sample->gyro[1] = (int16_t) -read_be_i16(&data[10]);
    sample->gyro[2] = read_be_i16(&data[12]);

    return 0;
}

void mpu6050_raw_convert(const mpu6050_raw_sample_t *raw,
    mpu6050_sample_t *sample)
{
    uint8_t axis;

    if ((raw == NULL) || (sample == NULL)) {
        return;
    }

    for (axis = 0U; axis < 3U; axis++) {
        sample->accel_g[axis] =
            (float) raw->accel[axis] / MPU6050_ACCEL_LSB_PER_G;
        sample->gyro_dps[axis] =
            (float) raw->gyro[axis] / MPU6050_GYRO_LSB_PER_DPS;
    }
    sample->temperature_c =
        (float) raw->temperature / MPU6050_TEMP_LSB_PER_C +
        MPU6050_TEMP_OFFSET_C;
}

void mpu6050_raw_disable_interrupt(void)
{
    (void) write_register(MPU6050_REG_INT_ENABLE, 0U);
}
