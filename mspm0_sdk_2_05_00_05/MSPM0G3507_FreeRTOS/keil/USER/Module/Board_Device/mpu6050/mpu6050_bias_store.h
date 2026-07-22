#ifndef MPU6050_BIAS_STORE_H
#define MPU6050_BIAS_STORE_H

#include <stdbool.h>

#define MPU6050_BIAS_FLASH_ADDRESS 0x0001FC00U

bool mpu6050_bias_load(float gyro_bias_dps[3]);
bool mpu6050_bias_save(const float gyro_bias_dps[3]);

#endif
