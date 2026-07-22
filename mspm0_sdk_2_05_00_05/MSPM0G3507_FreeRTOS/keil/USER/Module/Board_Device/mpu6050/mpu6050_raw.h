#ifndef MPU6050_RAW_H
#define MPU6050_RAW_H

#include <stdint.h>

#define MPU6050_SAMPLE_RATE_HZ 200U

typedef struct {
    /* Sensor counts after the board-axis mapping is applied. */
    int16_t accel[3];
    int16_t temperature;
    int16_t gyro[3];
} mpu6050_raw_sample_t;

typedef struct {
    float accel_g[3];
    float temperature_c;
    float gyro_dps[3];
} mpu6050_sample_t;

int mpu6050_raw_init(void);
int mpu6050_raw_read(mpu6050_raw_sample_t *sample);
void mpu6050_raw_convert(const mpu6050_raw_sample_t *raw,
    mpu6050_sample_t *sample);
void mpu6050_raw_disable_interrupt(void);

#endif
