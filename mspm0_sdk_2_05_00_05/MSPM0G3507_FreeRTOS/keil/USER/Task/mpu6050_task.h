#ifndef MPU6050_TASK_H
#define MPU6050_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "mpu6050_raw.h"

typedef struct {
    float x;
    float y;
    float z;
} mpu6050_vector3f_t;

typedef struct {
    mpu6050_raw_sample_t raw;
    /* Converted acceleration in g. */
    mpu6050_vector3f_t accel;
    /* Flash-bias-corrected angular velocity in degrees/s. */
    mpu6050_vector3f_t gyro;
    float temperature_c;
    float quaternion[4];
    float roll;
    float pitch;
    float yaw;
} Imu_t;

typedef enum {
    MPU6050_STATUS_UNINITIALIZED = 0,
    MPU6050_STATUS_INITIALIZING,
    MPU6050_STATUS_CALIBRATING,
    MPU6050_STATUS_RUNNING,
    MPU6050_STATUS_DEVICE_NOT_FOUND,
    MPU6050_STATUS_DATA_TIMEOUT,
    MPU6050_STATUS_I2C_ERROR,
    MPU6050_STATUS_CALIBRATION_MOTION
} mpu6050_status_t;

typedef struct {
    Imu_t imu;
    float gyro_bias_dps[3];
    uint32_t timestamp_ms;
    uint32_t sample_count;
    uint32_t interrupt_count;
    uint32_t i2c_error_count;
    uint32_t timeout_count;
    uint32_t flash_error_count;
    mpu6050_status_t status;
    bool valid;
    bool bias_from_flash;
    bool bias_persisted;
} mpu6050_snapshot_t;

extern Imu_t mpu6050;
extern volatile mpu6050_status_t mpu6050_status;

void mpu6050_task_create(void);
bool mpu6050_get_snapshot(mpu6050_snapshot_t *snapshot);
mpu6050_status_t mpu6050_get_status(void);
bool mpu6050_request_calibration(void);

#endif
