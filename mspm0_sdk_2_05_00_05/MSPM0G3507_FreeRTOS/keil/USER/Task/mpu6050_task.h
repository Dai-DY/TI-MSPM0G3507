#ifndef MPU6050_TASK_H
#define MPU6050_TASK_H

#include <stdbool.h>
#include <stdint.h>

#include "MPU6050.h"

typedef enum {
    MPU6050_STATUS_UNINITIALIZED = 0,
    MPU6050_STATUS_INITIALIZING,
    MPU6050_STATUS_RUNNING,
    MPU6050_STATUS_DEVICE_NOT_FOUND,
    MPU6050_STATUS_DMP_INIT_FAILED,
    MPU6050_STATUS_DATA_TIMEOUT,
    MPU6050_STATUS_FIFO_ERROR
} mpu6050_status_t;

typedef struct {
    Imu_t imu;
    uint32_t timestamp_ms;
    uint32_t sample_count;
    uint32_t interrupt_count;
    uint32_t i2c_error_count;
    uint32_t fifo_error_count;
    bool valid;
} mpu6050_snapshot_t;

void mpu6050_task_create(void);
bool mpu6050_get_snapshot(mpu6050_snapshot_t *snapshot);
mpu6050_status_t mpu6050_get_status(void);

#endif
