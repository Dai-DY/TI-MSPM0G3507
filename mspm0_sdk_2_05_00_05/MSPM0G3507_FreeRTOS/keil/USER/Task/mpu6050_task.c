#include "mpu6050_task.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050_port.h"
#include "ti_msp_dl_config.h"

#define MPU6050_TASK_STACK_SIZE          512U
#define MPU6050_TASK_PRIORITY            3U
#define MPU6050_STARTUP_DELAY_MS          100U
#define MPU6050_RETRY_DELAY_MS            500U
#define MPU6050_DATA_TIMEOUT_MS           100U
#define MPU6050_MAX_CONSECUTIVE_ERRORS    5U
#define MPU6050_Q30_SCALE                 1073741824.0f
#define MPU6050_RAD_TO_DEGREES            57.2957795f

static StaticTask_t mpu6050_task_tcb;
static StackType_t mpu6050_task_stack[MPU6050_TASK_STACK_SIZE];
static TaskHandle_t mpu6050_task_handle;

static volatile mpu6050_status_t current_status =
    MPU6050_STATUS_UNINITIALIZED;

static void set_status(mpu6050_status_t status)
{
    taskENTER_CRITICAL();
    current_status = status;
    taskEXIT_CRITICAL();
}

static int initialize_sensor(void)
{
    NVIC_DisableIRQ(MPU6050_INT_INT_IRQN);
    DL_GPIO_clearInterruptStatus(MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);

    mpu6050_port_delay_ms(MPU6050_STARTUP_DELAY_MS);
    if (mpu6050_port_init() != 0) {
        set_status(MPU6050_STATUS_DEVICE_NOT_FOUND);
        return -1;
    }

    if (MPU6050_initialize() != 0) {
        set_status(MPU6050_STATUS_DEVICE_NOT_FOUND);
        return -1;
    }

    if (DMP_Init() != 0) {
        set_status(MPU6050_STATUS_DMP_INIT_FAILED);
        return -1;
    }

    (void) ulTaskNotifyTake(pdTRUE, 0U);
    DL_GPIO_clearInterruptStatus(MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);
    NVIC_EnableIRQ(MPU6050_INT_INT_IRQN);
    return 0;
}

static int read_dmp_sample(Imu_t *sample)
{
    float gyro_sensitivity;
    unsigned long sensor_timestamp;
    unsigned char more;
    unsigned char valid_sample = 0U;
    short local_gyro[3];
    short local_accel[3];
    short sensors;
    long quaternion[4];

    if (sample == NULL) {
        return -1;
    }

    if ((mpu_get_gyro_sens(&gyro_sensitivity) != 0) ||
        (gyro_sensitivity <= 0.0f)) {
        return -1;
    }

    do {
        float q0;
        float q1;
        float q2;
        float q3;

        if (dmp_read_fifo(local_gyro, local_accel, quaternion,
                &sensor_timestamp, &sensors, &more) != 0) {
            return -1;
        }

        if ((sensors & INV_WXYZ_QUAT) == 0) {
            continue;
        }

        q0 = quaternion[0] / MPU6050_Q30_SCALE;
        q1 = quaternion[1] / MPU6050_Q30_SCALE;
        q2 = quaternion[2] / MPU6050_Q30_SCALE;
        q3 = quaternion[3] / MPU6050_Q30_SCALE;

        /* Keep the axis mapping used by the original WHEELTEC example. */
        sample->pitch = asinf(-2.0f * q1 * q3 + 2.0f * q0 * q2) *
            MPU6050_RAD_TO_DEGREES;
        sample->roll = atan2f(2.0f * q2 * q3 + 2.0f * q0 * q1,
                           -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) *
            MPU6050_RAD_TO_DEGREES;
        sample->yaw = atan2f(2.0f * (q1 * q2 + q0 * q3),
                          q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) *
            MPU6050_RAD_TO_DEGREES;

        sample->gyroRoll = (float) local_gyro[0] / gyro_sensitivity;
        sample->gyroPitch = (float) local_gyro[1] / gyro_sensitivity;
        sample->gyroYaw = (float) local_gyro[2] / gyro_sensitivity;
        valid_sample = 1U;
    } while (more != 0U);

    return (valid_sample != 0U) ? 0 : -1;
}

static void mpu6050_task(void *parameters)
{
    Imu_t sample;

    (void) parameters;
    memset(&sample, 0, sizeof(sample));

    for (;;) {
        uint32_t consecutive_errors = 0U;

        set_status(MPU6050_STATUS_INITIALIZING);
        if (initialize_sensor() != 0) {
            mpu6050_port_delay_ms(MPU6050_RETRY_DELAY_MS);
            continue;
        }

        set_status(MPU6050_STATUS_RUNNING);

        for (;;) {
            if (ulTaskNotifyTake(pdTRUE,
                    pdMS_TO_TICKS(MPU6050_DATA_TIMEOUT_MS)) == 0U) {
                set_status(MPU6050_STATUS_DATA_TIMEOUT);
                break;
            }

            if (read_dmp_sample(&sample) != 0) {
                consecutive_errors++;
                if (consecutive_errors >= MPU6050_MAX_CONSECUTIVE_ERRORS) {
                    set_status(MPU6050_STATUS_FIFO_ERROR);
                    break;
                }
                continue;
            }

            consecutive_errors = 0U;
            mpu6050 = sample;
        }

        NVIC_DisableIRQ(MPU6050_INT_INT_IRQN);
        DL_GPIO_clearInterruptStatus(
            MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);
        mpu6050_port_recover_bus();
        mpu6050_port_delay_ms(MPU6050_RETRY_DELAY_MS);
    }
}

void mpu6050_task_create(void)
{
    memset(&mpu6050, 0, sizeof(mpu6050));
    current_status = MPU6050_STATUS_UNINITIALIZED;

    mpu6050_task_handle = xTaskCreateStatic(mpu6050_task, "MPU6050",
        MPU6050_TASK_STACK_SIZE, NULL, MPU6050_TASK_PRIORITY,
        mpu6050_task_stack, &mpu6050_task_tcb);
}

mpu6050_status_t mpu6050_get_status(void)
{
    mpu6050_status_t status;

    taskENTER_CRITICAL();
    status = current_status;
    taskEXIT_CRITICAL();

    return status;
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);

    if ((pending & MPU6050_INT_PIN_PIN) != 0U) {
        BaseType_t higher_priority_task_woken = pdFALSE;

        DL_GPIO_clearInterruptStatus(
            MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);

        if (mpu6050_task_handle != NULL) {
            vTaskNotifyGiveFromISR(
                mpu6050_task_handle, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}
