#include "mpu6050_task.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#include "mahony.h"
#include "mpu6050_bias_store.h"
#include "mpu6050_port.h"
#include "ti_msp_dl_config.h"

#define MPU6050_TASK_STACK_SIZE               512U
#define MPU6050_TASK_PRIORITY                 6U
#define MPU6050_RETRY_DELAY_MS                500U
#define MPU6050_DATA_TIMEOUT_MS               100U
#define MPU6050_MAX_CONSECUTIVE_ERRORS        5U
#define MPU6050_CALIBRATION_SAMPLES           200U
#define MPU6050_ATTITUDE_INIT_SAMPLES         32U
#define MPU6050_CALIBRATION_GYRO_RANGE_DPS    5.0f
#define MPU6050_CALIBRATION_ACCEL_RANGE_G     0.20f
#define MPU6050_CALIBRATION_ACCEL_NORM_MIN_G  0.80f
#define MPU6050_CALIBRATION_ACCEL_NORM_MAX_G  1.20f
#define MPU6050_MAHONY_KP                     1.5f
#define MPU6050_MAHONY_KI                     0.0f
#define MPU6050_DEG_TO_RAD                    0.0174532925199f
#define MPU6050_NOMINAL_DT_S                  0.005f
#define MPU6050_MIN_DT_S                      0.002f
#define MPU6050_MAX_DT_S                      0.020f

typedef enum {
    INIT_RESULT_OK = 0,
    INIT_RESULT_IO_ERROR = -1,
    INIT_RESULT_DEVICE_NOT_FOUND = -2,
    INIT_RESULT_MOTION = -3
} initialization_result_t;

static StaticTask_t mpu6050_task_tcb;
static StackType_t mpu6050_task_stack[MPU6050_TASK_STACK_SIZE];
static TaskHandle_t mpu6050_task_handle;

volatile mpu6050_status_t mpu6050_status =
    MPU6050_STATUS_UNINITIALIZED;
static volatile uint32_t interrupt_count;
static volatile bool calibration_requested;
static mpu6050_snapshot_t latest_snapshot;

Imu_t mpu6050;

static void set_status(mpu6050_status_t status)
{
    taskENTER_CRITICAL();
    mpu6050_status = status;
    latest_snapshot.status = status;
    if (status != MPU6050_STATUS_RUNNING) {
        latest_snapshot.valid = false;
    }
    taskEXIT_CRITICAL();
}

static void increment_i2c_error(void)
{
    taskENTER_CRITICAL();
    latest_snapshot.i2c_error_count++;
    taskEXIT_CRITICAL();
}

static void prepare_data_ready_interrupt(void)
{
    NVIC_DisableIRQ(MPU6050_INT_INT_IRQN);
    NVIC_ClearPendingIRQ(MPU6050_INT_INT_IRQN);
    DL_GPIO_clearInterruptStatus(
        MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);
    (void) ulTaskNotifyTake(pdTRUE, 0U);
    NVIC_EnableIRQ(MPU6050_INT_INT_IRQN);
}

static int wait_for_sample(mpu6050_raw_sample_t *raw,
    mpu6050_sample_t *sample)
{
    if (ulTaskNotifyTake(pdTRUE,
            pdMS_TO_TICKS(MPU6050_DATA_TIMEOUT_MS)) == 0U) {
        taskENTER_CRITICAL();
        latest_snapshot.timeout_count++;
        taskEXIT_CRITICAL();
        return -1;
    }

    if (mpu6050_raw_read(raw) != 0) {
        increment_i2c_error();
        return -1;
    }

    mpu6050_raw_convert(raw, sample);
    return 0;
}

static bool accel_norm_is_valid(const float accel[3])
{
    const float norm = sqrtf(accel[0] * accel[0] +
        accel[1] * accel[1] + accel[2] * accel[2]);

    return (norm >= MPU6050_CALIBRATION_ACCEL_NORM_MIN_G) &&
        (norm <= MPU6050_CALIBRATION_ACCEL_NORM_MAX_G);
}

static initialization_result_t collect_accel_average(float average[3])
{
    mpu6050_raw_sample_t raw;
    mpu6050_sample_t sample;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    uint16_t sample_index;
    uint8_t axis;

    for (sample_index = 0U;
         sample_index < MPU6050_ATTITUDE_INIT_SAMPLES;
         sample_index++) {
        if (wait_for_sample(&raw, &sample) != 0) {
            return INIT_RESULT_IO_ERROR;
        }

        for (axis = 0U; axis < 3U; axis++) {
            sum[axis] += sample.accel_g[axis];
        }
    }

    for (axis = 0U; axis < 3U; axis++) {
        average[axis] = sum[axis] /
            (float) MPU6050_ATTITUDE_INIT_SAMPLES;
    }

    return accel_norm_is_valid(average) ?
        INIT_RESULT_OK : INIT_RESULT_MOTION;
}

static initialization_result_t calibrate_gyro(float gyro_bias[3],
    float accel_average[3])
{
    mpu6050_raw_sample_t raw;
    mpu6050_sample_t sample;
    float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
    float accel_sum[3] = {0.0f, 0.0f, 0.0f};
    float gyro_min[3] = {10000.0f, 10000.0f, 10000.0f};
    float gyro_max[3] = {-10000.0f, -10000.0f, -10000.0f};
    float accel_min[3] = {10000.0f, 10000.0f, 10000.0f};
    float accel_max[3] = {-10000.0f, -10000.0f, -10000.0f};
    uint16_t sample_index;
    uint8_t axis;

    set_status(MPU6050_STATUS_CALIBRATING);

    for (sample_index = 0U;
         sample_index < MPU6050_CALIBRATION_SAMPLES;
         sample_index++) {
        if (wait_for_sample(&raw, &sample) != 0) {
            return INIT_RESULT_IO_ERROR;
        }

        for (axis = 0U; axis < 3U; axis++) {
            gyro_sum[axis] += sample.gyro_dps[axis];
            accel_sum[axis] += sample.accel_g[axis];
            if (sample.gyro_dps[axis] < gyro_min[axis]) {
                gyro_min[axis] = sample.gyro_dps[axis];
            }
            if (sample.gyro_dps[axis] > gyro_max[axis]) {
                gyro_max[axis] = sample.gyro_dps[axis];
            }
            if (sample.accel_g[axis] < accel_min[axis]) {
                accel_min[axis] = sample.accel_g[axis];
            }
            if (sample.accel_g[axis] > accel_max[axis]) {
                accel_max[axis] = sample.accel_g[axis];
            }
        }
    }

    for (axis = 0U; axis < 3U; axis++) {
        gyro_bias[axis] = gyro_sum[axis] /
            (float) MPU6050_CALIBRATION_SAMPLES;
        accel_average[axis] = accel_sum[axis] /
            (float) MPU6050_CALIBRATION_SAMPLES;

        if (((gyro_max[axis] - gyro_min[axis]) >
                MPU6050_CALIBRATION_GYRO_RANGE_DPS) ||
            ((accel_max[axis] - accel_min[axis]) >
                MPU6050_CALIBRATION_ACCEL_RANGE_G)) {
            return INIT_RESULT_MOTION;
        }
    }

    return accel_norm_is_valid(accel_average) ?
        INIT_RESULT_OK : INIT_RESULT_MOTION;
}

static initialization_result_t initialize_filter(bool force_calibration,
    mahony_filter_t *filter, float gyro_bias[3], bool *bias_from_flash,
    bool *bias_persisted)
{
    float accel_average[3];
    initialization_result_t result;
    int sensor_result;

    set_status(MPU6050_STATUS_INITIALIZING);
    NVIC_DisableIRQ(MPU6050_INT_INT_IRQN);
    NVIC_ClearPendingIRQ(MPU6050_INT_INT_IRQN);
    DL_GPIO_clearInterruptStatus(
        MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);

    sensor_result = mpu6050_raw_init();
    if (sensor_result != 0) {
        increment_i2c_error();
        return (sensor_result == -2) ? INIT_RESULT_DEVICE_NOT_FOUND :
                                      INIT_RESULT_IO_ERROR;
    }

    prepare_data_ready_interrupt();
    mahony_reset(filter, MPU6050_MAHONY_KP, MPU6050_MAHONY_KI);

    *bias_from_flash = !force_calibration &&
        mpu6050_bias_load(gyro_bias);
    *bias_persisted = *bias_from_flash;

    if (*bias_from_flash) {
        result = collect_accel_average(accel_average);
    } else {
        result = calibrate_gyro(gyro_bias, accel_average);
        if (result == INIT_RESULT_OK) {
            NVIC_DisableIRQ(MPU6050_INT_INT_IRQN);
            *bias_persisted = mpu6050_bias_save(gyro_bias);
            if (!*bias_persisted) {
                taskENTER_CRITICAL();
                latest_snapshot.flash_error_count++;
                taskEXIT_CRITICAL();
            }
            prepare_data_ready_interrupt();
        }
    }

    if (result != INIT_RESULT_OK) {
        return result;
    }

    if (!mahony_init_from_accel(filter, accel_average[0],
            accel_average[1], accel_average[2])) {
        return INIT_RESULT_MOTION;
    }

    taskENTER_CRITICAL();
    latest_snapshot.gyro_bias_dps[0] = gyro_bias[0];
    latest_snapshot.gyro_bias_dps[1] = gyro_bias[1];
    latest_snapshot.gyro_bias_dps[2] = gyro_bias[2];
    latest_snapshot.bias_from_flash = *bias_from_flash;
    latest_snapshot.bias_persisted = *bias_persisted;
    taskEXIT_CRITICAL();

    return INIT_RESULT_OK;
}

static float calculate_sample_dt(TickType_t current_tick,
    TickType_t *last_tick)
{
    float dt;
    TickType_t elapsed_ticks;

    elapsed_ticks = current_tick - *last_tick;
    *last_tick = current_tick;
    dt = (float) elapsed_ticks * (float) portTICK_PERIOD_MS * 0.001f;

    if ((dt < MPU6050_MIN_DT_S) || (dt > MPU6050_MAX_DT_S)) {
        dt = MPU6050_NOMINAL_DT_S;
    }

    return dt;
}

static void publish_sample(const mpu6050_raw_sample_t *raw,
    const mpu6050_sample_t *sample, const float corrected_gyro_dps[3],
    const mahony_filter_t *filter)
{
    Imu_t output;

    output.raw = *raw;
    output.accel.x = sample->accel_g[0];
    output.accel.y = sample->accel_g[1];
    output.accel.z = sample->accel_g[2];
    output.gyro.x = corrected_gyro_dps[0];
    output.gyro.y = corrected_gyro_dps[1];
    output.gyro.z = corrected_gyro_dps[2];
    output.temperature_c = sample->temperature_c;
    output.quaternion[0] = filter->q[0];
    output.quaternion[1] = filter->q[1];
    output.quaternion[2] = filter->q[2];
    output.quaternion[3] = filter->q[3];
    mahony_get_euler_deg(filter,
        &output.roll, &output.pitch, &output.yaw);

    taskENTER_CRITICAL();
    mpu6050 = output;
    latest_snapshot.imu = output;
    latest_snapshot.timestamp_ms =
        (uint32_t) xTaskGetTickCount() * (uint32_t) portTICK_PERIOD_MS;
    latest_snapshot.sample_count++;
    latest_snapshot.interrupt_count = interrupt_count;
    latest_snapshot.valid = true;
    taskEXIT_CRITICAL();
}

static void stop_sensor(void)
{
    NVIC_DisableIRQ(MPU6050_INT_INT_IRQN);
    NVIC_ClearPendingIRQ(MPU6050_INT_INT_IRQN);
    DL_GPIO_clearInterruptStatus(
        MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);
    mpu6050_raw_disable_interrupt();
}

static void mpu6050_task(void *parameters)
{
    mahony_filter_t filter;
    mpu6050_raw_sample_t raw;
    mpu6050_sample_t sample;
    float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
    float corrected_gyro_dps[3];
    bool bias_from_flash;
    bool bias_persisted;
    bool force_calibration = false;
    uint8_t axis;

    (void) parameters;

    for (;;) {
        initialization_result_t init_result;
        bool calibration_attempt_forced;
        uint32_t consecutive_errors = 0U;
        TickType_t last_tick;

        calibration_attempt_forced = force_calibration;
        init_result = initialize_filter(force_calibration, &filter,
            gyro_bias, &bias_from_flash, &bias_persisted);
        force_calibration = false;

        if (init_result != INIT_RESULT_OK) {
            if (calibration_attempt_forced) {
                force_calibration = true;
            }
            if (init_result == INIT_RESULT_DEVICE_NOT_FOUND) {
                set_status(MPU6050_STATUS_DEVICE_NOT_FOUND);
            } else if (init_result == INIT_RESULT_MOTION) {
                set_status(MPU6050_STATUS_CALIBRATION_MOTION);
            } else {
                set_status(MPU6050_STATUS_I2C_ERROR);
            }
            stop_sensor();
            mpu6050_port_recover_bus();
            mpu6050_port_delay_ms(MPU6050_RETRY_DELAY_MS);
            continue;
        }

        calibration_requested = false;
        last_tick = xTaskGetTickCount();
        set_status(MPU6050_STATUS_RUNNING);

        for (;;) {
            float dt;

            if (calibration_requested) {
                force_calibration = true;
                break;
            }

            if (wait_for_sample(&raw, &sample) != 0) {
                consecutive_errors++;
                if (consecutive_errors >=
                    MPU6050_MAX_CONSECUTIVE_ERRORS) {
                    set_status(MPU6050_STATUS_DATA_TIMEOUT);
                    break;
                }
                continue;
            }

            consecutive_errors = 0U;
            for (axis = 0U; axis < 3U; axis++) {
                corrected_gyro_dps[axis] =
                    sample.gyro_dps[axis] - gyro_bias[axis];
            }

            dt = calculate_sample_dt(xTaskGetTickCount(), &last_tick);
            mahony_update_imu(&filter,
                corrected_gyro_dps[0] * MPU6050_DEG_TO_RAD,
                corrected_gyro_dps[1] * MPU6050_DEG_TO_RAD,
                corrected_gyro_dps[2] * MPU6050_DEG_TO_RAD,
                sample.accel_g[0], sample.accel_g[1], sample.accel_g[2],
                dt);
            publish_sample(&raw, &sample, corrected_gyro_dps, &filter);
        }

        stop_sensor();
        if (!force_calibration) {
            mpu6050_port_recover_bus();
            mpu6050_port_delay_ms(MPU6050_RETRY_DELAY_MS);
        }
    }
}

void mpu6050_task_create(void)
{
    memset(&mpu6050, 0, sizeof(mpu6050));
    memset(&latest_snapshot, 0, sizeof(latest_snapshot));
    interrupt_count = 0U;
    calibration_requested = false;
    mpu6050_status = MPU6050_STATUS_UNINITIALIZED;

    mpu6050_task_handle = xTaskCreateStatic(mpu6050_task, "MPU6050",
        MPU6050_TASK_STACK_SIZE, NULL, MPU6050_TASK_PRIORITY,
        mpu6050_task_stack, &mpu6050_task_tcb);
}

bool mpu6050_get_snapshot(mpu6050_snapshot_t *snapshot)
{
    bool valid;

    if (snapshot == NULL) {
        return false;
    }

    taskENTER_CRITICAL();
    *snapshot = latest_snapshot;
    snapshot->interrupt_count = interrupt_count;
    snapshot->status = mpu6050_status;
    valid = latest_snapshot.valid;
    taskEXIT_CRITICAL();

    return valid;
}

mpu6050_status_t mpu6050_get_status(void)
{
    mpu6050_status_t status;

    taskENTER_CRITICAL();
    status = mpu6050_status;
    taskEXIT_CRITICAL();

    return status;
}

bool mpu6050_request_calibration(void)
{
    if ((mpu6050_task_handle == NULL) ||
        (mpu6050_status != MPU6050_STATUS_RUNNING)) {
        return false;
    }

    calibration_requested = true;
    return true;
}

void GROUP1_IRQHandler(void)
{
    uint32_t pending = DL_GPIO_getEnabledInterruptStatus(
        MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);

    if ((pending & MPU6050_INT_PIN_PIN) != 0U) {
        BaseType_t higher_priority_task_woken = pdFALSE;

        DL_GPIO_clearInterruptStatus(
            MPU6050_INT_PORT, MPU6050_INT_PIN_PIN);
        interrupt_count++;

        if (mpu6050_task_handle != NULL) {
            vTaskNotifyGiveFromISR(
                mpu6050_task_handle, &higher_priority_task_woken);
            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }
}
