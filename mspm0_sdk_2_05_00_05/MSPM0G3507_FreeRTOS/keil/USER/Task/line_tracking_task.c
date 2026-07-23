#include "line_tracking_task.h"

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "eight_ir_task.h"
#include "line_tracking.h"
#include "motor.h"
#include "motor_task.h"

#define LINE_TRACKING_TASK_STACK_SIZE    256U
#define LINE_TRACKING_TASK_PRIORITY      3U

static StaticTask_t line_tracking_task_tcb;
static StackType_t line_tracking_task_stack[LINE_TRACKING_TASK_STACK_SIZE];
static TaskHandle_t line_tracking_task_handle;

static int16_t clamp_motor_target(int32_t target)
{
    if (target > MOTOR_SPEED_MAX) {
        return MOTOR_SPEED_MAX;
    }

    if (target < -MOTOR_SPEED_MAX) {
        return -MOTOR_SPEED_MAX;
    }

    return (int16_t) target;
}

static void make_motor_targets(const line_tracking_output_t *output,
    int16_t *left_target, int16_t *right_target)
{
    int32_t left;
    int32_t right;

    left = (int32_t) output->forward_speed + output->turn_command;
    right = (int32_t) output->forward_speed - output->turn_command;

    *left_target = clamp_motor_target(left);
    *right_target = clamp_motor_target(right);
}

static void line_tracking_task(void *parameters)
{
    bool hold_active = false;
    eight_ir_data_t sensor_data;
    int16_t held_left_target = 0;
    int16_t held_right_target = 0;
    int16_t left_target;
    int16_t right_target;
    line_tracking_config_t config;
    line_tracking_output_t output;
    line_tracking_t tracker;
    TickType_t hold_duration = 0U;
    TickType_t hold_start = 0U;
    TickType_t now;

    (void) parameters;

    line_tracking_get_default_config(&config);
    line_tracking_init(&tracker, &config);

    for (;;) {
        (void) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        sensor_data = ir_data;

        now = xTaskGetTickCount();

        if (hold_active) {
            if ((TickType_t) (now - hold_start) < hold_duration) {
                (void) motor_task_set_target(
                    held_left_target, held_right_target);
                continue;
            }

            hold_active = false;
        }

        if (!line_tracking_update(&tracker, sensor_data.values, &output)) {
            left_target = 0;
            right_target = 0;
            line_tracking_reset(&tracker);
        } else if (output.mode == LINE_TRACKING_MODE_LOST) {
            left_target = 0;
            right_target = 0;
            line_tracking_reset(&tracker);
        } else {
            make_motor_targets(&output, &left_target, &right_target);

            if (output.hold_time_ms != 0U) {
                hold_duration = pdMS_TO_TICKS(output.hold_time_ms);
                if (hold_duration == 0U) {
                    hold_duration = 1U;
                }

                hold_start = now;
                held_left_target = left_target;
                held_right_target = right_target;
                hold_active = true;
            }
        }

        (void) motor_task_set_target(left_target, right_target);
    }
}

void line_tracking_task_create(void)
{
    line_tracking_task_handle = xTaskCreateStatic(
        line_tracking_task,
        "LineTrack",
        LINE_TRACKING_TASK_STACK_SIZE,
        NULL,
        LINE_TRACKING_TASK_PRIORITY,
        line_tracking_task_stack,
        &line_tracking_task_tcb);
}

void line_tracking_task_notify_sensor_update(void)
{
    if (line_tracking_task_handle != NULL) {
        (void) xTaskNotifyGive(line_tracking_task_handle);
    }
}
