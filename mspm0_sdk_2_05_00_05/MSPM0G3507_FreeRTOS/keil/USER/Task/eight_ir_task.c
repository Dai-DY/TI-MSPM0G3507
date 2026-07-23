#include "eight_ir_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "eight_ir.h"
#include "line_tracking_task.h"

#define EIGHT_IR_TASK_STACK_SIZE    256U
#define EIGHT_IR_TASK_PRIORITY      2U
#define EIGHT_IR_SAMPLE_PERIOD_MS   5U

static StaticTask_t eight_ir_task_tcb;
static StackType_t eight_ir_task_stack[EIGHT_IR_TASK_STACK_SIZE];
eight_ir_data_t ir_data = { { 0U }, 0U };

static void eight_ir_task(void *parameters)
{
    eight_ir_data_t sample;
    TickType_t last_wake_time;

    (void) parameters;

    eight_ir_init();
    last_wake_time = xTaskGetTickCount();

    for (;;) {
        eight_ir_read(&sample);

        ir_data = sample;

        line_tracking_task_notify_sensor_update();

        vTaskDelayUntil(
            &last_wake_time, pdMS_TO_TICKS(EIGHT_IR_SAMPLE_PERIOD_MS));
    }
}

void eight_ir_task_create(void)
{
    (void) xTaskCreateStatic(
        eight_ir_task,
        "EightIRTask",
        EIGHT_IR_TASK_STACK_SIZE,
        NULL,
        EIGHT_IR_TASK_PRIORITY,
        eight_ir_task_stack,
        &eight_ir_task_tcb);
}
