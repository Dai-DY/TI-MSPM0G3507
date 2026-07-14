#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "motor.h"

#define MOTOR_TASK_STACK_SIZE    256U
#define MOTOR_TASK_PRIORITY      2U

static StaticTask_t motor_task_tcb;
static StackType_t motor_task_stack[MOTOR_TASK_STACK_SIZE];

static void motor_task(void *parameters)
{
    (void) parameters;

    motor_init();
    motor_set_speed(MOTOR_SPEED_MAX / 2, MOTOR_SPEED_MAX / 2);
    motor_standby(true);

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void motor_task_create(void)
{
    (void) xTaskCreateStatic(
        motor_task,
        "MotorTask",
        MOTOR_TASK_STACK_SIZE,
        NULL,
        MOTOR_TASK_PRIORITY,
        motor_task_stack,
        &motor_task_tcb);
}
