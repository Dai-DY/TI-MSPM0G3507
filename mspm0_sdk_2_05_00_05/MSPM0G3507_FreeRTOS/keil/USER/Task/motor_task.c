#include "motor_task.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

#include "motor.h"

#define MOTOR_TASK_STACK_SIZE    256U
#define MOTOR_TASK_PRIORITY      4U
#define MOTOR_COMMAND_TIMEOUT_MS 30U
#define MOTOR_COMMAND_QUEUE_LEN  1U

typedef struct {
    int16_t left_speed;
    int16_t right_speed;
} motor_command_t;

static StaticTask_t motor_task_tcb;
static StackType_t motor_task_stack[MOTOR_TASK_STACK_SIZE];
static StaticQueue_t motor_command_queue_tcb;
static uint8_t motor_command_queue_storage[
    MOTOR_COMMAND_QUEUE_LEN * sizeof(motor_command_t)];
static QueueHandle_t motor_command_queue;

static void motor_task(void *parameters)
{
    motor_command_t command;

    (void) parameters;

    motor_init();
    motor_standby(true);

    for (;;) {
        if (xQueueReceive(motor_command_queue, &command,
                pdMS_TO_TICKS(MOTOR_COMMAND_TIMEOUT_MS)) == pdPASS) {
            motor_set_speed(command.left_speed, command.right_speed);
        } else {
            motor_set_speed(0, 0);
        }
    }
}

void motor_task_create(void)
{
    motor_command_queue = xQueueCreateStatic(
        MOTOR_COMMAND_QUEUE_LEN,
        sizeof(motor_command_t),
        motor_command_queue_storage,
        &motor_command_queue_tcb);

    (void) xTaskCreateStatic(
        motor_task,
        "MotorTask",
        MOTOR_TASK_STACK_SIZE,
        NULL,
        MOTOR_TASK_PRIORITY,
        motor_task_stack,
        &motor_task_tcb);
}

bool motor_task_set_target(int16_t left_speed, int16_t right_speed)
{
    motor_command_t command;

    if (motor_command_queue == NULL) {
        return false;
    }

    command.left_speed = left_speed;
    command.right_speed = right_speed;

    return xQueueOverwrite(motor_command_queue, &command) == pdPASS;
}
