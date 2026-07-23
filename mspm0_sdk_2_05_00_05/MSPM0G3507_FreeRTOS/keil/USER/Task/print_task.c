#include "print_task.h"

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "UART0_Debug.h"
#include "eight_ir_task.h"

#define PRINT_TASK_STACK_SIZE    1024U
#define PRINT_TASK_PRIORITY      1U

static StaticTask_t print_task_tcb;
static StackType_t print_task_stack[PRINT_TASK_STACK_SIZE];

static void print_task(void *parameters)
{
    eight_ir_data_t sensor_data;

    (void) parameters;

    debug_uart_init();

    for (;;) {
        sensor_data = ir_data;

        printf("IR: %u%u%u%u%u%u%u%u  mask=0x%02X\n",
            (unsigned int) sensor_data.values[7],
            (unsigned int) sensor_data.values[6],
            (unsigned int) sensor_data.values[5],
            (unsigned int) sensor_data.values[4],
            (unsigned int) sensor_data.values[3],
            (unsigned int) sensor_data.values[2],
            (unsigned int) sensor_data.values[1],
            (unsigned int) sensor_data.values[0],
            (unsigned int) sensor_data.mask);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void print_task_create(void)
{
    (void) xTaskCreateStatic(
        print_task,
        "PrintTask",
        PRINT_TASK_STACK_SIZE,
        NULL,
        PRINT_TASK_PRIORITY,
        print_task_stack,
        &print_task_tcb);
}
