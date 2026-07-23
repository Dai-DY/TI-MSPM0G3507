#include "led_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "board_led.h"

#define LED_TASK_STACK_SIZE    configMINIMAL_STACK_SIZE
#define LED_TASK_PRIORITY      (tskIDLE_PRIORITY + 1U)
#define LED_BLINK_HALF_PERIOD  pdMS_TO_TICKS(500U)

static StaticTask_t led_task_tcb;
static StackType_t led_task_stack[LED_TASK_STACK_SIZE];

static void led_task(void *parameters)
{
    TickType_t last_wake_time;

    (void) parameters;

    last_wake_time = xTaskGetTickCount();
    board_led_on();

    for (;;) {
        vTaskDelayUntil(&last_wake_time, LED_BLINK_HALF_PERIOD);
        board_led_toggle();
    }
}

void led_task_create(void)
{
    (void) xTaskCreateStatic(
        led_task,
        "LedTask",
        LED_TASK_STACK_SIZE,
        NULL,
        LED_TASK_PRIORITY,
        led_task_stack,
        &led_task_tcb);
}
