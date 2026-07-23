#include "FreeRTOS.h"
#include "task.h"

#include "ti_msp_dl_config.h"
#include "eight_ir_task.h"
#include "led_task.h"
#include "line_tracking_task.h"
#include "mpu6050_task.h"
#include "motor_task.h"
#include "print_task.h"

int main(void)
{
    SYSCFG_DL_init();

    led_task_create();
    motor_task_create();
    line_tracking_task_create();
    eight_ir_task_create();
    print_task_create();
    mpu6050_task_create();

    vTaskStartScheduler();

    for (;;) {
    }
}

#if (configSUPPORT_STATIC_ALLOCATION == 1)
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
    StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t idle_task_tcb;
    static StackType_t idle_task_stack[configIDLE_TASK_STACK_DEPTH];

    *ppxIdleTaskTCBBuffer = &idle_task_tcb;
    *ppxIdleTaskStackBuffer = idle_task_stack;
    *pulIdleTaskStackSize = configIDLE_TASK_STACK_DEPTH;
}

#if (configUSE_TIMERS == 1)
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
    StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t timer_task_tcb;
    static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &timer_task_tcb;
    *ppxTimerTaskStackBuffer = timer_task_stack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
#endif

#if (configCHECK_FOR_STACK_OVERFLOW)
#if defined(__IAR_SYSTEMS_ICC__)
__weak void vApplicationStackOverflowHook(
    TaskHandle_t pxTask, char *pcTaskName)
#elif defined(__TI_COMPILER_VERSION__)
#pragma WEAK(vApplicationStackOverflowHook)
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#elif defined(__GNUC__) || defined(__ti_version__)
void __attribute__((weak))
vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
#endif
{
    (void) pxTask;
    (void) pcTaskName;

    for (;;) {
    }
}
#endif
