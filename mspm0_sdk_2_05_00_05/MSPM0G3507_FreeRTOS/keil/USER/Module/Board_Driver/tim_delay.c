#include "tim_delay.h"

void delay_us(uint32_t us)
{
    if (us != 0U) {
        DL_TimerA_setLoadValue(TIM_delay_ms_INST, us);
        DL_TimerA_setTimerCount(TIM_delay_ms_INST, us);
        DL_TimerA_startCounter(TIM_delay_ms_INST);

        while (DL_TimerA_getTimerCount(TIM_delay_ms_INST) != 0U) {
        }
    }
}

void delay_ms(uint16_t ms)
{
    while (ms != 0U) {
        delay_us(1000U);
        ms--;
    }
}
