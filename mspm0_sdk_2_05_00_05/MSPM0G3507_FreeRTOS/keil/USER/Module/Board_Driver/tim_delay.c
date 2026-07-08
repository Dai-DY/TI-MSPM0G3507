#include "tim_delay.h"

void delay_ms(uint16_t ms)
{
    uint16_t tick_count = ms * 2U;

    DL_TimerA_setLoadValue(TIM_delay_ms_INST, tick_count);
    DL_TimerA_setTimerCount(TIM_delay_ms_INST, tick_count);
    DL_TimerA_startCounter(TIM_delay_ms_INST);

    while (DL_TimerA_getTimerCount(TIM_delay_ms_INST) != 0U) {
    }
}
