#include "eight_ir.h"

#include "ti_msp_dl_config.h"
#include "tim_delay.h"

#define EIGHT_IR_SETTLE_DELAY_MS    (1U)

static void write_selector_pin(GPIO_Regs *port, uint32_t pin, uint8_t level)
{
    if (level != 0U) {
        DL_GPIO_setPins(port, pin);
    } else {
        DL_GPIO_clearPins(port, pin);
    }
}

static void select_channel(uint8_t channel)
{
    write_selector_pin(EIGHT_IR_AD0_PORT, EIGHT_IR_AD0_PIN,
        channel & 0x01U);
    write_selector_pin(EIGHT_IR_AD1_PORT, EIGHT_IR_AD1_PIN,
        (channel >> 1U) & 0x01U);
    write_selector_pin(EIGHT_IR_AD2_PORT, EIGHT_IR_AD2_PIN,
        (channel >> 2U) & 0x01U);
}

static uint8_t read_selected_channel(void)
{
    return (DL_GPIO_readPins(EIGHT_IR_OUT_PORT, EIGHT_IR_OUT_PIN) != 0U) ?
        1U : 0U;
}

void eight_ir_init(void)
{
    select_channel(0U);
}

void eight_ir_read(eight_ir_data_t *data)
{
    uint8_t channel;

    if (data == 0) {
        return;
    }

    data->mask = 0U;

    for (channel = 0U; channel < EIGHT_IR_CHANNEL_COUNT; channel++) {
        select_channel(channel);
        delay_ms(EIGHT_IR_SETTLE_DELAY_MS);
        data->values[channel] = read_selected_channel();

        if (data->values[channel] != 0U) {
            data->mask |= (uint8_t) (1U << channel);
        }
    }
}
