#include "board_led.h"

static led_t board_led;

void board_led_on(void)
{
    board_led.state = LED_ON;
    DL_GPIO_clearPins(CAR_LED_PORT, CAR_LED_PIN_LED_PIN);
}

void board_led_off(void)
{
    board_led.state = LED_OFF;
    DL_GPIO_setPins(CAR_LED_PORT, CAR_LED_PIN_LED_PIN);
}

void board_led_toggle(void)
{
    DL_GPIO_togglePins(CAR_LED_PORT, CAR_LED_PIN_LED_PIN);
    board_led.state = (DL_GPIO_readPins(CAR_LED_PORT, CAR_LED_PIN_LED_PIN) == 0U)
                          ? LED_ON
                          : LED_OFF;
}

led_state_e get_board_led_state(void)
{
	return board_led.state;
}
