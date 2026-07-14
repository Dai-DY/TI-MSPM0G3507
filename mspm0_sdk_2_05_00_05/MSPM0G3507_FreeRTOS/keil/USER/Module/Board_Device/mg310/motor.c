#include "motor.h"

#include "ti_msp_dl_config.h"

#define MOTOR_PWM_PERIOD    8000U
#define MOTOR_COUNT         2U
#define MOTOR_LEFT_INDEX    0U
#define MOTOR_RIGHT_INDEX   1U

typedef struct {
    DL_TIMER_CC_INDEX pwm_idx;
    GPIO_Regs *in1_port;
    uint32_t in1_pin;
    GPIO_Regs *in2_port;
    uint32_t in2_pin;
} motor_ctrl_t;

static const motor_ctrl_t motors[MOTOR_COUNT] = {
    {
        GPIO_PWM_0_C0_IDX,
        AIN_PORT, AIN_AIN1_PIN,
        AIN_PORT, AIN_AIN2_PIN
    },
    {
        GPIO_PWM_0_C1_IDX,
        BIN_PORT, BIN_BIN1_PIN,
        BIN_PORT, BIN_BIN2_PIN
    }
};

static int16_t clamp_speed(int16_t speed)
{
    if (speed > MOTOR_SPEED_MAX) {
        return MOTOR_SPEED_MAX;
    }

    if (speed < -MOTOR_SPEED_MAX) {
        return -MOTOR_SPEED_MAX;
    }

    return speed;
}

static void set_pwm(const motor_ctrl_t *motor, uint16_t speed)
{
    uint32_t compare_value = ((uint32_t) speed * MOTOR_PWM_PERIOD) /
        MOTOR_SPEED_MAX;

    DL_TimerA_setCaptureCompareValue(
        PWM_0_INST, compare_value, motor->pwm_idx);
}

static void set_one_motor(uint32_t index, int16_t speed)
{
    const motor_ctrl_t *motor = &motors[index];
    int16_t limited_speed = clamp_speed(speed);
    uint16_t pwm_speed;

    if (limited_speed > 0) {
        DL_GPIO_setPins(motor->in1_port, motor->in1_pin);
        DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);
        pwm_speed = (uint16_t) limited_speed;
    } else if (limited_speed < 0) {
        DL_GPIO_clearPins(motor->in1_port, motor->in1_pin);
        DL_GPIO_setPins(motor->in2_port, motor->in2_pin);
        pwm_speed = (uint16_t) (-limited_speed);
    } else {
        DL_GPIO_clearPins(motor->in1_port, motor->in1_pin);
        DL_GPIO_clearPins(motor->in2_port, motor->in2_pin);
        pwm_speed = 0U;
    }

    set_pwm(motor, pwm_speed);
}

void motor_init(void)
{
    motor_standby(false);
    motor_set_speed(0, 0);
}

void motor_standby(bool enable)
{
    if (enable) {
        DL_GPIO_setPins(MOTOR_STBY_PORT, MOTOR_STBY_STBY_PIN);
    } else {
        DL_GPIO_clearPins(MOTOR_STBY_PORT, MOTOR_STBY_STBY_PIN);
    }
}

void motor_set_speed(int16_t left_speed, int16_t right_speed)
{
    set_one_motor(MOTOR_LEFT_INDEX, left_speed);
    set_one_motor(MOTOR_RIGHT_INDEX, right_speed);
}

void motor_brake(void)
{
    uint32_t index;

    for (index = 0U; index < MOTOR_COUNT; index++) {
        const motor_ctrl_t *motor = &motors[index];

        DL_GPIO_setPins(motor->in1_port, motor->in1_pin);
        DL_GPIO_setPins(motor->in2_port, motor->in2_pin);
        set_pwm(motor, MOTOR_SPEED_MAX);
    }
}
