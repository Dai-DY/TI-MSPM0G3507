#include "motor.h"

#define MOTOR_PWM_PERIOD    (8000U)
#define MOTOR_PWMA_IDX      GPIO_PWM_0_C0_IDX
#define MOTOR_PWMB_IDX      GPIO_PWM_0_C1_IDX

#define MOTOR_AIN1_PORT     GPIOA
#define MOTOR_AIN1_PIN      DL_GPIO_PIN_13
#define MOTOR_AIN1_IOMUX    IOMUX_PINCM35

#define MOTOR_AIN2_PORT     GPIOA
#define MOTOR_AIN2_PIN      DL_GPIO_PIN_14
#define MOTOR_AIN2_IOMUX    IOMUX_PINCM36

#define MOTOR_BIN1_PORT     GPIOA
#define MOTOR_BIN1_PIN      DL_GPIO_PIN_16
#define MOTOR_BIN1_IOMUX    IOMUX_PINCM38

#define MOTOR_BIN2_PORT     GPIOA
#define MOTOR_BIN2_PIN      DL_GPIO_PIN_17
#define MOTOR_BIN2_IOMUX    IOMUX_PINCM39

typedef struct {
    DL_TIMER_CC_INDEX pwm_idx;
    GPIO_Regs *in1_port;
    uint32_t in1_pin;
    GPIO_Regs *in2_port;
    uint32_t in2_pin;
    motor_state_t state;
} motor_ctrl_t;

static motor_ctrl_t motors[2] = {
    {
        MOTOR_PWMA_IDX,
        MOTOR_AIN1_PORT, MOTOR_AIN1_PIN,
        MOTOR_AIN2_PORT, MOTOR_AIN2_PIN,
        { MOTOR_DIR_STOP, 0 }
    },
    {
        MOTOR_PWMB_IDX,
        MOTOR_BIN1_PORT, MOTOR_BIN1_PIN,
        MOTOR_BIN2_PORT, MOTOR_BIN2_PIN,
        { MOTOR_DIR_STOP, 0 }
    }
};

static uint16_t clamp_speed(uint16_t speed)
{
    return (speed > MOTOR_SPEED_MAX) ? MOTOR_SPEED_MAX : speed;
}

static void init_output_pin(uint32_t iomux, GPIO_Regs *port, uint32_t pin)
{
    DL_GPIO_initDigitalOutputFeatures(iomux,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_DISABLE);
    DL_GPIO_clearPins(port, pin);
    DL_GPIO_enableOutput(port, pin);
}

static void set_pwm_output(motor_id_t motor, uint16_t speed)
{
    motor_ctrl_t *ctrl = &motors[motor];
    uint32_t compare_value = ((uint32_t) clamp_speed(speed) * MOTOR_PWM_PERIOD) /
        MOTOR_SPEED_MAX;

    DL_TimerA_setCaptureCompareValue(PWM_0_INST, compare_value, ctrl->pwm_idx);
}

static void set_motor_output(motor_id_t motor, motor_dir_t dir, uint16_t speed)
{
    motor_ctrl_t *ctrl = &motors[motor];
    uint16_t limited_speed = clamp_speed(speed);

    switch (dir) {
        case MOTOR_DIR_FORWARD:
            DL_GPIO_setPins(ctrl->in1_port, ctrl->in1_pin);
            DL_GPIO_clearPins(ctrl->in2_port, ctrl->in2_pin);
            break;

        case MOTOR_DIR_BACKWARD:
            DL_GPIO_clearPins(ctrl->in1_port, ctrl->in1_pin);
            DL_GPIO_setPins(ctrl->in2_port, ctrl->in2_pin);
            break;

        case MOTOR_DIR_BRAKE:
            DL_GPIO_setPins(ctrl->in1_port, ctrl->in1_pin);
            DL_GPIO_setPins(ctrl->in2_port, ctrl->in2_pin);
            limited_speed = 0U;
            break;

        case MOTOR_DIR_STOP:
        default:
            DL_GPIO_clearPins(ctrl->in1_port, ctrl->in1_pin);
            DL_GPIO_clearPins(ctrl->in2_port, ctrl->in2_pin);
            limited_speed = 0U;
            dir = MOTOR_DIR_STOP;
            break;
    }

    set_pwm_output(motor, limited_speed);
    ctrl->state.dir = dir;
    ctrl->state.speed = (dir == MOTOR_DIR_BACKWARD) ?
        -(int16_t) limited_speed : (int16_t) limited_speed;
}

static void apply_to_motor(motor_id_t motor, motor_dir_t dir, uint16_t speed)
{
    if (motor == MOTOR_LEFT || motor == MOTOR_RIGHT) {
        set_motor_output(motor, dir, speed);
    } else if (motor == MOTOR_ALL) {
        set_motor_output(MOTOR_LEFT, dir, speed);
        set_motor_output(MOTOR_RIGHT, dir, speed);
    }
}

void motor_init(void)
{
    init_output_pin(MOTOR_AIN1_IOMUX, MOTOR_AIN1_PORT, MOTOR_AIN1_PIN);
    init_output_pin(MOTOR_AIN2_IOMUX, MOTOR_AIN2_PORT, MOTOR_AIN2_PIN);
    init_output_pin(MOTOR_BIN1_IOMUX, MOTOR_BIN1_PORT, MOTOR_BIN1_PIN);
    init_output_pin(MOTOR_BIN2_IOMUX, MOTOR_BIN2_PORT, MOTOR_BIN2_PIN);

    motor_stop_all();
}

void motor_standby(bool enable)
{
    (void) enable;
}

void motor_set_speed(motor_id_t motor, int16_t speed)
{
    motor_dir_t dir;
    uint16_t abs_speed;

    if (speed > MOTOR_SPEED_MAX) {
        speed = MOTOR_SPEED_MAX;
    } else if (speed < -MOTOR_SPEED_MAX) {
        speed = -MOTOR_SPEED_MAX;
    }

    if (speed > 0) {
        dir = MOTOR_DIR_FORWARD;
        abs_speed = (uint16_t) speed;
    } else if (speed < 0) {
        dir = MOTOR_DIR_BACKWARD;
        abs_speed = (uint16_t) (-speed);
    } else {
        dir = MOTOR_DIR_STOP;
        abs_speed = 0U;
    }

    apply_to_motor(motor, dir, abs_speed);
}

void motor_set_dir_speed(motor_id_t motor, motor_dir_t dir, uint16_t speed)
{
    apply_to_motor(motor, dir, speed);
}

void motor_stop(motor_id_t motor)
{
    apply_to_motor(motor, MOTOR_DIR_STOP, 0U);
}

void motor_brake(motor_id_t motor)
{
    apply_to_motor(motor, MOTOR_DIR_BRAKE, 0U);
}

void motor_stop_all(void)
{
    motor_stop(MOTOR_ALL);
}

void motor_brake_all(void)
{
    motor_brake(MOTOR_ALL);
}

motor_state_t motor_get_state(motor_id_t motor)
{
    motor_state_t empty_state = { MOTOR_DIR_STOP, 0 };

    if (motor == MOTOR_LEFT || motor == MOTOR_RIGHT) {
        return motors[motor].state;
    }

    return empty_state;
}
