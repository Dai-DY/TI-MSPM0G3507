#ifndef MG310_MOTOR_H
#define MG310_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_SPEED_MAX    1000

void motor_init(void);
void motor_standby(bool enable);

/* Positive: forward, negative: backward, zero: stop. */
void motor_set_speed(int16_t left_speed, int16_t right_speed);
void motor_brake(void);

#endif
