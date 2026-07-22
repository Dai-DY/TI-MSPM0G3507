#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include <stdbool.h>
#include <stdint.h>

void motor_task_create(void);
bool motor_task_set_target(int16_t left_speed, int16_t right_speed);

#endif
