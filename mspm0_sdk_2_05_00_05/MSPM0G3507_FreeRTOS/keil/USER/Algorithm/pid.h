#ifndef PID_H
#define PID_H

#include <stdbool.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_min;
    float integral_max;
    float output_min;
    float output_max;
} pid_config_t;

typedef struct {
    pid_config_t config;
    float integral;
    float previous_error;
    bool has_previous_error;
} pid_controller_t;

void pid_init(pid_controller_t *pid, const pid_config_t *config);
void pid_set_config(pid_controller_t *pid, const pid_config_t *config);
void pid_reset(pid_controller_t *pid);

/* Set integrate to false when the integral term should be held. */
float pid_calculate(pid_controller_t *pid, float error, bool integrate);

#endif
