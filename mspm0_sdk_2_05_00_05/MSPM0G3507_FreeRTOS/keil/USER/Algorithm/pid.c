#include "pid.h"

#include <stddef.h>
#include <string.h>

static float clamp_float(float value, float minimum, float maximum)
{
    if (value > maximum) {
        return maximum;
    }
    if (value < minimum) {
        return minimum;
    }
    return value;
}

void pid_init(pid_controller_t *pid, const pid_config_t *config)
{
    pid_config_t selected_config;

    if ((pid == NULL) || (config == NULL)) {
        return;
    }

    selected_config = *config;
    memset(pid, 0, sizeof(*pid));
    pid->config = selected_config;
}

void pid_set_config(pid_controller_t *pid, const pid_config_t *config)
{
    if ((pid == NULL) || (config == NULL)) {
        return;
    }

    pid->config = *config;
}

void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
    pid->has_previous_error = false;
}

float pid_calculate(pid_controller_t *pid, float error, bool integrate)
{
    float derivative = 0.0f;
    float output;

    if (pid == NULL) {
        return 0.0f;
    }

    if (integrate) {
        pid->integral += error;
        pid->integral = clamp_float(pid->integral,
            pid->config.integral_min, pid->config.integral_max);
    }

    if (pid->has_previous_error) {
        derivative = error - pid->previous_error;
    }

    output = (error * pid->config.kp) +
        (pid->integral * pid->config.ki) +
        (derivative * pid->config.kd);
    output = clamp_float(output, pid->config.output_min,
        pid->config.output_max);

    pid->previous_error = error;
    pid->has_previous_error = true;

    return output;
}
