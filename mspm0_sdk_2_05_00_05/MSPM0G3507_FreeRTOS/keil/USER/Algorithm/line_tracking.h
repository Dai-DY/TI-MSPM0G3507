#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include <stdbool.h>
#include <stdint.h>

#include "pid.h"

#define LINE_TRACKING_SENSOR_COUNT    (8U)

typedef enum {
    LINE_TRACKING_MODE_TRACK = 0,
    LINE_TRACKING_MODE_CORNER_LEFT,
    LINE_TRACKING_MODE_CORNER_RIGHT,
    LINE_TRACKING_MODE_LOST
} line_tracking_mode_t;

typedef struct {
    pid_config_t turn_pid;
    int16_t normal_speed;
    int16_t edge_speed;
    int16_t corner_speed;
    uint8_t active_level;
} line_tracking_config_t;

typedef struct {
    line_tracking_config_t config;
    pid_controller_t turn_pid;
    int16_t last_error;
} line_tracking_t;

typedef struct {
    uint8_t active_mask;
    int16_t error;
    int16_t turn_command;
    int16_t forward_speed;
    uint16_t hold_time_ms;
    line_tracking_mode_t mode;
} line_tracking_output_t;

void line_tracking_get_default_config(line_tracking_config_t *config);
void line_tracking_init(line_tracking_t *tracker,
    const line_tracking_config_t *config);
void line_tracking_reset(line_tracking_t *tracker);
void line_tracking_set_pid_config(line_tracking_t *tracker,
    const pid_config_t *config);

/*
 * Convert eight raw sensor levels into a motion request. The function has no
 * GPIO, motor, delay, or RTOS dependency. sensor_values[0] is X1 and
 * sensor_values[7] is X8.
 */
bool line_tracking_update(line_tracking_t *tracker,
    const uint8_t sensor_values[LINE_TRACKING_SENSOR_COUNT],
    line_tracking_output_t *output);

#endif
