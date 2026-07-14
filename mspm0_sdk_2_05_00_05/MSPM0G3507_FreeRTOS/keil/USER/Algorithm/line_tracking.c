#include "line_tracking.h"

#include <string.h>

#define LINE_TRACKING_DEFAULT_KP                (700.0f)
#define LINE_TRACKING_DEFAULT_KI                (0.02f)
#define LINE_TRACKING_DEFAULT_KD                (0.0f)
#define LINE_TRACKING_DEFAULT_INTEGRAL_LIMIT    (1000.0f)
#define LINE_TRACKING_DEFAULT_TURN_LIMIT        (1200.0f)
#define LINE_TRACKING_DEFAULT_NORMAL_SPEED      (350)
#define LINE_TRACKING_DEFAULT_EDGE_SPEED        (280)
#define LINE_TRACKING_DEFAULT_CORNER_SPEED      (-20)
#define LINE_TRACKING_DEFAULT_ACTIVE_LEVEL      (1U)
#define LINE_TRACKING_CORNER_HOLD_MS             (300U)

static uint8_t make_active_mask(const line_tracking_t *tracker,
    const uint8_t sensor_values[LINE_TRACKING_SENSOR_COUNT])
{
    uint8_t index;
    uint8_t mask = 0U;

    for (index = 0U; index < LINE_TRACKING_SENSOR_COUNT; index++) {
        if ((sensor_values[index] != 0U) ==
            (tracker->config.active_level != 0U)) {
            mask |= (uint8_t) (1U << index);
        }
    }

    return mask;
}

void line_tracking_get_default_config(line_tracking_config_t *config)
{
    if (config == NULL) {
        return;
    }

    config->turn_pid.kp = LINE_TRACKING_DEFAULT_KP;
    config->turn_pid.ki = LINE_TRACKING_DEFAULT_KI;
    config->turn_pid.kd = LINE_TRACKING_DEFAULT_KD;
    config->turn_pid.integral_min = -LINE_TRACKING_DEFAULT_INTEGRAL_LIMIT;
    config->turn_pid.integral_max = LINE_TRACKING_DEFAULT_INTEGRAL_LIMIT;
    config->turn_pid.output_min = -LINE_TRACKING_DEFAULT_TURN_LIMIT;
    config->turn_pid.output_max = LINE_TRACKING_DEFAULT_TURN_LIMIT;
    config->normal_speed = LINE_TRACKING_DEFAULT_NORMAL_SPEED;
    config->edge_speed = LINE_TRACKING_DEFAULT_EDGE_SPEED;
    config->corner_speed = LINE_TRACKING_DEFAULT_CORNER_SPEED;
    config->active_level = LINE_TRACKING_DEFAULT_ACTIVE_LEVEL;
}

void line_tracking_init(line_tracking_t *tracker,
    const line_tracking_config_t *config)
{
    line_tracking_config_t selected_config;

    if (tracker == NULL) {
        return;
    }

    if (config != NULL) {
        selected_config = *config;
    } else {
        line_tracking_get_default_config(&selected_config);
    }

    memset(tracker, 0, sizeof(*tracker));
    tracker->config = selected_config;
    pid_init(&tracker->turn_pid, &tracker->config.turn_pid);
}

void line_tracking_reset(line_tracking_t *tracker)
{
    if (tracker == NULL) {
        return;
    }

    tracker->last_error = 0;
    pid_reset(&tracker->turn_pid);
}

void line_tracking_set_pid_config(line_tracking_t *tracker,
    const pid_config_t *config)
{
    if ((tracker == NULL) || (config == NULL)) {
        return;
    }

    tracker->config.turn_pid = *config;
    pid_set_config(&tracker->turn_pid, config);
}

bool line_tracking_update(line_tracking_t *tracker,
    const uint8_t sensor_values[LINE_TRACKING_SENSOR_COUNT],
    line_tracking_output_t *output)
{
    bool x1;
    bool x2;
    bool x3;
    bool x4;
    bool x5;
    bool x6;
    bool x7;
    bool x8;
    int16_t error;
    uint8_t active_mask;

    if ((tracker == NULL) || (sensor_values == NULL) || (output == NULL)) {
        return false;
    }

    active_mask = make_active_mask(tracker, sensor_values);
    x1 = (active_mask & (1U << 0U)) != 0U;
    x2 = (active_mask & (1U << 1U)) != 0U;
    x3 = (active_mask & (1U << 2U)) != 0U;
    x4 = (active_mask & (1U << 3U)) != 0U;
    x5 = (active_mask & (1U << 4U)) != 0U;
    x6 = (active_mask & (1U << 5U)) != 0U;
    x7 = (active_mask & (1U << 6U)) != 0U;
    x8 = (active_mask & (1U << 7U)) != 0U;

    output->active_mask = active_mask;
    output->mode = LINE_TRACKING_MODE_TRACK;
    output->forward_speed = tracker->config.normal_speed;
    output->hold_time_ms = 0U;
    error = tracker->last_error;

    if (active_mask == 0U) {
        output->mode = LINE_TRACKING_MODE_LOST;
    } else if (x4 && x5 && !x1 && !x8) {
        error = 0;
    } else if ((x1 || x2) && (x4 || x5)) {
        error = -20;
        output->mode = LINE_TRACKING_MODE_CORNER_LEFT;
        output->forward_speed = tracker->config.corner_speed;
        output->hold_time_ms = LINE_TRACKING_CORNER_HOLD_MS;
    } else if ((x7 || x8) && (x4 || x5)) {
        error = 20;
        output->mode = LINE_TRACKING_MODE_CORNER_RIGHT;
        output->forward_speed = tracker->config.corner_speed;
        output->hold_time_ms = LINE_TRACKING_CORNER_HOLD_MS;
    } else if (!x1 && !x2 && !x3 && x4 && !x5 && !x6 && !x7 && !x8) {
        error = -1;
    } else if (!x1 && !x2 && x3 && x4 && !x5 && !x6 && !x7 && !x8) {
        error = -2;
    } else if (!x1 && !x2 && x3 && !x4 && !x5 && !x6 && !x7 && !x8) {
        error = -4;
    } else if (!x1 && x2 && !x3 && !x4 && !x5 && !x6 && !x7 && !x8) {
        error = -6;
    } else if (x1 && !x2 && !x3 && !x4 && !x5 && !x6 && !x7 && !x8) {
        error = -12;
        output->forward_speed = tracker->config.edge_speed;
    } else if (!x1 && !x2 && !x3 && !x4 && x5 && x6 && !x7 && !x8) {
        error = 3;
    } else if (!x1 && !x2 && !x3 && !x4 && !x5 && x6 && !x7 && !x8) {
        error = 5;
    } else if (!x1 && !x2 && !x3 && !x4 && !x5 && x6 && x7 && !x8) {
        error = 8;
    } else if (!x1 && !x2 && !x3 && !x4 && !x5 && !x6 && !x7 && x8) {
        error = 12;
        output->forward_speed = tracker->config.edge_speed;
    } else if (x4 && !x5) {
        error = -1;
    } else if (!x4 && x5) {
        error = 1;
    }

    if (output->mode != LINE_TRACKING_MODE_LOST) {
        tracker->last_error = error;
    }

    output->error = error;
    output->turn_command = (int16_t) pid_calculate(&tracker->turn_pid,
        (float) error, output->mode != LINE_TRACKING_MODE_LOST);

    return true;
}
