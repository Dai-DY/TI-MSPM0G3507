#ifndef MAHONY_H
#define MAHONY_H

#include <stdbool.h>

typedef struct {
    float q[4];
    float integral_error[3];
    float kp;
    float ki;
} mahony_filter_t;

void mahony_reset(mahony_filter_t *filter, float kp, float ki);
bool mahony_init_from_accel(mahony_filter_t *filter,
    float accel_x_g, float accel_y_g, float accel_z_g);
void mahony_update_imu(mahony_filter_t *filter,
    float gyro_x_rad_s, float gyro_y_rad_s, float gyro_z_rad_s,
    float accel_x_g, float accel_y_g, float accel_z_g, float dt_s);
void mahony_get_euler_deg(const mahony_filter_t *filter,
    float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif
