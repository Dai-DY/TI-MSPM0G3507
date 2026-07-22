#include "mahony.h"

#include <math.h>
#include <stddef.h>

#define MAHONY_RAD_TO_DEG       57.2957795131f
#define MAHONY_ACCEL_NORM_MIN   0.70f
#define MAHONY_ACCEL_NORM_MAX   1.30f
#define MAHONY_VECTOR_EPSILON   1.0e-6f
#define MAHONY_QUATERNION_EPSILON 1.0e-12f

static float clamp_unit(float value)
{
    if (value > 1.0f) {
        return 1.0f;
    }
    if (value < -1.0f) {
        return -1.0f;
    }
    return value;
}

static void normalize_quaternion(mahony_filter_t *filter)
{
    float norm_squared;
    float inverse_norm;

    norm_squared = filter->q[0] * filter->q[0] +
        filter->q[1] * filter->q[1] +
        filter->q[2] * filter->q[2] +
        filter->q[3] * filter->q[3];

    if (norm_squared < MAHONY_QUATERNION_EPSILON) {
        filter->q[0] = 1.0f;
        filter->q[1] = 0.0f;
        filter->q[2] = 0.0f;
        filter->q[3] = 0.0f;
        return;
    }

    inverse_norm = 1.0f / sqrtf(norm_squared);
    filter->q[0] *= inverse_norm;
    filter->q[1] *= inverse_norm;
    filter->q[2] *= inverse_norm;
    filter->q[3] *= inverse_norm;
}

void mahony_reset(mahony_filter_t *filter, float kp, float ki)
{
    if (filter == NULL) {
        return;
    }

    filter->q[0] = 1.0f;
    filter->q[1] = 0.0f;
    filter->q[2] = 0.0f;
    filter->q[3] = 0.0f;
    filter->integral_error[0] = 0.0f;
    filter->integral_error[1] = 0.0f;
    filter->integral_error[2] = 0.0f;
    filter->kp = kp;
    filter->ki = ki;
}

bool mahony_init_from_accel(mahony_filter_t *filter,
    float accel_x_g, float accel_y_g, float accel_z_g)
{
    float norm_squared;
    float inverse_norm;
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    float cosine_roll;
    float sine_roll;
    float cosine_pitch;
    float sine_pitch;

    if (filter == NULL) {
        return false;
    }

    norm_squared = accel_x_g * accel_x_g + accel_y_g * accel_y_g +
        accel_z_g * accel_z_g;
    if ((norm_squared < MAHONY_ACCEL_NORM_MIN * MAHONY_ACCEL_NORM_MIN) ||
        (norm_squared > MAHONY_ACCEL_NORM_MAX * MAHONY_ACCEL_NORM_MAX)) {
        return false;
    }

    inverse_norm = 1.0f / sqrtf(norm_squared);
    accel_x_g *= inverse_norm;
    accel_y_g *= inverse_norm;
    accel_z_g *= inverse_norm;

    roll = atan2f(accel_y_g, accel_z_g);
    pitch = atan2f(-accel_x_g,
        sqrtf(accel_y_g * accel_y_g + accel_z_g * accel_z_g));
    half_roll = 0.5f * roll;
    half_pitch = 0.5f * pitch;
    cosine_roll = cosf(half_roll);
    sine_roll = sinf(half_roll);
    cosine_pitch = cosf(half_pitch);
    sine_pitch = sinf(half_pitch);

    filter->q[0] = cosine_roll * cosine_pitch;
    filter->q[1] = sine_roll * cosine_pitch;
    filter->q[2] = cosine_roll * sine_pitch;
    filter->q[3] = -sine_roll * sine_pitch;
    filter->integral_error[0] = 0.0f;
    filter->integral_error[1] = 0.0f;
    filter->integral_error[2] = 0.0f;
    normalize_quaternion(filter);

    return true;
}

void mahony_update_imu(mahony_filter_t *filter,
    float gyro_x_rad_s, float gyro_y_rad_s, float gyro_z_rad_s,
    float accel_x_g, float accel_y_g, float accel_z_g, float dt_s)
{
    float accel_norm_squared;
    float inverse_norm;
    float gravity_x;
    float gravity_y;
    float gravity_z;
    float error_x;
    float error_y;
    float error_z;
    float q0;
    float q1;
    float q2;
    float q3;
    float half_dt;

    if ((filter == NULL) || (dt_s <= 0.0f)) {
        return;
    }

    accel_norm_squared = accel_x_g * accel_x_g +
        accel_y_g * accel_y_g + accel_z_g * accel_z_g;
    if ((accel_norm_squared >=
            MAHONY_ACCEL_NORM_MIN * MAHONY_ACCEL_NORM_MIN) &&
        (accel_norm_squared <=
            MAHONY_ACCEL_NORM_MAX * MAHONY_ACCEL_NORM_MAX) &&
        (accel_norm_squared > MAHONY_VECTOR_EPSILON)) {
        inverse_norm = 1.0f / sqrtf(accel_norm_squared);
        accel_x_g *= inverse_norm;
        accel_y_g *= inverse_norm;
        accel_z_g *= inverse_norm;

        gravity_x = 2.0f *
            (filter->q[1] * filter->q[3] -
                filter->q[0] * filter->q[2]);
        gravity_y = 2.0f *
            (filter->q[0] * filter->q[1] +
                filter->q[2] * filter->q[3]);
        gravity_z = filter->q[0] * filter->q[0] -
            filter->q[1] * filter->q[1] -
            filter->q[2] * filter->q[2] +
            filter->q[3] * filter->q[3];

        error_x = accel_y_g * gravity_z - accel_z_g * gravity_y;
        error_y = accel_z_g * gravity_x - accel_x_g * gravity_z;
        error_z = accel_x_g * gravity_y - accel_y_g * gravity_x;

        if (filter->ki > 0.0f) {
            filter->integral_error[0] += filter->ki * error_x * dt_s;
            filter->integral_error[1] += filter->ki * error_y * dt_s;
            filter->integral_error[2] += filter->ki * error_z * dt_s;
            gyro_x_rad_s += filter->integral_error[0];
            gyro_y_rad_s += filter->integral_error[1];
            gyro_z_rad_s += filter->integral_error[2];
        }

        gyro_x_rad_s += filter->kp * error_x;
        gyro_y_rad_s += filter->kp * error_y;
        gyro_z_rad_s += filter->kp * error_z;
    }

    q0 = filter->q[0];
    q1 = filter->q[1];
    q2 = filter->q[2];
    q3 = filter->q[3];
    half_dt = 0.5f * dt_s;

    filter->q[0] += (-q1 * gyro_x_rad_s - q2 * gyro_y_rad_s -
                        q3 * gyro_z_rad_s) *
        half_dt;
    filter->q[1] += (q0 * gyro_x_rad_s + q2 * gyro_z_rad_s -
                        q3 * gyro_y_rad_s) *
        half_dt;
    filter->q[2] += (q0 * gyro_y_rad_s - q1 * gyro_z_rad_s +
                        q3 * gyro_x_rad_s) *
        half_dt;
    filter->q[3] += (q0 * gyro_z_rad_s + q1 * gyro_y_rad_s -
                        q2 * gyro_x_rad_s) *
        half_dt;

    normalize_quaternion(filter);
}

void mahony_get_euler_deg(const mahony_filter_t *filter,
    float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    float roll;
    float pitch;
    float yaw;

    if (filter == NULL) {
        return;
    }

    roll = atan2f(
        2.0f * (filter->q[0] * filter->q[1] +
                   filter->q[2] * filter->q[3]),
        filter->q[0] * filter->q[0] -
            filter->q[1] * filter->q[1] -
            filter->q[2] * filter->q[2] +
            filter->q[3] * filter->q[3]);
    pitch = asinf(clamp_unit(2.0f *
        (filter->q[0] * filter->q[2] -
            filter->q[3] * filter->q[1])));
    yaw = atan2f(
        2.0f * (filter->q[0] * filter->q[3] +
                   filter->q[1] * filter->q[2]),
        filter->q[0] * filter->q[0] +
            filter->q[1] * filter->q[1] -
            filter->q[2] * filter->q[2] -
            filter->q[3] * filter->q[3]);

    if (roll_deg != NULL) {
        *roll_deg = roll * MAHONY_RAD_TO_DEG;
    }
    if (pitch_deg != NULL) {
        *pitch_deg = pitch * MAHONY_RAD_TO_DEG;
    }
    if (yaw_deg != NULL) {
        *yaw_deg = yaw * MAHONY_RAD_TO_DEG;
    }
}
