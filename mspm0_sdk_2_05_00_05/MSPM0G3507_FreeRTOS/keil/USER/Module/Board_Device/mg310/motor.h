#ifndef MG310_MOTOR_H
#define MG310_MOTOR_H

#include <stdbool.h>
#include <stdint.h>
#include "ti_msp_dl_config.h"

#define MOTOR_SPEED_MAX    (1000)

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT,
    MOTOR_ALL
} motor_id_t;

typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD,
    MOTOR_DIR_BRAKE
} motor_dir_t;

typedef struct {
    motor_dir_t dir;
    int16_t speed;
} motor_state_t;

void motor_init(void);
void motor_standby(bool enable);

void motor_set_speed(motor_id_t motor, int16_t speed);
void motor_set_dir_speed(motor_id_t motor, motor_dir_t dir, uint16_t speed);

void motor_stop(motor_id_t motor);
void motor_brake(motor_id_t motor);
void motor_stop_all(void);
void motor_brake_all(void);

motor_state_t motor_get_state(motor_id_t motor);

#endif
