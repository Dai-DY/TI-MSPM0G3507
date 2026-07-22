#ifndef MPU6050_PORT_H
#define MPU6050_PORT_H

#include <stdint.h>

int mpu6050_port_init(void);
int mpu6050_port_write(uint8_t address, uint8_t reg, uint8_t length,
    const uint8_t *data);
int mpu6050_port_read(uint8_t address, uint8_t reg, uint8_t length,
    uint8_t *data);
void mpu6050_port_delay_ms(uint32_t milliseconds);
void mpu6050_port_recover_bus(void);

#endif
