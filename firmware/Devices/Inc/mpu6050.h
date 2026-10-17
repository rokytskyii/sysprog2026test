#ifndef MPU6050_H
#define MPU6050_H

#include "bus_interfaces.h"

#define MPU6050_DEFAULT_ADDR 0x68
#define MPU6050_WHO_AM_I_VAL 0x68

typedef struct
{
    float ax; // Прискорення у g
    float ay;
    float az;
    float a_total; // Модуль вектора: sqrt(ax^2 + ay^2 + az^2)
} mpu6050_accel_data_t;

typedef struct
{
    const i2c_interface_t *bus;
    uint8_t dev_addr;
} mpu6050_t;

bool mpu6050_init(mpu6050_t *dev, const i2c_interface_t *bus, uint8_t dev_addr);
bool mpu6050_read_id(mpu6050_t *dev, uint8_t *id);
bool mpu6050_read_accel(mpu6050_t *dev, mpu6050_accel_data_t *accel_data);

#endif /* MPU6050_H */