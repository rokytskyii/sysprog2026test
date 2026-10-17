#include "mpu6050.h"
#include <math.h>

#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

bool mpu6050_init(mpu6050_t *dev, const i2c_interface_t *bus, uint8_t dev_addr)
{
    if (!dev || !bus)
        return false;
    dev->bus = bus;
    dev->dev_addr = dev_addr;

    uint8_t id = 0;
    if (!mpu6050_read_id(dev, &id) || id != MPU6050_WHO_AM_I_VAL)
    {
        return false;
    }

    // Пробудження чіпа: скидання біта SLEEP у регістрі PWR_MGMT_1 (0x6B)
    uint8_t pwr_wake = 0x00;
    return dev->bus->write_reg(dev->dev_addr, REG_PWR_MGMT_1, &pwr_wake, 1);
}

bool mpu6050_read_id(mpu6050_t *dev, uint8_t *id)
{
    if (!dev || !dev->bus || !id)
        return false;
    return dev->bus->read_reg(dev->dev_addr, REG_WHO_AM_I, id, 1);
}

bool mpu6050_read_accel(mpu6050_t *dev, mpu6050_accel_data_t *accel_data)
{
    if (!dev || !dev->bus || !accel_data)
        return false;

    uint8_t raw[6] = {0};
    if (!dev->bus->read_reg(dev->dev_addr, REG_ACCEL_XOUT_H, raw, 6))
    {
        return false;
    }

    int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);

    // При шкалі +/- 2g чутливість становить 16384 LSB/g
    accel_data->ax = (float)raw_ax / 16384.0f;
    accel_data->ay = (float)raw_ay / 16384.0f;
    accel_data->az = (float)raw_az / 16384.0f;

    accel_data->a_total = sqrtf(accel_data->ax * accel_data->ax +
                                accel_data->ay * accel_data->ay +
                                accel_data->az * accel_data->az);
    return true;
}