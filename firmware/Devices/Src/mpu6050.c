#include "mpu6050.h"
#include <math.h>

#define REG_SMPLRT_DIV 0x19
#define REG_CONFIG 0x1A
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_CONFIG 0x1C
#define REG_ACCEL_XOUT_H 0x3B
#define REG_PWR_MGMT_1 0x6B
#define REG_WHO_AM_I 0x75

/* ±8g => 4096 LSB/g; ±250 dps => 131 LSB/(deg/s) */
#define ACCEL_LSB_PER_G 4096.0f
#define GYRO_LSB_PER_DPS 131.0f

bool mpu6050_init(mpu6050_t *dev, const i2c_interface_t *bus, uint8_t dev_addr)
{
    if (!dev || !bus)
        return false;
    dev->bus = bus;
    dev->dev_addr = dev_addr;

    uint8_t id = 0;
    if (!mpu6050_read_id(dev, &id) || id != MPU6050_WHO_AM_I_VAL)
        return false;

    uint8_t pwr_wake = 0x00;
    if (!dev->bus->write_reg(dev->dev_addr, REG_PWR_MGMT_1, &pwr_wake, 1))
        return false;

    uint8_t smplrt = 9; /* 1 kHz / (1+9) = 100 Hz */
    if (!dev->bus->write_reg(dev->dev_addr, REG_SMPLRT_DIV, &smplrt, 1))
        return false;

    uint8_t dlpf = 0x03; /* ~44 Hz */
    if (!dev->bus->write_reg(dev->dev_addr, REG_CONFIG, &dlpf, 1))
        return false;

    uint8_t gyro_fs = 0x00; /* ±250 dps */
    if (!dev->bus->write_reg(dev->dev_addr, REG_GYRO_CONFIG, &gyro_fs, 1))
        return false;

    uint8_t accel_fs = 0x10; /* ±8g (AFS_SEL=2) */
    if (!dev->bus->write_reg(dev->dev_addr, REG_ACCEL_CONFIG, &accel_fs, 1))
        return false;

    return true;
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
        return false;

    int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);

    accel_data->ax = (float)raw_ax / ACCEL_LSB_PER_G;
    accel_data->ay = (float)raw_ay / ACCEL_LSB_PER_G;
    accel_data->az = (float)raw_az / ACCEL_LSB_PER_G;
    accel_data->a_total = sqrtf(accel_data->ax * accel_data->ax +
                                accel_data->ay * accel_data->ay +
                                accel_data->az * accel_data->az);
    return true;
}

bool mpu6050_read_motion(mpu6050_t *dev, mpu6050_motion_t *motion)
{
    if (!dev || !dev->bus || !motion)
        return false;

    uint8_t raw[14] = {0};
    if (!dev->bus->read_reg(dev->dev_addr, REG_ACCEL_XOUT_H, raw, 14))
        return false;

    int16_t raw_ax = (int16_t)((raw[0] << 8) | raw[1]);
    int16_t raw_ay = (int16_t)((raw[2] << 8) | raw[3]);
    int16_t raw_az = (int16_t)((raw[4] << 8) | raw[5]);
    int16_t raw_gx = (int16_t)((raw[8] << 8) | raw[9]);
    int16_t raw_gy = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t raw_gz = (int16_t)((raw[12] << 8) | raw[13]);

    motion->accel.ax = (float)raw_ax / ACCEL_LSB_PER_G;
    motion->accel.ay = (float)raw_ay / ACCEL_LSB_PER_G;
    motion->accel.az = (float)raw_az / ACCEL_LSB_PER_G;
    motion->accel.a_total = sqrtf(motion->accel.ax * motion->accel.ax +
                                  motion->accel.ay * motion->accel.ay +
                                  motion->accel.az * motion->accel.az);

    motion->gyro.gx = (float)raw_gx / GYRO_LSB_PER_DPS;
    motion->gyro.gy = (float)raw_gy / GYRO_LSB_PER_DPS;
    motion->gyro.gz = (float)raw_gz / GYRO_LSB_PER_DPS;
    return true;
}
