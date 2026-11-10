#include "bmp180.h"

#define REG_CHIP_ID 0xD0
#define REG_CTRL_MEAS 0xF4
#define REG_OUT_MSB 0xF6
#define CMD_TEMP 0x2E
#define CMD_PRESS_OSS0 0x34

#define REG_CALIB_AC1 0xAA

int16_t bmp180_calculate_temperature(int32_t ut, const bmp180_calib_t *calib)
{
    int32_t x1 = ((ut - (int32_t)calib->ac6) * (int32_t)calib->ac5) >> 15;
    int32_t x2 = ((int32_t)calib->mc << 11) / (x1 + calib->md);
    int32_t b5 = x1 + x2;
    return (int16_t)((b5 + 8) >> 4);
}

static bool read_i16(bmp180_t *dev, uint8_t reg, int16_t *out)
{
    uint8_t buf[2] = {0};
    if (!dev->bus->read_reg(dev->dev_addr, reg, buf, 2))
        return false;
    *out = (int16_t)((buf[0] << 8) | buf[1]);
    return true;
}

static bool read_u16(bmp180_t *dev, uint8_t reg, uint16_t *out)
{
    uint8_t buf[2] = {0};
    if (!dev->bus->read_reg(dev->dev_addr, reg, buf, 2))
        return false;
    *out = (uint16_t)((buf[0] << 8) | buf[1]);
    return true;
}

bool bmp180_init(bmp180_t *dev, const i2c_interface_t *bus, uint8_t dev_addr, void (*delay_ms)(uint32_t))
{
    if (!dev || !bus || !delay_ms)
        return false;
    dev->bus = bus;
    dev->dev_addr = dev_addr;
    dev->delay_ms = delay_ms;
    dev->b5 = 0;

    uint8_t id = 0;
    if (!bmp180_read_id(dev, &id) || id != BMP180_CHIP_ID)
        return false;

    if (!read_i16(dev, REG_CALIB_AC1 + 0, &dev->calib.ac1))
        return false;
    if (!read_i16(dev, REG_CALIB_AC1 + 2, &dev->calib.ac2))
        return false;
    if (!read_i16(dev, REG_CALIB_AC1 + 4, &dev->calib.ac3))
        return false;
    if (!read_u16(dev, REG_CALIB_AC1 + 6, &dev->calib.ac4))
        return false;
    if (!read_u16(dev, REG_CALIB_AC1 + 8, &dev->calib.ac5))
        return false;
    if (!read_u16(dev, REG_CALIB_AC1 + 10, &dev->calib.ac6))
        return false;
    if (!read_i16(dev, REG_CALIB_AC1 + 12, &dev->calib.b1))
        return false;
    if (!read_i16(dev, REG_CALIB_AC1 + 14, &dev->calib.b2))
        return false;
    if (!read_i16(dev, REG_CALIB_AC1 + 16, &dev->calib.mb))
        return false;
    if (!read_i16(dev, REG_CALIB_AC1 + 18, &dev->calib.mc))
        return false;
    if (!read_i16(dev, REG_CALIB_AC1 + 20, &dev->calib.md))
        return false;

    return true;
}

bool bmp180_read_id(bmp180_t *dev, uint8_t *id)
{
    if (!dev || !dev->bus || !id)
        return false;
    return dev->bus->read_reg(dev->dev_addr, REG_CHIP_ID, id, 1);
}

bool bmp180_read_temperature(bmp180_t *dev, int16_t *temp_c_x10)
{
    if (!dev || !dev->bus || !dev->delay_ms || !temp_c_x10)
        return false;

    uint8_t cmd = CMD_TEMP;
    if (!dev->bus->write_reg(dev->dev_addr, REG_CTRL_MEAS, &cmd, 1))
        return false;

    dev->delay_ms(5);

    uint8_t buf[2] = {0};
    if (!dev->bus->read_reg(dev->dev_addr, REG_OUT_MSB, buf, 2))
        return false;

    int32_t ut = (int32_t)((buf[0] << 8) | buf[1]);
    int32_t x1 = ((ut - (int32_t)dev->calib.ac6) * (int32_t)dev->calib.ac5) >> 15;
    int32_t x2 = ((int32_t)dev->calib.mc << 11) / (x1 + dev->calib.md);
    dev->b5 = x1 + x2;
    *temp_c_x10 = (int16_t)((dev->b5 + 8) >> 4);
    return true;
}

bool bmp180_read_pressure(bmp180_t *dev, int32_t *pressure_pa)
{
    if (!dev || !dev->bus || !dev->delay_ms || !pressure_pa)
        return false;

    /* Need fresh B5 */
    int16_t t_dummy = 0;
    if (!bmp180_read_temperature(dev, &t_dummy))
        return false;

    uint8_t cmd = CMD_PRESS_OSS0;
    if (!dev->bus->write_reg(dev->dev_addr, REG_CTRL_MEAS, &cmd, 1))
        return false;
    dev->delay_ms(5);

    uint8_t buf[3] = {0};
    if (!dev->bus->read_reg(dev->dev_addr, REG_OUT_MSB, buf, 3))
        return false;

    int32_t up = ((int32_t)buf[0] << 16 | (int32_t)buf[1] << 8 | buf[2]) >> 8;

    const bmp180_calib_t *c = &dev->calib;
    int32_t b6 = dev->b5 - 4000;
    int32_t x1 = (c->b2 * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (c->ac2 * b6) >> 11;
    int32_t x3 = x1 + x2;
    int32_t b3 = (((((int32_t)c->ac1) * 4 + x3) << 0) + 2) >> 2;
    x1 = (c->ac3 * b6) >> 13;
    x2 = (c->b1 * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    uint32_t b4 = (c->ac4 * (uint32_t)(x3 + 32768)) >> 15;
    uint32_t b7 = ((uint32_t)up - (uint32_t)b3) * 50000u;

    int32_t p;
    if (b7 < 0x80000000u)
        p = (int32_t)((b7 << 1) / b4);
    else
        p = (int32_t)((b7 / b4) << 1);

    x1 = (p >> 8) * (p >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * p) >> 16;
    *pressure_pa = p + ((x1 + x2 + 3791) >> 4);
    return true;
}
