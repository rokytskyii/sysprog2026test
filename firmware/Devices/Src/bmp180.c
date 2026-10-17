#include "bmp180.h"

#define REG_CHIP_ID 0xD0
#define REG_CTRL_MEAS 0xF4
#define REG_OUT_MSB 0xF6
#define CMD_TEMP 0x2E

#define REG_CALIB_AC5 0xB2
#define REG_CALIB_AC6 0xB4
#define REG_CALIB_MC 0xBC
#define REG_CALIB_MD 0xBE

int16_t bmp180_calculate_temperature(int32_t ut, const bmp180_calib_t *calib)
{
    // Офіційний алгоритм із даташиту Bosch Sensortec BMP180 (Section 3.5)
    int32_t x1 = ((ut - (int32_t)calib->ac6) * (int32_t)calib->ac5) >> 15;
    int32_t x2 = ((int32_t)calib->mc << 11) / (x1 + calib->md);
    int32_t b5 = x1 + x2;
    int16_t t = (int16_t)((b5 + 8) >> 4); // Повертає температуру в 0.1 °C
    return t;
}

bool bmp180_init(bmp180_t *dev, const i2c_interface_t *bus, uint8_t dev_addr, void (*delay_ms)(uint32_t))
{
    if (!dev || !bus || !delay_ms)
        return false;
    dev->bus = bus;
    dev->dev_addr = dev_addr;
    dev->delay_ms = delay_ms;

    uint8_t id = 0;
    if (!bmp180_read_id(dev, &id) || id != BMP180_CHIP_ID)
    {
        return false;
    }

    uint8_t buf[2] = {0};

    // Читання калібрувальних коефіцієнтів
    if (!dev->bus->read_reg(dev->dev_addr, REG_CALIB_AC5, buf, 2))
        return false;
    dev->calib.ac5 = (uint16_t)((buf[0] << 8) | buf[1]);

    if (!dev->bus->read_reg(dev->dev_addr, REG_CALIB_AC6, buf, 2))
        return false;
    dev->calib.ac6 = (uint16_t)((buf[0] << 8) | buf[1]);

    if (!dev->bus->read_reg(dev->dev_addr, REG_CALIB_MC, buf, 2))
        return false;
    dev->calib.mc = (int16_t)((buf[0] << 8) | buf[1]);

    if (!dev->bus->read_reg(dev->dev_addr, REG_CALIB_MD, buf, 2))
        return false;
    dev->calib.md = (int16_t)((buf[0] << 8) | buf[1]);

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
    {
        return false;
    }

    dev->delay_ms(5); // Апаратне очікування АЦП (4.5 мс)

    uint8_t buf[2] = {0};
    if (!dev->bus->read_reg(dev->dev_addr, REG_OUT_MSB, buf, 2))
    {
        return false;
    }

    int32_t ut = (int32_t)((buf[0] << 8) | buf[1]);
    *temp_c_x10 = bmp180_calculate_temperature(ut, &dev->calib);
    return true;
}