#ifndef BMP180_H
#define BMP180_H

#include "bus_interfaces.h"

#define BMP180_DEFAULT_ADDR 0x77
#define BMP180_CHIP_ID 0x55

typedef struct
{
    uint16_t ac5;
    uint16_t ac6;
    int16_t mc;
    int16_t md;
} bmp180_calib_t;

typedef struct
{
    const i2c_interface_t *bus;
    uint8_t dev_addr;
    bmp180_calib_t calib;
    void (*delay_ms)(uint32_t ms);
} bmp180_t;

bool bmp180_init(bmp180_t *dev, const i2c_interface_t *bus, uint8_t dev_addr, void (*delay_ms)(uint32_t));
bool bmp180_read_id(bmp180_t *dev, uint8_t *id);
bool bmp180_read_temperature(bmp180_t *dev, int16_t *temp_c_x10);

/** Функція розрахунку компенсації температури за формулою Bosch (для тестів на хості) */
int16_t bmp180_calculate_temperature(int32_t ut, const bmp180_calib_t *calib);

#endif /* BMP180_H */