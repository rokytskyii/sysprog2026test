/**
 * @file bus_interfaces.h
 * @brief Апаратні абстракції інтерфейсів шин (I2C, SPI, PWM, UART)
 * @note Цей файл НЕ залежить від STM32 HAL і може використовуватись у тестах.
 */

#ifndef BUS_INTERFACES_H
#define BUS_INTERFACES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ==================== 1. Інтерфейс I2C ==================== */
    typedef struct
    {
        /**
         * @brief Зчитування з регістра слейв-пристрою
         * @param dev_addr 7-бітна адреса пристрою
         * @param reg_addr Адреса внутрішнього регістра
         * @param data Буфер для зчитаних даних
         * @param len Кількість байтів
         * @return true у разі успіху, false у разі помилки/NACK
         */
        bool (*read_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);

        /**
         * @brief Запис у регістр слейв-пристрою
         */
        bool (*write_reg)(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len);
    } i2c_interface_t;

    /* ==================== 2. Інтерфейс SCCB (Камера) ==================== */
    typedef struct
    {
        bool (*read_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *val);
        bool (*write_reg)(uint8_t dev_addr, uint8_t reg_addr, uint8_t val);
    } sccb_interface_t;

    /* ==================== 3. Інтерфейс SPI ==================== */
    typedef struct
    {
        /** Керування лінією Chip Select (активний 0) */
        void (*cs_select)(void);
        void (*cs_deselect)(void);

        /** Повнодуплексна передача блоку даних */
        bool (*transfer)(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len);
    } spi_interface_t;

    /* ==================== 4. Інтерфейс ШІМ (PWM) ==================== */
    typedef struct
    {
        /** Встановлення тривалості активного імпульсу у мікросекундах */
        void (*set_pulse_us)(uint16_t pulse_us);
    } pwm_interface_t;

#ifdef __cplusplus
}
#endif

#endif /* BUS_INTERFACES_H */