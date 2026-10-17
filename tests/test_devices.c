#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <math.h>

#include "bus_interfaces.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "w25q64.h"

/* ========================================================================= */
/* 1. ТЕСТ BMP180: ЕТАЛОННИЙ РОЗРАХУНОК З ДАТАШИТУ BOSCH (SECTION 3.5)       */
/* ========================================================================= */
static void test_bmp180_calibration_datasheet_example(void)
{
    printf("[RUN] test_bmp180_calibration_datasheet_example...\n");

    // Еталонні калібрувальні коефіцієнти з розділу 3.5 документа BST-BMP180-DS000-09
    bmp180_calib_t calib = {
        .ac5 = 32757,
        .ac6 = 23153,
        .mc = -8711,
        .md = 2868};

    // Еталонне значення некомпенсованої температури з даташиту
    int32_t raw_ut = 27898;

    // Розрахунок за формулою драйвера
    int16_t temp_x10 = bmp180_calculate_temperature(raw_ut, &calib);

    // Очікуваний результат з даташиту Bosch: рівно 15.0 °C (у десятих долях: 150)
    assert(temp_x10 == 150);

    printf("      Дані даташиту Bosch: UT=27898, AC5=32757, AC6=23153, MC=-8711, MD=2868\n");
    printf("      Отримано: %d.%d C | Очікувалось: 15.0 C -> [100%% MATCH]\n",
           temp_x10 / 10, abs(temp_x10 % 10));
    printf("[PASS] test_bmp180_calibration_datasheet_example\n\n");
}

/* ========================================================================= */
/* 2. ТЕСТ MPU-6050: ВЕКТОР З РЕАЛЬНИМИ ДАНИМИ З ЛОГІЧНОГО АНАЛІЗАТОРА       */
/* ========================================================================= */
static uint8_t s_mock_mpu_reg = 0;
static uint8_t s_mock_mpu_bytes[6] = {0};

static bool mock_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    (void)dev_addr;
    if (reg_addr == 0x75)
    { // WHO_AM_I
        data[0] = 0x68;
        return true;
    }
    if (reg_addr == 0x3B && len == 6)
    { // ACCEL_X/Y/Z OUT
        memcpy(data, s_mock_mpu_bytes, 6);
        return true;
    }
    return false;
}

static bool mock_i2c_write(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len)
{
    (void)dev_addr;
    s_mock_mpu_reg = reg_addr;
    (void)data;
    (void)len;
    return true;
}

static void test_mpu6050_accel_real_trace_vector(void)
{
    printf("[RUN] test_mpu6050_accel_real_trace_vector...\n");

    i2c_interface_t mock_bus = {
        .read_reg = mock_i2c_read,
        .write_reg = mock_i2c_write};

    mpu6050_t dev;
    assert(mpu6050_init(&dev, &mock_bus, 0x68) == true);
    assert(s_mock_mpu_reg == 0x6B); // Перевірка пробудження через PWR_MGMT_1

    // Тестовий вектор №1: Ідеальна статика 1.00g (Z = 16384 LSB = 0x4000)
    s_mock_mpu_bytes[0] = 0x00;
    s_mock_mpu_bytes[1] = 0x00; // X = 0
    s_mock_mpu_bytes[2] = 0x00;
    s_mock_mpu_bytes[3] = 0x00; // Y = 0
    s_mock_mpu_bytes[4] = 0x40;
    s_mock_mpu_bytes[5] = 0x00; // Z = 16384 LSB (1.00g)

    mpu6050_accel_data_t data;
    assert(mpu6050_read_accel(&dev, &data) == true);
    assert(fabsf(data.a_total - 1.00f) < 0.001f);
    printf("      Вектор 1 (Ідеальна статика): |a| = %.2fg -> OK\n", data.a_total);

    // Тестовий вектор №2: РЕАЛЬНІ БАЙТИ ІЗ ЗАХОПЛЕННЯ SALEAE LOGIC (i2c_sensors_decode.png)
    // Байти з трейсу аналізатора: 0x0B 0xE4 (X), 0xC2 0x38 (Y), 0xEA 0x18 (Z)
    s_mock_mpu_bytes[0] = 0x0B;
    s_mock_mpu_bytes[1] = 0xE4; // X: raw = +3044  (+0.18g)
    s_mock_mpu_bytes[2] = 0xC2;
    s_mock_mpu_bytes[3] = 0x38; // Y: raw = -15816 (-0.96g)
    s_mock_mpu_bytes[4] = 0xEA;
    s_mock_mpu_bytes[5] = 0x18; // Z: raw = -5608  (-0.34g)

    assert(mpu6050_read_accel(&dev, &data) == true);

    // Перевірка: чи розраховує драйвер модуль вектора |a| = sqrt(0.18^2 + (-0.96)^2 + (-0.34)^2) ≈ 1.03g
    printf("      Вектор 2 (Дані з трейсу аналізатора): X:%.2fg, Y:%.2fg, Z:%.2fg\n", data.ax, data.ay, data.az);
    printf("      Розрахований модуль вектора: |a| = %.2fg (PuTTY лог: 1.03g-1.04g) -> [MATCH]\n", data.a_total);
    assert(data.a_total >= 1.01f && data.a_total <= 1.05f);

    printf("[PASS] test_mpu6050_accel_real_trace_vector\n\n");
}

/* ========================================================================= */
/* 3. MOCK-ТЕСТ W25Q64: ПЕРЕВІРКА ПОСЛІДОВНОСТІ КОМАНД SPI FLASH            */
/* ========================================================================= */
#define MOCK_SPI_LOG_SIZE 16
static uint8_t s_spi_log[MOCK_SPI_LOG_SIZE];
static uint8_t s_spi_log_len = 0;
static bool s_mock_cs_pin = false;

static void mock_spi_cs_select(void) { s_mock_cs_pin = true; }
static void mock_spi_cs_deselect(void) { s_mock_cs_pin = false; }

static bool mock_spi_transfer(const uint8_t *tx, uint8_t *rx, uint16_t len)
{
    // Апаратна вимога SPI: передача дозволена виключно при опущеному CS!
    assert(s_mock_cs_pin == true);

    if (tx && len > 0 && s_spi_log_len < MOCK_SPI_LOG_SIZE)
    {
        s_spi_log[s_spi_log_len++] = tx[0]; // Фіксуємо опкод команди
    }

    if (rx && tx && tx[0] == 0x9F && len >= 4)
    {                 // Відповідь JEDEC ID
        rx[1] = 0xEF; // Winbond
        rx[2] = 0x40; // SPI
        rx[3] = 0x17; // 64 Mbit
    }

    if (rx && tx && tx[0] == 0x05)
    {                 // Read Status Register 1
        rx[0] = 0x00; // Чіп вільний (BUSY = 0)
    }

    return true;
}

static void test_w25q64_mock_protocol_sequence(void)
{
    printf("[RUN] test_w25q64_mock_protocol_sequence (SPI Flash Protocol Verification)...\n");

    spi_interface_t mock_spi = {
        .cs_select = mock_spi_cs_select,
        .cs_deselect = mock_spi_cs_deselect,
        .transfer = mock_spi_transfer};

    w25q64_t flash;
    s_spi_log_len = 0;

    // 1. Перевірка ініціалізації
    assert(w25q64_init(&flash, &mock_spi) == true);
    assert(s_spi_log[0] == 0x9F); // Обов'язкове читання JEDEC ID

    // 2. Перевірка послідовності команд запису сторінки
    s_spi_log_len = 0;
    const uint8_t dummy[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    assert(w25q64_page_program(&flash, 0x007000, dummy, 4) == true);

    // Вимога стандарту SPI Flash: команда 0x06 (Write Enable) ПОВИННА передувати 0x02 (Page Program)
    assert(s_spi_log[0] == 0x06); // Write Enable
    assert(s_spi_log[1] == 0x02); // Page Program

    printf("      Послідовність команд на шині SPI:\n");
    printf("      [1] 0x%02X (Write Enable) -> [2] 0x%02X (Page Program) -> [СТРОГО ЗА СТАНДАРТОМ]\n",
           s_spi_log[0], s_spi_log[1]);
    printf("[PASS] test_w25q64_mock_protocol_sequence\n\n");
}

/* ========================================================================= */
/* 4. ТЕСТ ОБРОБКИ ПОМИЛОК ШИНИ (NACK / TIMEOUT / OFFLINE)                   */
/* ========================================================================= */
static bool mock_error_i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    (void)dev_addr;
    (void)reg_addr;
    (void)data;
    (void)len;
    return false; // Імітація апаратної відсутності слейва (NACK) або обриву дроту
}

static void test_bus_error_resilience(void)
{
    printf("[RUN] test_bus_error_resilience...\n");

    i2c_interface_t broken_bus = {
        .read_reg = mock_error_i2c_read,
        .write_reg = mock_i2c_write};

    mpu6050_t dev;
    // При обриві шини драйвер зобов'язаний повернути false, а не зависати в циклі
    bool status = mpu6050_init(&dev, &broken_bus, 0x68);
    assert(status == false);

    printf("      Збій шини (NACK) коректно перехоплено, драйвер повернув статус помилки -> OK\n");
    printf("[PASS] test_bus_error_resilience\n\n");
}

/* ========================================================================= */
/* ТОЧКА ВХОДУ ТЕСТІВ                                                        */
/* ========================================================================= */
int main(void)
{
    printf("=======================================================\n");
    printf("  Unit Tests: Host-based Sensor & Math Verification    \n");
    printf("=======================================================\n\n");

    test_bmp180_calibration_datasheet_example();
    test_mpu6050_accel_real_trace_vector();
    test_w25q64_mock_protocol_sequence();
    test_bus_error_resilience();

    printf("=======================================================\n");
    printf("  ALL 4 UNIT TESTS COMPLETED SUCCESSFULLY! (4 of 4)   \n");
    printf("=======================================================\n");
    return 0;
}