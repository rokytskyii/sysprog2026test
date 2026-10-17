#include "app.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "w25q64.h"
#include "servo.h"
#include "ov7670.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define L3GD20_ADDR 0x6B
#define LSM303D_ADDR 0x1D
#define FLASH_TEST_ADDR 0x00700000

/* Структури драйверів пристроїв */
static mpu6050_t s_mpu;
static bmp180_t s_bmp;
static w25q64_t s_flash;
static servo_t s_servo;
static ov7670_t s_camera;

static const i2c_interface_t *s_raw_i2c = NULL;
static uint32_t s_last_tick = 0;
static uint8_t s_step = 0;

void app_init(const i2c_interface_t *i2c_bus,
              const sccb_interface_t *sccb_bus,
              const spi_interface_t *spi_bus,
              const pwm_interface_t *pwm_bus,
              void (*delay_ms)(uint32_t))
{
    s_raw_i2c = i2c_bus;

    printf("\r\n=======================================================\r\n");
    printf("  Incident Recorder: Black Box Bring-Up & Self-Test    \r\n");
    printf("  Architecture: Layered (Core -> Platform -> Dev -> App)\r\n");
    printf("=======================================================\r\n");

    // 1. Ініціалізація сервоприводу
    if (servo_init(&s_servo, pwm_bus))
    {
        printf("[OK]     TIM3 PWM Servo Initialized (50Hz on PA6)\r\n");
    }

    // 2. Ініціалізація MPU-6050
    if (mpu6050_init(&s_mpu, i2c_bus, MPU6050_DEFAULT_ADDR))
    {
        printf("[OK]     MPU-6050 Detected and Initialized (Addr: 0x68)\r\n");
    }
    else
    {
        printf("[WARN]   MPU-6050 Init Failed\r\n");
    }

    // 3. Активація L3GD20 та LSM303D (GY-89) через прямі виклики i2c
    uint8_t l3g_wake = 0x0F;
    i2c_bus->write_reg(L3GD20_ADDR, 0x20, &l3g_wake, 1);
    uint8_t lsm_wake = 0x57;
    i2c_bus->write_reg(LSM303D_ADDR, 0x20, &lsm_wake, 1);

    // 4. Ініціалізація барометра BMP180
    if (bmp180_init(&s_bmp, i2c_bus, BMP180_DEFAULT_ADDR, delay_ms))
    {
        printf("[OK]     BMP180 Detected & Calibrated (Addr: 0x77)\r\n");
    }
    else
    {
        printf("[WARN]   BMP180 Init Failed\r\n");
    }

    // 5. Ініціалізація камери OV7670 через SCCB
    uint8_t cam_pid = 0, cam_ver = 0;
    ov7670_init(&s_camera, sccb_bus, OV7670_SCCB_ADDR);
    if (ov7670_read_id(&s_camera, &cam_pid, &cam_ver) && cam_pid == 0x76)
    {
        printf("[OK]     OV7670 Camera Detected (PID: 0x%02X, VER: 0x%02X)\r\n", cam_pid, cam_ver);
    }
    else
    {
        printf("[WARN]   OV7670 Check: PID=0x%02X, VER=0x%02X\r\n", cam_pid, cam_ver);
    }

    // 6. Ініціалізація та Self-Test SPI Flash W25Q64
    uint8_t mfg = 0, type = 0, cap = 0;
    if (w25q64_init(&s_flash, spi_bus) && w25q64_read_jedec_id(&s_flash, &mfg, &type, &cap))
    {
        printf("[OK]     Winbond W25Q64JV Detected (JEDEC: %02X %02X %02X)\r\n", mfg, type, cap);
        bool test_ok = w25q64_self_test(&s_flash, FLASH_TEST_ADDR);
        printf("[FLASH]  Data Integrity Write/Read Test: [%s]\r\n", test_ok ? "VERIFIED (100% MATCH)" : "FAILED");
    }
    else
    {
        printf("[ERROR]  Flash JEDEC Read Failed!\r\n");
    }

    printf("\r\nStarting physical plausibility monitoring loop (1.5s period)...\r\n\r\n");
}

void app_run_step(uint32_t current_tick_ms)
{
    if (current_tick_ms - s_last_tick >= 1500)
    {
        s_last_tick = current_tick_ms;

        // Зчитування ID сенсорів
        uint8_t mpu_id = 0, l3g_id = 0, lsm_id = 0, bmp_id = 0;
        mpu6050_read_id(&s_mpu, &mpu_id);
        bmp180_read_id(&s_bmp, &bmp_id);
        if (s_raw_i2c)
        {
            s_raw_i2c->read_reg(L3GD20_ADDR, 0x0F, &l3g_id, 1);
            s_raw_i2c->read_reg(LSM303D_ADDR, 0x0F, &lsm_id, 1);
        }

        // Зчитування даних акселерометра MPU-6050
        mpu6050_accel_data_t acc = {0};
        mpu6050_read_accel(&s_mpu, &acc);

        // Зчитування температури BMP180
        int16_t temp_x10 = 0;
        bmp180_read_temperature(&s_bmp, &temp_x10);

        // Керування сервоприводом
        const char *angle_str = " 90 deg";
        uint16_t pulse_us = 1500;
        if (s_step == 0)
        {
            servo_set_position(&s_servo, SERVO_ANGLE_0);
            angle_str = "  0 deg";
            pulse_us = SERVO_ANGLE_0;
            s_step = 1;
        }
        else if (s_step == 1)
        {
            servo_set_position(&s_servo, SERVO_ANGLE_90);
            angle_str = " 90 deg";
            pulse_us = SERVO_ANGLE_90;
            s_step = 2;
        }
        else
        {
            servo_set_position(&s_servo, SERVO_ANGLE_180);
            angle_str = "180 deg";
            pulse_us = SERVO_ANGLE_180;
            s_step = 0;
        }

        int a_int = (int)acc.a_total;
        int a_dec = (int)(fabsf(acc.a_total - (float)a_int) * 100.0f);

        printf("Tick %5lu | IDs[M:0x%02X L3:0x%02X LSM:0x%02X B:0x%02X] | Accel: |a|=%d.%02dg (X:%d Y:%d Z:%d) | Temp: %d.%d C | Servo: %s (%u us)\r\n",
               current_tick_ms,
               mpu_id, l3g_id, lsm_id, bmp_id,
               a_int, a_dec,
               (int)(acc.ax * 100.0f), (int)(acc.ay * 100.0f), (int)(acc.az * 100.0f),
               temp_x10 / 10, abs(temp_x10 % 10),
               angle_str, pulse_us);
    }
}