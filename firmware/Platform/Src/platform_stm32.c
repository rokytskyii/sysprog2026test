#include "platform_stm32.h"
#include "main.h"

/* Зовнішні дескриптори периферії CubeMX HAL */
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;

/* ==================== Реалізація I2C1 ==================== */
static bool stm32_i2c1_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
    if (!data || len == 0)
        return false;
    return (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(dev_addr << 1), reg_addr,
                             I2C_MEMADD_SIZE_8BIT, data, len, 100) == HAL_OK);
}

static bool stm32_i2c1_write_reg(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t len)
{
    if (!data || len == 0)
        return false;
    return (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(dev_addr << 1), reg_addr,
                              I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, 100) == HAL_OK);
}

static const i2c_interface_t s_i2c1_interface = {
    .read_reg = stm32_i2c1_read_reg,
    .write_reg = stm32_i2c1_write_reg};

/* ==================== Реалізація SCCB (Камера OV7670) ==================== */
static bool stm32_sccb_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *val)
{
    if (!val)
        return false;
    uint8_t write_addr = (uint8_t)(dev_addr << 1);         // 0x42
    uint8_t read_addr = (uint8_t)((dev_addr << 1) | 0x01); // 0x43

    if (HAL_I2C_Master_Transmit(&hi2c1, write_addr, &reg_addr, 1, 50) != HAL_OK)
    {
        return false;
    }

    for (volatile int i = 0; i < 300; i++)
    {
    } // Затримка між фазами SCCB

    if (HAL_I2C_Master_Receive(&hi2c1, read_addr, val, 1, 50) != HAL_OK)
    {
        return false;
    }
    return true;
}

static bool stm32_sccb_write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t val)
{
    uint8_t buf[2] = {reg_addr, val};
    return (HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(dev_addr << 1), buf, 2, 50) == HAL_OK);
}

static const sccb_interface_t s_sccb_interface = {
    .read_reg = stm32_sccb_read_reg,
    .write_reg = stm32_sccb_write_reg};

/* ==================== Реалізація SPI2 (Flash W25Q64) ==================== */
static void stm32_spi2_cs_select(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
}

static void stm32_spi2_cs_deselect(void)
{
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static bool stm32_spi2_transfer(const uint8_t *tx_data, uint8_t *rx_data, uint16_t len)
{
    HAL_StatusTypeDef status;
    if (tx_data && rx_data)
    {
        status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx_data, rx_data, len, 100);
    }
    else if (tx_data)
    {
        status = HAL_SPI_Transmit(&hspi2, (uint8_t *)tx_data, len, 100);
    }
    else if (rx_data)
    {
        status = HAL_SPI_Receive(&hspi2, rx_data, len, 100);
    }
    else
    {
        return false;
    }
    return (status == HAL_OK);
}

static const spi_interface_t s_spi2_interface = {
    .cs_select = stm32_spi2_cs_select,
    .cs_deselect = stm32_spi2_cs_deselect,
    .transfer = stm32_spi2_transfer};

/* ==================== Реалізація PWM (Сервопривід) ==================== */
static void stm32_servo_set_pulse_us(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}

static const pwm_interface_t s_servo_pwm_interface = {
    .set_pulse_us = stm32_servo_set_pulse_us};

/* ==================== Експорт інтерфейсів ==================== */
void platform_init(void)
{
    // Апаратна деактивація CS перед першим обміном
    stm32_spi2_cs_deselect();
}

const i2c_interface_t *platform_get_i2c1(void)
{
    return &s_i2c1_interface;
}

const sccb_interface_t *platform_get_sccb(void)
{
    return &s_sccb_interface;
}

const spi_interface_t *platform_get_spi2(void)
{
    return &s_spi2_interface;
}

const pwm_interface_t *platform_get_servo_pwm(void)
{
    return &s_servo_pwm_interface;
}

void platform_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}