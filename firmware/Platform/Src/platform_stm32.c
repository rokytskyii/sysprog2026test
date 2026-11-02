#include "platform_stm32.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

/* ==================== I2C1 ==================== */
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

/* ==================== SCCB ==================== */
static bool stm32_sccb_read_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t *val)
{
    if (!val)
        return false;
    uint8_t write_addr = (uint8_t)(dev_addr << 1);
    uint8_t read_addr = (uint8_t)((dev_addr << 1) | 0x01);

    if (HAL_I2C_Master_Transmit(&hi2c1, write_addr, &reg_addr, 1, 50) != HAL_OK)
        return false;

    for (volatile int i = 0; i < 300; i++)
    {
    }

    if (HAL_I2C_Master_Receive(&hi2c1, read_addr, val, 1, 50) != HAL_OK)
        return false;
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

/* ==================== SPI2 ==================== */
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
        status = HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx_data, rx_data, len, 100);
    else if (tx_data)
        status = HAL_SPI_Transmit(&hspi2, (uint8_t *)tx_data, len, 100);
    else if (rx_data)
        status = HAL_SPI_Receive(&hspi2, rx_data, len, 100);
    else
        return false;
    return (status == HAL_OK);
}

static const spi_interface_t s_spi2_interface = {
    .cs_select = stm32_spi2_cs_select,
    .cs_deselect = stm32_spi2_cs_deselect,
    .transfer = stm32_spi2_transfer};

/* ==================== PWM ==================== */
static void stm32_servo_set_pulse_us(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse_us);
}

static const pwm_interface_t s_servo_pwm_interface = {
    .set_pulse_us = stm32_servo_set_pulse_us};

/* ==================== Board IO ==================== */
static bool stm32_button_is_pressed(void)
{
    return (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET);
}

static void stm32_led_write(bool on)
{
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void stm32_led_toggle(void)
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

static const board_io_interface_t s_board_io = {
    .button_is_pressed = stm32_button_is_pressed,
    .led_write = stm32_led_write,
    .led_toggle = stm32_led_toggle};

/* ==================== Camera parallel (PC0-7, PB0=PCLK, PA4=HREF, PB1=VSYNC) ==================== */
#define CAM_PCLK_HIGH() ((GPIOB->IDR & GPIO_PIN_0) != 0u)
#define CAM_HREF_HIGH() ((GPIOA->IDR & GPIO_PIN_4) != 0u)
#define CAM_VSYNC_HIGH() ((GPIOB->IDR & GPIO_PIN_1) != 0u)
#define CAM_DATA8() ((uint8_t)(GPIOC->IDR & 0xFFu))

static void camera_gpio_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLDOWN;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 |
               GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_4;
    HAL_GPIO_Init(GPIOA, &gpio);
}

static bool wait_ms_deadline(uint32_t start, uint32_t timeout_ms)
{
    return (HAL_GetTick() - start) <= timeout_ms;
}

/**
 * QQVGA grayscale capture.
 * Критично: у циклі по PCLK НЕ можна викликати HAL_GetTick()/printf —
 * це зриває фронти і дає випадковий TIMEOUT. Таймаут лише на синхронізації
 * рядка/кадру; під час рядка — IRQ вимкнені.
 */
static bool stm32_camera_capture_gray(uint8_t *dst, uint16_t width, uint16_t height, uint32_t timeout_ms)
{
    if (!dst || width == 0 || height == 0)
        return false;

    const uint32_t t0 = HAL_GetTick();

    /* Дочекатись кінця поточного кадру (VSYNC high), потім старту нового (low) */
    while (!CAM_VSYNC_HIGH())
    {
        if (!wait_ms_deadline(t0, timeout_ms))
            return false;
    }
    while (CAM_VSYNC_HIGH())
    {
        if (!wait_ms_deadline(t0, timeout_ms))
            return false;
    }

    for (uint16_t row = 0; row < height; row++)
    {
        while (!CAM_HREF_HIGH())
        {
            if (!wait_ms_deadline(t0, timeout_ms))
                return false;
        }

        uint8_t *line = &dst[(uint32_t)row * width];
        __disable_irq();
        for (uint16_t col = 0; col < width; col++)
        {
            while (!CAM_PCLK_HIGH())
            {
            }
            line[col] = CAM_DATA8();
            while (CAM_PCLK_HIGH())
            {
            }

            while (!CAM_PCLK_HIGH())
            {
            }
            while (CAM_PCLK_HIGH())
            {
            }
        }
        __enable_irq();

        while (CAM_HREF_HIGH())
        {
            if (!wait_ms_deadline(t0, timeout_ms))
                return false;
        }
    }

    return true;
}

static const camera_parallel_interface_t s_camera_parallel = {
    .capture_gray = stm32_camera_capture_gray};

/* ==================== CLI UART RX (USART2) ==================== */
static bool stm32_cli_read_byte(uint8_t *out)
{
    if (!out)
        return false;
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) == RESET)
        return false;
    *out = (uint8_t)(huart2.Instance->DR & 0xFFu);
    return true;
}

static const uart_rx_interface_t s_cli_uart = {
    .read_byte = stm32_cli_read_byte};

/* ==================== Export ==================== */
void platform_init(void)
{
    stm32_spi2_cs_deselect();
    camera_gpio_init();
}

const i2c_interface_t *platform_get_i2c1(void) { return &s_i2c1_interface; }
const sccb_interface_t *platform_get_sccb(void) { return &s_sccb_interface; }
const spi_interface_t *platform_get_spi2(void) { return &s_spi2_interface; }
const pwm_interface_t *platform_get_servo_pwm(void) { return &s_servo_pwm_interface; }
const board_io_interface_t *platform_get_board_io(void) { return &s_board_io; }
const camera_parallel_interface_t *platform_get_camera_parallel(void) { return &s_camera_parallel; }
const uart_rx_interface_t *platform_get_cli_uart(void) { return &s_cli_uart; }

void platform_delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}
