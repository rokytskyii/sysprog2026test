/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - Lab 2 Sensor & SPI Flash Bring-Up
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define OV7670_WRITE_ADDR 0x42
#define OV7670_READ_ADDR 0x43
#define OV7670_REG_PID 0x0A
#define OV7670_REG_VER 0x0B

#define MPU6050_ADDR (0x68 << 1)
#define L3GD20_ADDR (0x6B << 1)
#define LSM303D_ADDR (0x1D << 1)
#define BMP180_ADDR (0x77 << 1)

#define FLASH_TEST_ADDR 0x00700000 // Тестовий сектор у верхній пам'яті (7 МБ)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Калібрувальні коефіцієнти BMP180 для розрахунку температури
static uint16_t bmp_AC5 = 0;
static uint16_t bmp_AC6 = 0;
static int16_t bmp_MC = 0;
static int16_t bmp_MD = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void I2C1_Bus_Clear(void);
uint8_t OV7670_ReadReg_SCCB(uint8_t reg_addr);
uint8_t ReadSensorRegister(uint16_t dev_addr, uint8_t reg_addr);
uint8_t WriteSensorRegister(uint16_t dev_addr, uint8_t reg_addr, uint8_t val);
uint8_t W25Q_ReadJEDEC(uint8_t *id_buf);
static void W25Q_WriteEnable(void);
static void W25Q_WaitBusy(void);
uint8_t W25Q_PerformSelfTest(void);
static void BMP180_ReadCalibration(void);
int16_t BMP180_GetTemperature(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int fd, char *p, int n)
{
  (void)fd;
  HAL_UART_Transmit(&huart2, (uint8_t *)p, n, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)p, n, 100);
  return n;
}

// Функція апаратного розблокування шини I2C
static void I2C1_Bus_Clear(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // PB8 -> SCL як вихід Open-Drain
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // PB9 -> SDA як вхід з підтяжкою
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // 9 імпульсів тактування SCL для виведення слейвів зі стану передачі біта
  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
    for (volatile int j = 0; j < 50; j++)
      ;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    for (volatile int j = 0; j < 50; j++)
      ;
  }

  // Генерація умови STOP
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
  for (volatile int j = 0; j < 50; j++)
    ;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
  for (volatile int j = 0; j < 50; j++)
    ;
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
  for (volatile int j = 0; j < 50; j++)
    ;
}

// Зчитування регістра I2C-сенсора
uint8_t ReadSensorRegister(uint16_t dev_addr, uint8_t reg_addr)
{
  uint8_t val = 0xFF;
  if (HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK)
  {
    return val;
  }
  return 0xFF;
}

// Запис регістра I2C-сенсора
uint8_t WriteSensorRegister(uint16_t dev_addr, uint8_t reg_addr, uint8_t val)
{
  return (HAL_I2C_Mem_Write(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK);
}

// Зчитування регістрів камери за протоколом SCCB (з умовою STOP замість Repeated Start)
uint8_t OV7670_ReadReg_SCCB(uint8_t reg_addr)
{
  uint8_t val = 0xFF;

  if (HAL_I2C_Master_Transmit(&hi2c1, OV7670_WRITE_ADDR, &reg_addr, 1, 50) != HAL_OK)
  {
    return 0xFF;
  }

  for (volatile int i = 0; i < 300; i++)
    ;

  if (HAL_I2C_Master_Receive(&hi2c1, OV7670_READ_ADDR, &val, 1, 50) != HAL_OK)
  {
    return 0xFF;
  }

  return val;
}

// Допоміжні функції W25Q64
static void W25Q_WriteEnable(void)
{
  uint8_t cmd = 0x06;
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
}

static void W25Q_WaitBusy(void)
{
  uint8_t cmd = 0x05;
  uint8_t status = 0;
  do
  {
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, &cmd, 1, 100);
    HAL_SPI_Receive(&hspi2, &status, 1, 100);
    HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  } while (status & 0x01);
}

uint8_t W25Q_ReadJEDEC(uint8_t *id_buf)
{
  uint8_t tx[4] = {0x9F, 0x00, 0x00, 0x00};
  uint8_t rx[4] = {0};

  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    id_buf[0] = rx[1]; // Winbond = 0xEF
    id_buf[1] = rx[2]; // Type = 0x40
    id_buf[2] = rx[3]; // Capacity = 0x17
    return (id_buf[0] == 0xEF && id_buf[1] == 0x40 && id_buf[2] == 0x17);
  }
  return 0;
}

// Тест побайтового збігу запису/читання W25Q64
uint8_t W25Q_PerformSelfTest(void)
{
  uint8_t test_tx[8] = {0x02, (FLASH_TEST_ADDR >> 16) & 0xFF, (FLASH_TEST_ADDR >> 8) & 0xFF, FLASH_TEST_ADDR & 0xFF,
                        0xDE, 0xAD, 0xBE, 0xEF};
  uint8_t test_rx[4] = {0};
  uint8_t erase_cmd[4] = {0x20, (FLASH_TEST_ADDR >> 16) & 0xFF, (FLASH_TEST_ADDR >> 8) & 0xFF, FLASH_TEST_ADDR & 0xFF};
  uint8_t read_cmd[4] = {0x03, (FLASH_TEST_ADDR >> 16) & 0xFF, (FLASH_TEST_ADDR >> 8) & 0xFF, FLASH_TEST_ADDR & 0xFF};

  // 1. Sector Erase 4KB
  W25Q_WriteEnable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, erase_cmd, 4, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  W25Q_WaitBusy();

  // 2. Page Program
  W25Q_WriteEnable();
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, test_tx, 8, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);
  W25Q_WaitBusy();

  // 3. Read Data
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2, read_cmd, 4, 100);
  HAL_SPI_Receive(&hspi2, test_rx, 4, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  return (memcmp(&test_tx[4], test_rx, 4) == 0);
}

// Зчитування калібрувальних коефіцієнтів BMP180
static void BMP180_ReadCalibration(void)
{
  uint8_t buf[2] = {0};

  // AC5 (0xB2)
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, 0xB2, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
  bmp_AC5 = (uint16_t)((buf[0] << 8) | buf[1]);

  // AC6 (0xB4)
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, 0xB4, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
  bmp_AC6 = (uint16_t)((buf[0] << 8) | buf[1]);

  // MC (0xBC)
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, 0xBC, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
  bmp_MC = (int16_t)((buf[0] << 8) | buf[1]);

  // MD (0xBE)
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, 0xBE, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
  bmp_MD = (int16_t)((buf[0] << 8) | buf[1]);
}

// Отримання температури з BMP180 за офіційною формулою Bosch BST-BMP180
int16_t BMP180_GetTemperature(void)
{
  uint8_t cmd = 0x2E;
  uint8_t buf[2] = {0};

  // 1. Запуск перетворення температури
  HAL_I2C_Mem_Write(&hi2c1, BMP180_ADDR, 0xF4, I2C_MEMADD_SIZE_8BIT, &cmd, 1, 50);
  HAL_Delay(5); // Очікування завершення вимірювання (4.5 мс по даташиту)

  // 2. Зчитування сирого значення UT
  HAL_I2C_Mem_Read(&hi2c1, BMP180_ADDR, 0xF6, I2C_MEMADD_SIZE_8BIT, buf, 2, 50);
  int32_t UT = (int32_t)((buf[0] << 8) | buf[1]);

  // 3. Формула розрахунку температури (Розділ 3.5 даташиту BMP180)
  int32_t X1 = ((UT - (int32_t)bmp_AC6) * (int32_t)bmp_AC5) >> 15;
  int32_t X2 = ((int32_t)bmp_MC << 11) / (X1 + bmp_MD);
  int32_t B5 = X1 + X2;
  int16_t T = (int16_t)((B5 + 8) >> 4); // Температура в 0.1 °C

  return T;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  // 1. Апаратне очищення шини I2C
  I2C1_Bus_Clear();

  // 2. Активація генерації такту MCO1 16 МГц на PA8 (MCLK) перед увімкненням периферії
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStructMCO = {0};
  GPIO_InitStructMCO.Pin = GPIO_PIN_8;
  GPIO_InitStructMCO.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStructMCO.Pull = GPIO_NOPULL;
  GPIO_InitStructMCO.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStructMCO.Alternate = GPIO_AF0_MCO;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructMCO);

  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
  HAL_Delay(50); // Пауза для стабілізації камери

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  printf("\r\n=======================================================\r\n");
  printf("  Incident Recorder: Black Box Bring-Up & Self-Test    \r\n");
  printf("=======================================================\r\n");

  // Перевірка шини I2C1
  if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
  {
    printf("[HAL_OK] I2C1 Bus Initialized (PB8/PB9 @ 100kHz)\r\n");
  }
  else
  {
    printf("[ERROR]  I2C1 Init Failed!\r\n");
  }

  // Перевірка шини SPI2
  if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_READY)
  {
    printf("[HAL_OK] SPI2 Bus Initialized (PB13-PB15)\r\n");
  }
  else
  {
    printf("[ERROR]  SPI2 Init Failed!\r\n");
  }

  // Перевірка телеметрії USART1
  if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
  {
    printf("[HAL_OK] USART1 Initialized for FT232RL (PA9/PA10 @ 115200)\r\n");
  }
  else
  {
    printf("[ERROR]  USART1 Init Failed!\r\n");
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  // Старт ШІМ сервоприводу (50 Гц)
  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK)
  {
    printf("[HAL_OK] TIM3 PWM CH1 Started (50Hz on PA6)\r\n");
  }
  else
  {
    printf("[ERROR]  TIM3 PWM Start Failed!\r\n");
  }

  // --- МІНІМАЛЬНА ІНІЦІАЛІЗАЦІЯ СЕНСОРІВ ДО РОБОЧОГО СТАНУ (Вимога 4.2) ---

  // 1. Пробудження MPU-6050 (скидання біта SLEEP у регістрі 0x6B)
  WriteSensorRegister(MPU6050_ADDR, 0x6B, 0x00);

  // 2. Увімкнення L3GD20 (активація нормального режиму та 3 осей: Zen=1, Yen=1, Xen=1, PD=1)
  WriteSensorRegister(L3GD20_ADDR, 0x20, 0x0F);

  // 3. Увімкнення акселерометра LSM303D (50 Hz, BDU=1, all axes active)
  WriteSensorRegister(LSM303D_ADDR, 0x20, 0x57);

  // 4. Зчитування калібрування BMP180
  BMP180_ReadCalibration();

  // 5. Опитування ідентифікаторів камери OV7670 через SCCB
  uint8_t init_cam_pid = OV7670_ReadReg_SCCB(OV7670_REG_PID);
  uint8_t init_cam_ver = OV7670_ReadReg_SCCB(OV7670_REG_VER);
  if (init_cam_pid == 0x76)
  {
    printf("[HAL_OK] OV7670 Camera Detected (PID: 0x%02X, VER: 0x%02X)\r\n", init_cam_pid, init_cam_ver);
  }
  else
  {
    printf("[WARN]   OV7670 Check: PID=0x%02X, VER=0x%02X (Expected 0x76, 0x70)\r\n", init_cam_pid, init_cam_ver);
  }

  // 6. Тест Flash W25Q64: читання JEDEC ID та тест запису/читання побайтного збігу
  uint8_t jedec[3] = {0};
  uint8_t jedec_ok = W25Q_ReadJEDEC(jedec);
  if (jedec_ok)
  {
    printf("[HAL_OK] Winbond W25Q64JV Detected (JEDEC: %02X %02X %02X)\r\n", jedec[0], jedec[1], jedec[2]);
    uint8_t rw_ok = W25Q_PerformSelfTest();
    printf("[FLASH]  Data Integrity Write/Read Test: [%s]\r\n", rw_ok ? "VERIFIED (100% MATCH)" : "FAILED");
  }
  else
  {
    printf("[ERROR]  Flash JEDEC Read Failed!\r\n");
  }

  printf("\r\nStarting physical plausibility monitoring loop (1.5s period)...\r\n");
  printf("Format: [Time] | Accel Vector Magnitude | BMP180 Temp | Servo Position\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t step = 0;
  uint32_t last_sensor_tick = 0;
  uint8_t rx_byte = 0;

  while (1)
  {
    // Перевірка прийому символів по резервному UART (FT232RL echo)
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
    {
      __HAL_UART_CLEAR_OREFLAG(&huart1);
    }

    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 5) == HAL_OK)
    {
      printf("\r\n>>> [FT232RL Echo]: Received '%c' (ASCII %d / 0x%02X) <<<\r\n\r\n", rx_byte, rx_byte, rx_byte);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    // Періодичне виведення фізичних даних сенсорів (раз на 1.5 с)
    if (HAL_GetTick() - last_sensor_tick >= 1500)
    {
      last_sensor_tick = HAL_GetTick();

      // 1. Читання ID сенсорів
      uint8_t mpu_id = ReadSensorRegister(MPU6050_ADDR, 0x75);
      uint8_t l3g_id = ReadSensorRegister(L3GD20_ADDR, 0x0F);
      uint8_t lsm_id = ReadSensorRegister(LSM303D_ADDR, 0x0F);
      uint8_t bmp_id = ReadSensorRegister(BMP180_ADDR, 0xD0);

      // 2. Зчитування 6 байтів прискорення MPU-6050 (0x3B..0x40)
      uint8_t raw_acc[6] = {0};
      HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, 0x3B, I2C_MEMADD_SIZE_8BIT, raw_acc, 6, 100);
      int16_t ax = (int16_t)((raw_acc[0] << 8) | raw_acc[1]);
      int16_t ay = (int16_t)((raw_acc[2] << 8) | raw_acc[3]);
      int16_t az = (int16_t)((raw_acc[4] << 8) | raw_acc[5]);

      // Переведення в g (при шкалі +/-2g чутливість 16384 LSB/g)
      float ax_g = (float)ax / 16384.0f;
      float ay_g = (float)ay / 16384.0f;
      float az_g = (float)az / 16384.0f;
      float a_total = sqrtf(ax_g * ax_g + ay_g * ay_g + az_g * az_g);

      // 3. Зчитування реальної температури BMP180
      int16_t temp_x10 = BMP180_GetTemperature();

      // 4. Керування положенням сервоприводу (0 -> 90 -> 180 deg)
      uint16_t servo_pulse = 1500;
      const char *angle_str = "90 deg";

      if (step == 0)
      {
        servo_pulse = 1000;
        angle_str = "  0 deg";
        step = 1;
      }
      else if (step == 1)
      {
        servo_pulse = 1500;
        angle_str = " 90 deg";
        step = 2;
      }
      else
      {
        servo_pulse = 2000;
        angle_str = "180 deg";
        step = 0;
      }
      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse);

      // Друк структурованого рядка (цілочисельний вивід для сумісності з newlib-nano)
      int a_tot_int = (int)a_total;
      int a_tot_dec = (int)(fabsf(a_total - (float)a_tot_int) * 100.0f);

      printf("Tick %5lu | IDs[M:0x%02X L3:0x%02X LSM:0x%02X B:0x%02X] | Accel: |a|=%d.%02dg (X:%d Y:%d Z:%d) | Temp: %d.%d C | Servo: %s (%u us)\r\n",
             HAL_GetTick(),
             mpu_id, l3g_id, lsm_id, bmp_id,
             a_tot_int, a_tot_dec,
             (int)(ax_g * 100.0f), (int)(ay_g * 100.0f), (int)(az_g * 100.0f),
             temp_x10 / 10, abs(temp_x10 % 10),
             angle_str, servo_pulse);

      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FLASH_CS_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif /* USE_FULL_ASSERT */