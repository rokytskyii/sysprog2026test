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
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int fd, char *p, int n)
{
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

// Зчитування для звичайних I2C-сенсорів (MPU, L3G, LSM, BMP)
uint8_t ReadSensorRegister(uint16_t dev_addr, uint8_t reg_addr)
{
  uint8_t val = 0;
  if (HAL_I2C_Mem_Read(&hi2c1, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, &val, 1, 100) == HAL_OK)
  {
    return val;
  }
  return 0xFF;
}

// Зчитування регістрів камери за протоколом SCCB (без Repeated Start)
uint8_t OV7670_ReadReg_SCCB(uint8_t reg_addr)
{
  uint8_t val = 0xFF;

  // 1. Запис адреси регістра із формуванням умови STOP
  if (HAL_I2C_Master_Transmit(&hi2c1, OV7670_WRITE_ADDR, &reg_addr, 1, 50) != HAL_OK)
  {
    return 0xFF;
  }

  // Пауза для перемикання внутрішнього автомата камери
  for (volatile int i = 0; i < 300; i++)
    ;

  // 2. Окреме зчитування значення із формуванням умови STOP
  if (HAL_I2C_Master_Receive(&hi2c1, OV7670_READ_ADDR, &val, 1, 50) != HAL_OK)
  {
    return 0xFF;
  }

  return val;
}

// Зчитування JEDEC ID з Flash-пам'яті W25Q64JV
uint8_t W25Q_ReadJEDEC(uint8_t *id_buf)
{
  uint8_t tx[4] = {0x9F, 0x00, 0x00, 0x00};
  uint8_t rx[4] = {0};

  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 100);
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    id_buf[0] = rx[1]; // Manufacturer ID (Winbond = 0xEF)
    id_buf[1] = rx[2]; // Memory Type (0x40)
    id_buf[2] = rx[3]; // Capacity (64Mbit = 0x17)
    return 1;
  }
  return 0;
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
  printf("\r\n==================================\r\n");
  printf("System Bring-Up & Peripheral Check\r\n");
  printf("==================================\r\n");

  if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY)
  {
    printf("[HAL_OK] I2C1 Bus Initialized (PB8/PB9 @ 100kHz)\r\n");
  }
  else
  {
    printf("[ERROR]  I2C1 Init Failed!\r\n");
  }

  if (HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_READY)
  {
    printf("[HAL_OK] SPI2 Bus Initialized (PB13-PB15)\r\n");
  }
  else
  {
    printf("[ERROR]  SPI2 Init Failed!\r\n");
  }

  if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_READY)
  {
    printf("[HAL_OK] USART1 Initialized for FT232RL (PA9/PA10 @ 115200)\r\n");
  }
  else
  {
    printf("[ERROR]  USART1 Init Failed!\r\n");
  }

  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

  if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) == HAL_OK)
  {
    printf("[HAL_OK] TIM3 PWM CH1 Started (50Hz on PA6)\r\n");
  }
  else
  {
    printf("[ERROR]  TIM3 PWM Start Failed!\r\n");
  }

  // Опитування ідентифікаторів камери через протокол SCCB
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

  printf("Starting diagnostic loop for sensors, camera & FT232RL echo test...\r\n");
  printf("Tip: type any key in PuTTY (FT232RL COM port) to test RX line!\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint8_t step = 0;
  uint32_t last_sensor_tick = 0;
  uint8_t rx_byte = 0;

  while (1)
  {
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE))
    {
      __HAL_UART_CLEAR_OREFLAG(&huart1);
    }

    if (HAL_UART_Receive(&huart1, &rx_byte, 1, 5) == HAL_OK)
    {
      printf("\r\n>>> [FT232RL RX Echo]: Received '%c' (ASCII %d / 0x%02X) <<<\r\n\r\n", rx_byte, rx_byte, rx_byte);
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }

    if (HAL_GetTick() - last_sensor_tick >= 1500)
    {
      last_sensor_tick = HAL_GetTick();

      uint8_t mpu_id = ReadSensorRegister(0x68 << 1, 0x75);
      uint8_t l3g_id = ReadSensorRegister(0x6B << 1, 0x0F);
      uint8_t lsm_id = ReadSensorRegister(0x1D << 1, 0x0F);
      uint8_t bmp_id = ReadSensorRegister(0x77 << 1, 0xD0);

      // Опитування регістрів камери за протоколом SCCB
      uint8_t cam_pid = OV7670_ReadReg_SCCB(OV7670_REG_PID);
      uint8_t cam_ver = OV7670_ReadReg_SCCB(OV7670_REG_VER);

      uint8_t flash_id[3] = {0, 0, 0};
      uint8_t flash_ok = W25Q_ReadJEDEC(flash_id);

      uint16_t servo_pulse = 1500;
      const char *angle_str = "90 deg";

      if (step == 0)
      {
        servo_pulse = 1000;
        angle_str = " 0 deg";
        step = 1;
      }
      else if (step == 1)
      {
        servo_pulse = 1500;
        angle_str = "90 deg";
        step = 2;
      }
      else
      {
        servo_pulse = 2000;
        angle_str = "180 deg";
        step = 0;
      }

      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, servo_pulse);

      printf("Tick %lu | MPU: 0x%02X | L3G: 0x%02X | LSM: 0x%02X | BMP: 0x%02X | CAM: 0x%02X:0x%02X | Flash: [%s] | Servo: %s (Pulse %u)\r\n",
             HAL_GetTick(),
             mpu_id,
             l3g_id,
             lsm_id,
             bmp_id,
             cam_pid,
             cam_ver,
             flash_ok ? "OK" : "ERR",
             angle_str,
             servo_pulse);

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
}
#endif /* USE_FULL_ASSERT */