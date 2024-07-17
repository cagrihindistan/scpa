/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Max31865.h"
#include<stdio.h>
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* --------- DEFINE FOR MOTOR DRIVER --------- */
#define ARR_Value 10800

/* --------- DEFINE FOR TEMPERATURE SENSOR --------- */
#define RREF 430.0
#define RNOMINAL 100.0
#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

uint8_t USB_TX_Buf[50];
uint8_t USB_RX_Buf[50];

FLOATUNION_t data1_send;
FLOATUNION_t data2_send;

FLOATUNION_t data1_rec;
FLOATUNION_t data2_rec;
FLOATUNION_t data3_rec;
FLOATUNION_t data4_rec;
uint8_t usb_read_error;

float time;
uint16_t CCR_Value;
uint16_t Duty_Cycle;
float Voltage;

/* --------- VARIABLE FOR TEMPERATURE SENSOR --------- */

float temperature_t = 0.0;
uint16_t rtd = 0;
float ratio;
bool initialized = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* --------- FUNCTION FOR TEMPERATURE SENSOR --------- */

void begin_t();
uint16_t readRTD2(void);
float calculateTemperature2(uint16_t RTDraw, float RTDnominal,
		float refResistor);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim5);


  /* --------- CONFIG FOR TEMPERATURE SENSOR --------- */
	void begin_t() {
		max31865_numwires_t wires = MAX31865_2WIRE;  // Varsayılan wires değeri

		if (HAL_SPI_Init(&hspi1) == HAL_OK) {
			initialized = true;
		} else {
			initialized = false;
		}

		uint8_t t;  // t değişkeni burada tanımlandı

		// readRegister8 fonksiyonunun içeriği doğrudan eklendi
		uint8_t addr = MAX31865_CONFIG_REG; // Adres değişkeni
		addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_DELAY);
		HAL_SPI_Receive(&hspi1, &t, 1, SPI_DELAY); // t değişkeni burada doğrudan kullanıldı
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		if (wires == MAX31865_3WIRE) {
			t |= MAX31865_CONFIG_3WIRE;
		} else {
			t &= ~MAX31865_CONFIG_3WIRE;
		}

		// writeRegister8 fonksiyonunun içeriği doğrudan eklendi
		addr = MAX31865_CONFIG_REG; // Adres değişkeni
		addr |= 0x80; // MSB=1 for write, make sure top bit is set

		uint8_t buffer[2] = { addr, t };

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		// enableBias fonksiyonunun içeriği doğrudan eklendi
		t = t; // Okunan değeri kullanarak devam ediyoruz
		bool b = false; // Enable or disable bias
		if (b) {
			t |= MAX31865_CONFIG_BIAS; // Enable bias
		} else {
			t &= ~MAX31865_CONFIG_BIAS; // Disable bias
		}

		addr = MAX31865_CONFIG_REG; // Adres değişkeni
		addr |= 0x80; // MSB=1 for write, make sure top bit is set

		buffer[0] = addr;
		buffer[1] = t;

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		// autoConvert fonksiyonunun içeriği doğrudan eklendi
		t = t; // Okunan değeri kullanarak devam ediyoruz
		b = false; // Enable or disable autoConvert
		if (b) {
			t |= MAX31865_CONFIG_MODEAUTO; // Enable autoConvert
		} else {
			t &= ~MAX31865_CONFIG_MODEAUTO; // Disable autoConvert
		}

		addr = MAX31865_CONFIG_REG; // Adres değişkeni
		addr |= 0x80; // MSB=1 for write, make sure top bit is set

		buffer[0] = addr;
		buffer[1] = t;

		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		// setThresholds fonksiyonunun içeriği doğrudan eklendi
		addr = MAX31865_LFAULTLSB_REG; // Adres değişkeni
		addr |= 0x80; // MSB=1 for write, make sure top bit is set
		buffer[0] = addr;
		buffer[1] = 0 & 0xFF; // lower & 0xFF
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		addr = MAX31865_LFAULTMSB_REG; // Adres değişkeni
		addr |= 0x80; // MSB=1 for write, make sure top bit is set
		buffer[0] = addr;
		buffer[1] = 0 >> 8; // lower >> 8
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		addr = MAX31865_HFAULTLSB_REG; // Adres değişkeni
		addr |= 0x80; // MSB=1 for write, make sure top bit is set
		buffer[0] = addr;
		buffer[1] = 0xFFFF & 0xFF; // upper & 0xFF
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

		addr = MAX31865_HFAULTMSB_REG; // Adres değişkeni
		addr |= 0x80; // MSB=1 for write, make sure top bit is set
		buffer[0] = addr;
		buffer[1] = 0xFFFF >> 8; // upper >> 8
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
		HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
	}
  	begin_t();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		rtd = readRTD2();
		ratio = rtd;
		ratio /= 32768;
		temperature_t = calculateTemperature2(readRTD2(), RNOMINAL, RREF);
		HAL_Delay(1000);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10800-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 108-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* --------- FUNCTION FOR TEMPERATURE SENSOR --------- */

uint16_t readRTD2(void) {
	// enableBias fonksiyonunun içeriği
	uint8_t t;

	// readRegister8 fonksiyonunun içeriği
	uint8_t addr = MAX31865_CONFIG_REG; // Adres değişkeni
	addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_DELAY);
	HAL_SPI_Receive(&hspi1, &t, 1, SPI_DELAY); // t değişkeni burada doğrudan kullanıldı
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	t |= MAX31865_CONFIG_BIAS; // Enable bias

	// writeRegister8 fonksiyonunun içeriği
	addr = MAX31865_CONFIG_REG; // Adres değişkeni
	addr |= 0x80; // MSB=1 for write, make sure top bit is set

	uint8_t buffer[2] = { addr, t };

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	HAL_Delay(10);

	// readRegister8 fonksiyonunun içeriği
	addr = MAX31865_CONFIG_REG; // Adres değişkeni
	addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_DELAY);
	HAL_SPI_Receive(&hspi1, &t, 1, SPI_DELAY); // t değişkeni burada doğrudan kullanıldı
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	t |= MAX31865_CONFIG_1SHOT;

	// writeRegister8 fonksiyonunun içeriği
	addr = MAX31865_CONFIG_REG; // Adres değişkeni
	addr |= 0x80; // MSB=1 for write, make sure top bit is set

	buffer[0] = addr;
	buffer[1] = t;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	HAL_Delay(65);

	// readRegister16 fonksiyonunun içeriği
	addr = MAX31865_RTDMSB_REG; // Adres değişkeni
	addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

	uint8_t buffer16[2] = { 0, 0 };
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_DELAY);
	HAL_SPI_Receive(&hspi1, buffer16, 2, SPI_DELAY); // rtd değişkeni burada doğrudan kullanıldı
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	uint16_t rtd = (uint16_t) buffer16[0]; // Cast to uint16_t before left shift
	rtd <<= 8;
	rtd |= buffer16[1];

	// enableBias(false) kısmı
	// readRegister8 fonksiyonunun içeriği
	addr = MAX31865_CONFIG_REG; // Adres değişkeni
	addr &= 0x7F; // MSB=0 for read, make sure top bit is not set

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &addr, 1, SPI_DELAY);
	HAL_SPI_Receive(&hspi1, &t, 1, SPI_DELAY); // t değişkeni burada doğrudan kullanıldı
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	t &= ~MAX31865_CONFIG_BIAS; // Disable bias

	// writeRegister8 fonksiyonunun içeriği
	addr = MAX31865_CONFIG_REG; // Adres değişkeni
	addr |= 0x80; // MSB=1 for write, make sure top bit is set

	buffer[0] = addr;
	buffer[1] = t;

	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, buffer, 2, SPI_DELAY);
	HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

	rtd >>= 1; // Remove fault bit

	return rtd;
}
float calculateTemperature2(uint16_t RTDraw, float RTDnominal,
		float refResistor) {
	float Z1, Z2, Z3, Z4, Rt, temp;

	Rt = RTDraw;
	Rt /= 32768;
	Rt *= refResistor;

	Z1 = -RTD_A;
	Z2 = RTD_A * RTD_A - (4 * RTD_B);
	Z3 = (4 * RTD_B) / RTDnominal;
	Z4 = 2 * RTD_B;

	temp = Z2 + (Z3 * Rt);
	temp = (sqrt(temp) + Z1) / Z4;

	if (temp >= 0)
		return temp;

	Rt /= RTDnominal;
	Rt *= 100; // Normalize to 100 ohm

	float rpoly = Rt;

	temp = -242.02;
	temp += 2.2228 * rpoly;
	rpoly *= Rt; // Square
	temp += 2.5859e-3 * rpoly;
	rpoly *= Rt; // ^3
	temp -= 4.8260e-6 * rpoly;
	rpoly *= Rt; // ^4
	temp -= 2.8183e-8 * rpoly;
	rpoly *= Rt; // ^5
	temp += 1.5243e-10 * rpoly;

	return temp;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
