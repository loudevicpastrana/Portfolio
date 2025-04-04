/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "app_bluenrg_ms.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hci_tl.h"
#include "link_layer.h"
#include "bluenrg_def.h"
#include "bluenrg_gap_aci.h"
#include "bluenrg_gap.h"
#include "string.h"
#include <stdio.h>
#include "hci.h"
#include "hci_le.h"
#include "bluenrg_gatt_aci.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define  ADV_INTERVAL_MIN_MS  800
#define  ADV_INTERVAL_MAX_MS  900
#define  CONN_INTERVAL_MIN_MS 100
#define  CONN_INTERVAL_MAX_MS 300

#define HTS221_ADDR_READ 0xBF
#define HTS221_ADDR_WRITE 0xBE

// HTS221 Registers
#define HTS221_TEMP_OUT_L  0x2A   // Low byte of the temperature data
#define HTS221_TEMP_OUT_H  0x2B   // High byte of the temperature data
#define HTS221_T0_DEGC_X8  0x32   // T0 temperature calibration (x8)
#define HTS221_T1_DEGC_X8  0x33   // T1 temperature calibration (x8)
#define HTS221_T0_OUT_L    0x3C   // T0 output calibration (ADC value)
#define HTS221_T0_OUT_H	   0x3D
#define HTS221_T1_OUT_L    0x3E   // T1 output calibration (ADC value)
#define HTS221_T1_OUT_H    0x3F

#define HTS221_WHO_AM_I_REG  0x0F
#define HTS221_CTRL_REG1     0x20
#define HTS221_CTRL_REG2     0x21
#define HTS221_CTRL_REG3     0x22

int32_t ble_temperature = 0;

// Function to read 16-bit data from HTS221 (I2C read)
int16_t HTS221_Read16(uint8_t regL, uint8_t regH)
{
    uint8_t tempL;
    uint8_t tempH;

    HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_READ, regL, I2C_MEMADD_SIZE_8BIT, &tempL, 1, HAL_MAX_DELAY);
    HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_READ, regH, I2C_MEMADD_SIZE_8BIT, &tempH, 1, HAL_MAX_DELAY);

    uint16_t result = (tempH << 8) | tempL;

    return result;  // Combine high and low byte
}


// Function to read 16-bit data from HTS221 (I2C read)
int16_t HTS221_Read8(uint8_t reg)
{
    uint8_t data;


    HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_READ, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);


    return data;  // Combine high and low byte
}

// Function to convert the raw ADC value to Celsius
float HTS221_ConvertToCelsius(int16_t adc_value)
{
    // Read calibration values from HTS221
    int16_t T0_OUT = HTS221_Read16(HTS221_T0_OUT_L, HTS221_T0_OUT_H);  // T0 ADC value
    int16_t T1_OUT = HTS221_Read16(HTS221_T1_OUT_L, HTS221_T1_OUT_H);  // T1 ADC value
    int16_t T0_degC = HTS221_Read8(HTS221_T0_DEGC_X8);  // T0 temperature (divided by 8)
    int16_t T1_degC = HTS221_Read8(HTS221_T1_DEGC_X8);  // T1 temperature (divided by 8)


    // Apply linear interpolation formula
    float T_degC = T0_degC + ((adc_value - T0_OUT) * (T1_degC - T0_degC)) / (float)(T1_OUT - T0_OUT);
    return T_degC;
}

void HTS221_Init(void)
{
	uint8_t data[2];

	// Step 1: Check if the HTS221 device is present
	HAL_I2C_Mem_Read(&hi2c2, HTS221_ADDR_READ, HTS221_WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
	if (data[0] != 0xBC)  // Expected WHO_AM_I response for HTS221 is 0xBC
	{
		printf("HTS221 not found!\n");
		return;
	}

	// Step 2: Configure CTRL_REG1 for operation
	// Power ON, Block Data Update enabled, Output Data Rate 10 Hz
	data[0] = 0x85;  // 0x85 = 0b10000101
	// Bit breakdown: PD = 0 (Power On), BDU = 1 (Block Data Update), ODR = 01 (10 Hz)
	HAL_I2C_Mem_Write(&hi2c2, HTS221_ADDR_WRITE, HTS221_CTRL_REG1, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);

}

void Read_Temperature(void)
{

	 char buffer[50];  // A buffer to hold the string representation of the variable

    // Read raw temperature data (from TEMP_OUT_L and TEMP_OUT_H registers)
    int16_t raw_temp = HTS221_Read16(HTS221_TEMP_OUT_L, HTS221_TEMP_OUT_H);  // Read raw temperature value

    // Convert to Celsius using the conversion function
    float temperature = HTS221_ConvertToCelsius(raw_temp);

    ble_temperature = temperature * 100; // to transmit in a whole number on bluetooth

    // Print the temperature to the debug console (via UART or other interface)
    //printf("\r! temperature \r\n");temperature
    snprintf(buffer, sizeof(buffer), "Value: %.2f\r\n", temperature);

     HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    //HAL_Delay(1000);
}

volatile uint32_t sysTickCount = 0;  // Global variable to track SysTick interrupts
volatile uint32_t delayTime = 0;     // Variable used by SysTick_Handler to count down

// SysTick timer initialization
void SysTick_Init(void) {
    // Set SysTick to interrupt every 1 ms
    SysTick->LOAD = 4000 - 1;         // Set reload value for 1 ms (4000 clock cycles) default clock of systick is 4MHz
    SysTick->VAL = 0;     // Clear current value register
    SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;  // Enable SysTick interrupt and the timer
}
//// SysTick interrupt handler
//void SysTick_Handler(void) {
//    if (delayTime > 0) {
//        delayTime--;
//    }
//    sysTickCount++;  // Increment SysTick counter
//}

// Delay function
void delay(uint32_t ms) {
    delayTime = ms;  // Set the number of SysTick interrupts to wait for
    while (delayTime > 0) {
        // Busy-wait until delayTime reaches 0
    }
}


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
  MX_I2C2_Init();
  MX_BlueNRG_MS_Init();
  /* USER CODE BEGIN 2 */
  MX_SPI3_Init(&hspi3);

  HTS221_Init();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
   Read_Temperature();
   MX_BlueNRG_MS_Process();
  // Read and convert the temperature

  // delay(500);  // Delay 500 milliseconds using systick interrupt
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00100D14;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|BLE_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLE_SPI_CS_GPIO_Port, BLE_SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin BLE_RST_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|BLE_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BLE_SPI_CS_Pin */
  GPIO_InitStruct.Pin = BLE_SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLE_SPI_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
