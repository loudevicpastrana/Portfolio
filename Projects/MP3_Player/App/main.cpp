/*
 * Loudevic Pastrana
 * 
 *
 * (c) 2021 Cristian Pop
 */

#include <stdio.h>

#include "main.h"
#include "test.h"

const char clr_scrn[] = { 27, 91, 50, 74, 0 };              // esc[2J

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 40
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 4
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* MSI is enabled after System reset, activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    /* Initialization Error */
    ErrorHandler();
  }
}

/**
  * @brief UART console init function
  */
static void Console_UART_Init(void)
{
  COM_InitTypeDef com_init;
  
  com_init.BaudRate = 115200;
  com_init.WordLength = COM_WORDLENGTH_8B;
  com_init.StopBits = COM_STOPBITS_1;
  com_init.Parity = COM_PARITY_NONE;
  com_init.HwFlowCtl = COM_HWCONTROL_NONE;
  BSP_COM_Init(COM1, &com_init);
}

void ErrorHandler()
{
    printf("PANIC!\n");
    while(1)
    {
        BSP_LED_Toggle(LED_GREEN);
        HAL_Delay(200);
    }
}

extern char _user_heap_base asm("_user_heap_base");
extern char _user_heap_end asm("_user_heap_end");

int main() 
{
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Off(LED_GREEN);
  
  /* UART console init */
  Console_UART_Init();

  printf("%sUniversity of Washington: Hello RTOS World!\r\n", clr_scrn);
  #ifdef DEBUG
  const char* build_type = "DEBUG";
  #else
  const char* build_type = "RELEASE";
  #endif

  printf("\t%s Build, %s %s.\r\n", build_type, __DATE__, __TIME__);
  printf("\tBSP version 0x%lx\r\n", BSP_GetVersion());
  printf("\tReserved heap: %d\r\n", &_user_heap_end - &_user_heap_base); // To modify, use _Min_Heap_Size in the *.LD file.
  printf("\tCPU clock: %ldMHz\r\n", HAL_RCC_GetSysClockFreq() / 1000000);

  // Quick blink once for a simple visual indicator that initialization was successful.
  BSP_LED_On(LED_GREEN);
  HAL_Delay(500);
  BSP_LED_Off(LED_GREEN);

  test_all();

  printf("\r\n -- STOP --\r\n");

  while(1);
  return 0;
}
