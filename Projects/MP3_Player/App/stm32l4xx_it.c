/**
  ******************************************************************************
  * @file    Templates/Src/stm32l4xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_it.h"
#include "main.h"
#include "stm32l475e_iot01_audio.h"

#ifdef UCOS2
  #include "ucos_ii.h"
  #include "es_wifi_conf.h"

  /* UW:  See Configuration/app_cfg.h for more details around CPU_CFG_KA_IPL_BOUNDARY:
   *      1. Interrupts with priority < CPU_CFG_KA_IPL_BOUNDARY MUST not use uCOS-2 features.
   *         These are Non-Kernel aware and do not require aditional handling.
   *      2. Kernel aware interupts have priority >=CPU_CFG_KA_IPL_BOUNDARY and MUST 
   *         implement proper uCOS-2 header/trailer as defined at 
   *         https://www.weston-embedded.com/company/media-articles/20-cortex-m-migrate-to-new-armv7m-port
   */

  #if 0
  // uCOS ISR requirements for Kernel-Aware interrupts:

  // Header:
  #if OS_CRITICAL_METHOD == 3u                                    /* Allocate storage for CPU status register             */
    OS_CPU_SR  cpu_sr;
  #endif

  OS_ENTER_CRITICAL();
  OSIntEnter(); /* Tell OS that we are starting an ISR */
  OS_EXIT_CRITICAL();

  // Process ISR - uCOS2 Constructs _can_ be called here.
  // ...

  OSIntExit(); /* Tell OS that we are leaving the ISR */
  #endif

#endif

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

#if 0 // UW: Defined in startup. Remapped to uCOS2.
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
  
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

#endif 


/******************************************************************************/
/*                 STM32L4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32l4xxxx.s).                                             */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 

/**
  * @}
  */

/************ Inventek WiFi Interrupts ****************/

// Defined in net_conf_es_wifi_spi.c
extern SPI_HandleTypeDef hspi;

void SPI3_IRQHandler(void)
{
  // UW_TODO: Kernel aware interrupt: wrap in header/trailer 
  //UW_SOLUTION:

  HAL_SPI_IRQHandler(&hspi);

  // Check if SPI has received data (RXNE flag set)
  if (__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_RXNE))  
  {
      SEM_SIGNAL(spi_rx_sem);  // Signal RX semaphore
  }

  // // Check if SPI has completed transmission (TXE flag set)
  if (__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_TXE))  
  {
    SEM_SIGNAL(spi_tx_sem); // Signal TX semaphore
  }


}

void EXTI1_IRQHandler(void)
{
  // UW_TODO: Kernel aware interrupt: wrap in header/trailer 
  //UW_SOLUTION

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);

  // // Check if SPI peripheral signals "Data Ready"
  if (__HAL_SPI_GET_FLAG(&hspi, SPI_FLAG_OVR))  // Example: Overrun means new data available
  {
    SEM_SIGNAL(cmddata_rdy_rising_sem); // Signal Data Ready semaphore
  }
}
