/**
  ******************************************************************************
  * @file    stm32l475e_iot01_conf.h
  * @author  MCD Application Team
  * @brief   configuration file.
  *          This file should be copied to the application folder and renamed
  *          to stm32l475e_iot01_conf.h
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STM32L475E_IOT01_CONF_H
#define STM32L475E_IOT01_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* AUDIO define */
#define BSP_AUDIO_IN_INSTANCE               0U
#define AUDIO_INSTANCE                      BSP_AUDIO_IN_INSTANCE
#define AUDIO_CHANNELS                      1
#define AUDIO_VOLUME_VALUE                  32
#define AUDIO_SAMPLING_FREQUENCY            16000
#define AUDIO_DFSDM_DMAx_MIC1_IRQHandler    DMA1_Channel4_IRQHandler

/* COM define */
#define USE_BSP_COM_FEATURE                  1U

#define USE_COM_LOG                          1U

/* IRQ priorities */
#define BSP_BUTTON_USER_IT_PRIORITY          0x04UL

/* Audio interrupt priorities */
#define BSP_AUDIO_IN_IT_PRIORITY             0x02UL

/* I2C2 Frequency in Hz  */
#define BUS_I2C1_FREQUENCY                   100000UL /* Frequency of I2C1 = 100 KHz*/
#define BUS_I2C2_FREQUENCY                   100000UL /* Frequency of I2C2 = 100 KHz*/

/* SPI1 Baud rate in bps  */
#define BUS_SPI1_BAUDRATE                    16000000UL /* baud rate of SPIn = 16 Mbps */

/* SPI3 Baud rate in bps  */
#define BUS_SPI3_BAUDRATE                    16000000U /* baud rate of SPIn = 16 Mbps */

/* Default AUDIO IN internal buffer size in 32-bit words per micro */
#define BSP_AUDIO_IN_DEFAULT_BUFFER_SIZE     2048UL /* 2048*4 = 8Kbytes */

#ifdef __cplusplus
}
#endif

#endif /* STM32L475E_IOT01_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
