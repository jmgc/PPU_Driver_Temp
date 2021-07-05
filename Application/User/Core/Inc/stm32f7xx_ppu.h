/** 
  ******************************************************************************
  * @file    stm32f7xx_nucleo_144.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-November-2015
  * @brief   This file contains definitions for:
  *          - LEDs and push-button available on STM32F7XX-Nucleo-144 Kit 
  *            from STMicroelectronics
  *          - LCD, joystick and microSD available on Adafruit 1.8" TFT LCD 
  *            shield (reference ID 802)
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************  
  */ 
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F7XX_NUCLEO_144_H
#define __STM32F7XX_NUCLEO_144_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
   
/** @addtogroup BSP
  * @{
  */

/** @addtogroup STM32F7XX_PPU
  * @{
  */

/** @addtogroup STM32F7XX_PPU_LOW_LEVEL
  * @{
  */ 

/** @defgroup STM32F7XX_PPU_LOW_LEVEL_Exported_Types
  * @{
  */

/**
  * @}
  */ 

/** @defgroup STM32F7XX_NUCLEO_144_LOW_LEVEL_Exported_Constants
  * @{
  */ 

/** 
  * @brief Define for STM32F7XX_NUCLEO_144 board  
  */ 
#if !defined (USE_STM32F7XX_PPU)
 #define USE_STM32F7XX_PPU
#endif

/** @addtogroup STM32F7XX_PPU_LOW_LEVEL_TRF
  * @{
  */

#define CONTROL_N_P_PIN                         GPIO_PIN_0
#define CONTROL_N_P_GPIO_PORT                   GPIOA
#define CONTROL_N_P_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define CONTROL_N_P_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define CONTROL_N_N_PIN                         GPIO_PIN_1
#define CONTROL_N_N_GPIO_PORT                   GPIOA
#define CONTROL_N_N_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define CONTROL_N_N_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define CONTROL_P_P_PIN                         GPIO_PIN_2
#define CONTROL_P_P_GPIO_PORT                   GPIOA
#define CONTROL_P_P_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define CONTROL_P_P_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

#define CONTROL_P_N_PIN                         GPIO_PIN_3
#define CONTROL_P_N_GPIO_PORT                   GPIOA
#define CONTROL_P_N_GPIO_CLK_ENABLE()           __HAL_RCC_GPIOA_CLK_ENABLE()
#define CONTROL_P_N_GPIO_CLK_DISABLE()          __HAL_RCC_GPIOA_CLK_DISABLE()

/**
  * @}
  */ 
/**
  * @brief USB Pins definition
  */


#define OTG_FS1_OVER_CURRENT_PIN                  GPIO_PIN_7
#define OTG_FS1_OVER_CURRENT_PORT                 GPIOG
#define OTG_FS1_OVER_CURRENT_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()

#define OTG_FS1_POWER_SWITCH_PIN                  GPIO_PIN_6
#define OTG_FS1_POWER_SWITCH_PORT                 GPIOG
#define OTG_FS1_POWER_SWITCH_PORT_CLK_ENABLE()     __HAL_RCC_GPIOG_CLK_ENABLE()

/**
  * @}
  */ 

/*################################ ADC3 ######################################*/
/**
  * @brief  ADC Interface pins
  *         used to detect motion of Joystick available on Adafruit 1.8" TFT shield
  */
  
#ifdef HAL_ADC_MODULE_ENABLED
  
#define NUCLEO_ADCx                        ADC3
#define NUCLEO_ADCx_CLK_ENABLE()         __HAL_RCC_ADC3_CLK_ENABLE()
#define NUCLEO_ADCx_CLK_DISABLE()        __HAL_RCC_ADC3_CLK_DISABLE()

#define NUCLEO_ADCx_CHANNEL                ADC_CHANNEL_9
   
#define NUCLEO_ADCx_GPIO_PORT              GPIOF
#define NUCLEO_ADCx_GPIO_PIN               GPIO_PIN_3
#define NUCLEO_ADCx_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOF_CLK_ENABLE()
#define NUCLEO_ADCx_GPIO_CLK_DISABLE()   __HAL_RCC_GPIOF_CLK_DISABLE()

#endif /* HAL_ADC_MODULE_ENABLED */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32F7XX_NUCLEO_144_LOW_LEVEL_Exported_Macros
  * @{
  */  
/**
  * @}
  */ 


  
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F7XX_NUCLEO_144_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
