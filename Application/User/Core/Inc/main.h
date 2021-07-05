/**
  ******************************************************************************
  * @file    USB_Device/CDC_ECM_Server/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                      www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

// #define TARGET_NUCLEO_F746ZG
#define TARGET_PPU_F746VG
#define DEVICE_PWMOUT
#define DEVICE_ANALOGIN
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc_ecm.h"
#include "usbd_cdc_ecm_if.h"
#include "lwip/netif.h"

#ifdef TARGET_NUCLEO_F746ZG
#include "stm32f7xx_nucleo_144.h"
#endif

#ifdef TARGET_PPU_F746VG
#include "stm32f7xx_ppu.h"
#endif

#include "stdio.h"

//#include "analogin_api.h"
#include "pwmout_api.h"
#include "bipolar_pwmout_api.h"
#include "tle9201sg_pwmdriver_api.h"

#include "peripheral_names.h"
#include "pin_names.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define LED_TIMER_LONG  ((uint32_t)(0x3FFFC))
#define LED_TIMER_SHORT ((uint32_t)(LED_TIMER_LONG/2))

/* User can use this section to tailor USARTx/UARTx instance used and associated
   resources */
/* Definition for USARTx clock resources */
#define USARTx                           USART3
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART3_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART3_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_8
#define USARTx_TX_GPIO_PORT              GPIOD
#define USARTx_TX_AF                     GPIO_AF7_USART3
#define USARTx_RX_PIN                    GPIO_PIN_9
#define USARTx_RX_GPIO_PORT              GPIOD
#define USARTx_RX_AF                     GPIO_AF7_USART3

/* Definition for ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()

#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_0
#define ADCx_CHANNEL_GPIO_PORT          GPIOC

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_10

/* Definition for ADCx's NVIC */
#define ADCx_IRQn                       ADC_IRQn


/* Exported macro ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
extern __IO uint32_t LEDTimer;
extern volatile int ACCESSING_READINGS_LIST;
extern volatile uint32_t state_machine_cnt_ms;
extern USBD_HandleTypeDef USBD_Device;
extern uint8_t ACCESSING_PARAMETERS;

extern struct netif gnetif;
/* Exported functions ------------------------------------------------------- */
void Toggle_Leds(void);
void Error_Handler(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
