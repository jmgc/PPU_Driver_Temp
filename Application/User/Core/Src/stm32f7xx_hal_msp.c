/**
 ******************************************************************************
 * @file    UART/UART_Printf/Src/stm32f7xx_hal_msp.c
 * @author  MCD Application Team
 * @brief   HAL MSP module.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2016 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_RCC_SYSCFG_CLK_ENABLE();
}

/**
 * @brief ADC MSP Initialization
 *        This function configures the hardware resources used in this example:
 *           - Peripheral's clock enable
 *           - Peripheral's GPIO Configuration
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc) {
	GPIO_InitTypeDef GPIO_InitStruct;

	ADCx_CLK_ENABLE();
	ADCx_CHANNEL_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Pin = ADCx_CHANNEL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(ADCx_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(ADCx_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADCx_IRQn);
}

/**
 * @brief ADC MSP De-Initialization
 *        This function frees the hardware resources used in this example:
 *          - Disable the Peripheral's clock
 *          - Revert GPIO to their default state
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc) {
	ADCx_FORCE_RESET();
	ADCx_RELEASE_RESET();

	HAL_GPIO_DeInit(ADCx_CHANNEL_GPIO_PORT, ADCx_CHANNEL_PIN);
}

/**
 * @brief TIM_Base MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	__HAL_RCC_DAC_CLK_ENABLE();

	if (htim_base->Instance==TIM1) {
		__HAL_RCC_TIM1_CLK_ENABLE();
	}
	else if (htim_base->Instance==TIM3) {
		__HAL_RCC_TIM3_CLK_ENABLE();
	}
	/*else if (htim_base->Instance==TIM8) {
		__HAL_RCC_TIM8_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(TIM8_CC_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM8_CC_IRQn);
	}*/
}

/**
 * @brief TIM_PWM MSP Initialization
 * This function configures the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm) {
	if (htim_pwm->Instance==TIM4)
		__HAL_RCC_TIM4_CLK_ENABLE();
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if (htim->Instance==TIM3) {
		__HAL_RCC_GPIOA_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	else if (htim->Instance==TIM4) {
		__HAL_RCC_GPIOD_CLK_ENABLE();

		GPIO_InitStruct.Pin = GPIO_PIN_14;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	}
}

/**
 * @brief TIM_Base MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_base: TIM_Base handle pointer
 * @retval None
 */
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
	if(htim_base->Instance==TIM1)
		__HAL_RCC_TIM1_CLK_DISABLE();
	if(htim_base->Instance==TIM3)
		__HAL_RCC_TIM3_CLK_DISABLE();
	if(htim_base->Instance==TIM8) {
		__HAL_RCC_TIM8_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);
		HAL_NVIC_DisableIRQ(TIM8_CC_IRQn);
	}
}

/**
 * @brief TIM_PWM MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param htim_pwm: TIM_PWM handle pointer
 * @retval None
 */
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm) {
	if(htim_pwm->Instance==TIM4)
		__HAL_RCC_TIM4_CLK_DISABLE();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
