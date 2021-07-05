#include "tle9201sg_pwmdriver_api.h"

/* mbed Microcontroller Library
 *******************************************************************************
 * Copyright (c) 2015, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
 *******************************************************************************
 */

#include "pinmap.h"
#include <assert.h>
#include <stdlib.h>

#include "stm32f7xx_hal_tim.h"

static const int32_t time_unit = 1000000; // us

static TIM_HandleTypeDef timHandlePWM;
static TIM_HandleTypeDef timHandleDir;

void tle9201sg_pwmdriver_init(tle9201sg_pwmdriver_t* obj, PinName pin_pwm,
		PinName pin_dir) {
	// Get the peripheral name from the pin and assign it to the object
	obj->pwm = (PWMName) pinmap_peripheral(pin_pwm, PinMap_PWM);
	assert(obj->pwm != (PWMName)NC);
	obj->dir = (PWMName) pinmap_peripheral(pin_dir, PinMap_PWM);
	assert(obj->pwm != obj->dir);

	// Get the functions (timer channel, (non)inverted) from the pin and assign it to the object
	uint32_t function_pwm = pinmap_function(pin_pwm, PinMap_PWM);
	uint32_t function_dir = pinmap_function(pin_dir, PinMap_PWM);
	assert(function_pwm != (uint32_t)NC);
	assert(function_dir != (uint32_t)NC);
	obj->channel_pwm = STM_PIN_CHANNEL(function_pwm);
	obj->channel_dir = STM_PIN_CHANNEL(function_dir);
	assert(obj->channel_pwm != obj->channel_dir);

	// Enable TIM clock
#if defined(TIM1_BASE)
	if (obj->pwm == PWM_1 || obj->dir == PWM_1) {
		__HAL_RCC_TIM1_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM2_BASE)
	if (obj->pwm == PWM_2 || obj->dir == PWM_2) {
		__HAL_RCC_TIM2_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM3_BASE)
	if (obj->pwm == PWM_3 || obj->dir == PWM_3) {
		__HAL_RCC_TIM3_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM4_BASE)
	if (obj->pwm == PWM_4 || obj->dir == PWM_4) {
		__HAL_RCC_TIM4_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM5_BASE)
	if (obj->pwm == PWM_5 || obj->dir == PWM_5) {
		__HAL_RCC_TIM5_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM8_BASE)
	if (obj->pwm == PWM_8 || obj->dir == PWM_8) {
		__HAL_RCC_TIM8_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM9_BASE)
	if (obj->pwm == PWM_9 || obj->dir == PWM_9) {
		__HAL_RCC_TIM9_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM10_BASE)
	if (obj->pwm == PWM_10 || obj->dir == PWM_10) {
		__HAL_RCC_TIM10_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM11_BASE)
	if (obj->pwm == PWM_11 || obj->dir == PWM_11) {
		__HAL_RCC_TIM11_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM12_BASE)
	if (obj->pwm == PWM_12 || obj->dir == PWM_12) {
		__HAL_RCC_TIM12_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM13_BASE)
	if (obj->pwm == PWM_13 || obj->dir == PWM_13) {
		__HAL_RCC_TIM10_CLK_ENABLE()
				;
	}
#endif
#if defined(TIM14_BASE)
	if (obj->pwm == PWM_14 || obj->dir == PWM_14) {
		__HAL_RCC_TIM11_CLK_ENABLE()
				;
	}
#endif
	// Configure GPIO
	pinmap_pinout(pin_pwm, PinMap_PWM);
	pinmap_pinout(pin_dir, PinMap_PWM);
	obj->pin_pwm = pin_pwm;
	obj->pin_dir = pin_dir;
	obj->period = 0;
	obj->pulse = 0;
	obj->prescaler = 1;

	tle9201sg_pwmdriver_period_us(obj, 20000); // 20 ms per default
}

void tle9201sg_pwmdriver_free(tle9201sg_pwmdriver_t* obj) {
	// Configure GPIO
	pin_function(obj->pin_pwm, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
	pin_function(obj->pin_dir, STM_PIN_DATA(STM_MODE_INPUT, GPIO_NOPULL, 0));
}

static int getChannel(uint8_t channel) {
	switch (channel) {
	case 1:
		return TIM_CHANNEL_1;
	case 2:
		return TIM_CHANNEL_2;
	case 3:
		return TIM_CHANNEL_3;
	case 4:
		return TIM_CHANNEL_4;
	default:
		return 0;
	}
}

void tle9201sg_pwmdriver_pulse(tle9201sg_pwmdriver_t* obj, float pulse) {
	TIM_OC_InitTypeDef sConfig;
	uint8_t channel_pwm = 0;
	uint8_t channel_dir = 0;
	int pwm_pulse = abs((int) (pulse * time_unit));
	timHandlePWM.Instance = (TIM_TypeDef *) (obj->pwm);
	timHandleDir.Instance = (TIM_TypeDef *) (obj->dir);

	channel_pwm = getChannel(obj->channel_pwm);
	channel_dir = getChannel(obj->channel_dir);

	HAL_TIM_PWM_Stop(&timHandlePWM, channel_pwm);
	HAL_TIM_PWM_Stop(&timHandleDir, channel_dir);

	if (pwm_pulse <= 0) {
		pwm_pulse = 1;
	}
	obj->pulse = pwm_pulse;
	if (pwm_pulse >= (obj->period)) {
		pwm_pulse = (obj->period / 2) - 1;
	}

	// Configure channels
	sConfig.OCMode = TIM_OCMODE_PWM2;
	sConfig.Pulse = pwm_pulse;
	sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode = TIM_OCFAST_DISABLE;
	sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	if (HAL_TIM_PWM_ConfigChannel(&timHandlePWM, &sConfig, channel_pwm)
			!= HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}

	sConfig.OCMode = TIM_OCMODE_PWM2;
	sConfig.Pulse = (timHandleDir.Init.Period + 1) / 2 ;

	if (HAL_TIM_PWM_ConfigChannel(&timHandleDir, &sConfig, channel_dir)
			!= HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}

	HAL_TIM_PWM_Start(&timHandleDir, channel_dir);
	HAL_TIM_PWM_Start(&timHandlePWM, channel_pwm);
}

float tle9201sg_pwmdriver_read(tle9201sg_pwmdriver_t* obj) {
	return 2 * obj->pulse / (float) time_unit;
}

void tle9201sg_pwmdriver_period(tle9201sg_pwmdriver_t* obj, float seconds) {
	tle9201sg_pwmdriver_write(obj, seconds);
}

void tle9201sg_pwmdriver_period_ms(tle9201sg_pwmdriver_t* obj, int ms) {
	tle9201sg_pwmdriver_write(obj, ms * 1e-3);
}

void tle9201sg_pwmdriver_period_us(tle9201sg_pwmdriver_t* obj, int us) {
	tle9201sg_pwmdriver_write(obj, us * 1e-6);
}

static int32_t period_calculation(tle9201sg_pwmdriver_t* obj, int32_t period) {
	period = period / obj->prescaler;
	int delta = (period % 2);
	period += delta == 0 ? 0 : 2 - delta;
	period = period * obj->prescaler;
	return period;
}

void tle9201sg_pwmdriver_write(tle9201sg_pwmdriver_t* obj, float value) {
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };

	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	uint32_t PclkFreq = 0;
	uint32_t APBxCLKDivider = RCC_HCLK_DIV1;
	uint8_t i = 0;
	uint32_t half_period = abs((uint32_t) (value * time_unit / 2));
	float pulse = tle9201sg_pwmdriver_read(obj);
	timHandlePWM.Instance = (TIM_TypeDef *) (obj->pwm);
	timHandleDir.Instance = (TIM_TypeDef *) (obj->dir);

	//__HAL_TIM_DISABLE(&timHandle);
	if (HAL_TIM_Base_Stop(&timHandleDir) != HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}
	if (HAL_TIM_Base_Stop(&timHandlePWM) != HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}

	uint8_t channel_pwm = getChannel(obj->channel_pwm);
	uint8_t channel_dir = getChannel(obj->channel_dir);

	HAL_TIM_PWM_Stop(&timHandlePWM, channel_pwm);
	HAL_TIM_PWM_Stop(&timHandleDir, channel_dir);

	// Get clock configuration
	// Note: PclkFreq contains here the Latency (not used after)
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &PclkFreq);

	/*  Parse the pwm / apb mapping table to find the right entry */
	while (pwm_apb_map_table[i].pwm != obj->pwm) {
		i++;
	}

	if (pwm_apb_map_table[i].pwm == 0)
		assert(0); //"Unknown PWM instance");

	if (pwm_apb_map_table[i].pwmoutApb == PWMOUT_ON_APB1) {
		PclkFreq = HAL_RCC_GetPCLK1Freq();
		APBxCLKDivider = RCC_ClkInitStruct.APB1CLKDivider;
	} else {
#if !defined(PWMOUT_APB2_NOT_SUPPORTED)
		PclkFreq = HAL_RCC_GetPCLK2Freq();
		APBxCLKDivider = RCC_ClkInitStruct.APB2CLKDivider;
#endif
	}

	/* By default use, 1us as SW pre-scaler */
	obj->prescaler = 1;
	// TIMxCLK = PCLKx when the APB prescaler = 1 else TIMxCLK = 2 * PCLKx
	if (APBxCLKDivider == RCC_HCLK_DIV1) {
		timHandlePWM.Init.Prescaler = (((PclkFreq) / time_unit)) - 1; // 1 us tick
		timHandleDir.Init.Prescaler = timHandlePWM.Init.Prescaler;
	} else {
		timHandlePWM.Init.Prescaler = (((PclkFreq * 2) / time_unit)) - 1; // 1 us tick
		timHandleDir.Init.Prescaler = timHandlePWM.Init.Prescaler;
	}

	half_period = period_calculation(obj, half_period);

	timHandlePWM.Init.Period = half_period;
	timHandleDir.Init.Period = half_period * 2;

	/*  In case period or pre-scalers are out of range, loop-in to get valid values */
	while ((timHandleDir.Init.Period > 0xFFFF)
			|| (timHandleDir.Init.Prescaler > 0xFFFF)) {
		obj->prescaler = obj->prescaler * 2;
		if (APBxCLKDivider == RCC_HCLK_DIV1) {
			timHandleDir.Init.Prescaler = (((PclkFreq) / 1000000)
					* obj->prescaler) - 1;
		} else {
			timHandleDir.Init.Prescaler = (((PclkFreq * 2) / 1000000)
					* obj->prescaler) - 1;
		}
		half_period = period_calculation(obj, half_period / 2);

		timHandleDir.Init.Period = half_period * 2;
		/*  Period decreases and prescaler increases over loops, so check for
		 *  possible out of range cases */
		if ((timHandleDir.Init.Period < 0xFFFF)
				&& (timHandleDir.Init.Prescaler > 0xFFFF)) {
			assert(0); //"Cannot initialize PWM\n");
			break;
		}
	}
	timHandlePWM.Init.Prescaler = timHandleDir.Init.Prescaler;
	timHandleDir.Init.Period = half_period * 2 - 1;
	timHandlePWM.Init.Period = half_period - 1;
	timHandlePWM.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timHandlePWM.Init.CounterMode = TIM_COUNTERMODE_UP;
	timHandlePWM.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	timHandleDir.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	timHandleDir.Init.CounterMode = TIM_COUNTERMODE_UP;
	timHandleDir.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (HAL_TIM_Base_Init(&timHandleDir) != HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ITR3;
	if (HAL_TIM_SlaveConfigSynchronization(&timHandleDir, &sSlaveConfig) != HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}
	if (HAL_TIM_Base_Init(&timHandlePWM) != HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&timHandlePWM, &sMasterConfig)
			!= HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}

	// Save for future use
	obj->period = half_period;
	if (obj->pulse == 0)
		obj->pulse = 1;

	// Set duty cycle again
	tle9201sg_pwmdriver_pulse(obj, pulse);
/*
	if (HAL_TIM_Base_Start(&timHandleDir) != HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}
	if (HAL_TIM_Base_Start(&timHandlePWM) != HAL_OK) {
		assert(0); //"Cannot initialize PWM\n");
	}*/
	//__HAL_TIM_ENABLE(&timHandle);
}
