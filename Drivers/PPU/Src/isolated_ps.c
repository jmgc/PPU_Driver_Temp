/*
 * isolated_ps.c
 *
 *  Created on: May 4, 2021
 *      Author: Guillaume
 */

#include "isolated_ps.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void TLE_Isolated_Init(uint32_t pwm_period_us) {
	uint32_t CLK_Freq_MHz = 108;
	uint16_t PSC_multiplier = 1;
	uint32_t PSC = CLK_Freq_MHz * PSC_multiplier - 1;

	uint16_t PWM_PSC;
	uint16_t PWM_ARR;
	uint16_t DIR_PSC;
	uint16_t DIR_ARR;

	while (pwm_period_us > PSC_multiplier * 0xFFFF) {
		PSC_multiplier++;
		PSC = CLK_Freq_MHz * PSC_multiplier - 1;
		if (PSC > 0xFFFF) {
			assert(0);
			break;
		}
	}
	PWM_PSC = PSC;
	if (PSC_multiplier == 1)
		PWM_ARR = pwm_period_us - 1;
	else
		PWM_ARR = (pwm_period_us * CLK_Freq_MHz - (PWM_PSC + 1)) / (PWM_PSC + 1);

	if (PWM_ARR > (0xFFFF >> 1)) {
		if (PWM_PSC > 0xFFFF / 2)
			assert(0);
		else {
			DIR_PSC = 2 * PWM_PSC + 1;
			DIR_ARR = PWM_ARR;
		}
	}
	else {
		DIR_PSC = PWM_PSC;
		DIR_ARR = 2 * PWM_ARR + 1;
	}

	MX_TIM3_Init(DIR_PSC, DIR_ARR);
	MX_TIM4_Init(PWM_PSC, PWM_ARR);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
}


void MX_TIM3_Init(uint16_t PSC, uint16_t ARR) {
	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = PSC;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = ARR;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
		Error_Handler();

	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_COMBINED_RESETTRIGGER;
	sSlaveConfig.InputTrigger = TIM_TS_ITR3;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
		Error_Handler();

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = (htim3.Init.Period + 1) / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	HAL_TIM_MspPostInit(&htim3);
}

void MX_TIM4_Init(uint16_t PSC, uint16_t ARR) {
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = PSC;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = ARR;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
		Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_ENABLE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
		Error_Handler();

	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = (htim4.Init.Period + 1) / 2;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();

	HAL_TIM_MspPostInit(&htim4);
}

