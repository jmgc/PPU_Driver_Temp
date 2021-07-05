#include "bipolar_pwmout_api.h"

#include "math.h"
#include "pinmap.h"
#include "peripheral_pins.h"

extern TIM_HandleTypeDef htim1;
static TIM_HandleTypeDef timHandle;

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

static void enable_clk(bipolar_pwmout_t* obj) {
	// Enable TIM clock
#if defined(TIM1_BASE)
	if (obj->pwm == PWM_1) {
		__HAL_RCC_TIM1_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM2_BASE)
	if (obj->pwm == PWM_2) {
		__HAL_RCC_TIM2_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM3_BASE)
	if (obj->pwm == PWM_3) {
		__HAL_RCC_TIM3_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM4_BASE)
	if (obj->pwm == PWM_4) {
		__HAL_RCC_TIM4_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM5_BASE)
	if (obj->pwm == PWM_5) {
		__HAL_RCC_TIM5_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM8_BASE)
	if (obj->pwm == PWM_8) {
		__HAL_RCC_TIM8_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM9_BASE)
	if (obj->pwm == PWM_9) {
		__HAL_RCC_TIM9_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM10_BASE)
	if (obj->pwm == PWM_10) {
		__HAL_RCC_TIM10_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM11_BASE)
	if (obj->pwm == PWM_11) {
		__HAL_RCC_TIM11_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM12_BASE)
	if (obj->pwm == PWM_12) {
		__HAL_RCC_TIM12_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM13_BASE)
	if (obj->pwm == PWM_13) {
		__HAL_RCC_TIM13_CLK_ENABLE()
														;
	}
#endif
#if defined(TIM14_BASE)
	if (obj->pwm == PWM_14) {
		__HAL_RCC_TIM14_CLK_ENABLE()
														;
	}
#endif
}

static void disable_clk(bipolar_pwmout_t* obj) {
	// Enable TIM clock
#if defined(TIM1_BASE)
	if (obj->pwm == PWM_1) {
		__HAL_RCC_TIM1_CLK_DISABLE();
	}
#endif
#if defined(TIM2_BASE)
	if (obj->pwm == PWM_2) {
		__HAL_RCC_TIM2_CLK_DISABLE();
	}
#endif
#if defined(TIM3_BASE)
	if (obj->pwm == PWM_3) {
		__HAL_RCC_TIM3_CLK_DISABLE();
	}
#endif
#if defined(TIM4_BASE)
	if (obj->pwm == PWM_4) {
		__HAL_RCC_TIM4_CLK_DISABLE();
	}
#endif
#if defined(TIM5_BASE)
	if (obj->pwm == PWM_5) {
		__HAL_RCC_TIM5_CLK_DISABLE();
	}
#endif
#if defined(TIM8_BASE)
	if (obj->pwm == PWM_8) {
		__HAL_RCC_TIM8_CLK_DISABLE();
	}
#endif
#if defined(TIM9_BASE)
	if (obj->pwm == PWM_9) {
		__HAL_RCC_TIM9_CLK_DISABLE();
	}
#endif
#if defined(TIM10_BASE)
	if (obj->pwm == PWM_10) {
		__HAL_RCC_TIM10_CLK_DISABLE();
	}
#endif
#if defined(TIM11_BASE)
	if (obj->pwm == PWM_11) {
		__HAL_RCC_TIM11_CLK_DISABLE();
	}
#endif
#if defined(TIM12_BASE)
	if (obj->pwm == PWM_12) {
		__HAL_RCC_TIM12_CLK_DISABLE();
	}
#endif
#if defined(TIM13_BASE)
	if (obj->pwm == PWM_13) {
		__HAL_RCC_TIM13_CLK_DISABLE();
	}
#endif
#if defined(TIM14_BASE)
	if (obj->pwm == PWM_14) {
		__HAL_RCC_TIM14_CLK_DISABLE();
	}
#endif
}

void bipolar_pwmout_allocate(bipolar_pwmout_t* obj) {
	pinmap_pinout(obj->pin, PinMap_PWM);
	pinmap_pinout(obj->pinn, PinMap_PWM);
}

void bipolar_pwmout_free(bipolar_pwmout_t* obj) {
	// Configure GPIO
	pin_function(obj->pin, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
	pin_function(obj->pinn, STM_PIN_DATA(STM_MODE_OUTPUT_PP, GPIO_NOPULL, 0));
}

static void bipolar_pwmout_configure(bipolar_pwmout_t* obj) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	TIM_OC_InitTypeDef sConfig;
	uint32_t htim1_prescaler;
	uint32_t PclkFreq = 0;
	uint32_t APBxCLKDivider = RCC_HCLK_DIV1;
	uint8_t i = 0;
	timHandle.Instance = (TIM_TypeDef *) (obj->pwm);

	uint8_t channel = getChannel(obj->channel);
	uint8_t channeln = getChannel(obj->channeln);

	enable_clk(obj);

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
	htim1_prescaler = 1;
	// TIMxCLK = PCLKx when the APB prescaler = 1 else TIMxCLK = 2 * PCLKx
	if (APBxCLKDivider == RCC_HCLK_DIV1) {
		timHandle.Init.Prescaler = (((PclkFreq) / obj->time_unit)) - 1; // 1 us tick
	} else {
		timHandle.Init.Prescaler = (((PclkFreq << 1) / obj->time_unit)) - 1; // 1 us tick
	}
	htim1.Init.Prescaler = timHandle.Init.Prescaler;

	timHandle.Init.Period = obj->half_period;
	htim1.Init.Period = obj->half_period * 2;

	/*  In case period or pre-scalers are out of range, loop-in to get valid values */
	while ((timHandle.Init.Period > 0xFFFF)
			|| (timHandle.Init.Prescaler > 0xFFFF)) {
		obj->prescaler = obj->prescaler << 1;
		if (APBxCLKDivider == RCC_HCLK_DIV1) {
			timHandle.Init.Prescaler = ((PclkFreq) / obj->time_unit) - 1;
		} else {
			timHandle.Init.Prescaler = ((PclkFreq << 1) / obj->time_unit) - 1;
		}
		timHandle.Init.Period = obj->half_period / obj->prescaler;
		/*  Period decreases and prescaler increases over loops, so check for
		 *  possible out of range cases */
		if ((timHandle.Init.Period < 0xFFFF)
				&& (timHandle.Init.Prescaler > 0xFFFF)) {
			assert(0); //"Cannot initialize PWM\n");
			break;
		}
	}
	while ((htim1.Init.Period > 0xFFFF)
			|| (htim1.Init.Prescaler > 0xFFFF)) {
		htim1_prescaler = htim1_prescaler << 1;
		if (APBxCLKDivider == RCC_HCLK_DIV1) {
			htim1.Init.Prescaler = ((PclkFreq) / obj->time_unit) - 1;
		} else {
			htim1.Init.Prescaler = ((PclkFreq << 1) / obj->time_unit) - 1;
		}
		htim1.Init.Period = (obj->half_period * 2) / htim1_prescaler;
		/*  Period decreases and prescaler increases over loops, so check for
		 *  possible out of range cases */
		if ((htim1.Init.Period < 0xFFFF)
				&& (htim1.Init.Prescaler > 0xFFFF)) {
			assert(0); //"Cannot initialize PWM\n");
			break;
		}
	}

	if ((obj->half_period > 0) && (obj->half_pulse > 0)
			&& (obj->half_period >= obj->half_pulse + obj->deadtime)) {

		timHandle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		timHandle.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
		timHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

		htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
		htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
		htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

		// Configure channels
		sConfig.OCMode = TIM_OCMODE_PWM1;
		sConfig.Pulse = obj->half_pulse;
		sConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
		sConfig.OCFastMode = TIM_OCFAST_DISABLE;
		sConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
		sConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

		if (HAL_TIM_PWM_ConfigChannel(&timHandle, &sConfig, channel)
				!= HAL_OK) {
			assert(0); //"Cannot initialize PWM\n");
		}

		sConfig.OCMode = TIM_OCMODE_PWM2;
		if (obj->half_pulse > 0)
			sConfig.Pulse = timHandle.Init.Period - obj->half_pulse;
		else
			sConfig.Pulse = 0xFFFFFFFF; // Really bad but should work
		/* Ideally, in this case, we would want to set Pulse to
		 * something greater than the period of the other bipolar
		 */

		if (HAL_TIM_PWM_ConfigChannel(&timHandle, &sConfig, channeln)
				!= HAL_OK) {
			assert(0); //"Cannot initialize PWM\n");
		}
		HAL_TIM_PWM_Stop(&timHandle, channel);
		HAL_TIM_PWM_Stop(&timHandle, channeln);
		HAL_TIM_PWM_Start(&timHandle, channel);
		HAL_TIM_PWM_Start(&timHandle, channeln);

		// This step ensures that the TIM registers are updated with the new
		// values.
		if (HAL_TIM_Base_Init(&timHandle) != HAL_OK) {
			assert(0); //"Cannot initialize PWM\n");
		}

		if (HAL_TIM_Base_Start(&timHandle) != HAL_OK) {
			assert(0); //"Cannot initialize PWM\n");
		}

		if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
			assert(0);

		bipolar_pwmout_allocate(obj);
	} else {
		bipolar_pwmout_free(obj);

		disable_clk(obj);
		if (HAL_TIM_Base_Stop(&timHandle) != HAL_OK) {
			assert(0); //"Cannot initialize PWM\n");
		}

		HAL_TIM_PWM_Stop(&timHandle, channel);
		HAL_TIM_PWM_Stop(&timHandle, channeln);
		//timHandle.Instance->CNT = 0;
	}
}

void bipolar_pwmout_init(bipolar_pwmout_t* obj, PinName pin, PinName pinn,
		float deadtime) {
	obj->time_unit = SystemCoreClock >> 2;
	obj->deadtime = (uint32_t) ceil(deadtime * obj->time_unit);
	// Get the peripheral name from the pin and assign it to the object
	obj->pwm = (PWMName) pinmap_peripheral(pin, PinMap_PWM);
	assert(obj->pwm != (PWMName)NC);
	assert(obj->pwm == (PWMName) pinmap_peripheral(pinn, PinMap_PWM));

	// Get the functions (timer channel, (non)inverted) from the pin and assign it to the object
	uint32_t function = pinmap_function(pin, PinMap_PWM);
	volatile uint32_t functionn = pinmap_function(pinn, PinMap_PWM);
	assert(function != (uint32_t)NC);
	assert(functionn != (uint32_t)NC);
	obj->channel = STM_PIN_CHANNEL(function);
	obj->channeln = STM_PIN_CHANNEL(functionn);
	assert(obj->channel != obj->channeln);

	// Configure GPIO
	obj->pin = pin;
	obj->pinn = pinn;
	obj->half_period = 0;
	obj->half_pulse = 0;
	obj->prescaler = 1;

	bipolar_pwmout_configure(obj);
}

uint32_t bipolar_pwmout_check_pulse_bu(bipolar_pwmout_t* obj, float pulse) {
	return (uint32_t) fabs(pulse * obj->time_unit);
}

uint32_t bipolar_pwmout_check_period_bu(bipolar_pwmout_t* obj, float period) {
	return (uint32_t) fabs(period * obj->time_unit);
}

float bipolar_pwmout_get_base_period(bipolar_pwmout_t* obj) {
	return 1.0 / obj->time_unit;
}

uint32_t bipolar_pwmout_get_base_frequency(bipolar_pwmout_t* obj) {
	return obj->time_unit;
}

uint32_t bipolar_pwmout_get_pulse_bu(bipolar_pwmout_t* obj) {
	return (obj->half_pulse << 1);
}

float bipolar_pwmout_get_pulse(bipolar_pwmout_t* obj) {
	return bipolar_pwmout_get_pulse_bu(obj)
			/ (float) obj->time_unit;
}

void bipolar_pwmout_set_pulse_bu(bipolar_pwmout_t* obj, uint32_t pulse) {
	obj->half_pulse = pulse >> 1;
	bipolar_pwmout_configure(obj);
}

void bipolar_pwmout_set_pulse(bipolar_pwmout_t* obj, float pulse) {
	bipolar_pwmout_set_pulse_bu(obj, (uint32_t) fabs(pulse * obj->time_unit));
}

uint32_t bipolar_pwmout_get_period_bu(bipolar_pwmout_t* obj) {
	return (obj->half_pulse << 1);
}

float bipolar_pwmout_get_period(bipolar_pwmout_t* obj) {
	return bipolar_pwmout_get_period_bu(obj)
			/ (float) obj->time_unit;
}

void bipolar_pwmout_set_period_bu(bipolar_pwmout_t* obj, uint32_t period) {
	obj->half_period = period >> 1;
	bipolar_pwmout_configure(obj);
}

void bipolar_pwmout_set_period(bipolar_pwmout_t* obj, float seconds) {
	bipolar_pwmout_set_period_bu(obj, (uint32_t) fabs(seconds * obj->time_unit));
}

void bipolar_pwmout_set_period_ms(bipolar_pwmout_t* obj, int ms) {
	bipolar_pwmout_set_period(obj, ms * 1e-3);
}

void bipolar_pwmout_set_period_us(bipolar_pwmout_t* obj, int us) {
	bipolar_pwmout_set_period(obj, us * 1e-6);
}

void bipolar_pwmout_set_period_pulse_bu(bipolar_pwmout_t* obj, uint32_t period,
		uint32_t pulse) {
	obj->half_period = period >> 1;
	obj->half_pulse = pulse >> 1;
	bipolar_pwmout_configure(obj);
}

void bipolar_pwmout_set_period_pulse(bipolar_pwmout_t* obj, float period,
		float pulse) {
	bipolar_pwmout_set_period_pulse_bu(obj,
			bipolar_pwmout_check_period_bu(obj, period),
			bipolar_pwmout_check_pulse_bu(obj, pulse));
}
