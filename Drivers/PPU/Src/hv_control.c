/*
 * hv_control.c
 *
 *  Created on: 13 avr. 2021
 *      Author: Guillaume
 */
#include "hv_control.h"
#include "ppu.h"

hv_control_t HV_Control;
tle9201sg_pwmdriver_t TLE_Driver;
volatile int ACCESSING_READINGS_LIST = 0;
uint8_t ACCESSING_PARAMETERS = 0;


void TLE_Driver_Init(void) {
	tle9201sg_pwmdriver_init(&TLE_Driver, ISOLATED_DIS, ISOLATED_DIR);
	const float isolatedPeriod = 2e-04;
	const float isolatedPulse = isolatedPeriod / 4.;

	tle9201sg_pwmdriver_period(&TLE_Driver, isolatedPeriod);
	tle9201sg_pwmdriver_pulse(&TLE_Driver, isolatedPulse);
}

void HV_Control_Init(uint16_t readingsSize) {
	/* Init Readings List */
	HV_Control.Output_Readings_List_Size = readingsSize;
	HV_Control.Output_Readings_List = Init_ADC_Readings_List(readingsSize);

	/* Bipolar PWM Init*/
	bipolar_pwmout_init(&HV_Control.pwm_P, CONTROL_P_P, CONTROL_P_N, HV_Control.Bipolar_PWM_Deadtime);
	bipolar_pwmout_init(&HV_Control.pwm_N, CONTROL_N_P, CONTROL_N_N, HV_Control.Bipolar_PWM_Deadtime);

	/* Gate PWM Init*/
	pwmout_init(&HV_Control.gate_P, POSITIVE_EN);
	pwmout_init(&HV_Control.gate_N, NEGATIVE_EN);

	/* Analog Input Init*/
	// The ADC1 is already configured in 'main.c', so we can't redefine it here.
	// We can't use analogin_t structure type
	// Solution -> Use the adc directly
	//analogin_init(&HV_Control.output_V, PPU_OUTPUT);
}

float HV_Control_check_positive_pulse(float pulse) {
	return 2. * (bipolar_pwmout_check_pulse_bu(&HV_Control.pwm_P, pulse) >> 1) / ((float)HV_Control.pwm_P.time_unit);
}

float HV_Control_check_negative_pulse(float pulse) {
	return 2. * (bipolar_pwmout_check_pulse_bu(&HV_Control.pwm_N, pulse) >> 1) / ((float)HV_Control.pwm_N.time_unit);
}


float HV_Control_check_positive_period(float period) {
	return 2. * (bipolar_pwmout_check_period_bu(&HV_Control.pwm_P, period) >> 1) / ((float)HV_Control.pwm_P.time_unit);
}

float HV_Control_check_negative_period(float period) {
	return 2. * (bipolar_pwmout_check_period_bu(&HV_Control.pwm_N, period) >> 1) / ((float)HV_Control.pwm_N.time_unit);
}

void HV_Control_Enable_Output(pwmout_t* gate, float period, float dutyCycle) {
	pwmout_period(gate, period);
	pwmout_write(gate, dutyCycle);
}

void HV_Control_Disable_Output(pwmout_t* gate) {
	pwmout_write(gate, 0.0);
}

void HV_Control_Start_BipolarPWM(bipolar_pwmout_t* pwm, float period, float pulse) {
	bipolar_pwmout_set_period_pulse(pwm, period, pulse);
}

void HV_Control_Stop_BipolarPWM(bipolar_pwmout_t* pwm) {
	if (HV_Control.Status == POSITIVE_ON) {
		bipolar_pwmout_free(&HV_Control.pwm_P);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
	}
	else if (HV_Control.Status == NEGATIVE_ON) {
		bipolar_pwmout_free(&HV_Control.pwm_N);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	}

	bipolar_pwmout_set_pulse(pwm, 0.0);
}

int HV_Control_Is_Voltage_Below_Threshold(uint32_t threshold) {
	return (HV_Control.Mean_Output_Value <= threshold);
}


/*
 * @brief  Disable both MOS outputs and reset the HV_Control parameters
 * @param  None
 * @retval None
 */
void HV_Control_Reset(void) {
	if (HV_Control.Positive_Output_Enabled) {
		HV_Control_Disable_Output(&HV_Control.gate_P);
		HV_Control.Positive_Output_Enabled = 0;
	}
	else if (HV_Control.Negative_Output_Enabled) {
		HV_Control_Disable_Output(&HV_Control.gate_N);
		HV_Control.Negative_Output_Enabled = 0;
	}

	HV_Control.Enabled = 0; // Disable HV Control
	HV_Control.Stopping = 0; // Successfully stopped
	state_machine_cnt_ms = 0;
}

void HV_Control_State_Machine(void) {
	if (HV_Control.Enabled && !ACCESSING_PARAMETERS)
		switch (HV_Control.Status) {
		case POSITIVE_START:
			if (!HV_Control.Stopping) {
				if (!HV_Control.Positive_Output_Enabled) {
					if (state_machine_cnt_ms >= HV_Control.Polarity_Switch_Delta_Time) {
						HV_Control_Enable_Output(&HV_Control.gate_P, HV_Control.Gate_Period, HV_Control.Gate_DutyCycle);
						HV_Control.Positive_Output_Enabled = 1;
						state_machine_cnt_ms = 0;
					}
				}
				else if (state_machine_cnt_ms >= HV_Control.Positive_MOS_Charge_Time) {
					HV_Control_Start_BipolarPWM(&HV_Control.pwm_P, HV_Control.Positive_Bipolar_PWM_Period, HV_Control.Positive_Bipolar_PWM_Pulse);
					HV_Control.Positive_Bipolar_Enabled = 1;
					HV_Control.Status = POSITIVE_ON;
					state_machine_cnt_ms = 0;
				}
			}
			else if (!HV_Control.Positive_Bipolar_Enabled) {
				HV_Control_Reset();
			}
			break;

		case POSITIVE_ON:
			if (state_machine_cnt_ms >= HV_Control.Positive_MOS_On_Time || HV_Control.Stopping) {
				HV_Control_Stop_BipolarPWM(&HV_Control.pwm_P);
				HV_Control.Positive_Bipolar_Enabled = 0;
				HV_Control.Status = POSITIVE_WAIT_THRESHOLD;
			}
			break;

		case POSITIVE_WAIT_THRESHOLD:
			if (HV_Control_Is_Voltage_Below_Threshold(HV_Control.Voltage_Threshold)) {
				HV_Control_Disable_Output(&HV_Control.gate_P);
				HV_Control.Positive_Output_Enabled = 0;
				state_machine_cnt_ms = 0;
				HV_Control.Status = NEGATIVE_START;
			}
			break;

		case NEGATIVE_START:
			if (!HV_Control.Stopping) {
				if (!HV_Control.Negative_Output_Enabled) {
					if (state_machine_cnt_ms >= HV_Control.Polarity_Switch_Delta_Time) {
						HV_Control_Enable_Output(&HV_Control.gate_N, HV_Control.Gate_Period, HV_Control.Gate_DutyCycle);
						HV_Control.Negative_Output_Enabled = 1;
						state_machine_cnt_ms = 0;
					}
				}
				else if (state_machine_cnt_ms >= HV_Control.Negative_MOS_Charge_Time) {
					HV_Control_Start_BipolarPWM(&HV_Control.pwm_N, HV_Control.Negative_Bipolar_PWM_Period, HV_Control.Negative_Bipolar_PWM_Pulse);
					HV_Control.Negative_Bipolar_Enabled = 1;
					HV_Control.Status = NEGATIVE_ON;
					state_machine_cnt_ms = 0;
				}
			}
			else if (!HV_Control.Negative_Bipolar_Enabled) {
				HV_Control_Reset();
			}
			break;

		case NEGATIVE_ON:
			if (state_machine_cnt_ms >= HV_Control.Negative_MOS_On_Time || HV_Control.Stopping) {
				HV_Control_Stop_BipolarPWM(&HV_Control.pwm_N);
				HV_Control.Negative_Bipolar_Enabled = 0;
				HV_Control.Status = NEGATIVE_WAIT_THRESHOLD;
			}
			break;

		case NEGATIVE_WAIT_THRESHOLD:
			if (HV_Control_Is_Voltage_Below_Threshold(HV_Control.Voltage_Threshold)) {
				HV_Control_Disable_Output(&HV_Control.gate_N);
				HV_Control.Negative_Output_Enabled = 0;
				state_machine_cnt_ms = 0;
				HV_Control.Status = POSITIVE_START;
			}
			break;

		default:
			break;
		}
}

HV_ADC_Readings_List* Init_ADC_Readings_List(uint16_t size) {
	if (size == 0)
		assert(0);

	HV_ADC_Readings_List* list = malloc(sizeof(*list));
	HV_ADC_Reading* reading = malloc(sizeof(*reading));

	if (list == NULL || reading == NULL)
		assert(0);

	reading->val = 0;
	reading->next = NULL;
	list->first = reading;
	list->last = reading;

	for (uint16_t i=0; i<size-1; i++)
		AddTo_ADC_Readings_List(list, 0);

	return list;
}

// NOT FINISHED YET
void HV_Control_ClosedLoop() {
	// If NEGATIVE_ON or POSITIVE_ON
	// AND IF T>ON_TIME/2 (at least)
	// Do only once per polarity switch
	// Add Positive_Bipolar_PWM_Pulse_Offset or Negative_Bipolar_PWM_Pulse_Offset
	// Will be computed in next polarity switch
	if (HV_Control.Mean_Output_Value >= HV_Control.Target_Voltage) {
		if (HV_Control.Mean_Output_Value - HV_Control.Target_Voltage > HV_Control.Target_Voltage_Tolerance) {
			// Reduce Output
			HV_Control.Positive_Bipolar_PWM_Pulse_Offset -= 5e-06;
			HV_Control.Negative_Bipolar_PWM_Pulse_Offset -= 5e-06;
		}
	}
	else {
		if (HV_Control.Target_Voltage - HV_Control.Mean_Output_Value > HV_Control.Target_Voltage_Tolerance) {
			// Increase Output
			HV_Control.Positive_Bipolar_PWM_Pulse_Offset += 5e-06;
			HV_Control.Negative_Bipolar_PWM_Pulse_Offset += 5e-06;
		}
	}
}

void AddTo_ADC_Readings_List(HV_ADC_Readings_List* list, uint16_t value) {
	HV_ADC_Reading* newReading = malloc(sizeof(*newReading));

	if (newReading == NULL)
		assert(0);
	newReading->val = value;
	newReading->next = NULL;

	list->last->next = newReading;
	list->last = newReading;
}

void FreeFirstOf_ADC_Readings_List(HV_ADC_Readings_List* list) {
	if (list == NULL)
		assert(0);

	if (list->first != NULL) {
		HV_ADC_Reading* readingToFree = list->first;
		list->first = list->first->next;
		free(readingToFree);
	}
}

void Update_ADC_Readings_List(HV_ADC_Readings_List* list, uint16_t value) {
	FreeFirstOf_ADC_Readings_List(list);
	AddTo_ADC_Readings_List(list, value);
}
