/*
 * hv_control.h
 *
 *  Created on: 13 avr. 2021
 *      Author: Guillaume
 */

#ifndef PPU_INC_HV_CONTROL_H_
#define PPU_INC_HV_CONTROL_H_

#include "main.h"

typedef enum {
	POSITIVE_START,
	POSITIVE_ON,
	POSITIVE_WAIT_THRESHOLD,
	NEGATIVE_START,
    NEGATIVE_ON,
	NEGATIVE_WAIT_THRESHOLD,
} HV_Control_State;

typedef struct HV_ADC_Reading HV_ADC_Reading;
struct HV_ADC_Reading {
	uint16_t val;
	HV_ADC_Reading* next;
};

typedef struct HV_ADC_Readings_List HV_ADC_Readings_List;
struct HV_ADC_Readings_List {
	HV_ADC_Reading* first;
	HV_ADC_Reading* last;
};

HV_ADC_Readings_List* Init_ADC_Readings_List(uint16_t size);
void AddTo_ADC_Readings_List(HV_ADC_Readings_List* list, uint16_t value);
void FreeFirstOf_ADC_Readings_List(HV_ADC_Readings_List* list);
void Update_ADC_Readings_List(HV_ADC_Readings_List* list, uint16_t value);

struct hv_control_s {
	int Enabled;
	int Stopping;
	int Positive_Bipolar_Enabled;
	int Negative_Bipolar_Enabled;
	int Positive_Output_Enabled;
	int Negative_Output_Enabled;

	float Gate_Period;
	float Gate_DutyCycle;
	float Bipolar_PWM_Deadtime;
	float Positive_Bipolar_PWM_Period;
	float Positive_Bipolar_PWM_Pulse;
	float Positive_Bipolar_PWM_Pulse_Offset;
	float Negative_Bipolar_PWM_Period;
	float Negative_Bipolar_PWM_Pulse;
	float Negative_Bipolar_PWM_Pulse_Offset;
	uint32_t Target_Voltage;
	uint32_t Target_Voltage_Tolerance;
	uint32_t Voltage_Threshold;
	uint32_t Positive_MOS_On_Time;
	uint32_t Negative_MOS_On_Time;
	uint32_t Positive_MOS_Charge_Time;
	uint32_t Negative_MOS_Charge_Time;
	uint32_t Polarity_Switch_Delta_Time;

	HV_Control_State Status;

	uint16_t Output_Readings_List_Size;
	HV_ADC_Readings_List* Output_Readings_List;
	uint16_t Mean_Output_Value;
	uint16_t Mean_Output_Value_Long;

	bipolar_pwmout_t pwm_P;
	bipolar_pwmout_t pwm_N;
	pwmout_t gate_P;
	pwmout_t gate_N;
};
typedef struct hv_control_s hv_control_t;

extern ADC_HandleTypeDef hadc1;

extern hv_control_t HV_Control;
extern tle9201sg_pwmdriver_t TLE_Driver;
extern volatile int ACCESSING_READINGS_LIST;

void TLE_Driver_Init();

/*
 * @brief  Init the Bipolar PWM, the Gate PWM, and the Analog Input.
 * @param  readingsSize number of past adc readings kept in memory
 * @retval None
 */
void HV_Control_Init(uint16_t readingsSize);

float HV_Control_check_positive_pulse(float pulse);
float HV_Control_check_negative_pulse(float pulse);
float HV_Control_check_positive_period(float period);
float HV_Control_check_negative_period(float period);

/*
 * @brief  Enable the given MOS
 * @param  gate pointer to a pwmout object that contains
 *         the configuration information for the specified gate PWM.
 * @param  period floating-point pwm period (in seconds)
 * @param  dutyCycle floating-point duty cycle value (percent)
 * @retval None
 */
void HV_Control_Enable_Output(pwmout_t* gate, float period, float dutyCycle);

/*
 * @brief  Disable the given MOS
 * @param  gate pointer to a pwmout object that contains
 *         the configuration information for the specified gate PWM.
 * @retval None
 */
void HV_Control_Disable_Output(pwmout_t* gate);

/*
 * @brief  Start the given bipolar PWM with the given parameters
 * @param  pwm pointer to the bipolar pwmout object
 * @param  period The floating-point seconds period
 * @param  pulse The floating-point seconds pulse
 * @retval None
 */
void HV_Control_Start_BipolarPWM(bipolar_pwmout_t* pwm, float period, float pulse);

/*
 * @brief  Stop the given bipolar PWM
 * @param  pwm pointer to the bipolar pwmout object
 * @param  period The floating-point seconds period
 * @param  pulse The floating-point seconds pulse
 * @retval None
 */
void HV_Control_Stop_BipolarPWM(bipolar_pwmout_t* pwm);

/*
 * @brief  Check if the voltage read by the ADC is below a given threshold
 * @param  threshold ADC value below which it is safe to switch polarity
 * @retval Int value representing a boolean (0 for false, 1 for true)
 */
int HV_Control_Is_Voltage_Below_Threshold(uint32_t threshold);

/*
 * @brief  State Machine of the HV Control - Should be called in the ADC interrupt handler
 * @param  None
 * @retval None
 */
void HV_Control_State_Machine(void);

void HV_Control_ClosedLoop(void);


#endif /* PPU_INC_HV_CONTROL_H_ */
