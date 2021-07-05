
/** \addtogroup hal */
/** @{*/
/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef BIPOLAR_PWMOUT_API_H
#define BIPOLAR_PWMOUT_API_H

#include "stm32f7xx_hal.h"

#ifdef HAL_TIM_MODULE_ENABLED
#define DEVICE_PWMOUT

#include "pin_names.h"
#include "peripheral_names.h"
#include "gpio_object.h"
#include "pwmout_device.h"

/** Pwmout hal structure. pwmout_s is declared in the target's hal
 */
struct bipolar_pwmout_s {
	PWMName pwm;
    PinName pin;
    PinName pinn;
    uint32_t prescaler;
    uint32_t half_period;
    uint32_t deadtime;
    uint32_t half_pulse;
    uint32_t time_unit;
    uint8_t channel;
    uint8_t channeln;
    gpio_t gpio;
    gpio_t gpion;
};

typedef struct bipolar_pwmout_s bipolar_pwmout_t;

/**
 * \defgroup hal_pwmout Pwmout hal functions
 * @{
 */

/** Initialize the bipolar pwm out peripheral and configure the pin
 *
 * @param obj The bipolar pwmout object to initialize
 * @param pin The positive polarity pwmout pin to initialize
 * @param pinn The negative polarity pwmout pin to initialize
 * @param base_period The default period of the timer
 */
void bipolar_pwmout_init(bipolar_pwmout_t *obj, PinName pin, PinName pinn, float deadtime);

uint32_t bipolar_pwmout_check_pulse_bu(bipolar_pwmout_t* obj, float pulse);
uint32_t bipolar_pwmout_check_period_bu(bipolar_pwmout_t* obj, float period);


/** Deinitialize the bipolar pwmout object
 *
 * @param obj The bipolar pwmout object
 */
void bipolar_pwmout_free(bipolar_pwmout_t *obj);

/** Get the current float_point output base period in seconds
 *
 * @param obj     The pwmout object
 * @return A floating_point output base period
 */
float bipolar_pwmout_get_base_period(bipolar_pwmout_t* obj);

/** Get the current output base period in seconds
 *
 * @param obj     The pwmout object
 * @return Output base period
 */
uint32_t bipolar_pwmout_get_base_frequency(bipolar_pwmout_t* obj);

/** Get the current output pulse in base period
 *
 * @param obj     The pwmout object
 * @return The output pulse in base period
 */
uint32_t bipolar_pwmout_get_pulse_bu(bipolar_pwmout_t* obj);

/** Get the current float_point output pulse in seconds
 *
 * @param obj     The pwmout object
 * @return A floating_point output pulse
 */
float bipolar_pwmout_get_pulse(bipolar_pwmout_t* obj);

/** Set the output pulse in base period
 *
 * @param obj     The pwmout object
 * @param pulse The pulse in base period
 */
void bipolar_pwmout_set_pulse_bu(bipolar_pwmout_t* obj, uint32_t pulse);

/** Set the output pulse in seconds
 *
 * @param obj     The pwmout object
 * @param pulse The floating-point pulse in seconds
 */
void bipolar_pwmout_set_pulse(bipolar_pwmout_t* obj, float pulse);

/** Get the current output period in base period
 *
 * @param obj     The pwmout object
 * @return The output period in base period
 */
uint32_t bipolar_pwmout_get_period_bu(bipolar_pwmout_t* obj);

/** Read the current float-point output period in seconds
 *
 * @param obj The bipolar pwmout object
 * @return A floating-point output period
 */
float bipolar_pwmout_get_period(bipolar_pwmout_t *obj);

/** Set the PWM period specified in base units
 *
 * Periods smaller than base unit (the lowest resolution) are set to zero.
 * @param obj     The bipolar pwmout object
 * @param seconds The period in base units
 */
void bipolar_pwmout_set_period_bu(bipolar_pwmout_t* obj, uint32_t period);

/** Set the PWM period specified in seconds
 *
 * Periods smaller than microseconds (the lowest resolution) are set to zero.
 * @param obj     The bipolar pwmout object
 * @param seconds The floating-point seconds period
 */
void bipolar_pwmout_set_period(bipolar_pwmout_t *obj, float seconds);

/** Set the PWM period specified in miliseconds
 *
 * @param obj The bipolar pwmout object
 * @param ms  The milisecond period
 */
void bipolar_pwmout_set_period_ms(bipolar_pwmout_t *obj, int ms);

/** Set the PWM period specified in microseconds
 *
 * @param obj The bipolar pwmout object
 * @param us  The microsecond period
 */
void bipolar_pwmout_set_period_us(bipolar_pwmout_t *obj, int us);

/** Set the PWM period specified in base untis
 *
 * Periods smaller than base unit (the lowest resolution) are set to zero.
 * @param obj     The bipolar pwmout object
 * @param period The period in base units
 * @param pulse The pulse in base units
 */
void bipolar_pwmout_set_period_pulse_bu(bipolar_pwmout_t* obj, uint32_t period,
		uint32_t pulse);

/** Set the PWM period specified in seconds
 *
 * Periods smaller than microseconds (the lowest resolution) are set to zero.
 * @param obj     The bipolar pwmout object
 * @param period The floating-point seconds period
 * @param pulse The floating-point seconds pulse
 */
void bipolar_pwmout_set_period_pulse(bipolar_pwmout_t* obj, float period, float pulse);
/**@}*/

#endif

#endif

/** @}*/
