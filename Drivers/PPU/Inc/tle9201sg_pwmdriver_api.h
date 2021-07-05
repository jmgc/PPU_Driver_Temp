
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
#ifndef TLE9201SG_PWMDRIVER_API_H
#define TLE9201SG_PWMDRIVER_API_H

#include "stm32f7xx_hal.h"

#ifdef HAL_TIM_MODULE_ENABLED
#define DEVICE_PWMOUT

#include "pin_names.h"
#include "peripheral_names.h"
#include "peripheral_pins.h"
#include "stm32f7xx_hal_tim.h"
#include "pwmout_device.h"

/** Pwmout hal structure. pwmout_s is declared in the target's hal
 */
struct tle9201sg_pwmdriver_s {
	PWMName pwm;
	PWMName dir;
    PinName pin_pwm;
    PinName pin_dir;
    uint32_t prescaler;
    uint32_t period;
    uint32_t pulse;
    uint8_t channel_pwm;
    uint8_t channel_dir;
};

typedef struct tle9201sg_pwmdriver_s tle9201sg_pwmdriver_t;

/**
 * \defgroup hal_pwmout Pwmout hal functions
 * @{
 */

/** Initialize the bipolar pwm out peripheral and configure the pin
 *
 * @param obj The bipolar pwmout object to initialize
 * @param pin_pwm The pwm pin to initialize
 * @param pin_dir The dir pin to initialize
 */
void tle9201sg_pwmdriver_init(tle9201sg_pwmdriver_t *obj, PinName pin_pwm, PinName pin_dir);

/** Deinitialize the bipolar pwmout object
 *
 * @param obj The bipolar pwmout object
 */
void tle9201sg_pwmdriver_free(tle9201sg_pwmdriver_t *obj);

/** Set the output pulse in seconds
 *
 * @param obj     The pwmout object
 * @param pulse The floating-point pulse in seconds
 */
void tle9201sg_pwmdriver_pulse(tle9201sg_pwmdriver_t* obj, float pulse);

/** Set the output period in seconds
 *
 * @param obj     The pwmout object
 * @param value The floating-point period in seconds
 */
void tle9201sg_pwmdriver_write(tle9201sg_pwmdriver_t *obj, float value);

/** Read the current float-point output period in seconds
 *
 * @param obj The bipolar pwmout object
 * @return A floating-point output period
 */
float tle9201sg_pwmdriver_read(tle9201sg_pwmdriver_t *obj);

/** Set the PWM period specified in seconds, keeping the duty cycle the same
 *
 * Periods smaller than microseconds (the lowest resolution) are set to zero.
 * @param obj     The bipolar pwmout object
 * @param seconds The floating-point seconds period
 */
void tle9201sg_pwmdriver_period(tle9201sg_pwmdriver_t *obj, float seconds);

/** Set the PWM period specified in miliseconds, keeping the duty cycle the same
 *
 * @param obj The bipolar pwmout object
 * @param ms  The milisecond period
 */
void tle9201sg_pwmdriver_period_ms(tle9201sg_pwmdriver_t *obj, int ms);

/** Set the PWM period specified in microseconds, keeping the duty cycle the same
 *
 * @param obj The bipolar pwmout object
 * @param us  The microsecond period
 */
void tle9201sg_pwmdriver_period_us(tle9201sg_pwmdriver_t *obj, int us);

#endif

#endif
/** @}*/
