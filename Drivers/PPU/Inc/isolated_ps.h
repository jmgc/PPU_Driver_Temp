/*
 * isolated_ps.h
 *
 *  Created on: May 4, 2021
 *      Author: Guillaume
 */

#ifndef PPU_INC_ISOLATED_PS_H_
#define PPU_INC_ISOLATED_PS_H_

#include "main.h"

void TLE_Isolated_Init(uint32_t pwm_half_period_us);
void MX_TIM3_Init(uint16_t PSC, uint16_t ARR);
void MX_TIM4_Init(uint16_t PSC, uint16_t ARR);


#endif /* PPU_INC_ISOLATED_PS_H_ */
