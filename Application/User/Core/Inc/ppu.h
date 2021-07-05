/*
 * ppu.h
 *
 *  Created on: Sep 9, 2019
 *      Author: chema
 */

#ifndef PPU_H_
#define PPU_H_

#if defined(TARGET_PPU_F746VG)

#define CONTROL_N_P PA_0
#define CONTROL_N_N PA_1
#define CONTROL_P_P PA_2
#define CONTROL_P_N PA_3
#define CURRENT_SENSE_1 PA_4
#define CURRENT_SENSE_2 PA_5
#define CURRENT_SENSE_3 PB_0
#define CURRENT_SENSE_4 PB_1

// !! POSITIVE_EN is PB_9 on the board, and NEGATIVE_EN is PB_8 !!!
//#define POSITIVE_EN PB_8_ALT0
//#define NEGATIVE_EN PB_9_ALT0
#define POSITIVE_EN PB_9_ALT0
#define NEGATIVE_EN PB_8_ALT0

#define I_OUTPUT PA_7
#define PPU_OUTPUT PC_0
#define V12_HK PC_1
#define TR1_COMP_A PC_6
#define TR1_COMP_B PC_7
#define TR2_COMP_A PC_8
#define TR2_COMP_B PC_9
#define ISOLATED_DIR PA_6
#define ISOLATED_DIS PD_14
#define TX PD_5
#define RX PD_6
#define SCLK PE_2
#define MISO PE_5
#define MOSI PE_6
#define SELECT_P PE_7
#define SELECT_N PE_8
#define SELECT_I PE_9

#elif defined(TARGET_NUCLEO_F746ZG)

#define CONTROL_N_P PB_3
#define CONTROL_N_N PA_15
#define CONTROL_P_P PB_10
#define CONTROL_P_N PB_11
#define CURRENT_SENSE_1 PA_4
#define CURRENT_SENSE_2 PA_5
#define POSITIVE_EN PB_8_ALT0
#define NEGATIVE_EN PB_9_ALT0
#define PPU_OUTPUT PC_0
#define V12_HK PC_1
#define ISOLATED_DIR PA_6
#define ISOLATED_DIS PD_14

#endif
#endif /* PPU_H_ */
