/**
 ******************************************************************************
 * @file    USB_Device/CDC_ECM_Server/Src/main.c
 * @author  MCD Application Team
 * @brief   USB device CDC_ECM demo main file
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                      www.st.com/SLA0044
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "hv_control.h"
#include "isolated_ps.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

// ADC handler declaration
ADC_HandleTypeDef hadc1;

// TIM handler declaration
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim8;

// USB handler declaration
USBD_HandleTypeDef       USBD_Device;

struct netif             gnetif;
__IO uint32_t LEDTimer = LED_TIMER_LONG;
volatile uint32_t cnt_10ms = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void BSP_Config(void);
static void CPU_CACHE_Enable(void);
static void MX_TIM1_Init();
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program
 * @param  None
 * @retval None
 */
int main(void) {
	/* Enable the CPU Cache */
	CPU_CACHE_Enable();

	/* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Low Level Initialization
	 */
	if (HAL_Init() != HAL_OK)
		Error_Handler();

	/* Configure the System clock to have a frequency of 216 MHz */
	SystemClock_Config();

	/* Configure the BSP */
	BSP_Config();

	/* COnfigure the High Voltage Control */
	// HV_Control_Config(); -> The stuff below should be acquired from the ETH connection

	/* Initialize the HV Control */
	HV_Control_Init(25);

	/*
	HV_Control.Enabled = 0;
	HV_Control.Bipolar_PWM_Deadtime = 0.6e-6;
	HV_Control.Gate_DutyCycle = 0.5;
	HV_Control.Gate_Period = 1e-04;
	HV_Control.Negative_Bipolar_PWM_Period = HV_Control_check_positive_period(1.3e-06);
	HV_Control.Negative_Bipolar_PWM_Pulse = HV_Control_check_positive_pulse(320e-09);
	HV_Control.Negative_MOS_On_Time = 2000;
	HV_Control.Negative_Output_Enabled = 0;
	HV_Control.Positive_Bipolar_PWM_Period = HV_Control_check_negative_period(1.3e-06);
	HV_Control.Positive_Bipolar_PWM_Pulse = HV_Control_check_negative_pulse(320e-09);
	HV_Control.Positive_MOS_On_Time = 2000;
	HV_Control.Positive_Output_Enabled = 0;
	HV_Control.Status = POSITIVE_START;
	HV_Control.Voltage_Threshold = 4096;

	HV_Control.Enabled = 1;
	 */

	/* Initialize all configured peripherals */
	MX_USB_Device_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_ADC1_Init();

	/* Configure the Isolated TLE (power supply) */
	TLE_Isolated_Init(100);

	/* Start TIM1, base start */
	if (HAL_TIM_Base_Start(&htim1) != HAL_OK)
		Error_Handler();

	/* Start all 4 channels of TIM8, Input Capture (IC) with interrupt (IT) */
	if (HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_4) != HAL_OK)
		Error_Handler();

	/* Start ADC1, with interrupt (IT) */
	if (HAL_ADC_Start_IT(&hadc1) != HAL_OK)
		Error_Handler();

	while (1) {
		Toggle_Leds();

		/* Background process of CDC_ECM class */
		USBD_CDC_ECM_fops.Process(&USBD_Device);
	}
}

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 216000000
 *            HCLK(Hz)                       = 216000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 4
 *            APB2 Prescaler                 = 2
 *            HSE Frequency(Hz)              = 25000000
 *            PLL_M                          = 25
 *            PLL_N                          = 432
 *            PLL_P                          = 2
 *            PLLSAI_N                       = 384
 *            PLLSAI_P                       = 8
 *            VDD(V)                         = 3.3
 *            Main regulator output voltage  = Scale1 mode
 *            Flash Latency(WS)              = 7
 * @param  None
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 432;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;

	if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
		Error_Handler();

	/* Activate the OverDrive to reach the 216 Mhz Frequency */
	if(HAL_PWREx_EnableOverDrive() != HAL_OK)
		Error_Handler();

	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
	if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK)
		Error_Handler();

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
			RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
		Error_Handler();
}

/**
 * @brief  TIM1 Initialization Function
 * @note   TIM1 role us to trigger an ADC1 conversion on update (default period = 1ms)
 * @note   The period is then updated in 'bipolar_pwmout_configure()', to meet the bipolar frequency
 * @param  None
 * @retval None
 */
static void MX_TIM1_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 54000 - 1;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
		Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
		Error_Handler();

}

/**
 * @brief  TIM8 Initialization Function
 * @note   TIM8 is used to handle the trigger input on comparator pins (2x falling and 2x rising)
 * @param  None
 * @retval None
 */
static void MX_TIM8_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 0;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 0xffff;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
		Error_Handler();

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
		Error_Handler();

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
		Error_Handler();

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
		Error_Handler();

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
		Error_Handler();

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
		Error_Handler();

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
		Error_Handler();
}

/**
 * @brief  ADC1 Initialization Function
 * @param  None
 * @retval None
 */
static void MX_ADC1_Init(void) {
	ADC_ChannelConfTypeDef sConfig;

	// Configure the ADC peripheral
	hadc1.Instance                   = ADC1;
	hadc1.Init.ClockPrescaler        = ADC_CLOCKPRESCALER_PCLK_DIV4;
	hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode    = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIGCONV_T1_TRGO;
	hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion       = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
		Error_Handler();

	// Configure ADC regular channel
	sConfig.Channel      = ADC_CHANNEL_10;
	sConfig.Rank         = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	sConfig.Offset       = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		Error_Handler();
}

/**
 * @brief  Setup the BSP.
 * @param  None
 * @retval None
 */
static void BSP_Config(void) {
#ifdef TARGET_NUCLEO_F746ZG
	/* Configure LED1, LED2, LED3 and LED4 */
	BSP_LED_Init(LED1);
	BSP_LED_Init(LED2);
	BSP_LED_Init(LED3);

	BSP_PB_Init(BUTTON_USER, BUTTON_MODE_GPIO);

	/* Set Systick Interrupt to the highest priority */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0x0, 0x0);
#endif
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* User may add here some code to deal with this error */
	while(1) {};
}

/*
 * @brief  Regular conversion complete callback in non blocking mode
 * @param  hadc pointer to a ADC_HandleTypeDef structure that contains
 *         the configuration information for the specified ADC.
 * @retval None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
	static uint8_t tick = 0;
	static uint8_t tickLong = 0;
	static uint32_t meanVal = 0;
	static uint32_t meanValLong = 0;

	/* Store ADC Value */
	uint32_t adcVal = HAL_ADC_GetValue(hadc);
	meanVal += adcVal;
	meanValLong += adcVal;

	/* Compute mean ADC value using the last 4 readings */
	if (tick >= 3) {
		/* ADC Mean Value calculation */
		HV_Control.Mean_Output_Value = (meanVal + 2) >> 2;
		meanVal = 0;

		tick = 0;

		/* HV Control State Machine */
		HV_Control_State_Machine();
	}
	else {
		tick ++;
	}

	/* Compute mean ADC value using the last 256 readings */
	if (tickLong == 0xFF) {
		/* ADC Mean Value calculation */
		HV_Control.Mean_Output_Value_Long = (meanValLong + 128) >> 8;
		meanValLong = 0;

		tickLong = 0;
	}
	else {
		tickLong ++;
	}
}

/*
 * @brief  CPU L1-Cache enable.
 * @param  None
 * @retval None
 */
static void CPU_CACHE_Enable(void) {
	/* Enable I-Cache */
	SCB_EnableICache();

	/* Enable D-Cache */
	SCB_EnableDCache();
}

/*
 * @brief  Toggle LEDs to show user input state.
 * @param  None
 * @retval None
 */
void Toggle_Leds(void) {
#ifdef TARGET_NUCLEO_F746ZG
	static uint32_t ticks;
	ticks++ ;
	if(ticks >= LEDTimer) {
		BSP_LED_Toggle(LED1);
		ticks = 0;
	}
#endif
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
