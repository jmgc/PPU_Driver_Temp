/**
 ******************************************************************************
 * @file    USB_Device/CDC_ECM_Server/Src/httpd_cg_ssi.c
 * @author  MCD Application Team
 * @brief   Webserver SSI and CGI handlers
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
#include "lwip/debug.h"
#include "lwip/tcp.h"
#include "lwip/apps/httpd.h"
#include "http_cgi_ssi.h"
#include "hv_control.h"

#include <string.h>
#include <stdlib.h>

/* Array of tags for the SSI handler */
#define numSSItags 17
char const *theSSItags[numSSItags] = {"tag1", "tag2", "tag3", "tag4", "tag5", "tag6", "tag7", "tag8", "tag9", "tag10",
		"tag11", "tag12", "tag13", "tag14", "tag15", "tag16", "tag17"};

void myCGIinit(void);
u16_t mySSIHandler(int iIndex, char *pcInsert, int iInsertLen);
void mySSIinit(void);

/* CGI handler for LED control */
const char * LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

/* CGI handlers for HV Control */
const char * HVControl_params_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char * HVControl_en_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);

/* The following html requests will start appropriate handlers */
const tCGI LEDS_CGI={"/leds.cgi", LEDS_CGI_Handler};
const tCGI HVControl_params_CGI={"/hvcontrol_params.cgi", HVControl_params_CGI_Handler};
const tCGI HVControl_en_CGI={"/hvcontrol_en.cgi", HVControl_en_CGI_Handler};

/* Cgi call table, the 3 CGI used */
tCGI CGI_TAB[3];

/* Initialize the CGI handlers */
void myCGIinit(void)
{
	/* Add LED control CGI to the table */
	CGI_TAB[0] = LEDS_CGI;

	/* Add HV control CGIs to the table */
	CGI_TAB[1] = HVControl_params_CGI;
	CGI_TAB[2] = HVControl_en_CGI;

	/* Give the table to the HTTP server */
	http_set_cgi_handlers(CGI_TAB, 3);
} /* myCGIinit */

/**
 * @brief  CGI handler for LEDs control
 */
const char * LEDS_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
	uint32_t i;

	if (iIndex==0) {
		/* All LEDs off, or when only one LED used, slower toggling timer value */
		/* BSP_LED_Off(LED1); */
		LEDTimer = LED_TIMER_LONG;

		/* Check cgi parameter : example GET /leds.cgi?led=2&led=4 */
		for (i=0; i<(uint32_t)iNumParams; i++) {
			/* check parameter "led" */
			if (strcmp(pcParam[i] , "led")== 0) {
				/* Switch LED1 ON if 1, or when only one LED used, faster toggling timer value */
				if(strcmp(pcValue[i], "1") == 0) {
					/* BSP_LED_On(LED1); */
					LEDTimer = LED_TIMER_SHORT;
				}
				else {
					LEDTimer = LED_TIMER_LONG;
				}
			}
		}
	}
	/* uri to send after cgi call*/
	return "/STM32F7xxLED.html";
}

const char * HVControl_params_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
	uint32_t i;

	if (iIndex == 1) {
		ACCESSING_PARAMETERS = 1;
		for (i=0; i<(uint32_t)iNumParams; i++) {
			if (strcmp(pcParam[i], "Gate_Period") == 0)
				HV_Control.Gate_Period = atoff(pcValue[i]);
			else if (strcmp(pcParam[i], "Gate_DutyCycle") == 0)
				HV_Control.Gate_DutyCycle = atoff(pcValue[i]);
			else if (strcmp(pcParam[i], "Bipolar_PWM_Deadtime") == 0)
				HV_Control.Bipolar_PWM_Deadtime = atoff(pcValue[i]);
			else if (strcmp(pcParam[i], "Positive_Bipolar_PWM_Period") == 0) {
				HV_Control.Positive_Bipolar_PWM_Period = HV_Control_check_positive_period(atoff(pcValue[i]));
				if (HV_Control.Positive_Bipolar_PWM_Period > 2.4e-03)
					HV_Control.Positive_Bipolar_PWM_Period = HV_Control_check_positive_period(2.4e-03);
			}
			else if (strcmp(pcParam[i], "Positive_Bipolar_PWM_Pulse") == 0)
				HV_Control.Positive_Bipolar_PWM_Pulse = HV_Control_check_positive_pulse(atoff(pcValue[i]));
			else if (strcmp(pcParam[i], "Negative_Bipolar_PWM_Period") == 0) {
				HV_Control.Negative_Bipolar_PWM_Period = HV_Control_check_negative_period(atoff(pcValue[i]));
				if (HV_Control.Positive_Bipolar_PWM_Period > 2.4e-03)
					HV_Control.Negative_Bipolar_PWM_Period = HV_Control_check_negative_period(2.4e-03);
			}
			else if (strcmp(pcParam[i], "Negative_Bipolar_PWM_Pulse") == 0)
				HV_Control.Negative_Bipolar_PWM_Pulse = HV_Control_check_negative_pulse(atoff(pcValue[i]));
			else if (strcmp(pcParam[i], "Voltage_Threshold") == 0)
				HV_Control.Voltage_Threshold = (uint32_t)atoi(pcValue[i]);
			else if (strcmp(pcParam[i], "Positive_MOS_On_Time") == 0)
				HV_Control.Positive_MOS_On_Time = (uint32_t)atoi(pcValue[i]);
			else if (strcmp(pcParam[i], "Negative_MOS_On_Time") == 0)
				HV_Control.Negative_MOS_On_Time = (uint32_t)atoi(pcValue[i]);

			else if (strcmp(pcParam[i], "Positive_MOS_Charge_Time") == 0)
				HV_Control.Positive_MOS_Charge_Time = (uint32_t)atoi(pcValue[i]);
			else if (strcmp(pcParam[i], "Negative_MOS_Charge_Time") == 0)
				HV_Control.Negative_MOS_Charge_Time = (uint32_t)atoi(pcValue[i]);

			else if (strcmp(pcParam[i], "Polarity_Switch_Delta_Time") == 0)
				HV_Control.Polarity_Switch_Delta_Time = (uint32_t)atoi(pcValue[i]);
		}
		ACCESSING_PARAMETERS = 0;
	}

	return "/STM32F7xxHVCONTROL.shtml";
}

const char * HVControl_en_CGI_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]) {
	uint32_t i;

	if (iIndex == 2) {
		for (i=0; i<(uint32_t)iNumParams; i++) {
			if (strcmp(pcParam[i], "Enable") == 0) {
				if ((!HV_Control.Enabled) && (HV_Control.Gate_Period > 0.) && (HV_Control.Positive_Bipolar_PWM_Period > 0.) && (HV_Control.Negative_Bipolar_PWM_Period > 0.) &&
						(HV_Control.Positive_MOS_On_Time > 0.) && (HV_Control.Negative_MOS_On_Time > 0.)) {
					HV_Control.Enabled = 1;
				}
			}
			else if (strcmp(pcParam[i], "Disable") == 0) {
				if (HV_Control.Enabled)
					HV_Control.Stopping = 1;
			}
		}
	}

	return "/STM32F7xxHVCONTROL.shtml";
}


/**** SSI handler ****/
u16_t mySSIHandler(int iIndex, char *pcInsert, int iInsertLen) {
	uint16_t pos;
	HV_ADC_Reading* currentReading;
	char Tag_Str[20];
	char Tag_Str_Long[250];

	switch (iIndex) {
	// Current HV_Control values
	case 0:
		sprintf(Tag_Str, "%g", HV_Control.Gate_Period);
		break;
	case 1:
		sprintf(Tag_Str, "%g", HV_Control.Gate_DutyCycle);
		break;
	case 2:
		sprintf(Tag_Str, "%g", HV_Control.Bipolar_PWM_Deadtime);
		break;
	case 3:
		sprintf(Tag_Str, "%g", HV_Control.Positive_Bipolar_PWM_Period);
		break;
	case 4:
		sprintf(Tag_Str, "%g", HV_Control.Positive_Bipolar_PWM_Pulse);
		break;
	case 5:
		sprintf(Tag_Str, "%g", HV_Control.Negative_Bipolar_PWM_Period);
		break;
	case 6:
		sprintf(Tag_Str, "%g", HV_Control.Negative_Bipolar_PWM_Pulse);
		break;
	case 7:
		sprintf(Tag_Str, "%lu", HV_Control.Voltage_Threshold);
		break;
	case 8:
		sprintf(Tag_Str, "%lu", HV_Control.Positive_MOS_On_Time);
		break;
	case 9:
		sprintf(Tag_Str, "%lu", HV_Control.Negative_MOS_On_Time);
		break;
	case 12:
		sprintf(Tag_Str, "%lu", HV_Control.Positive_MOS_Charge_Time);
		break;
	case 13:
		sprintf(Tag_Str, "%lu", HV_Control.Negative_MOS_Charge_Time);
		break;
	case 14:
		sprintf(Tag_Str, "%lu", HV_Control.Polarity_Switch_Delta_Time);
		break;
	case 15:
		sprintf(Tag_Str, "%d", HV_Control.Enabled + HV_Control.Stopping);
		break;

		// ADC values plot
	case 10:
		ACCESSING_READINGS_LIST = 1;
		pos = 0;
		currentReading = HV_Control.Output_Readings_List->first;
		pos += sprintf(&Tag_Str_Long[pos], "var values=[%u", currentReading->val);
		currentReading = currentReading->next;
		while (currentReading != NULL) {
			pos += sprintf(&Tag_Str_Long[pos], ",%u", currentReading->val);
			currentReading = currentReading->next;
		}
		ACCESSING_READINGS_LIST = 0;
		pos += sprintf(&Tag_Str_Long[pos], "];");
		break;

	case 11:
		sprintf(Tag_Str, "var length=%u;", HV_Control.Output_Readings_List_Size);
		break;

		// ADV Values GUI update
	case 16:
		// Return the mean value of the values in the HV_Control.Output_Readings_List array
		ACCESSING_READINGS_LIST = 1;
		uint32_t returnVal;
		currentReading = HV_Control.Output_Readings_List->first;
		while (currentReading != NULL) {
			returnVal += currentReading->val;
			currentReading = currentReading->next;
		}
		ACCESSING_READINGS_LIST = 0;

		returnVal = returnVal / HV_Control.Output_Readings_List_Size;
		sprintf(Tag_Str, "@%lu", returnVal);
		break;

		// Default
	default:
		return 0;
		break;
	}

	if (iIndex != 10) {//&& iIndex != 16) {
		(void)strcpy(pcInsert, Tag_Str);
		return (u16_t)strlen(Tag_Str);
	}
	else {
		(void)strcpy(pcInsert, Tag_Str_Long);
		return (u16_t)strlen(Tag_Str_Long);
	}
} /* mySSIHandler */

/**** Initialize SSI handlers ****/
void mySSIinit(void)
{
	/* Configure SSI handler function */
	/* theSSItags is an array of SSI tag strings to search for in SSI-enabled files */
	http_set_ssi_handler(mySSIHandler, (char const **)theSSItags, numSSItags);
} /* mySSIinit */

/**
 * @brief  Http webserver Init
 */
void http_server_init(void)
{
	/* Httpd Init */
	httpd_init();

	/* configure CGI handlers (LEDs control CGI) */
	myCGIinit();

	/* configure SSI handlers */
	mySSIinit();
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
