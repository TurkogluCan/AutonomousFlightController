/*
 * PowerModule.c
 *
 *  Created on: 6 Mart 2020
 *      Author: Yusuf TUNC
 */

#include "common.h"

void pw_init()
{
	HAL_ADC_Start_DMA(&hadc1,pw.adcBuf,2);
}

void pw_calc()
{
	filter_adc_voltaj((float*)&pw.adcBuf[0]);
	pw.volt = (pw.adcBuf[0] * 3.3f) / 4095.0f;
	pw.volt = (pw.volt * PW_REF_VOLT) / 5.0f;
	pw.filt_volt = (float)(((uint16_t)(pw.volt * 100)) / 100.0f);

	filter_adc_akim((float*)&pw.adcBuf[1]);
	pw.amper = (pw.adcBuf[1] * 3.3f) / 4095.0f;
	pw.amper = (pw.amper * PW_REF_AMP) / 5.0f;
	pw.filt_amper  = (float)(((uint16_t)(pw.amper * 100)) / 100.0f);
}


