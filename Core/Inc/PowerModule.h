/*
 * PowerModule.h
 *
 *  Created on: 6 Mart 2020
 *      Author: Yusuf TUNC
 */

#ifndef POWER_MODULE_H_
#define POWER_MODULE_H_

//#include "stdint.h"

#define PW_REF_VOLT 52.5f
#define PW_REF_AMP  90.0f
// 50mV akým çýkýþýndan elde edilen gerilim

struct pw_t
{
	uint32_t adcBuf[2];
	float volt,amper;
	float filt_volt,filt_amper;
	uint8_t CH;
}pw;

void pw_init(void);
void pw_calc(void);

#endif /* POWERMODULE_H_ */
