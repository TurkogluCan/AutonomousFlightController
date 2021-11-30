/*
 * read_ppm.c
 *
 *  Created on: 2 Mar 2020
 *      Author: Yusuf TUNC
 */

#include "common.h"

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(GPIOA->IDR & GPIO_PIN_6)
		{
			ch.t[ch.pulse] = htim->Instance->CNT;

			switch (ch.pulse){
			// -------------- channel 1 --------------- //
			case 1:
			ch.roll = ch.t[1] - ch.t[0];
			ch.pulse++;
			if (ch.roll > 3000) {
				ch.t[0] = ch.t[1];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 2 --------------- //
			case 2:
			ch.pitch = ch.t[2] - ch.t[1];
			ch.pulse++;
			if (ch.pitch > 3000) {
				ch.t[0] = ch.t[2];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 3 --------------- //
			case 3:
			ch.throttle = ch.t[3] - ch.t[2];
			ch.pulse++;
			if (ch.throttle > 3000) {
				ch.t[0] = ch.t[3];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 4 --------------- //
			case 4:
			ch.yaw = ch.t[4] - ch.t[3];
			ch.pulse++;
			if (ch.yaw > 3000) {
				ch.t[0] = ch.t[4];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 5 --------------- //
			case 5:
			ch.swA = ch.t[5] - ch.t[4];
			ch.pulse++;
			if (ch.swA > 3000) {
				ch.t[0] = ch.t[5];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 6 --------------- //
			case 6:
			ch.swB = ch.t[6] - ch.t[5];
			ch.pulse++;
			if (ch.swB > 3000) {
				ch.t[0] = ch.t[6];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 7 --------------- //
			case 7:
			ch.swC = ch.t[7] - ch.t[6];
			ch.pulse++;
			if (ch.swC > 3000) {
				ch.t[0] = ch.t[7];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 8 --------------- //
			case 8:
			ch.swD = ch.t[8] - ch.t[7];
			ch.pulse++;
			if (ch.swD > 3000) {
				ch.t[0] = ch.t[8];
				ch.pulse = 1;
			}
			break;

			// -------------- channel 0 --------------- //
			case 9:
			ch.elps_time = ch.t[9] - ch.t[8];
			ch.t[0] = 0;
			ch.pulse = 1;
			TIM3->CNT = 0;
			break;

			default:
			ch.pulse++;
			break;

			}
		}

		Flight_Mode_Control();
	}
}


void Flight_Mode_Control(void)
{
	if (ch.swB > 950 && ch.swB < 1050)
	{
		FLIGHT_MODE = MANUEL;
	}
	else
	{
		FLIGHT_MODE = ModeSelect;
	}

	if(FLIGHT_MODE == ModeSelect)
	{
		if (ch.swC > 950 && ch.swC < 1050)
		{
			FLIGHT_MODE = STABILIZE;
		}
		else if (ch.swC > 1450 && ch.swC < 1550)
		{
			FLIGHT_MODE = AUTO;
		}
		else
		{
			FLIGHT_MODE = AUTOTUNE;
		}
	}
}








