/*
 * read_ppm.h
 *
 *  Created on: 4 Kas 2019
 *      Author: Yusuf TUNC
 */

#ifndef READPPM_H_
#define READPPM_H_

typedef struct {
	uint16_t t[10];
	uint16_t roll,pitch,throttle,yaw,swA,swB,swC,swD;
	uint16_t elps_time;
	uint16_t pulse;

	uint16_t roll_max,  roll_min;
	uint16_t pitch_max, pitch_min;
	uint16_t throttle_max, throttle_min;
	uint16_t yaw_max, yaw_min;
	uint16_t swA_max, swA_min;
	uint16_t swB_max, swB_min;
	uint16_t swC_max, swC_min;
	uint16_t swD_max, swD_min;
}ch_t;

ch_t ch;

enum flight_modes_t
{
	MANUEL,			// --> swB
	STABILIZE,		// --> swC
	AUTO,			// --> swC
	AUTOTUNE,		// --> swC
	ModeSelect		// --> swB
};
enum flight_modes_t FLIGHT_MODE;

void Flight_Mode_Control(void);

#endif /* READPPM_H_ */
