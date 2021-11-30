/*
 * QMC5883L.h
 *
 *  Created on: 21 Eki 2019
 *      Author: Huseyin Koc
 */

//#include "stdint.h"

#ifndef QMC5883L_H_
#define QMC5883L_H_

#define QMC5883L_ADDR 		0x1A

#define QMC5883L_OUT_X_L	0x00
#define QMC5883L_OUT_X_H	0x01
#define QMC5883L_OUT_Y_L	0x02
#define QMC5883L_OUT_Y_H	0x03
#define QMC5883L_OUT_Z_L	0x04
#define QMC5883L_OUT_Z_H	0x05
#define QMC5883L_STATUS		0x06
#define QMC5883L_OUT_TEMP_L	0x07
#define QMC5883L_OUT_TEMP_H	0x08
#define QMC5883L_CNTRL		0x09
#define QMC5883L_CNTRL1		0x0A
#define QMC5883L_PERIOD_REG 0x0B
#define QMC5883L_WHO_AM_I	0x0D
#define QMC5883L_ID 		0xFF


struct raw_t
{
	float mag[3];

};

struct calib_t
{
	int16_t mag_offset[3];
	float 	mag_scale_factor[3];
};

typedef struct
{
	struct raw_t raw;
	struct calib_t calib;

}QMC5883L_t;

QMC5883L_t qmc5883;



void QMC5883L_init(void);
void QMC5883L_readMag(float* manyetik_ham_veri);
void QMC5883L_reset(void);
bool QMC5883L_isReady();
void QMC5883L_calibration(int16_t* mag_offset, float* mag_scale_factor);

#endif /* QMC5883L_H_ */
