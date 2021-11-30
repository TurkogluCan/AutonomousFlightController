/*
 * QMC5883L.c
 *
 *  Created on: 21 Eki 2019
 *      Author: Huseyin Koc
 */

#include "common.h"

extern uint8_t kalibrasyon_modu;

void QMC5883L_init()
{
	qmc5883.calib.mag_scale_factor[0] = 0.97848;
	qmc5883.calib.mag_scale_factor[1] = 1.00189;
	qmc5883.calib.mag_scale_factor[2] = 1.02050;

	qmc5883.calib.mag_offset[0] = 525;
	qmc5883.calib.mag_offset[1] = -102;
	qmc5883.calib.mag_offset[2] = -200;


	if(QMC5883L_isReady())
	{
		QMC5883L_reset();

		uint8_t giden_veri_u8 = 0;

		giden_veri_u8 = 0x0D; // Over sample ratio : 512, FS : 2Gauss, Output data rate : 200Hz, Mode : Continuous
		HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR, QMC5883L_CNTRL, I2C_MEMADD_SIZE_8BIT, &giden_veri_u8, 1, 50);

	}
}

bool QMC5883L_isReady()
{
	uint8_t gelen_veri_u8 = 0;

	HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDR, QMC5883L_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &gelen_veri_u8, 1, 50);

	if(gelen_veri_u8 == QMC5883L_ID)
	{
		return true; 	// The sensor is QMC5883L
	}
	else
	{
		return false;	// The sensor is not QMC5883L
	}

}


void QMC5883L_reset()
{
	uint8_t giden_veri_u8 = 0;

	giden_veri_u8 = 0x80; // Software reset mode is enabled.
	HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR, QMC5883L_CNTRL1, I2C_MEMADD_SIZE_8BIT, &giden_veri_u8, 1, 50);
	HAL_Delay(100);

	giden_veri_u8 = 0x00; // Normal mode is enabled.
	HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR, QMC5883L_CNTRL1, I2C_MEMADD_SIZE_8BIT, &giden_veri_u8, 1, 50);
	HAL_Delay(100);

	giden_veri_u8 = 0x40; // Roll-over function is enabled.
	HAL_I2C_Mem_Write(&hi2c2, QMC5883L_ADDR, QMC5883L_PERIOD_REG, I2C_MEMADD_SIZE_8BIT, &giden_veri_u8, 1, 50);

}

void QMC5883L_readMag(float* manyetik_ham_veri)
{
	uint8_t gelen_veri_u8[6];

	HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDR, QMC5883L_STATUS, I2C_MEMADD_SIZE_8BIT, &gelen_veri_u8[0], 1, 50);

	if(gelen_veri_u8[0] & 0x01) // if new data ready
	{
		HAL_I2C_Mem_Read(&hi2c2, QMC5883L_ADDR, QMC5883L_OUT_X_L, I2C_MEMADD_SIZE_8BIT, gelen_veri_u8, 6, 50);

		if(comm.calib_mode.mag)
		{
			CALIB_LED_ENABLE
			manyetik_ham_veri[0] = (int16_t)(gelen_veri_u8[1] << 8 | gelen_veri_u8[0]);
			manyetik_ham_veri[1] = (int16_t)(gelen_veri_u8[3] << 8 | gelen_veri_u8[2]);
			manyetik_ham_veri[2] = (int16_t)(gelen_veri_u8[5] << 8 | gelen_veri_u8[4]);

			QMC5883L_calibration(qmc5883.calib.mag_offset,qmc5883.calib.mag_scale_factor);
		}
		else
		{
		CALIB_LED_DISABLE

		manyetik_ham_veri[0] = ((int16_t)(gelen_veri_u8[1] << 8 | gelen_veri_u8[0]) - qmc5883.calib.mag_offset[0]) *
																					  qmc5883.calib.mag_scale_factor[0];
		manyetik_ham_veri[1] = ((int16_t)(gelen_veri_u8[3] << 8 | gelen_veri_u8[2]) - qmc5883.calib.mag_offset[1]) *
																					  qmc5883.calib.mag_scale_factor[1];
		manyetik_ham_veri[2] = ((int16_t)(gelen_veri_u8[5] << 8 | gelen_veri_u8[4]) - qmc5883.calib.mag_offset[2]) *
																					  qmc5883.calib.mag_scale_factor[2];
		}
	}
}

void QMC5883L_calibration(int16_t* mag_offset, float* mag_scale_factor)
{
	static int16_t mx_min = 32767, my_min = 32767, mz_min = 32767;
	static int16_t mx_max =-32767, my_max =-32767, mz_max =-32767;

	float mag_scale_avg = 0;

	mx_min = min(qmc5883.raw.mag[0],mx_min);
	my_min = min(qmc5883.raw.mag[1],my_min);
	mz_min = min(qmc5883.raw.mag[2],mz_min);

	mx_max = max(qmc5883.raw.mag[0],mx_max);
	my_max = max(qmc5883.raw.mag[1],my_max);
	mz_max = max(qmc5883.raw.mag[2],mz_max);


	// ----- Calculate hard-iron offsets
	mag_offset[0] = (mx_max + mx_min) >> 1;
	mag_offset[1] = (my_max + my_min) >> 1;
	mag_offset[2] = (mz_max + mz_min) >> 1;

	  // ----- Calculate soft-iron scale factors
	mag_scale_factor[0] = (float)((mx_max - mx_min) >> 1);
	mag_scale_factor[1] = (float)((my_max - my_min) >> 1);
	mag_scale_factor[2] = (float)((mz_max - mz_min) >> 1);

	mag_scale_avg = ( mag_scale_factor[0] + mag_scale_factor[1] + mag_scale_factor[2] ) / 3.0f;

	mag_scale_factor[0] = mag_scale_avg / mag_scale_factor[0];
	mag_scale_factor[1] = mag_scale_avg / mag_scale_factor[1];
	mag_scale_factor[2] = mag_scale_avg / mag_scale_factor[2];

}
