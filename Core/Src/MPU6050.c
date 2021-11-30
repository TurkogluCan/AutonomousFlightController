/*
 * MPU6050.c
 *
 *  Created on: 19 Eki 2019
 *      Author: Huseyin Koc
 */

#include "common.h"

/**
  * @brief MPU6050 sensorunun baslangic ayarlarini yapar.
  * @param  (void)
  * @retval (void)
  */
void mpu6050_init(void)
{
	uint8_t giden_veri_u8;

	giden_veri_u8 = 0x80;
	mpu6050_write(PWR_MGMT,giden_veri_u8,1);

	HAL_Delay(100);

	if(mpu6050_isReady())
	{
		MPU6050_LED_ENABLE

		giden_veri_u8 = 0x80;
		mpu6050_write(PWR_MGMT,giden_veri_u8,1); // Reset MPU6050
		HAL_Delay(100);

		giden_veri_u8 = 0x00;
		mpu6050_write(PWR_MGMT,giden_veri_u8,1); // Wake-up MPU6050
		HAL_Delay(100);

		giden_veri_u8 = 0x09;
		mpu6050_write(CONFIG,giden_veri_u8,1); // 10Hz bandwidth Accelometer - 1kHz Fs, 10Hz bandwidth Gyroscope - 1kHz Fs

		giden_veri_u8 = 0x02;
		mpu6050_write(SMPLRT_DIV,giden_veri_u8,1); // Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV) --> 200Hz data output rate.

		giden_veri_u8 = 0x10;
		mpu6050_write(GYRO_CONFIG,giden_veri_u8,1); // Gyro FS : 1000 degree/s

		giden_veri_u8 = 0x00;
		mpu6050_write(ACCEL_CONFIG,giden_veri_u8,1); // Accel FS : +/- 2g

		giden_veri_u8 = 0x01;
		mpu6050_write(INT_ENABLE,giden_veri_u8,1);	// Data ready interrupt enable.

		giden_veri_u8 = 0x00;
		mpu6050_write(USER_CTRL,giden_veri_u8,1);

		giden_veri_u8 = 0x02;
		mpu6050_write(INT_PIN_CFG,giden_veri_u8,1); // int_bypass_mode is enabled. Magnetometer sensor is enabled.

	}
	else
	{
		MPU6050_LED_DISABLE
	}

}

/**
  * @brief MPU6050 sensoru veri yazma fonksiyonu.
  * @param  MPU6050 register adresi
  * @param  Registera yazilacak veri
  * @param  Veri uzunlugu
  * @retval (void)
  */
void mpu6050_write(uint8_t addr, uint8_t txdata, uint8_t length)
{
	uint8_t reg = addr;

	HAL_I2C_Mem_Write(&hi2c2,MPU6050_ADDR,reg,I2C_MEMADD_SIZE_8BIT,&txdata,1,100);

}

/**
  * @brief MPU6050 sensoru veri okuma fonksiyonu.
  * @param  MPU6050 register adresi
  * @param  Veri uzunlugu
  * @retval (void)
  */
uint8_t* mpu6050_read(uint8_t addr, uint8_t length)
{
	uint8_t reg = addr;
	static uint8_t rxdata[14];
	HAL_I2C_Mem_Read(&hi2c2,MPU6050_ADDR,reg,I2C_MEMADD_SIZE_8BIT,rxdata,length,50);
	return rxdata;
}


/**
  * @brief MPU6050 ivme ve jiroskop verilerini okur.
  * @param  (void)
  * @retval (void)
  */
void mpu6050_readAccelGyro()
{
	uint8_t* gelen_veri_u8;

	gelen_veri_u8 = mpu6050_read(INT_STATUS,1);

	if(gelen_veri_u8[0] & 0x01) // if new data ready
	{
		gelen_veri_u8 = mpu6050_read(ACCEL_X_OUT_H,14);

		if(comm.calib_mode.accel)
		{
			mpu6050.raw.accel[0] = (int16_t)(gelen_veri_u8[0] << 8 | gelen_veri_u8[1]);
			mpu6050.raw.accel[1] = (int16_t)(gelen_veri_u8[2] << 8 | gelen_veri_u8[3]);
			mpu6050.raw.accel[2] = (int16_t)(gelen_veri_u8[4] << 8 | gelen_veri_u8[5]);

			mpu6050.raw.gyro[0] = (int16_t)(gelen_veri_u8[8]  << 8 | gelen_veri_u8[9]);
			mpu6050.raw.gyro[1] = (int16_t)(gelen_veri_u8[10] << 8 | gelen_veri_u8[11]);
			mpu6050.raw.gyro[2] = (int16_t)(gelen_veri_u8[12] << 8 | gelen_veri_u8[13]);
		}
		else
		{
			mpu6050.raw.accel[0] = (int16_t)(gelen_veri_u8[0] << 8 | gelen_veri_u8[1]) - mpu6050.offset.ax;
			mpu6050.raw.accel[1] = (int16_t)(gelen_veri_u8[2] << 8 | gelen_veri_u8[3]) - mpu6050.offset.ay;
			mpu6050.raw.accel[2] = (int16_t)(gelen_veri_u8[4] << 8 | gelen_veri_u8[5]) - mpu6050.offset.az;

			mpu6050.raw.gyro[0] = (int16_t)(gelen_veri_u8[8]  << 8 | gelen_veri_u8[9])   - mpu6050.offset.gx;
			mpu6050.raw.gyro[1] = (int16_t)(gelen_veri_u8[10] << 8 | gelen_veri_u8[11])  - mpu6050.offset.gy;
			mpu6050.raw.gyro[2] = (int16_t)(gelen_veri_u8[12] << 8 | gelen_veri_u8[13])  - mpu6050.offset.gz;
		}

	}

	  mpu6050.raw.accel_f32[0] = (float)mpu6050.raw.accel[0] / ACCEL_SCALE;
	  mpu6050.raw.accel_f32[1] = (float)mpu6050.raw.accel[1] / ACCEL_SCALE;
	  mpu6050.raw.accel_f32[2] = (float)mpu6050.raw.accel[2] / ACCEL_SCALE;

	  mpu6050.raw.gyro_f32[0] = (float)mpu6050.raw.gyro[0] / GYRO_SCALE;
	  mpu6050.raw.gyro_f32[1] = (float)mpu6050.raw.gyro[1] / GYRO_SCALE;
	  mpu6050.raw.gyro_f32[2] = (float)mpu6050.raw.gyro[2] / GYRO_SCALE;

}

/**
  * @brief MPU6050 jiroskop kalibrasyonu yapar.
  * @param  Ornekleme miktari
  * @retval	(void)
  */
void mpu6050_gyroCalibration(int16_t sampling)
{
	int32_t gyro_temp[3] = {0};

	for(int16_t i = 0; i < sampling; i++)
	{
		mpu6050_readAccelGyro();
		HAL_Delay(6);
		gyro_temp[0] += mpu6050.raw.gyro[0];
		gyro_temp[1] += mpu6050.raw.gyro[1];
		gyro_temp[2] += mpu6050.raw.gyro[2];
	}


	mpu6050.offset.gx = gyro_temp[0] / sampling;
	mpu6050.offset.gy = gyro_temp[1] / sampling;
	mpu6050.offset.gz = gyro_temp[2] / sampling;
}

/**
  * @brief MPU6050 ivme kalibrasyonu yapar.
  * @param  Ornekleme miktari
  * @retval	(void)
  */
void mpu6050_accelCalibration(int16_t sampling)
{
	int32_t ivme_temp[3] = {0};

	for(int16_t i = 0; i < sampling; i++)
	{
		mpu6050_readAccelGyro();
		HAL_Delay(5);

		ivme_temp[0] += mpu6050.raw.accel[0];
		ivme_temp[1] += mpu6050.raw.accel[1];
		ivme_temp[2] += mpu6050.raw.accel[2];
	}

	mpu6050.offset.ax = ivme_temp[0]  / sampling;
	mpu6050.offset.ay = ivme_temp[1]  / sampling;
	mpu6050.offset.az = (ivme_temp[2] / sampling) - ACCEL_SCALE;
}

/**
  * @brief MPU6050 ile iletisim olup olmadigina bakar.
  * @param  (void)
  * @retval	True or False
  */
bool mpu6050_isReady()
{
	uint8_t gelen_veri_u8 = 0;

	HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, MPU6050_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &gelen_veri_u8, 1, 50);

	if(gelen_veri_u8 == MPU6050_ID)
	{
		return true;
	}

	return false;
}
