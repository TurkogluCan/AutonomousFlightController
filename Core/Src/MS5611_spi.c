
#include "common.h"

//TODO : set a proper timing


static uint16_t prom[6];


//min OSR by default
static uint8_t pressAddr = PRESSURE_OSR_4096;
static uint8_t tempAddr = TEMP_OSR_4096;

/**
 * @brief init the pressure sensor with default parameters
 */
static void ms5611_init();

/**
 * @brief write the command passed in parameters to the SPI Bus
 *
 * @param data the command to write
 */
static void ms5611_write(uint8_t data);

/**
 * @brief read the n bits of the SPI Bus on  register reg
 *
 * @return the value read on the SPI bus
 */
static uint16_t ms5611_read16bits(uint8_t reg);
static uint32_t ms5611_read24bits(uint8_t reg);

/**
 * @brief read the raw value
 *
 * @return the value read on the SPI bus
 */

static void ms5611_init()
{
	MS5611_DIS
	HAL_Delay(10);

	ms5611_write(CMD_RESET);
	HAL_Delay(10);

	prom[0] = ms5611_read16bits(CMD_PROM_C1);
	prom[1] = ms5611_read16bits(CMD_PROM_C2);
	prom[2] = ms5611_read16bits(CMD_PROM_C3);
	prom[3] = ms5611_read16bits(CMD_PROM_C4);
	prom[4] = ms5611_read16bits(CMD_PROM_C5);
	prom[5] = ms5611_read16bits(CMD_PROM_C6);
}

static void ms5611_write(uint8_t data)
{
	MS5611_EN
	HAL_SPI_Transmit(&hspi1, &data, 1, 50);
	MS5611_DIS
}

static uint16_t ms5611_read16bits(uint8_t reg)
{
	uint8_t byte[3];
	uint16_t return_value;
	MS5611_EN
	HAL_SPI_TransmitReceive(&hspi1, &reg, byte, 3, 50);
	MS5611_DIS
	/**
	 * We dont care about byte[0] because that is what was recorded while
	 * we were sending the first byte of the cmd. Since the baro wasn't sending
	 * actual data at that time (it was listening for command), data[0] will
	 * contain garbage data (probably all 0's).
	 */
	return_value = ((uint16_t)byte[1]<<8) | (byte[2]);
	return return_value;
}

static uint32_t ms5611_read24bits(uint8_t reg)
{
	uint8_t byte[4];
	uint32_t return_value;
	MS5611_EN
	HAL_SPI_TransmitReceive(&hspi1, &reg, byte, 4, 50);
	MS5611_DIS
	return_value = ((uint32_t)byte[1]<<16) | ((uint32_t)(byte[2]<<8)) | (byte[3]);
	return return_value;
}


void Barometer_init()
{
	ms5611_init();
	Barometer_setOSR(OSR_4096);

	for(uint8_t i = 0; i < 20; i++)
	{
		ms5611_getRawPressure();
		HAL_Delay(9);
		ms5611.D1 = ms5611_readRawPressure();

		ms5611_getRawTemp();
		HAL_Delay(9);
		ms5611.D2 = ms5611_readRawTemp();

	}

	ms5611.referans_basinc_oku = false;
	Barometer_calculate();

	ms5611_getRawPressure();
	HAL_Delay(10);
}

void Barometer_setOSR(OSR osr)
{
	switch(osr)
	{
		default:
		case OSR_256:
			pressAddr = PRESSURE_OSR_256;
			tempAddr = TEMP_OSR_256;
			break;
		case OSR_512:
			pressAddr = PRESSURE_OSR_512;
			tempAddr = TEMP_OSR_512;
			break;
		case OSR_1024:
			pressAddr = PRESSURE_OSR_1024;
			tempAddr = TEMP_OSR_1024;
			break;
		case OSR_2048:
			pressAddr = PRESSURE_OSR_2048;
			tempAddr = TEMP_OSR_2048;
			break;
		case OSR_4096:
			pressAddr = PRESSURE_OSR_4096;
			tempAddr = TEMP_OSR_4096;
			break;
	}
}



void Barometer_calculate()
{
	int32_t dT;
	int64_t TEMP, OFF, SENS, P;
	float press, r1, c1, r2;
	int64_t T2,OFF2,SENS2;

	dT = ms5611.D2-(((long)prom[4] << 8));
	TEMP = 2000 + (((int64_t)dT * prom[5]) >> 23);
	OFF = ((int64_t)prom[1] << 16) + (((int64_t)prom[3] * dT ) >> 7);
	SENS = ((int64_t)prom[0] << 15) + (((int64_t)prom[2] * dT) >> 8);

	if (TEMP < 2000) // second order temperature compensation
	{
		T2 = (((int64_t)dT)*dT) >> 31;
		int64_t Aux_64 = (TEMP-2000)*(TEMP-2000);
		OFF2 = (5*Aux_64) >> 1;
		SENS2 = (5*Aux_64) >> 2;

		if(TEMP < -1500)
		{
			OFF2 = OFF2 + 7 * sqr((TEMP + 1500));
			SENS2 = SENS2 + ( 11 * sqr((TEMP + 1500)) >> 1);
		}
	}
	else
	{
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	P = (( (ms5611.D1 * SENS >> 21) - OFF) >> 15);
	ms5611.temperature = TEMP;
	ms5611.pressure = P;

	if(!ms5611.referans_basinc_oku)
	{
		ms5611.ref_pressure = ms5611.pressure;
		ms5611.referans_basinc_oku = true;
	}

	press = (float)ms5611.pressure;
	r1= press/101325.0f;
	r2 = press / ms5611.ref_pressure;
	c1 = 1.0/5.255f;

	ms5611.relative_altitude = (1 - pow(r2,c1)) * 44330.77;
	ms5611.altitude = (1 - pow(r1,c1)) * 44330.77;

	filter_baro(ms5611.relative_altitude);
}

uint32_t ms5611_readRawTemp()
{
	uint32_t D2;
	D2 = ms5611_read24bits(0x00);
	return D2;
}

void ms5611_getRawTemp()
{
	ms5611_write(tempAddr);
}

uint32_t ms5611_readRawPressure()
{
	uint32_t D1;
	//Convert pressure
	D1 = ms5611_read24bits(0x00);
	return D1;
}

void ms5611_getRawPressure()
{
	ms5611_write(pressAddr);
}
