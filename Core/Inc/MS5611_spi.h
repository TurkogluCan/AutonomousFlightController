
//#include <stdint.h>
#include <stdbool.h>

#define MS5611_EN HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
#define MS5611_DIS HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

#define CMD_RESET 0x1E
#define CMD_PROM_C1 0xA2
#define CMD_PROM_C2 0xA4
#define CMD_PROM_C3 0xA6
#define CMD_PROM_C4 0xA8
#define CMD_PROM_C5 0xAA
#define CMD_PROM_C6 0xAC

#define PRESSURE_OSR_256  0x40
#define PRESSURE_OSR_512  0x42
#define PRESSURE_OSR_1024 0x44
#define PRESSURE_OSR_2048 0x46
#define PRESSURE_OSR_4096 0x48

#define TEMP_OSR_256      0x50
#define TEMP_OSR_512  	  0x52
#define TEMP_OSR_1024 	  0x54
#define TEMP_OSR_2048     0x56
#define TEMP_OSR_4096     0x58

#define CONVERSION_OSR_256  1
#define CONVERSION_OSR_512  2
#define CONVERSION_OSR_1024 3
#define CONVERSION_OSR_2048 5
#define CONVERSION_OSR_4096 10

/**
 * @brief The oversampling rate
 * @warn an higher value means a longer conversion
 */
typedef enum OSR {
	OSR_256,
	OSR_512,
	OSR_1024,
	OSR_2048,
	OSR_4096
}OSR;


struct ms5611_t
{
	uint32_t D1, D2;
	int32_t temperature;
	int32_t pressure;
	int32_t ref_pressure;
	float altitude;
	float relative_altitude;

	bool referans_basinc_oku;

} ms5611;

/**
 * @brief Init the Barometer with default parameters
 */
 void Barometer_init();

/**
 * @brief Set the OSR (Oversampling rate)
 * 		  Setting another value from the enumeration will put the min OSR
 * @warn setting an higher value means taking more time to read the data
 * @param osr the oversampling rate (refers to OSR enumeration from barometer.h)
 */
 void Barometer_setOSR(OSR osr);

/**
 * @brief calculate/update the altitude/pressure/temperature
 * 		  using the barometer
 */
 void Barometer_calculate();

  uint32_t ms5611_readRawTemp();
  uint32_t ms5611_readRawPressure();
  void ms5611_getRawTemp();
  void ms5611_getRawPressure();
