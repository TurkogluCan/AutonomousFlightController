/*
 * common.h
 *
 *  Created on: 13 Sub 2020
 *      Author: Huseyin Koc
 */

#ifndef COMMON_H_
#define COMMON_H_

#include "stm32f1xx_hal.h"
#include "math.h"
#include "main.h"

extern IWDG_HandleTypeDef hiwdg;

extern SPI_HandleTypeDef hspi1;
extern I2C_HandleTypeDef hi2c2;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

#ifndef M_PI
#define M_PI       3.14159265358979323846f
#endif /* M_PI */

#define RAD2DEG 180.0f / M_PI
#define DEG2RAD M_PI / 180.0f

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define abs(x) ((x) > 0 ? (x) : -(x))
#define sqr(x) (x*x)

#define dt 		0.005 // Sample times = 200Hz
#define wGyro 	250


/******* SENSOR HEADEAR FILES *******/
#include "mpu6050.h"
#include "QMC5883L.h"
#include "ReadPPM.h"
#include "GPS.h"
#include "IMU.h"
#include "Communication.h"
#include "LowpassFilter.h"
#include "MS5611_spi.h"
#include "PowerModule.h"
#include "Paket_Islemleri.h"
#include "pid_controller.h"
#include "AFC_main.h"
#include "TrajectoryTracking.h"
/******* SENSOR HEADEAR FILES *******/

/******* TASK DEFINES *******/
#define TASK_05HZ 	2000
#define TASK_1HZ 	1000
#define TASK_5HZ 	200
#define TASK_10HZ 	100
#define TASK_20HZ 	50
#define TASK_50HZ 	20
#define TASK_100HZ 	10
#define TASK_200HZ  5

enum state_t
{
	AFC_MAIN		= 0,
	UART_RX 		= 1,
	UART_TX	 		= 2
}state;


typedef struct
{
	volatile uint8_t task_05hz;
	volatile uint8_t task_1hz;
	volatile uint8_t task_5hz;
	volatile uint8_t task_10hz;
	volatile uint8_t task_20hz;
	volatile uint8_t task_50hz;
	volatile uint8_t task_100hz;
	volatile uint8_t task_200hz;
	volatile uint32_t ms;
	uint32_t kacan_cevrim;
}tick_t;
/******* TASK DEFINES *******/

typedef struct
{
	uint32_t now_time;
	uint32_t baro_time;
	uint8_t  baro_state;
	bool 	 baro_dead_time;
	uint16_t elapsed_time;
	uint8_t  tick_5ms;
	uint32_t uart_cevrim_suresi;
	uint8_t  pid_cevrim_suresi;
}GL_t;

tick_t tick;
GL_t GL;

#define MPU6050_LED_ENABLE	HAL_GPIO_WritePin(MPU6050_OK_GPIO_Port,MPU6050_OK_Pin,GPIO_PIN_SET);
#define MPU6050_LED_DISABLE	HAL_GPIO_WritePin(MPU6050_OK_GPIO_Port,MPU6050_OK_Pin,GPIO_PIN_RESET);

#define GPS_LED_ENABLE 		HAL_GPIO_WritePin(GPS_OK_GPIO_Port, GPS_OK_Pin, GPIO_PIN_SET);
#define GPS_LED_DISABLE 	HAL_GPIO_WritePin(GPS_OK_GPIO_Port, GPS_OK_Pin, GPIO_PIN_RESET);

#define CALIB_LED_ENABLE	HAL_GPIO_WritePin(CALIB_OK_GPIO_Port,CALIB_OK_Pin,GPIO_PIN_RESET);
#define CALIB_LED_DISABLE	HAL_GPIO_WritePin(CALIB_OK_GPIO_Port,CALIB_OK_Pin,GPIO_PIN_SET);

#endif /* COMMON_H_ */
