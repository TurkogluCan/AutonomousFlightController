/*
 * LowpassFilter.h
 *
 *  Created on: 6 Þub 2020
 *      Author: Huseyin Koc
 */

#ifndef LOWPASSFILTER_H_
#define LOWPASSFILTER_H_


typedef struct
{
	float input;
	float output_last;
	float output;
	float cf;       		// lower value, lower damping
	float delta_t;        // delta time
}baro_filter_t;

typedef struct
{
	float input[3];
	float output_last[3];
	float output[3];
	float cf;       		// lower value, lower damping
	float delta_t;       	// delta time
}acc_filter_t;

typedef struct
{
	float input[3];
	float output_last[3];
	float output[3];
	float cf;       		// lower value, lower damping
	float delta_t;       	// delta time
}gyro_filter_t;

typedef struct
{
	float input;
	float output_last;
	float output;
	float cf;       		// lower value, lower damping
	float delta_t;        // delta time
}adc_filter_t;

typedef struct
{
	float input;
	float output_last;
	float output;
	float cf;       		// lower value, lower damping
	float delta_t;        // delta time
}heading_filter_t;


baro_filter_t 	baro_filter;
acc_filter_t 	acc_filter;
gyro_filter_t 	gyro_filter;
adc_filter_t 	voltaj_filter;
adc_filter_t 	akim_filter;
heading_filter_t heading_filter;

void filter_baro(float unfiltered_value);
void filter_acc(float* unfiltered_value);
void filter_gyro(float* unfiltered_value);
void filter_adc_voltaj(float* unfiltered_value);
void filter_adc_akim(float* unfiltered_value);
void filter_heading(float* unfiltered_value);
#endif /* LOWPASSFILTER_H_ */
