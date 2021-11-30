/*
 * LowpassFilter.c
 *
 *  Created on: 6 Þub 2020
 *      Author: Huseyin Koc
 */

#include "common.h"

void filter_baro(float unfiltered_value)
{
	baro_filter.input = unfiltered_value;

    const float alpha = 0.990;

    baro_filter.output = baro_filter.output_last + (1-alpha)*(baro_filter.input - baro_filter.output_last);
    ms5611.relative_altitude = baro_filter.output;
    baro_filter.output_last = baro_filter.output;
}

void filter_acc(float* unfiltered_value)
{
	acc_filter.input[0] = unfiltered_value[0];
	acc_filter.input[1] = unfiltered_value[1];
	acc_filter.input[2] = unfiltered_value[2];

    const float alpha = 0.925;

    for(uint8_t i=0; i<3; i++)
    {
        acc_filter.output[i] = acc_filter.output_last[i] + (1-alpha)*(acc_filter.input[i] - acc_filter.output_last[i]);
        mpu6050.raw.accel_f32[i] = acc_filter.output[i];;
        acc_filter.output_last[i] = acc_filter.output[i];
    }

}

void filter_gyro(float* unfiltered_value)
{
	gyro_filter.input[0] = unfiltered_value[0];
	gyro_filter.input[1] = unfiltered_value[1];
	gyro_filter.input[2] = unfiltered_value[2];

    const float alpha = 0.925;

    for(uint8_t i=0; i<3; i++)
    {
    	gyro_filter.output[i] = gyro_filter.output_last[i] + (1-alpha)*(gyro_filter.input[i] - gyro_filter.output_last[i]);
    	mpu6050.raw.gyro_f32[i] = gyro_filter.output[i];;
    	gyro_filter.output_last[i] = gyro_filter.output[i];
    }

}

void filter_adc_voltaj(float* unfiltered_value)
{
	voltaj_filter.input = *unfiltered_value;

    const float alpha = 0.965;

    voltaj_filter.output = voltaj_filter.output_last + (1-alpha)*(voltaj_filter.input - voltaj_filter.output_last);
    *unfiltered_value = voltaj_filter.output;
    voltaj_filter.output_last = voltaj_filter.output;
}

void filter_adc_akim(float* unfiltered_value)
{
	akim_filter.input = *unfiltered_value;

    const float alpha = 0.965;

    akim_filter.output = akim_filter.output_last + (1-alpha)*(akim_filter.input - akim_filter.output_last);
    *unfiltered_value = akim_filter.output;
    akim_filter.output_last = akim_filter.output;
}

void filter_heading(float* unfiltered_value)
{
	heading_filter.input = *unfiltered_value;

    const float alpha = 0.945;

    heading_filter.output = heading_filter.output_last + (1-alpha)*(heading_filter.input - heading_filter.output_last);
    *unfiltered_value = heading_filter.output;
    heading_filter.output_last = heading_filter.output;
}




