/*
 * Task.c
 *
 *  Created on: 20 Mar 2020
 *      Author: Yusuf TUNC
 */

#include "common.h"

void HAL_SYSTICK_Callback(void)
{
	tick.ms++; // 1ms

	if(!(tick.ms % TASK_05HZ))
	{
		if(tick.task_05hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_05hz = 1;
	}

	if(!(tick.ms % TASK_1HZ))
	{
		if(tick.task_1hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_1hz = 1;
	}

	if(!(tick.ms % TASK_5HZ))
	{
		if(tick.task_5hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_5hz = 1;
	}

	if(!(tick.ms % TASK_10HZ))
	{
		if(tick.task_10hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_10hz = 1;
	}

	if(!(tick.ms % TASK_20HZ))
	{
		if(tick.task_20hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_20hz = 1;
	}

	if(!(tick.ms % TASK_50HZ))
	{
		if(tick.task_50hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_50hz = 1;
	}

	if(!(tick.ms % TASK_100HZ))
	{
		if(tick.task_100hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_100hz = 1;
	}

	if(!(tick.ms % TASK_200HZ))
	{
		if(tick.task_200hz)
		{
			tick.kacan_cevrim++;
		}
		tick.task_200hz = 1;
	}

}


