/*
 * time_since_boot.c
 *
 *  Created on: 19 Jul 2016
 *      Author: raffael
 */

#include "time_since_boot.h"
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>

// default values are for 48 MHz clock frequency
static Types_FreqHz 	freq1 = {0, 48000000};   /* Timestamp frequency */
static uint32_t sec_scaling_lo = 48000000;
static uint32_t msec_scaling_lo = 48000;
static uint32_t usec_scaling_lo = 48;
static float sec_scaling_hi = 89.478485333;
static float msec_scaling_hi = 89478.485333;
static float usec_scaling_hi = 89478485.333;

static Types_Timestamp64 stamp;

void time_since_boot_init()
{
	Timestamp_getFreq(&freq1);
	sec_scaling_lo = freq1.lo;
	msec_scaling_lo = freq1.lo/1000;
	usec_scaling_lo = freq1.lo/1000000;

	sec_scaling_hi = ((float)UINT32_MAX+1)/sec_scaling_lo;
	msec_scaling_hi = ((float)UINT32_MAX+1)/msec_scaling_lo;
	usec_scaling_hi = ((float)UINT32_MAX+1)/usec_scaling_lo;

}

uint32_t sec_since_boot()
{
	Timestamp_get64(&stamp);
	return (uint32_t)(stamp.hi*sec_scaling_hi + stamp.lo/sec_scaling_lo);
}

uint32_t ms_since_boot()
{
	Timestamp_get64(&stamp);
	return (uint32_t)(stamp.hi*msec_scaling_hi + stamp.lo/msec_scaling_lo);
}

uint64_t us_since_boot()
{
	Timestamp_get64(&stamp);
	return (uint64_t)((uint64_t)stamp.hi*usec_scaling_hi + stamp.lo/usec_scaling_lo);
}


