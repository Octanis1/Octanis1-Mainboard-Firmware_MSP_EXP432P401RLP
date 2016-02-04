/*
 *  File: geiger.c
 *  Description: track the count of the geiger counter
 *  Author: Eloi
 */


#include "../../Board.h"
#include "geiger.h"
#include <stdio.h>
#include <stdlib.h>
#include "../../fw/core/interrupts.h"

static uint16_t last_minute_count;

void geiger_init(){
    /* Install callback and enable interrupts */
    GPIO_setCallback(Board_GEIGER_COUNTER, port1_isr);

    GPIO_enableInt(Board_GEIGER_COUNTER);

    last_minute_count = 0;
}

void geiger_count(){

	static uint16_t start_sec = 0;
	static uint8_t first_time = 1;
	static uint16_t current_count = 0;
	uint16_t current_time = Seconds_get();

	//Taking care of the overflow. The max value of uint16 is 65535
	if (!first_time && (current_time < start_sec)){
		start_sec = 0;
		current_time += (65535 - start_sec) + 1;
	}

	if (first_time){
		first_time = 0;
		start_sec = Seconds_get();
		current_count = 1;
	}
	else if(start_sec - current_time <= 60){
		current_count++;
	}
	else{
		start_sec = Seconds_get();
		last_minute_count=current_count;
		current_count = 1;
	}
}

uint16_t get_last_minute_count(){
	return last_minute_count;
}
