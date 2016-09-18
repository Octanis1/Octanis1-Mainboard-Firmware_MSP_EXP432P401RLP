/*
 *  File: geiger.c
 *  Description: track the count of the geiger counter
 *  Author: Eloi
 */


#include "../../Board.h"
#include "geiger.h"
#include "../core/eps.h"
#include <stdio.h>
#include <stdlib.h>
#include "../../fw/core/interrupts.h"

#define NOP10() __asm__("nop;nop;nop;nop;nop;nop;nop;nop;nop;nop")

static uint16_t last_minute_count = 0;
static uint16_t start_sec = 0;
static uint8_t valid_data = 0;

void geiger_turn_on_off(uint8_t on_off){
	uint16_t i = 0;

	if (on_off == GEIGER_ON){
		Task_sleep(2000); //need to give EPS some time to finish boot procedure.

		eps_switch_module(M3V3_2_ON);

		//turn on geiger counter then wait 200 microsec
		GPIO_write(Board_GEIGER_EN, Board_GEIGER_ON);
		for (i=0;i<1000;i++)
			NOP10();
		/* Install callback and enable interrupts */
		GPIO_enableInt(Board_GEIGER_COUNTER);
	}
	else {
		GPIO_disableInt(Board_GEIGER_COUNTER);
		GPIO_write(Board_GEIGER_EN, Board_GEIGER_OFF);
	}
}


void geiger_count(){

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
	else if((current_time - start_sec) <= SAMPLING_TIME){
		current_count++;
	}
	else{
		start_sec = Seconds_get();
		last_minute_count=current_count;
		valid_data = 1;
		current_count = 1;
	}
}

uint16_t get_last_minute_count(){

	//We check if last_minute_count has relevant data
	if (!valid_data)
		return TOO_SOON;

	/* If there's no "tick" during 1 minutes, we will return
	 * The count number of 2 minutes before ---> fixed */
	if ((Seconds_get() - start_sec) >= 2*SAMPLING_TIME)
		last_minute_count = 0;

	return last_minute_count;
}
