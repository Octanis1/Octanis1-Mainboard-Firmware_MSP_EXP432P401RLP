/*
 * state_machine.h
 *
 *  Created on: 09.05.2016
 *      Author: beat
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include "eps_hal.h"

typedef struct _eps_status { //stores the answers to be sent to an eventual i2c request
	uint16_t v_bat;
	uint16_t t_bat;
	uint16_t v_solar;
	uint16_t current_in;
	uint16_t current_out;
	uint16_t analog_ext1;
	uint16_t analog_ext2;
	uint16_t analog_ext3;
	uint16_t analog_ext4;

	uint8_t v_bat_8;
	uint8_t t_bat_8;
	uint8_t v_solar_8;
	uint8_t current_in_8;
	uint8_t current_out_8;
	uint8_t analog_ext1_8;
	uint8_t analog_ext2_8;
	uint8_t analog_ext3_8;
	uint8_t analog_ext4_8;
} eps_status_t;

enum _module_status{
	OFF,
	TURN_OFF,
	TURN_ON,
	ON,
	FAULT
} module_status_t;

extern eps_status_t eps_status;
extern module_status_t module_status[N_MODULES]; //stores the answers to be sent to an eventual i2c request



#endif /* STATE_MACHINE_H_ */
