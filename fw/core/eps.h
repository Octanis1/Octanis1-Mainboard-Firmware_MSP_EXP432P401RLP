/*
 * eps.h
 *
 *  Created on: 08 Mar 2016
 *      Author: raffael
 */

#ifndef FW_CORE_EPS_H_
#define FW_CORE_EPS_H_

/* Commands */
// Switch modules on / off
#define M3V3_1_ON	0x31
#define M3V3_2_ON	0xD1
#define M3V3_1_OFF	0x30
#define M3V3_2_OFF	0xD0
#define M11V_ON		0xB1
#define M5V_ON		0x51
#define M11V_OFF     0xB0
#define M5V_OFF		0x50

#define SC_ON		0xC1
#define SC_OFF		0xC0

//switch heaters on / off
#define HEAT_1_ON	0x41
#define HEAT_1_OFF  0x40
#define HEAT_2_ON	0x43
#define HEAT_2_OFF  0x42
#define HEAT_3_ON	0x45
#define HEAT_3_OFF  0x44

// Confirmations (EPS responses)
#define COMM_OK				0x1A
#define LOW_VOLTAGE			0x99
#define UNKNOWN_COMMAND		0x77
#define COMM_ERROR			0xFA

// Ask for status data
#define V_BAT		0x0B
#define V_SC	    0x0C
#define I_IN		0x11
#define I_OUT		0x10
#define AEXT1		0xA1
#define AEXT2		0xA2
#define T_BAT		0xBB

// mainboard confirming "im still alive"
#define ALIVE 		0xA0

void eps_init();
uint8_t eps_switch_module(uint8_t command);

uint16_t eps_get_vbat();
uint16_t eps_get_vsolar();
uint16_t eps_get_iin();
uint16_t eps_get_iout();

void eps_ISR();


#endif /* FW_CORE_EPS_H_ */
