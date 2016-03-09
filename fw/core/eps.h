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
#define M11V_OFF		0xB0
#define M5V_OFF		0x50

#define SC_ON		0xC1
#define SC_OFF		0xC0

#define OFF	0;
#define ON  	1;

void eps_init();
uint8_t eps_switch_module(uint8_t command);


#endif /* FW_CORE_EPS_H_ */
