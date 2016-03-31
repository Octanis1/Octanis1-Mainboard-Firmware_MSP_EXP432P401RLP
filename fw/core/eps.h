/*
 * eps.h
 *
 *  Created on: 08 Mar 2016
 *      Author: raffael
 */

#ifndef FW_CORE_EPS_H_
#define FW_CORE_EPS_H_

/* Commands --> in eps.c*/


void eps_init();
uint8_t eps_switch_module(uint8_t command);

uint16_t eps_get_vbat();
uint16_t eps_get_vsolar();
uint16_t eps_get_iin();
uint16_t eps_get_iout();

void eps_ISR();


#endif /* FW_CORE_EPS_H_ */
