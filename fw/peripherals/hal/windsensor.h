/*
 * windsensor.h
 *
 *  Created on: 01 Nov 2015
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_WINDSENSOR_H_
#define FW_PERIPHERALS_HAL_WINDSENSOR_H_

#include "../../../Board.h"

void windsensor_ccr_ISR(uint16_t timestamp, uint8_t edgetype);
void windsensor_init();

void windsensor_getvalue();



#endif /* FW_PERIPHERALS_HAL_WINDSENSOR_H_ */
