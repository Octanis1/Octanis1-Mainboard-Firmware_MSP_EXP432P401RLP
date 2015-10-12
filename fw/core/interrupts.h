/*
 * interrupt.h
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */

#ifndef FW_CORE_INTERRUPTS_H_
#define FW_CORE_INTERRUPTS_H_

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <msp432.h>




void port1_isr();
void port4_isr();

//void timer_a2_isr();
//void timer_a3_isr();

#endif /* FW_CORE_INTERRUPTS_H_ */
