/*
 * interrupts.c
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */

#include "interrupts.h"

void port1_isr()
{
	switch( P1IV ) {
	        case P1IV__NONE:   break;                               // None
	        case P1IV__P1IFG0:                                      // Pin 0
	             __no_operation();
	             break;
	        case P1IV__P1IFG1:                                       // Pin 1 (button0)
//	        	 GPIO_clearInt(Board_BUTTON0);
	             break;
	        case P1IV__P1IFG2:                                       // Pin 2
	        	__no_operation();
	             break;
	        case P1IV__P1IFG3:                                       // Pin 3
	        	__no_operation();
	             break;
	        case P1IV__P1IFG4:                                       // Pin 4 (button1)
//	        	 GPIO_clearInt(Board_BUTTON1);
	             break;
	        case P1IV__P1IFG5:                                       // Pin 5
	             __no_operation();
	             break;
	        case P1IV__P1IFG6:                                       // Pin 6
	             __no_operation();
	             break;
	        case P1IV__P1IFG7:                                       // Pin 7
	             __no_operation();
	             break;
	        default:   break;
	    }


	System_printf("port1\n");
	System_flush();
}


void port4_isr()
{
	switch( P4IV ) {
	        case P4IV__NONE:   break;                               // None
	        case P4IV__P4IFG0:                                      // Pin 0 (ultrasonic input 0)
//	        	ultrasonic_ISR(0);
//	        	GPIO_clearInt(Board_ULTRASONIC_IN0);
	            break;
	        case P4IV__P4IFG1:                                       // Pin 1
	        	__no_operation();
	             break;
	        case P4IV__P4IFG2:                                       // Pin 2
	        	__no_operation();
	             break;
	        case P4IV__P4IFG3:                                       // Pin 3
	        	__no_operation();
	             break;
	        case P4IV__P4IFG4:                                       // Pin 4
	        	__no_operation();
	             break;
	        case P4IV__P4IFG5:                                       // Pin 5
	             __no_operation();
	             break;
	        case P4IV__P4IFG6:                                       // Pin 6
	             __no_operation();
	             break;
	        case P4IV__P4IFG7:                                       // Pin 7
	             __no_operation();
	             break;
	        default:   break;
	    }


	System_printf("port4\n");
	System_flush();
}




void port5_isr()
{
	switch( P5IV ) {
	        case P5IV__NONE:   break;                               // None
	        case P5IV__P5IFG0:                                      // Pin 0 (ultrasonic input 0)
	        	__no_operation();
	            break;
	        case P5IV__P5IFG1:                                       // Pin 1
	        	__no_operation();
	             break;
	        case P5IV__P5IFG2:                                       // Pin 2
	        	__no_operation();
	             break;
	        case P5IV__P5IFG3:                                       // Pin 3
	        	__no_operation();
	             break;
	        case P5IV__P5IFG4:                                       // Pin 4
	        	__no_operation();
	             break;
	        case P5IV__P5IFG5:                                       // Pin 5
	             __no_operation();
	             break;
	        case P5IV__P5IFG6:                                       // Pin 6
	             __no_operation();
	             break;
	        case P5IV__P5IFG7:                                       // Pin 7
//	             ultrasonic_ISR(0);
//	             GPIO_clearInt(Board_ULTRASONIC_IN0);
	        	__no_operation();
	             break;
	        default:   break;
	    }


	System_printf("port7\n");
	System_flush();
}

void timer_a2_isr()
{
	uint16_t ta2_iv = TA2IV;
	ultrasonic_ccr_ISR();

}

