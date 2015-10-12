/*
 * interrupts.c
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */
#include "interrupts.h"

#include "../../Board.h"
/* include all modules with interrupt service routines */
#include "../peripherals/hal/ultrasonic.h"


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


uint16_t ccr_timestamp = 0;
uint8_t  ccr_edgetype  = 0;

void timer_a2_isr()
{
	switch( TA2IV ) {
// check if board definitions are still the same
#if(Board_ULTRASONIC_IN_0_TAx_MODULE==TIMER_A2_MODULE && Board_ULTRASONIC_IN_1_TAx_MODULE==TIMER_A2_MODULE)
		case Board_ULTRASONIC_IN_0_IV:                                      // ultrasonic input capture 0
			/* record timestamp and logic state of input pin to get edgetype */
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_MODULE, Board_ULTRASONIC_IN_0_CCR );
			ccr_edgetype  = Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_MODULE,Board_ULTRASONIC_IN_0_CCR,TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT);
			ultrasonic_ccr_ISR(0,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE,Board_ULTRASONIC_IN_0_CCR);
			break;
		case Board_ULTRASONIC_IN_1_IV:                                      // ultrasonic input capture 1
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_MODULE, Board_ULTRASONIC_IN_1_CCR );
			ccr_edgetype  = Timer_A_getSynchronizedCaptureCompareInput(TIMER_A2_MODULE,Board_ULTRASONIC_IN_1_CCR,TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT);
			ultrasonic_ccr_ISR(1,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE,Board_ULTRASONIC_IN_1_CCR);
			break;
#else
	#error("Timer module changed. Check this code section and move to appropriate timer module ISR")
#endif
		default:   break;
	}
	MAP_Timer_A_clearInterruptFlag(TIMER_A2_MODULE);
}


void timer_a3_isr()
{
	switch( TA3IV ) {
// check if board definitions are still the same
#if(Board_ULTRASONIC_IN_2_TAx_MODULE==TIMER_A3_MODULE && Board_ULTRASONIC_IN_3_TAx_MODULE==TIMER_A3_MODULE)
		case Board_ULTRASONIC_IN_2_IV:                                      // ultrasonic input capture 0
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A3_MODULE, Board_ULTRASONIC_IN_2_CCR );
			ccr_edgetype  = Timer_A_getSynchronizedCaptureCompareInput(TIMER_A3_MODULE,Board_ULTRASONIC_IN_2_CCR,TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT);
			ultrasonic_ccr_ISR(2,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A3_MODULE,Board_ULTRASONIC_IN_2_CCR);
			break;
		case Board_ULTRASONIC_IN_3_IV:                                      // ultrasonic input capture 1
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A3_MODULE, Board_ULTRASONIC_IN_3_CCR );
			ccr_edgetype  = Timer_A_getSynchronizedCaptureCompareInput(TIMER_A3_MODULE,Board_ULTRASONIC_IN_3_CCR,TIMER_A_READ_SYNCHRONIZED_CAPTURECOMPAREINPUT);
			ultrasonic_ccr_ISR(3,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A3_MODULE,Board_ULTRASONIC_IN_3_CCR);
			break;
#else
	#error("Timer module changed. Check this code section and move to appropriate timer module ISR")
#endif

		default:   break;
	}
	MAP_Timer_A_clearInterruptFlag(TIMER_A3_MODULE);
}

