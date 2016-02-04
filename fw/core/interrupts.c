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
#include "../peripherals/hal/windsensor.h"
#include "../peripherals/hal/AS3935.h"
#include "../peripherals/geiger.h"



void port1_isr()
{
	switch( P1IV ) {
	        case P1IV__NONE:   break;                               // None
	        case P1IV__P1IFG0:                                      // Pin 0
	             geiger_count();
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
	        case Board_LIGHTNING_INT_IV:                             // Pin 4 (lightning detector)
	        		as3935_ISR();
	        		GPIO_clearInt(Board_LIGHTNING_INT);
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
//	        case P5IV__P5IFG0:                                      // Pin 0 (ultrasonic input 0)
//	        	__no_operation();
//	            break;
//	        case P5IV__P5IFG1:                                       // Pin 1
//	        	__no_operation();
//	             break;
//	        case P5IV__P5IFG2:                                       // Pin 2
//	        	__no_operation();
//	             break;
//	        case P5IV__P5IFG3:                                       // Pin 3
//	        	__no_operation();
//	             break;
//	        case P5IV__P5IFG4:                                       // Pin 4
//	        	__no_operation();
//	             break;
//	        case P5IV__P5IFG5:                                       // Pin 5
//	             __no_operation();
//	             break;
//	        case P5IV__P5IFG6:                                       // Pin 6
//	             __no_operation();
//	             break;
//	        case P5IV__P5IFG7:                                       // Pin 7
////	             ultrasonic_ISR(0);
//	             GPIO_clearInt(Board_ULTRASONIC_IN0);
//	        	__no_operation();
//	             break;
	        default:   break;
	    }


	System_printf("port7\n");
	System_flush();
}


uint16_t ccr_timestamp = 0;
uint8_t  ccr_edgetype  = 0;


void timer_a0_isr()
{
	switch( TA0IV ) {
// check if board definitions are still the same
#if(Board_WINDSENSOR_IN_TAx_MODULE==TIMER_A0_MODULE)
	case Board_WINDSENSOR_IN_IV:
			ccr_timestamp = Timer_A_getCaptureCompareCount(TIMER_A0_MODULE, Board_WINDSENSOR_IN_CCR );
			ccr_edgetype  = Board_WINDSENSOR_IN_CCTL & CCI; //1=rising, 0=falling
			windsensor_ccr_ISR(ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE,Board_WINDSENSOR_IN_CCR);
			break;
#if(Board_ULTRASONIC_IN_4_TAx_MODULE==TIMER_A0_MODULE && Board_ULTRASONIC_IN_6_TAx_MODULE==TIMER_A0_MODULE && Board_ULTRASONIC_IN_7_TAx_MODULE==TIMER_A0_MODULE)
	case Board_ULTRASONIC_IN_4_IV:                                      // ultrasonic input capture 4
			/* record timestamp and logic state of input pin to get edgetype */
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A0_MODULE, Board_ULTRASONIC_IN_4_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_4_CCTL & CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(4,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE,Board_ULTRASONIC_IN_4_CCR);
			break;
	case Board_ULTRASONIC_IN_6_IV:                                      // ultrasonic input capture 6
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A0_MODULE, Board_ULTRASONIC_IN_6_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_6_CCTL & CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(6,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE,Board_ULTRASONIC_IN_6_CCR);
			break;
	case Board_ULTRASONIC_IN_7_IV:                                      // ultrasonic input capture 7
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A0_MODULE, Board_ULTRASONIC_IN_7_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_7_CCTL & CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(7,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A0_MODULE,Board_ULTRASONIC_IN_7_CCR);
			break;
#endif
	default:   break;
	}
}


void timer_a2_isr()
{
	/* TODO: add detection of capture before previous event was read.
	 * From the MSP430 users guide, p363:
	 *
	 * Overflow logic is provided in each capture/compare register to indicate if a second capture was
	 * performed before the value from the first capture was read. Bit COV is set when this occurs as shown in
	 * Figure 14-11. COV must be reset with software.
	 */


	switch( TA2IV ) {
// check if board definitions are still the same
#if(Board_ULTRASONIC_IN_0_TAx_MODULE==TIMER_A2_MODULE && Board_ULTRASONIC_IN_1_TAx_MODULE==TIMER_A2_MODULE && Board_ULTRASONIC_IN_5_TAx_MODULE==TIMER_A2_MODULE)
	case Board_ULTRASONIC_IN_0_IV:                                      // ultrasonic input capture 0
			/* record timestamp and logic state of input pin to get edgetype */
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_MODULE, Board_ULTRASONIC_IN_0_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_0_CCTL & CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(0,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE,Board_ULTRASONIC_IN_0_CCR);
			break;
	case Board_ULTRASONIC_IN_1_IV:                                      // ultrasonic input capture 1
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_MODULE, Board_ULTRASONIC_IN_1_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_1_CCTL & CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(1,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE,Board_ULTRASONIC_IN_1_CCR);
			break;
	case Board_ULTRASONIC_IN_5_IV:                                      // ultrasonic input capture 5
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_MODULE, Board_ULTRASONIC_IN_5_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_5_CCTL & CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(5,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE,Board_ULTRASONIC_IN_5_CCR);
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
		case Board_ULTRASONIC_IN_2_IV:                                      // ultrasonic input capture 2
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A3_MODULE, Board_ULTRASONIC_IN_2_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_2_CCTL & CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(2,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A3_MODULE,Board_ULTRASONIC_IN_2_CCR);
			break;
		case Board_ULTRASONIC_IN_3_IV:                                      // ultrasonic input capture 3
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A3_MODULE, Board_ULTRASONIC_IN_3_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_3_CCTL & CCI; //1=rising, 0=falling
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

