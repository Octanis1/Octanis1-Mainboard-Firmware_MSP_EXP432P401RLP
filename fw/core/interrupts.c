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
#include "eps.h"
#include "driverlib.h"

/***** !!! don't forget to add the interrupt vector to app.cfg if you add a new ISR here !!! *****************/


void port1_isr()
{
	switch( P1->IV ) {
	        case P1IV__NONE:   break;                               // None
	        case DIO_PORT_IV__IFG0:                                      // Pin 0
	             geiger_count();
	             break;
	        case DIO_PORT_IV__IFG1:                                       // Pin 1 (button0)
//	        	 GPIO_clearInt(Board_BUTTON0);
	             break;
	        case DIO_PORT_IV__IFG2:                                       // Pin 2
	        	__no_operation();
	             break;
	        case DIO_PORT_IV__IFG3:                                       // Pin 3
	        	__no_operation();
	             break;
#ifndef VERSION_1
	        case Board_LIGHTNING_INT_IV:                             // Pin 4 (lightning detector)
	        		as3935_ISR();
	        		GPIO_clearInt(Board_LIGHTNING_INT);
	             break;
#endif
	        case DIO_PORT_IV__IFG5:                                       // Pin 5
	             __no_operation();
	             break;
	        case DIO_PORT_IV__IFG6:                                       // Pin 6
	             __no_operation();
	             break;
	        case DIO_PORT_IV__IFG7:                                       // Pin 7
	             __no_operation();
	             break;
	        default:   break;
	}
}

void port2_isr()
{
	switch(P2->IV) {
#ifdef VERSION_1
	case Board_LIGHTNING_INT_IV:                             // Pin 4 (lightning detector)
		as3935_ISR();
		GPIO_clearInt(Board_LIGHTNING_INT);
	break;
#endif
	        default:   break;
	}
}


void port4_isr()
{
	switch( P4->IV ) {
	        case DIO_PORT_IV__NONE:   break;                               // None
	        case DIO_PORT_IV__IFG0:                                      // Pin 0 (ultrasonic input 0)
//	        	ultrasonic_ISR(0);
//	        	GPIO_clearInt(Board_ULTRASONIC_IN0);
	            break;
	        case DIO_PORT_IV__IFG1:                                       // Pin 1
	        	__no_operation();
	             break;
	        case DIO_PORT_IV__IFG2:                                       // Pin 2
	        	__no_operation();
	             break;
	        case DIO_PORT_IV__IFG3:                                       // Pin 3
	        	__no_operation();
	             break;
	        case DIO_PORT_IV__IFG4:                                       // Pin 4
	        	__no_operation();
	             break;
	        case DIO_PORT_IV__IFG5:                                       // Pin 5
	             __no_operation();
	             break;
	        case DIO_PORT_IV__IFG6:                                       // Pin 6
	             __no_operation();
	             break;
	        case DIO_PORT_IV__IFG7:                                       // Pin 7
	             __no_operation();
	             break;
	        default:   break;
	    }
}


uint16_t ccr_timestamp = 0;
uint8_t  ccr_edgetype  = 0;


void timer_a0_isr()
{
	switch( TIMER_A0->IV ) {
// check if board definitions are still the same
//#if((Board_WINDSENSOR_IN_TAx_MODULE)==(TIMER_A0_BASE))
	case Board_WINDSENSOR_IN_IV:
			ccr_timestamp = Timer_A_getCaptureCompareCount(TIMER_A0_BASE, Board_WINDSENSOR_IN_CCR );
			ccr_edgetype  = Board_WINDSENSOR_IN_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			windsensor_ccr_ISR(ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,Board_WINDSENSOR_IN_CCR);
			break;
//#endif

			//#if(Board_ULTRASONIC_IN_4_TAx_MODULE==TIMER_A0_BASE && Board_ULTRASONIC_IN_6_TAx_MODULE==TIMER_A0_BASE && Board_ULTRASONIC_IN_7_TAx_MODULE==TIMER_A0_BASE)

	case Board_ULTRASONIC_IN_6_IV:                                      // ultrasonic input capture 6
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A0_BASE, Board_ULTRASONIC_IN_6_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_6_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_6,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,Board_ULTRASONIC_IN_6_CCR);
			break;
	case Board_ULTRASONIC_IN_7_IV:                                      // ultrasonic input capture 7
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A0_BASE, Board_ULTRASONIC_IN_7_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_7_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_7,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE,Board_ULTRASONIC_IN_7_CCR);
			break;
			//#endif
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


	switch( TIMER_A2->IV ) {
// check if board definitions are still the same
	//#if(Board_ULTRASONIC_IN_0_TAx_MODULE==TIMER_A2_BASE && Board_ULTRASONIC_IN_1_TAx_MODULE==TIMER_A2_BASE && Board_ULTRASONIC_IN_5_TAx_MODULE==TIMER_A2_BASE)
	case Board_ULTRASONIC_IN_0_IV:                                      // ultrasonic input capture 0
			/* record timestamp and logic state of input pin to get edgetype */
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_BASE, Board_ULTRASONIC_IN_0_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_0_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_0,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,Board_ULTRASONIC_IN_0_CCR);
			break;
	case Board_ULTRASONIC_IN_1_IV:                                      // ultrasonic input capture 1
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_BASE, Board_ULTRASONIC_IN_1_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_1_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_1,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,Board_ULTRASONIC_IN_1_CCR);
			break;
	case Board_ULTRASONIC_IN_4_IV:                                      // ultrasonic input capture 4
			/* record timestamp and logic state of input pin to get edgetype */
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_BASE, Board_ULTRASONIC_IN_4_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_4_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_4,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,Board_ULTRASONIC_IN_4_CCR);
			break;
	case Board_ULTRASONIC_IN_5_IV:                                      // ultrasonic input capture 5
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A2_BASE, Board_ULTRASONIC_IN_5_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_5_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_5,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A2_BASE,Board_ULTRASONIC_IN_5_CCR);
			break;
			//#else
	//#error("Timer module changed. Check this code section and move to appropriate timer module ISR")
			//#endif
		default:   break;
	}
	Timer_A_clearInterruptFlag(TIMER_A2_BASE);
}


void timer_a3_isr()
{
	switch( TIMER_A3->IV ) {
// check if board definitions are still the same
	//#if(Board_ULTRASONIC_IN_2_TAx_MODULE==TIMER_A3_BASE && Board_ULTRASONIC_IN_3_TAx_MODULE==TIMER_A3_BASE)
		case Board_ULTRASONIC_IN_2_IV:                                      // ultrasonic input capture 2
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A3_BASE, Board_ULTRASONIC_IN_2_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_2_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_2,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,Board_ULTRASONIC_IN_2_CCR);
			break;
		case Board_ULTRASONIC_IN_3_IV:                                      // ultrasonic input capture 3
			ccr_timestamp = Timer_A_getCaptureCompareCount ( TIMER_A3_BASE, Board_ULTRASONIC_IN_3_CCR );
			ccr_edgetype  = Board_ULTRASONIC_IN_3_CCTL & TIMER_A_CCTLN_CCI; //1=rising, 0=falling
			ultrasonic_ccr_ISR(ULTRASONIC_INDEX_3,ccr_timestamp,ccr_edgetype);
			Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE,Board_ULTRASONIC_IN_3_CCR);
			break;
			//#else
	//#error("Timer module changed. Check this code section and move to appropriate timer module ISR")
			//#endif

		default:   break;
	}
	Timer_A_clearInterruptFlag(TIMER_A3_BASE);
}

