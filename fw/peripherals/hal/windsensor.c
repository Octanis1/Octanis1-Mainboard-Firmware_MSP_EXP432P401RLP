/*
 * windsensor.c
 *
 *  Created on: 01 Nov 2015
 *      Author: raffael
 */




#include "windsensor.h"
#include "../../../Board.h"
#include <xdc/runtime/Timestamp.h>
//#include <xdc/runtime/Types.h>
//#include <msp432.h> //to access the registers
//#include <driverlib/timer_a.h>

/* DriverLib Includes */
#include "driverlib.h"


void windsensor_ccr_ISR(uint16_t timestamp, uint8_t edgetype);
Timer_Handle timer;

const Timer_A_ContinuousModeConfig continuousWindModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           		// ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      		// ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_DISABLE,     		// Enable Overflow ISR ???
        TIMER_A_DO_CLEAR                    		// Clear Counter
};

Timer_A_CaptureModeConfig captureWindModeConfig =
{
		Board_WINDSENSOR_IN_CCR,        		// CC Register for the input pin to be initialized.
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,
		TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

void windsensor_init()
{
	/* enable sleep mode of wind sensor  */
	GPIO_write(Board_WINDSENSOR_SLEEP, 1);

	/* configure input pins as capture/compare functions: */
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_WINDSENSOR_IN_PORT,
			Board_WINDSENSOR_IN_PIN,  GPIO_PRIMARY_MODULE_FUNCTION);

	/* configure the timer as continuous Mode and capture mode */
	Timer_A_configureContinuousMode(Board_WINDSENSOR_IN_TAx_MODULE, &continuousWindModeConfig);
	Timer_A_initCapture(Board_WINDSENSOR_IN_TAx_MODULE, &captureWindModeConfig);

	/* Enabling global timer interrupts and starting timer */
//#if(Board_WINDSENSOR_IN_TAx_MODULE==TIMER_A0_MODULE)
	Interrupt_enableInterrupt(INT_TA0_N);
	/* Starting the Timer_A0 in continuous mode */
	Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
	//#else
	//#error("Timer module changed. Check this code section and adapt the interrupt enables")
	//#endif

	/* Enable CCR interrupt */
	Timer_A_enableCaptureCompareInterrupt(Board_WINDSENSOR_IN_TAx_MODULE, Board_WINDSENSOR_IN_CCR );

	/* Enabling MASTER interrupts */
	MAP_Interrupt_enableMaster();


}


void windsensor_getvalue()
{
	//disable sleep mode
	GPIO_write(Board_WINDSENSOR_SLEEP, 0);

	Task_sleep(1000);

	//enable sleep mode
	GPIO_write(Board_WINDSENSOR_SLEEP, 1);
}


void windsensor_ccr_ISR(uint16_t timestamp, uint8_t edgetype)
{
//	GPIO_toggle(Board_LED_GREEN);

	// if(edgetype)
	// {
	// 	GPIO_write(Board_LED_GREEN, 0);

	// }
	// else
	// {
	// 	GPIO_write(Board_LED_GREEN, 1);

	// }
}

