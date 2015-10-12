/*
 * ultrasonic.c
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */

#include "ultrasonic.h"
#include "../../../Board.h"
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <msp432.h> //to access the registers
//#include <driverlib/timer_a.h>

/* DriverLib Includes */
#include "driverlib.h"


uint16_t pulse_rising_time[8]; //record timestamp when pulse was sent out
uint16_t pulse_falling_time[8]; //record timestamp when the interrupt was triggered by the echoed pulse


uint8_t running=0;
uint8_t index=0; // the index of the ultrasonic sensor who sends out the pulse.

void ultrasonic_ccr_ISR(uint8_t index, uint16_t timestamp, uint8_t edgetype);
Timer_Handle timer;

const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           		// ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      		// ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_DISABLE,     		// Enable Overflow ISR ???
        TIMER_A_DO_CLEAR                    		// Clear Counter
};

Timer_A_CaptureModeConfig captureModeConfig =
{
		Board_ULTRASONIC_IN_0_CCR,        		// CC Register for the first input pin to be initialized.
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,
		TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};

void ultrasonic_init()
{
	/* configure input pins as capture/compare functions: */
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_0_PORT,
			Board_ULTRASONIC_IN_0_PIN,  GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_1_PORT,
			Board_ULTRASONIC_IN_1_PIN,  GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_2_PORT,
			Board_ULTRASONIC_IN_2_PIN,  GPIO_PRIMARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_3_PORT,
			Board_ULTRASONIC_IN_3_PIN,  GPIO_PRIMARY_MODULE_FUNCTION);

	/* configure the timer as continuous Mode and capture mode for each input pin separately */
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_0_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_0_TAx_MODULE, &captureModeConfig);

	captureModeConfig.captureRegister = Board_ULTRASONIC_IN_1_CCR;
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_1_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_1_TAx_MODULE, &captureModeConfig);

	captureModeConfig.captureRegister = Board_ULTRASONIC_IN_2_CCR;
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_2_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_2_TAx_MODULE, &captureModeConfig);

	captureModeConfig.captureRegister = Board_ULTRASONIC_IN_3_CCR;
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_3_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_3_TAx_MODULE, &captureModeConfig);

	/* Enabling global timer interrupts and starting timer */
#if(Board_ULTRASONIC_IN_0_TAx_MODULE==TIMER_A2_MODULE && Board_ULTRASONIC_IN_1_TAx_MODULE==TIMER_A2_MODULE)
	MAP_Interrupt_enableInterrupt(INT_TA2_N);
	/* Starting the Timer_A0 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A2_MODULE, TIMER_A_CONTINUOUS_MODE);
#else
	#error("Timer module changed. Check this code section and adapt the interrupt enables")
#endif

#if(Board_ULTRASONIC_IN_2_TAx_MODULE==TIMER_A3_MODULE && Board_ULTRASONIC_IN_3_TAx_MODULE==TIMER_A3_MODULE)
	MAP_Interrupt_enableInterrupt(INT_TA3_N);
	/* Starting the Timer_A0 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A3_MODULE, TIMER_A_CONTINUOUS_MODE);
#else
	#error("Timer module changed. Check this code section and adapt the interrupt enables")
#endif

	/* Enable each CCR interrupt */
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_0_TAx_MODULE, Board_ULTRASONIC_IN_0_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_1_TAx_MODULE, Board_ULTRASONIC_IN_1_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_2_TAx_MODULE, Board_ULTRASONIC_IN_2_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_3_TAx_MODULE, Board_ULTRASONIC_IN_3_CCR );

	/* Enabling MASTER interrupts */
	MAP_Interrupt_enableMaster();

	//For reference: direct register operation instructions:
	/*
	Timer_Params prms;
	xdc_runtime_Types_FreqHz  freq;

	Timer_Params_init(&prms);
	prms.period = 100000; //prms.periodType is PeriodType_MICROSECS


	timer = Timer_create(2, &(ultrasonic_ccr_ISR), &prms, NULL); //create timer A2
	Timer_getFreq(timer, &freq);


	TA2CCTL2 |= CM__BOTH + CCIE + CAP;// CCIS__CCIA;
	*/

}


void ultrasonic_send_pulse(uint8_t index)
{
	uint32_t start_time;
	uint32_t stop_time;

	start_time = Timestamp_get32();
	GPIO_write(Board_ULTRASONIC_OUT_0, 1);
	stop_time = Timestamp_get32();
	running = 0;
	while((stop_time-start_time)<10*CYCLES_PER_US)
	{
		stop_time = Timestamp_get32();
	}

	GPIO_write(Board_ULTRASONIC_OUT_0, 0);
	index=0;
}

/* TODO: this code is just here for reference. remove later
UInt32 time;

void ultrasonic_ISR(uint8_t index){
	if(running)
	{
		//pulse_falling_time[index]=Timer_getCount(timer);
		pulse_falling_time[index]=TA2CCR2;
		TA2CCTL2 &= ~CCIFG;
		running=0;
	}
	else
	{
		//pulse_rising_time[index]=Timer_getCount(timer);
		pulse_rising_time[index]=TA2CCR2;
		TA2CCTL2 &= ~CCIFG;
		running=1;
	}
}*/

int16_t diff=0;

/*
 * ultrasonic_ccr_ISR() gets called in the TIMER_Ax ISR if it was triggered by an ultrasonic pin
 *
 * Arguments:
 *
 * index : ultrasonic input pin that triggered the CCR (0-3)
 * timestamp : ...
 * edgetype : 1=falling, 0=rising
 */
void ultrasonic_ccr_ISR(uint8_t pin_index, uint16_t timestamp, uint8_t edgetype)
{
	if(edgetype)
	{//bit was high -> falling edge
		pulse_falling_time[pin_index]=timestamp;
		diff=(int16_t)pulse_falling_time[pin_index] - (int16_t)pulse_rising_time[pin_index]; //TODO: remove. this is only for debug.
	}
	else
	{
		pulse_rising_time[pin_index]=timestamp;
	}

}
