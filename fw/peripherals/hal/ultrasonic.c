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

void ultrasonic_ccr_ISR();
Timer_Handle timer;

const Timer_A_ContinuousModeConfig continuousModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,           // ACLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,      // ACLK/1 = 32.768khz
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Enable Overflow ISR
        TIMER_A_DO_CLEAR                    // Clear Counter
};

const Timer_A_CaptureModeConfig captureModeConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_2,        // CC Register 2
		TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE,
		TIMER_A_CAPTURE_INPUTSELECT_CCIxA,        // CCIxB Input Select
        TIMER_A_CAPTURE_SYNCHRONOUS,              // Synchronized Capture
        TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE,  // Enable interrupt
        TIMER_A_OUTPUTMODE_OUTBITVALUE            // Output bit value
};



void ultrasonic_init()
{
	 //GPIO_enableInt(Board_ULTRASONIC_IN0);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5 ,
	             GPIO_PIN7,  GPIO_PRIMARY_MODULE_FUNCTION);

/*
	 Timer_Params prms;
	 xdc_runtime_Types_FreqHz  freq;

	 Timer_Params_init(&prms);
	 prms.period = 100000; //prms.periodType is PeriodType_MICROSECS


	 timer = Timer_create(2, &(ultrasonic_ccr_ISR), &prms, NULL); //create timer A2
	 Timer_getFreq(timer, &freq);


	 TA2CCTL2 |= CM__BOTH + CCIE + CAP;// CCIS__CCIA;
*/

	 /* Configuring Continuous Mode  and capture mode*/
	    MAP_Timer_A_configureContinuousMode(TIMER_A2_MODULE, &continuousModeConfig);
	    Timer_A_initCapture(TIMER_A2_MODULE, &captureModeConfig);

	    /* Enabling interrupts and going to sleep */
	 //   MAP_Interrupt_enableSleepOnIsrExit();
	    MAP_Interrupt_enableInterrupt(INT_TA2_N);
	    Timer_A_enableCaptureCompareInterrupt( TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_2 );

	    /* Enabling MASTER interrupts */
	     MAP_Interrupt_enableMaster();

	    /* Starting the Timer_A0 in continuous mode */
	    MAP_Timer_A_startCounter(TIMER_A2_MODULE, TIMER_A_CONTINUOUS_MODE);

}


void ultrasonic_send_pulse(uint8_t index)
{
	uint32_t start_time;
	uint32_t stop_time;

	start_time = Timestamp_get32();
	GPIO_write(Board_ULTRASONIC_OUT0, 1);
	stop_time = Timestamp_get32();
	running = 0;
	while((stop_time-start_time)<10*CYCLES_PER_US)
	{
		stop_time = Timestamp_get32();
	}

	GPIO_write(Board_ULTRASONIC_OUT0, 0);
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

void ultrasonic_ccr_ISR(){

	if(running)
		{//bit was high -> falling edge
			//pulse_falling_time[index]=Timer_getCount(timer);
			pulse_falling_time[index]=Timer_A_getCaptureCompareCount ( TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_2 );
			running = 0;
			diff=(int16_t)pulse_falling_time[index] - (int16_t)pulse_rising_time[index];
		}
		else
		{
			//pulse_rising_time[index]=Timer_getCount(timer);
			pulse_rising_time[index]=Timer_A_getCaptureCompareCount ( TIMER_A2_MODULE, TIMER_A_CAPTURECOMPARE_REGISTER_2 );
			running=1;
		}
	MAP_Timer_A_clearInterruptFlag(TIMER_A2_MODULE);
	Timer_A_clearCaptureCompareInterrupt(TIMER_A2_MODULE,TIMER_A_CAPTURECOMPARE_REGISTER_2);
}
