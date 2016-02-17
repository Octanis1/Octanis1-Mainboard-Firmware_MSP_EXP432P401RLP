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


uint16_t pulse_rising_time[N_ULTRASONIC_SENSORS]; //record timestamp when pulse was sent out
uint16_t pulse_falling_time[N_ULTRASONIC_SENSORS]; //record timestamp when the interrupt was triggered by the echoed pulse

//uint16_t

void ultrasonic_ccr_ISR(uint8_t pin_index, uint16_t timestamp, uint8_t edgetype);
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
			Board_ULTRASONIC_IN_0_PIN,  Board_ULTRASONIC_IN_0_SELECT);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_1_PORT,
			Board_ULTRASONIC_IN_1_PIN,  Board_ULTRASONIC_IN_1_SELECT);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_2_PORT,
			Board_ULTRASONIC_IN_2_PIN,  Board_ULTRASONIC_IN_2_SELECT);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_3_PORT,
			Board_ULTRASONIC_IN_3_PIN,  Board_ULTRASONIC_IN_3_SELECT);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_4_PORT,
			Board_ULTRASONIC_IN_4_PIN,  Board_ULTRASONIC_IN_4_SELECT);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_5_PORT,
			Board_ULTRASONIC_IN_5_PIN,  Board_ULTRASONIC_IN_5_SELECT);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_6_PORT,
			Board_ULTRASONIC_IN_6_PIN,  Board_ULTRASONIC_IN_6_SELECT);
	GPIO_setAsPeripheralModuleFunctionInputPin(Board_ULTRASONIC_IN_7_PORT,
			Board_ULTRASONIC_IN_7_PIN,  Board_ULTRASONIC_IN_7_SELECT);

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

	captureModeConfig.captureRegister = Board_ULTRASONIC_IN_4_CCR;
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_4_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_4_TAx_MODULE, &captureModeConfig);

	captureModeConfig.captureRegister = Board_ULTRASONIC_IN_5_CCR;
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_5_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_5_TAx_MODULE, &captureModeConfig);

	captureModeConfig.captureRegister = Board_ULTRASONIC_IN_6_CCR;
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_6_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_6_TAx_MODULE, &captureModeConfig);

	captureModeConfig.captureRegister = Board_ULTRASONIC_IN_7_CCR;
	MAP_Timer_A_configureContinuousMode(Board_ULTRASONIC_IN_7_TAx_MODULE, &continuousModeConfig);
	Timer_A_initCapture(Board_ULTRASONIC_IN_7_TAx_MODULE, &captureModeConfig);

	/* Enabling global timer interrupts and starting timer */
#if(Board_ULTRASONIC_IN_4_TAx_MODULE==TIMER_A2_MODULE && Board_ULTRASONIC_IN_0_TAx_MODULE==TIMER_A2_MODULE && Board_ULTRASONIC_IN_1_TAx_MODULE==TIMER_A2_MODULE && Board_ULTRASONIC_IN_5_TAx_MODULE==TIMER_A2_MODULE)
	MAP_Interrupt_enableInterrupt(INT_TA2_N);
	/* Starting the Timer_A2 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A2_MODULE, TIMER_A_CONTINUOUS_MODE);
#else
	#error("Timer module changed. Check this code section and adapt the interrupt enables")
#endif

#if(Board_ULTRASONIC_IN_2_TAx_MODULE==TIMER_A3_MODULE && Board_ULTRASONIC_IN_3_TAx_MODULE==TIMER_A3_MODULE)
	MAP_Interrupt_enableInterrupt(INT_TA3_N);
	/* Starting the Timer_A3 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A3_MODULE, TIMER_A_CONTINUOUS_MODE);
#else
	#error("Timer module changed. Check this code section and adapt the interrupt enables")
#endif

#if(Board_ULTRASONIC_IN_6_TAx_MODULE==TIMER_A0_MODULE && Board_ULTRASONIC_IN_7_TAx_MODULE==TIMER_A0_MODULE)
	MAP_Interrupt_enableInterrupt(INT_TA0_N);
	/* Starting the Timer_A0 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A0_MODULE, TIMER_A_CONTINUOUS_MODE);
#else
	#error("Timer module changed. Check this code section and adapt the interrupt enables")
#endif

	/* Enable each CCR interrupt */
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_0_TAx_MODULE, Board_ULTRASONIC_IN_0_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_1_TAx_MODULE, Board_ULTRASONIC_IN_1_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_2_TAx_MODULE, Board_ULTRASONIC_IN_2_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_3_TAx_MODULE, Board_ULTRASONIC_IN_3_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_4_TAx_MODULE, Board_ULTRASONIC_IN_4_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_5_TAx_MODULE, Board_ULTRASONIC_IN_5_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_6_TAx_MODULE, Board_ULTRASONIC_IN_6_CCR );
	Timer_A_enableCaptureCompareInterrupt(Board_ULTRASONIC_IN_7_TAx_MODULE, Board_ULTRASONIC_IN_7_CCR );

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

/*
 * distance_values: integer times of flight in ms for each sensor
 */
bool ultrasonic_get_distance(int32_t distance_values[])
{
	bool retval=1;
	uint8_t j=0, i=0;

	for(i=0; i < N_ULTRASONIC_SENSORS; i++)
	{
		pulse_rising_time[i]=0;
		pulse_falling_time[i]=0;
	}
	/* iterate over the number of sensor arrays */
	for(j=0; j<N_ULTRASONIC_ARRAYS; j++)
	{
		ultrasonic_send_pulses(j);
	}
	for(i=0; i < N_ULTRASONIC_SENSORS; i++)
	{
		/* Check if pulse was detected and timestamp recorded */
		if(pulse_rising_time[i]==0 || pulse_falling_time[i]==0) //assuming timestamp = 0 is very unlikely
		{
			distance_values[i]=0;
			retval = 0; //return ERROR
		}
		else
		{
			/* Check that no timer overflow happened */
			if(pulse_rising_time[i] < pulse_falling_time[i])
			{ //calculate time difference normally
				distance_values[i]=(int32_t)pulse_falling_time[i] - (int32_t)pulse_rising_time[i];
			}
			else
			{ //compensate for the overflow
				distance_values[i]=(int32_t)pulse_falling_time[i] - (int32_t)pulse_rising_time[i] + 0xFFFFFFFF;
			}

			cli_printf("d%d : %d \n", i, distance_values[i]);
		}
	}

	return retval;
}

/*
 * distance_values: integer times of flight in ms for each sensor
 *
 * return an array of boolean values (or possibly more information) if the direction corresponding to an array of ultrasonic sensors is cleared or not
 */
void ultrasonic_check_distance(int32_t distance_values[], int8_t directions_array[])
{
	uint8_t	i=0;


	/* iterate over the number of sensor arrays */
	for(i=0; i<N_ULTRASONIC_SENSORS; i++)
	{
		if(distance_values[i] <= CRITICAL_DISTANCE_THRESHOLD_TIMESTAMP)
		{
			directions_array[i] = 0;
		}
		else
		{
			directions_array[i] = 1;
		}
	//	cli_printf("us: cleared? - for %d \n", directions_array[i+arrayoffset]);
	}
}

/*
 * Function triggers 4 out of 8 sensors to send out Ultrasonic pulse and
 * sends task to sleep for the necessary time. Then it triggers the other
 * 4 sensors and waits again before the calling function can resume
 *
 *
 */
void ultrasonic_send_pulses(uint8_t index)
{
	// TODO: this part of the code is only left here in case we have two different triggers
	const static uint8_t sleep_pins[2] = {Board_ULTRASONIC_SLEEP,Board_ULTRASONIC_SLEEP};
	const static uint8_t trigger_pins[2] = {Board_ULTRASONIC_TRIGGER,Board_ULTRASONIC_TRIGGER};

	uint32_t start_time;
	uint32_t stop_time;

	/* Turn on sensors 0-3 */
	GPIO_write(sleep_pins[index], 0);
	Task_sleep(600); //assure supply voltage could rise high.

	//TODO: maybe would be useful to turn on/off CCR interrupts

	/* Send pulse for 10us */
	start_time = Timestamp_get32();
	GPIO_write(trigger_pins[index], 1);
	stop_time = Timestamp_get32();
	while((stop_time-start_time)<10*CYCLES_PER_US)
	{ //wait 10us
		stop_time = Timestamp_get32();
	}

	GPIO_write(Board_WINDSENSOR_SLEEP, 1);
	Task_sleep(10);
	GPIO_write(Board_WINDSENSOR_SLEEP, 0);

	GPIO_write(trigger_pins[index], 0);

	/* Wait for all pulses to arrive and ISR to finish (max pulse lenght: 23ms) */
	Task_sleep(80);

	/* Turn off sensors 0-3 (or 4-7) */
	GPIO_write(sleep_pins[index], 0);
}


int16_t diff=0;

/*
 * ultrasonic_ccr_ISR() gets called in the TIMER_Ax ISR if it was triggered by an ultrasonic pin
 *
 * Arguments:
 *
 * index : ultrasonic input pin that triggered the CCR (0-7)
 * timestamp : ...
 * edgetype : 1=rising, 0=falling
 */
void ultrasonic_ccr_ISR(uint8_t pin_index, uint16_t timestamp, uint8_t edgetype)
{
	if(edgetype)
	{//bit is high -> rising edge
		pulse_rising_time[pin_index]=timestamp;

	}
	else
	{//input bit is low -> falling edge
		pulse_falling_time[pin_index]=timestamp;
	}

}
