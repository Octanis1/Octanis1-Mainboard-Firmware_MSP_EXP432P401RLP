/*
 * ultrasonic.c
 *
 *  Created on: Sep 11, 2015
 *      Author: vagrant
 */

#include "ultrasonic.h"
#include "motors.h"
#include "../../../Board.h"
#include "../../core/eps.h"
#include <xdc/runtime/Timestamp.h>
#include <xdc/runtime/Types.h>
#include <msp432.h> //to access the registers
//#include <driverlib/timer_a.h>

/* DriverLib Includes */
#include "driverlib.h"

#define TRIGGER_HIGH		0
#define TRIGGER_LOW		1


uint16_t pulse_rising_time[N_ULTRASONIC_SENSORS]; //record timestamp when pulse was sent out
uint16_t pulse_falling_time[N_ULTRASONIC_SENSORS]; //record timestamp when the interrupt was triggered by the echoed pulse

static float bl[N_ULTRASONIC_SENSORS]; //braitenberg_weights_left
static float br[N_ULTRASONIC_SENSORS]; //braitenberg_weights_right

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
	/* Set the trigger pins to LOW (i.e. turn on NMOS -> set port high) and turn off 5V */
	GPIO_write(Board_ULTRASONIC_TRIGGER0, TRIGGER_LOW);
	GPIO_write(Board_ULTRASONIC_TRIGGER1, TRIGGER_LOW);

	GPIO_write(Board_ULTRASONIC_ENABLE, 0);


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
	//#if(Board_ULTRASONIC_IN_0_TAx_MODULE==TIMER_A2_BASE && Board_ULTRASONIC_IN_1_TAx_MODULE==TIMER_A2_BASE && Board_ULTRASONIC_IN_5_TAx_MODULE==TIMER_A2_BASE)
	MAP_Interrupt_enableInterrupt(INT_TA2_N);
	/* Starting the Timer_A2 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A2_BASE, TIMER_A_CONTINUOUS_MODE);
	//#else
	//#error("Timer module changed. Check this code section and adapt the interrupt enables")
	//#endif

	//#if(Board_ULTRASONIC_IN_2_TAx_MODULE==TIMER_A3_BASE && Board_ULTRASONIC_IN_3_TAx_MODULE==TIMER_A3_BASE)
	MAP_Interrupt_enableInterrupt(INT_TA3_N);
	/* Starting the Timer_A3 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_CONTINUOUS_MODE);
	//#else
	//#error("Timer module changed. Check this code section and adapt the interrupt enables")
	//#endif

	//#if(Board_ULTRASONIC_IN_4_TAx_MODULE==TIMER_A0_BASE && Board_ULTRASONIC_IN_6_TAx_MODULE==TIMER_A0_BASE && Board_ULTRASONIC_IN_7_TAx_MODULE==TIMER_A0_BASE)
	MAP_Interrupt_enableInterrupt(INT_TA0_N);
	/* Starting the Timer_A0 in continuous mode */
	MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_CONTINUOUS_MODE);
	//#else
	//#error("Timer module changed. Check this code section and adapt the interrupt enables")
	//#endif

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


	TIMER_A2->CCTL[2] |= TIMER_A_CCTLN_CM__BOTH + CCIE + CAP;// TIMER_A_CCTLN_CCIS__CCIA;
	*/

	ultrasonic_set_bl(0,-0.5,-0.5,1,-0.5,2,1,0); //braitenberg_weights_left
	ultrasonic_set_br(0,1,2,-0.5,1,-0.5,-0.5,0);

}

void ultrasonic_set_bl(float a, float b, float c, float d, float e, float f, float g, float h)
{
	bl[0]=a;
	bl[1]=b;
	bl[2]=c;
	bl[3]=d;
	bl[4]=e;
	bl[5]=f;
	bl[6]=g;
	bl[7]=h;
}

void ultrasonic_set_br(float a, float b, float c, float d, float e, float f, float g, float h)
{
	br[0]=a;
	br[1]=b;
	br[2]=c;
	br[3]=d;
	br[4]=e;
	br[5]=f;
	br[6]=g;
	br[7]=h;
}

int32_t ultrasonic_get_smallest (int32_t *distance_values, uint8_t size){
	int32_t smallest = ULTRASONIC_MAX_SENSOR_VALUE;

#ifdef USE_ULTRASONIC
	int i = 0;
	//Since some of the sensors are not connected and thus return 0, we have to weed them out
	for(i=0; i<size; i++){
		if((distance_values[i] < smallest) && (distance_values[i] != 0))
			smallest = distance_values[i];
	}
#endif

	return smallest;
}

/*
 * distance_values: integer times of flight in ms for each sensor
 */
uint8_t ultrasonic_get_distance(int32_t distance_values[])
{
	uint8_t retval=1;
#ifdef USE_ULTRASONIC
	uint8_t j=0,
			i=0,
			arrayoffset=0; //the index offset to store the distance_value's for each array

	for(i=0; i < N_ULTRASONIC_SENSORS; i++)
	{
		distance_values[i]=0;
	}

	/* Turn on 5V supply for sensors */
	eps_switch_module(M5V_ON);
	GPIO_write(Board_ULTRASONIC_ENABLE, 1);
	Task_sleep(300); //assure supply voltage could rise high.

	/* iterate over the number of sensor arrays */
	for(j=0; j<N_ULTRASONIC_ARRAYS*N_ULTRASONIC_SAMPLES; j++)
	{
		arrayoffset = (j%N_ULTRASONIC_ARRAYS)*N_ULTRASONIC_SENSORS_PER_ARRAY;
		for(i=0; i < N_ULTRASONIC_SENSORS_PER_ARRAY; i++)
		{
			pulse_rising_time[i+arrayoffset]=0;
			pulse_falling_time[i+arrayoffset]=0;
		}

		ultrasonic_send_pulses((j%N_ULTRASONIC_ARRAYS));

		for(i=0; i < N_ULTRASONIC_SENSORS_PER_ARRAY; i++)
		{
			/* Check if pulse was detected and timestamp recorded */
			if(pulse_rising_time[i+arrayoffset]==0 || pulse_falling_time[i+arrayoffset]==0) //assuming timestamp = 0 is very unlikely
			{
				distance_values[i+arrayoffset] = 0;
				//retval = 0; //return ERROR
			}
			else
			{
				/* Check that no timer overflow happened */
				if(pulse_rising_time[i+arrayoffset] < pulse_falling_time[i+arrayoffset])
				{ //calculate time difference normally
					distance_values[i+arrayoffset] += ((int32_t)pulse_falling_time[i+arrayoffset] - (int32_t)pulse_rising_time[i+arrayoffset]);
				}
				else
				{ //compensate for the overflow
					distance_values[i+arrayoffset] += ((int32_t)pulse_falling_time[i+arrayoffset] - (int32_t)pulse_rising_time[i+arrayoffset] + 0xFFFF);
				}

			//	serial_printf(cli_stdout, "us : distance : %d \n", distance_values[i+arrayoffset]);
			}
		}

	}

	/* Turn off sensors */
	GPIO_write(Board_ULTRASONIC_ENABLE, 0);
	eps_switch_module(M5V_OFF);

#endif

	return retval;
}

/*
 * input:
 * - dist: integer times of flight in ms for each sensor
 * - motor_scaling_factor: 100% PWM signal number.
 *
 * returns:
 * - motor_values[0] = left speed
 * - motor_values[1] = right speed
 * - return value: true if obstacle is near, false if not.
 */
uint8_t ultrasonic_check_distance(int32_t dist[], int32_t motor_values[], int32_t motor_scaling_factor)
{

//	static uint8_t bl[N_ULTRASONIC_SENSORS] = {0,2,0.5,-0.5,1,-0.5,-0.5,0}; //braitenberg_weights_left
//	static uint8_t br[N_ULTRASONIC_SENSORS] = {0,-0.5,-0.5,1,-0.5,0.5,2,0}; //braitenberg_weights_right

	uint8_t j=0,
			i=0,
			arrayoffset=0; //the index offset to store the distance_value's for each array

	int32_t tmp_motor_values[2] = {0, 0};


	/* iterate over the number of sensor arrays */
	for(j=0; j<N_ULTRASONIC_ARRAYS; j++)
	{
		arrayoffset = j*N_ULTRASONIC_SENSORS_PER_ARRAY;
		for(i=0; i < N_ULTRASONIC_SENSORS_PER_ARRAY; i++)
		{
			if(dist[i+arrayoffset] <= CRITICAL_DISTANCE_THRESHOLD_TICKS) //if any of the sensors is below a threshold,
			{
				int32_t average_val = (dist[2] + dist[5]); //look ahead and react stronger when something is near just ahead
				tmp_motor_values[0] = bl[0] * dist[0] + bl[1] * dist[1] + bl[2] * dist[2] + bl[3] * dist[3] +
						bl[4] * dist[4] + bl[5] * dist[5] + bl[6] * dist[6] + bl[7] * dist[7] + average_val;
				tmp_motor_values[1] = br[0] * dist[0] + br[1] * dist[1] + br[2] * dist[2] + br[3] * dist[3] +
						br[4] * dist[4] + br[5] * dist[5] + br[6] * dist[6] + br[7] * dist[7] + average_val;

				if(tmp_motor_values[0] == 0 || tmp_motor_values[1] == 0)
				{
					// There are no updated ultrasonic sensor values. --> keep old values
					return 1;
				}

				//scaling with absolute maximum and 100% speed:
				int32_t maxval = tmp_motor_values[0];
				if(tmp_motor_values[1] > maxval)
					maxval = tmp_motor_values[1];

				motor_values[0] = tmp_motor_values[0] * motor_scaling_factor / maxval;
				motor_values[1] = tmp_motor_values[1] * motor_scaling_factor / maxval;


				// assure a certain minimum speed
				if(motor_values[0] < PWM_MINIMUM_SPEED)
					motor_values[0] = PWM_MINIMUM_SPEED;

				if(motor_values[1] < PWM_MINIMUM_SPEED)
					motor_values[1] = PWM_MINIMUM_SPEED;

				return 1;
			}

		//	serial_printf(cli_stdout, "us: cleared? - for %d \n", directions_array[i+arrayoffset]);
		}

	}

	// If this point is reached, it means  there is no close obstacle --> full speed ahead.
	motor_values[0] = motor_scaling_factor;
	motor_values[1] = motor_scaling_factor;

	return 0;
}

/*
 * Function triggers 4 out of 8 sensors to send out Ultrasonic pulse and
 * sends task to sleep for the necessary time. Then it triggers the other
 * 4 sensors and waits again before the calling function can resume
 *
 * Trigger0 is connected to sensors 6,0,4 and 2,
 * Trigger1 is connected to sensors 3,1,5 and 7.
 */
void ultrasonic_send_pulses(uint8_t index)
{
	// TODO: this part of the code is only left here in case we have two different triggers
	const static uint8_t trigger_pins[2] = {Board_ULTRASONIC_TRIGGER0,Board_ULTRASONIC_TRIGGER1};

	uint32_t start_time;
	uint32_t stop_time;

	//TODO: maybe would be useful to turn on/off CCR interrupts

	/* Send pulse for 10us */
	start_time = Timestamp_get32();
	GPIO_write(trigger_pins[index], TRIGGER_HIGH);
	stop_time = Timestamp_get32();
	while((stop_time-start_time)<10*CYCLES_PER_US)
	{ //wait 10us
		stop_time = Timestamp_get32();
	}
	GPIO_write(trigger_pins[index], TRIGGER_LOW);

	/* Wait for all pulses to arrive and ISR to finish (max pulse lenght: 23ms) */
	Task_sleep(50);



}

/* TODO: this code is just here for reference. remove later
UInt32 time;

void ultrasonic_ISR(uint8_t index){
	if(running)
	{
		//pulse_falling_time[index]=Timer_getCount(timer);
		pulse_falling_time[index]=TIMER_A2->CCR[2];
		TIMER_A2->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;
		running=0;
	}
	else
	{
		//pulse_rising_time[index]=Timer_getCount(timer);
		pulse_rising_time[index]=TIMER_A2->CCR[2];
		TIMER_A2->CCTL[2] &= ~TIMER_A_CCTLN_CCIFG;
		running=1;
	}
}*/

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
