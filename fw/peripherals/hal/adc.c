/*
 * adc.c
 *
 *  Created on: 16 Dec 2015
 *      Author: raffael
 */

#include "adc.h"
#include "driverlib.h"
#include "motors.h"
#include "../../Board.h"

#define VOFFSET_VAL_12BIT			2021 // offset value from Vref (1.25V in 12 bits); empirically determined. (TODO: verify!)
#define CURRENT_SCALING_FACTOR_uA	(float)125000/2048	//divide by this factor to get from ADC counts to mA (full scale = ±125mA)

static uint64_t adc_int_status;

static uint16_t curADCResult;
static double normalizedADCRes;

/* motor current sensor variables */
//static uint16_t motorvalues[N_WHEELS]; // for motors 5 to 8
static uint8_t wheel_conversion_done;
static uint8_t strut_conversion_done;


void adc_isr()
{
	/* ADC Interrupt Handler. This handler is called whenever there is a conversion
	* that is finished for ADC_MEMx.
	*/
	adc_int_status = ADC14_getEnabledInterruptStatus();
	if (ADC_INT4 & adc_int_status) //strut4 current sensor measurement ready
	{
		ADC14_clearInterruptFlag(ADC_INT4);
		strut_conversion_done = 1;
		/***** END OF CONVERSION SEQUENCE 1 *****/
	}
	else if (ADC_INT23 & adc_int_status) //motor5 current sensor; 2nd priority
	{
//		motorvalues[0] = ADC14->MEM[23];
		ADC14_clearInterruptFlag(ADC_INT23);
		wheel_conversion_done = 1;
		/***** END OF CONVERSION SEQUENCE 2 *****/
	}

	else if (ADC_INT2 & adc_int_status) //test only
	{
		curADCResult = ADC14->MEM[2];
		normalizedADCRes = (curADCResult * 3.3) / 16384;
		ADC14_clearInterruptFlag(ADC_INT2);
		/***** END OF CONVERSION SEQUENCE 1 *****/
	}


	else if (ADC_INT20 & adc_int_status) //motor8 current sensor
	{
		/***** BEGIN OF CONVERSION SEQUENCE 2 *****/
//		motorvalues[3] = ADC14->MEM[20];
		ADC14_clearInterruptFlag(ADC_INT20);
		ADC14_toggleConversionTrigger();
	}
	else if (ADC_INT21 & adc_int_status) //motor7 current sensor
	{
//		motorvalues[2] = ADC14->MEM[21];
		ADC14_clearInterruptFlag(ADC_INT21);
		ADC14_toggleConversionTrigger();
	}
	else if (ADC_INT22 & adc_int_status) //motor6 current sensor
	{
//		motorvalues[1] = ADC14->MEM[22];
		ADC14_clearInterruptFlag(ADC_INT22);
		ADC14_toggleConversionTrigger();
	}
	else
	{
		ADC14_clearInterruptFlag(adc_int_status);
	}

}

// function returns motor current values in uA
uint8_t adc_read_motor_sensors(int32_t* wheel_sensor_values)
{
	static int i,j; //counter variables

	ADC14_disableConversion(); //just as a safety

	if(ADC14_configureMultiSequenceMode(ADC_MEM20,ADC_MEM23, false)) //Last argument is "repeat mode". only has effect in AUTOMATIC_ITERATION. Silicon Bug present too!
	{
		if(ADC14_enableConversion())
		{
			for(j=0;j<N_ADC_AVG_WHEEL;j++)
			{
				i = ADC14_toggleConversionTrigger();
				wheel_conversion_done = 0;

				while(i<N_WHEELS)
				{
					if(ADC14_isBusy())
					{continue;}
					else
					{
						i += (int)ADC14_toggleConversionTrigger();
					}
				}
				wheel_sensor_values[3] += ADC14->MEM[20];
				wheel_sensor_values[2] += ADC14->MEM[21];
				wheel_sensor_values[1] += ADC14->MEM[22];
				while(wheel_conversion_done == 0);
				wheel_sensor_values[0] += ADC14->MEM[23];

				Task_sleep(1);
			}

			ADC14_disableConversion();
			for(j = 0; j<N_WHEELS;j++)
			{
				wheel_sensor_values[j] -= VOFFSET_VAL_12BIT*N_ADC_AVG_WHEEL;
				wheel_sensor_values[j] = (float)wheel_sensor_values[j] * (float)((float)CURRENT_SCALING_FACTOR_uA/(float)N_ADC_AVG_WHEEL );
			}
			return ADC_SUCCESS;
		}
	}
	return ADC_ERROR;
}


uint8_t adc_read_strut_sensor_values(int32_t strut_sensor_values[N_STRUTS])
{
	static int i,j; //counter variables

	ADC14_disableConversion(); //just as a safety

	if(ADC14_configureMultiSequenceMode(ADC_MEM1,ADC_MEM4, false)) //Last argument is "repeat mode". only has effect in AUTOMATIC_ITERATION. Silicon Bug present too!
	{
		if(ADC14_enableConversion())
		{
			for(j=0;j<N_ADC_AVG_STRUT;j++)
			{
				i = ADC14_toggleConversionTrigger();
				strut_conversion_done = 0;

				while(i<N_STRUTS)
				{
					if(ADC14_isBusy())
					{continue;}
					else
					{
						i += (int)ADC14_toggleConversionTrigger();
					}
				}
				strut_sensor_values[0] += ADC14->MEM[1];
				strut_sensor_values[1] += ADC14->MEM[2];
				strut_sensor_values[2] += ADC14->MEM[3];
				while(strut_conversion_done == 0);
				strut_sensor_values[3] += ADC14->MEM[4];
			}

			ADC14_disableConversion();
			return ADC_SUCCESS;
		}
	}
	return ADC_ERROR;
}




void adc_init(void)
{
	/* Initializing ADC (MCLK/1/4) */
	ADC14_enableModule();
	ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_4, ADC_NOROUTE);
	//ADC14_setPowerMode(ADC_EXTREME_LOW_POWER_MODE);
	ADC14_setResolution(ADC_12BIT);

	/******** enable reference voltage and set to 2.5V ********/
	REF_A_enableReferenceVoltage();
	REF_A_setReferenceVoltage(REF_A_VREF2_5V);

	/******** Motor (wheel current sensor) ADC inputs *********/
	/* Configuring GPIOs (8.5 A20) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN5, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring GPIOs (8.4 A21) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring GPIOs (8.3 A22) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring GPIOs (8.2 A23) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P8, GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring ADC Memory */
	ADC14_configureMultiSequenceMode(ADC_MEM20,ADC_MEM23, false); //Last argument is "repeat mode". only has effect in AUTOMATIC_ITERATION. Silicon Bug present too!
	ADC14_configureConversionMemory(ADC_MEM20, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
	ADC_INPUT_A20, ADC_NONDIFFERENTIAL_INPUTS);
	ADC14_configureConversionMemory(ADC_MEM21, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
	ADC_INPUT_A21, ADC_NONDIFFERENTIAL_INPUTS);
	ADC14_configureConversionMemory(ADC_MEM22, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
	ADC_INPUT_A22, ADC_NONDIFFERENTIAL_INPUTS);
	ADC14_configureConversionMemory(ADC_MEM23, ADC_VREFPOS_INTBUF_VREFNEG_VSS,
	ADC_INPUT_A23, ADC_NONDIFFERENTIAL_INPUTS);

	/******** Strut position sensor ADC inputs *********/
	/* Configuring GPIOs (8.5 A1) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring GPIOs (8.4 A2) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN3, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring GPIOs (8.3 A3) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring GPIOs (8.2 A4) */
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
	/* Configuring ADC Memory */
	ADC14_configureMultiSequenceMode(ADC_MEM1,ADC_MEM4, false); //Last argument is "repeat mode". only has effect in AUTOMATIC_ITERATION. Silicon Bug present too!
	ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS,
	ADC_INPUT_A1, ADC_NONDIFFERENTIAL_INPUTS);
	ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS,
	ADC_INPUT_A2, ADC_NONDIFFERENTIAL_INPUTS);
	ADC14_configureConversionMemory(ADC_MEM3, ADC_VREFPOS_AVCC_VREFNEG_VSS,
	ADC_INPUT_A3, ADC_NONDIFFERENTIAL_INPUTS);
	ADC14_configureConversionMemory(ADC_MEM4, ADC_VREFPOS_AVCC_VREFNEG_VSS,
	ADC_INPUT_A4, ADC_NONDIFFERENTIAL_INPUTS);

	/* Configuring Sample Timer */
	ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);

	/* Enable interrupt */
	/* Struts */
	ADC14_enableInterrupt(ADC_INT4);

	/*motors*/
	/*ADC14_enableInterrupt(ADC_INT20);
	ADC14_enableInterrupt(ADC_INT21);
	ADC14_enableInterrupt(ADC_INT22);*/
	ADC14_enableInterrupt(ADC_INT23);

	/* Enabling Interrupts */
	Interrupt_enableInterrupt(INT_ADC14);
	Interrupt_enableMaster();

	/* Enabling/Toggling Conversion */
	//ADC14_enableConversion();
	//ADC14_toggleConversionTrigger();

}


