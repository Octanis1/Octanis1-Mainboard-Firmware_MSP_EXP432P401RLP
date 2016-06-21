/*
 * eps_hal.c
 *
 *  Created on: 09.05.2016
 *      Author: beat
 */

#include "eps_hal.h"

volatile unsigned char RXData[I2C_BUFFER_SIZE];
volatile unsigned int RXData_ptr_start = 0; //points to oldest, unread char
volatile unsigned int RXData_ptr_end = 0; //points to free spot
volatile unsigned char TXData = 0xff;

volatile int ADC_summing = 0; //
volatile unsigned int ADC_sum[ANALOG_PORTS];
float ADC_result[ANALOG_PORTS];
//volatile unsigned int TXData_ptr_start = 0; //points to next char to be sent
//volatile unsigned int TXData_ptr_end = 0; //points to end (free spot)

//----------------------------------------------------------------------------------------------------------------------------------------------------
void gpio_init()
{
	//TODO amke dependent of definitions
	//P1OUT = 0xC0; //uncomment if internal i2c pullup required
	//P1REN = 0xC0;
	P1DIR = 0x00; //all inputs
	P1SEL0 = 0x3f; //p1.0-5 a0-a5, p1.6-7 i2c
	P1SEL1 = 0xff;

	P2OUT = 0x00; //external digital, default all pulldown inpputs
	P2REN = 0xff;
	P2DIR = 0x00;
	P2SEL0 = 0x00; //standard gpio
	P2SEL1 = 0x00;

	P3OUT = 0x04; //module pins p3.0-4 output, rest pulldown. everything off, main is active low
	P3REN = 0xe0;
	P3DIR = 0x1f;
	P3SEL0 = 0x00; //standard gpio
	P3SEL1 = 0x00;

	P4OUT = 0x00; //module pin p4.7, external analog p4.0-3, rest pulldown
	P4REN = 0x70;
	P4DIR = 0x80;
	P4SEL0 = 0x0f; //p4.0-3 analog in, rest standard gpio
	P4SEL1 = 0x0f;

	PJOUT = 0x00; //heater pins pj.0-2, pulldown on pj.3
	PJREN = 0x08;
	PJDIR = 0x07;
	PJSEL0 = 0x00; //standard gpio
	PJSEL1 = 0x00;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------

void i2c_send(unsigned char data)
{
	TXData = data;
}

void i2c_init()
{
	//TODO verify interrupts
	UCB0CTL1 |= UCSWRST; // eUSCI_B in reset state
	UCB0CTLW0 |= UCMODE_3; // I2C slave mode
	UCB0I2COA0 = 0x0048; // own address is 48hex
	UCB0CTL1 &= ~UCSWRST; // eUSCI_B in operational state

	UCB0IE |= UCTXIE + UCRXIE; // enable TX&RX-interrupt
}

int i2c_read()
{
	char buffer = RXData[RXData_ptr_start];
	if(RXData_ptr_start != RXData_ptr_end) //data available
	{
		//increment pointer as char was read;
		RXData_ptr_start++;
		while(RXData_ptr_start >= I2C_BUFFER_SIZE)
			RXData_ptr_start = RXData_ptr_start - I2C_BUFFER_SIZE;
		//return char
		return buffer;
	}
	//return -1 if no char read, unblocking call
	return -1;
}

int i2c_glimpse()
{
	if(RXData_ptr_start != RXData_ptr_end) //data available
	{
		//if bufffer not empty, return char
		return RXData[RXData_ptr_start];
	}
	//return -1 if no char read, unblocking call
	return -1;
}

int i2c_available()
{
	int available = RXData_ptr_end - RXData_ptr_start;
	while(available < 0)
		available += I2C_BUFFER_SIZE;
	return available;
}


// USCI_B0 Data ISR
#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
	if(UCB0IFG & UCTXIFG0) //TX
	{
		UCB0TXBUF = TXData;                     // TX data to send
	} else //RX
	{
		RXData[RXData_ptr_end++] = UCB0RXBUF;	  //read command
		while(RXData_ptr_end >= I2C_BUFFER_SIZE)
			RXData_ptr_end = RXData_ptr_end - I2C_BUFFER_SIZE;

		//check if no more space in buffer. if so, overwrite oldest (increment start pointer)
		if(RXData_ptr_start == RXData_ptr_end)
		{
			RXData_ptr_start++;
			while(RXData_ptr_start >= I2C_BUFFER_SIZE)
				RXData_ptr_start = RXData_ptr_start - I2C_BUFFER_SIZE;
		}
		//check_i2c_command();
		//TODO: i2c callback?
	}
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

void timer0_A_init()
{
	TA0R = 0x0000;                       // Reset timer counter
	TA0CCR0 = TIMER0_A0_DELAY;           // Set timing offset of A0
	TA0CCR1 = TIMER0_A0_DELAY + TIMER0_A1_DELAY / 2;                    // Set timing offset of A1
	TA0CCTL0 = CCIE;                     // TA0CCR0 interrupt enabled

#if TIMER0_A1_ENABLE
	TA0CCTL1 = CCIE;                     // TA0CCR1 interrupt enabled
#endif

	TA0EX0 = TAIDEX_7;					 // Prescaler: divide by 8
	TA0CTL = TASSEL_2 + MC_2 + ID_3;     // activate timer, SMCLK, contmode, prescaler 1:8
}

// Timer0 A interrupt service routine CC0
// Wake up processor to spin main loop once
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer0_A0 ()
{
	TA0CCR0 += TIMER0_A0_DELAY;
	__bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

// Timer0 A interrupt service routine CC1-2, OV
// Timer interrupt currently not in use
#pragma vector=TIMER0_A1_VECTOR
__interrupt void Timer0_A1 ()
{
	//TODO find use for this timer
	TA0CCR1 += TIMER0_A1_DELAY;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------

//enum adc_status_{
//	IDLE, 	//not in use, and not to be triggered
//	ADC_BUSY, 	//wait for it to finish
//	START, 	//the periodic timer interrupt requires a new measurement, therefore start a new ADC conversion
//	DONE		//ADC conversion is done, read values to status variable
//} adc_status;

void ADC_init()
{

	//adc_status = IDLE;
	//reference voltage
	REFCTL0 = REFVSEL_2;
	//4 adclock sh, msc set, adcon;
	ADC12CTL0 = ADC12SHT1_0 + ADC12SHT1_0 + ADC12MSC + ADC12ON;
	//prediv 1:4, adcshp, adc12div 1:8, SMCLK, sequence
	ADC12CTL1 = ADC12PDIV_1 + ADC12SHP + ADC12DIV_7 + ADC12SSEL_3 + ADC12CONSEQ_1;
	//12bit res, low power mode (max 50ksps)
	ADC12CTL2 = ADC12RES_2 + ADC12PWRMD;
	//
	ADC12CTL3 = 0x00;

	//Vref as reference
	ADC12MCTL0 = ADC12VRSEL_1 + AIN_I_OUT_CH;
	ADC12MCTL1 = ADC12VRSEL_1 + AIN_I_IN_CH;
	ADC12MCTL2 = ADC12VRSEL_1 + AIN_V_SC_CH;
	ADC12MCTL3 = ADC12VRSEL_1 + AIN_V_BAT_CH;
	ADC12MCTL4 = ADC12VRSEL_1 + AIN_A_EXT0_CH;
	ADC12MCTL5 = ADC12VRSEL_1 + AIN_A_EXT1_CH;
	ADC12MCTL6 = ADC12VRSEL_1 + AIN_A_EXT2_CH;
	ADC12MCTL7 = ADC12VRSEL_1 + AIN_A_EXT3_CH;
	ADC12MCTL8 = ADC12VRSEL_1 + AIN_A_EXT4_CH;
	ADC12MCTL9 = ADC12VRSEL_1 + AIN_A_EXT5_CH + ADC12EOS;

	// interrupt
	ADC12IER0 = 0x0200; //interrupt generated after conversion of last value
}

void ADC_update()
{
	int i;
	//reset buffer
	ADC_summing = 0;
	for(i = 0; i < ANALOG_PORTS; i++)
	{
		ADC_sum[i] = 0;
	}

	//start ADC
	ADC12CTL0 |= ADC12ENC + ADC12SC;

	//wait for ADC to finish
	while(ADC_summing < ANALOG_NUM_AVG)
	{
		__bis_SR_register(CPUOFF + GIE);        // Enter LPM0 w/ interrupts
		__no_operation();                       // Set breakpoint >>here<< and read
	}

	//TODO: calculate result for right units
	for(i = 0; i < ANALOG_PORTS; i++)
	{
		ADC_result[i] = ADC_sum[i] / ANALOG_NUM_AVG;
	}
}

float ADC_read(int port)
{
	if(port >= 0 && port < ANALOG_PORTS)
		return ADC_result[port];
	return 0;
}

// ADC interrupt after all values are read.
#pragma vector=ADC12_VECTOR
__interrupt void ADC_ISR ()
{
	//add new values to the buffer
	ADC_sum[0] += ADC12MEM0;
	ADC_sum[1] += ADC12MEM1;
	ADC_sum[2] += ADC12MEM2;
	ADC_sum[3] += ADC12MEM3;
	ADC_sum[4] += ADC12MEM4;
	ADC_sum[5] += ADC12MEM5;
	ADC_sum[6] += ADC12MEM6;
	ADC_sum[7] += ADC12MEM7;
	ADC_sum[8] += ADC12MEM8;
	ADC_sum[9] += ADC12MEM9;

	if(++ADC_summing < ANALOG_NUM_AVG)
	{
		//start another conversion series
		ADC12CTL0 |= ADC12ENC + ADC12SC;
	}
	else
	{
		//stop ADC (automatic)
		ADC12CTL0 &= ~ADC12ENC;
		//wake up CPU
		__bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
	}
}

void module_set_state(int module_number, char state)
{
	switch(module_number)
	{
		case M_M:
			if(state) CLR_PIN(PORT_3V3_M_EN, PIN_3V3_M_EN);
			else SET_PIN(PORT_3V3_M_EN, PIN_3V3_M_EN);
			break;
		case M_SC:
			if(state) SET_PIN(PORT_3V3_1_EN, PIN_3V3_1_EN);
			else CLR_PIN(PORT_3V3_1_EN, PIN_3V3_1_EN);
			break;
		case M_331:
			if(state) SET_PIN(PORT_3V3_2_EN, PIN_3V3_2_EN);
			else CLR_PIN(PORT_3V3_2_EN, PIN_3V3_2_EN);
			break;
		case M_332:
			if(state) SET_PIN(PORT_5V_EN, PIN_5V_EN);
			else CLR_PIN(PORT_5V_EN, PIN_5V_EN);
			break;
		case M_5:
			if(state) SET_PIN(PORT_11V_EN, PIN_11V_EN);
			else CLR_PIN(PORT_11V_EN, PIN_11V_EN);
			break;
		case M_11:
			if(state) SET_PIN(PORT_SC_EN, PIN_SC_EN);
			else CLR_PIN(PORT_SC_EN, PIN_SC_EN);
			break;
		case H_T1:
			if(state) SET_PIN(PORT_HEATER_1_EN, PIN_HEATER_1_EN);
			else CLR_PIN(PORT_HEATER_1_EN, PIN_HEATER_1_EN);
			break;
		case H_T2:
			if(state) SET_PIN(PORT_HEATER_2_EN, PIN_HEATER_2_EN);
			else CLR_PIN(PORT_HEATER_2_EN, PIN_HEATER_2_EN);
			break;
		case H_T3:
			if(state) SET_PIN(PORT_HEATER_3_EN, PIN_HEATER_3_EN);
			else CLR_PIN(PORT_HEATER_3_EN, PIN_HEATER_3_EN);
			break;
	}
}
