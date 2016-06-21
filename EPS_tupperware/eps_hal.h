/*
 * eps_hal.h
 *
 *  Created on: 09.05.2016
 *      Author: beat
 */

#ifndef EPS_HAL_H_
#define EPS_HAL_H_

#include <msp430fr5969.h>

//---------------------------------------------------------------
// definitions

#define I2C_BUFFER_SIZE		8

#define N_MODULES	9
//indexes:
#define M_M			0
#define M_SC		1
#define M_331		2
#define M_332		3
#define M_5			4
#define M_11			5
#define H_T1			6
#define H_T2			7
#define H_T3			8

#define ON		1
// turning a module off is always allowed
#define OFF		0

#define PORT_3V3_M_EN		P3OUT
#define PIN_3V3_M_EN		BIT2

#define PORT_3V3_1_EN		P3OUT
#define PIN_3V3_1_EN		BIT3

#define PORT_3V3_2_EN		P4OUT
#define PIN_3V3_2_EN		BIT7

#define PORT_5V_EN			P3OUT
#define PIN_5V_EN			BIT1

#define PORT_11V_EN			P3OUT
#define PIN_11V_EN			BIT0

#define PORT_SC_EN			P3OUT
#define PIN_SC_EN			BIT4

#define PORT_HEATER_1_EN	PJOUT
#define PIN_HEATER_1_EN		BIT0

#define PORT_HEATER_2_EN	PJOUT
#define PIN_HEATER_2_EN		BIT1

#define PORT_HEATER_3_EN	PJOUT
#define PIN_HEATER_3_EN		BIT2

#define MASTER_POKE_PORT	P3OUT
//on Analog EXT4
#define MASTER_POKE_PIN		BIT7
//on Analog EXT3
#define MASTER_POKE_PIN2	BIT6

#define PORT_DIGITAL_OUT	P2OUT
#define PIN_DIGITAL_1		BIT5
#define PIN_DIGITAL_2		BIT6
#define PIN_DIGITAL_3		BIT0
#define PIN_DIGITAL_4		BIT1
#define PIN_DIGITAL_5		BIT3
#define PIN_DIGITAL_6		BIT4

#define PORT_MB_POKE		PORT_DIGITAL_OUT
#define PIN_MB_POKE			PIN_DIGITAL_1

#define TIMER0_A0_DELAY		0x2000
#define TIMER0_A1_ENABLE	0
#define TIMER0_A1_DELAY		0x2000

#define POKE_COUNTER_LIMIT	5

#define ANALOG_PORTS		10
#define ANALOG_NUM_AVG		16

#define AIN_I_OUT_CH		0
#define AIN_I_IN_CH			1
#define AIN_V_SC_CH			2
#define AIN_V_BAT_CH		3
#define AIN_A_EXT0_CH		4
#define AIN_A_EXT1_CH		5
#define AIN_A_EXT2_CH		8
#define AIN_A_EXT3_CH		9
#define AIN_A_EXT4_CH		10
#define AIN_A_EXT5_CH		11

#define AIN_I_OUT_ADDR		0
#define AIN_I_IN_ADDR		1
#define AIN_V_SC_ADDR		2
#define AIN_V_BAT_ADDR		3
#define AIN_A_EXT0_ADDR		4
#define AIN_A_EXT1_ADDR		5
#define AIN_A_EXT2_ADDR		6
#define AIN_A_EXT3_ADDR		7
#define AIN_A_EXT4_ADDR		8
#define AIN_A_EXT5_ADDR		9
//-----------------------------------------------------------
//battery voltage threshold levels
#define THRESHOLD_80	585	 	//80% of charge (100% is 4.2V & 636 adc counts)
#define THRESHOLD_60	534		//60% of charge
#define THRESHOLD_40	481		//40% of charge
#define THRESHOLD_20	430		//20% of charge (0% is 2.5V)
#define THRESHOLD_0		378		//0% of charge
//battery temperature thresholds in ADC counts
#define COLD_20			385		//too cold for charging
#define COLD_0			672		//ok for charging
#define HOT_30			1020	//too hot, shut lower the PID duty cycle
#define T_BAT_OK		800
//-----------------------------------------------------------

#define SET_PIN(port, pin) port|=pin
#define CLR_PIN(port, pin) port&=~(pin)

//-----------------------------------------------------------

void i2c_init();
int i2c_read();
int i2c_glimpse();
int i2c_available();
void i2c_send(unsigned char data);

void gpio_init();

void timer0_A_init();

void ADC_init();
void ADC_update();
float ADC_read(int channel);

void module_set_state(int module_number, char state);
void module_update();

//TODO s:
//module interface

#endif /* EPS_HAL_H_ */
