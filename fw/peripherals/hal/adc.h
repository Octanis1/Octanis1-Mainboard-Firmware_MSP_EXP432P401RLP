/*
 * adc.h
 *
 *  Created on: 16 Dec 2015
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_ADC_H_
#define FW_PERIPHERALS_HAL_ADC_H_

#include <stdint.h>

#define ADC_SUCCESS 	1
#define ADC_ERROR	0
#define N_ADC_AVG	10

void adc_isr();
uint8_t adc_read_motor_sensors(uint16_t sensor_values[]);


void adc_init(void);


#endif /* FW_PERIPHERALS_HAL_ADC_H_ */
