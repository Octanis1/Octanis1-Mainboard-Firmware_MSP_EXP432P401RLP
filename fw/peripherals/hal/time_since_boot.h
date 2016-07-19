/*
 * time_since_boot.h
 *
 *  Created on: 19 Jul 2016
 *      Author: raffael
 */

#ifndef FW_PERIPHERALS_HAL_TIME_SINCE_BOOT_H_
#define FW_PERIPHERALS_HAL_TIME_SINCE_BOOT_H_

#include <stdint.h>

void time_since_boot_init();

uint32_t sec_since_boot();
uint32_t ms_since_boot();
uint64_t us_since_boot();

#endif /* FW_PERIPHERALS_HAL_TIME_SINCE_BOOT_H_ */
