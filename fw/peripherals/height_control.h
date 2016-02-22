/*
 * height_control.h
 *
 *  Created on: Feb 8, 2016
 *      Author: Eloi
 */

#ifndef FW_PERIPHERALS_HAL_HEIGHT_CONTROL_H_
#define FW_PERIPHERALS_HAL_HEIGHT_CONTROL_H_

#define OPERATION_UNSAFE 0
#define OPERATION_SAFE 1

#include <stdint.h>

typedef struct height_rover_status{
	uint16_t height_left; //in mm
	uint16_t height_right;
	float body_angle; //relative to the horizontal
}height_rover_status;

void height_control_update();

/* Check if rotating the body won't result in a collision with the ground */
uint8_t height_control_check_collision(float angle);

#endif /* FW_PERIPHERALS_HAL_HEIGHT_CONTROL_H_ */
