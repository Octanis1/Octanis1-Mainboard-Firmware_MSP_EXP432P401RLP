/*
 * height_control.c
 *
 *  Created on: Feb 8, 2016
 *      Author: Eloi
 *
 *      Units: all length are in millimeter and all angle are in 100 of a degree
 */

#include <math.h>

#include "height_control.h"
#include "hal/motors.h"
#include "imu.h"

#define WHEEL_RADIUS 80
#define WHEEL_THICKNESS 40
#define ARM_LENGTH 400
#define ARM_THICKNESS 20 //near the rover's body
#define BODY_LENGTH 400
#define BODY_HEIGHT 100
#define BODY_WIDTH 300

//Struct holding the data
static height_rover_status height_status;

void height_control_update(){
	uint16_t arms_degree [N_STRUTS];
	motors_struts_degrees_get (arms_degree);

	//TODO: compute the height based on the degree of the arms

	height_status.body_angle = ((float)imu_get_pitch())/100;
}

uint8_t height_control_check_collision(float angle){
	//We assume a plane under the rover

	uint16_t smallest_height = 0;
        uint16_t projected_height = 0;
        uint8_t ret = 0;

	if (height_status.height_right <= height_status.height_left)
		smallest_height = height_status.height_right;
	else
		smallest_height = height_status.height_left;

        projected_height = sin(angle)/(BODY_LENGTH/2);

        //We take 1mm of safety margin
        if((projected_height+1) >= smallest_height)
            ret = OPERATION_UNSAFE;
        else
            ret = OPERATION_SAFE;

	return ret;
}


