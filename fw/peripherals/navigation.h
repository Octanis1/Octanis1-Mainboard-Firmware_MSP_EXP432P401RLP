/*
 * navigation.h
 *
 *  Created on: Oct 24, 2015
 *      Author: Eloi Benvenuti
 */

#ifndef FW_PERIPHERALS_NAVIGATION_H_
#define FW_PERIPHERALS_NAVIGATION_H_

//Compute angle and distance from the rover to the target
float nav_get_distance(float lat_current, float lon_current, float lat_target, float lon_target);
float nav_get_angle(float lat_current, float lon_current, float lat_target, float lon_target);

//"Main" task of the file
void navigation_task();

#endif /* FW_PERIPHERALS_NAVIGATION_H_ */
