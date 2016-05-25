/*
 * navigation.h
 *
 *  Created on: Oct 24, 2015
 *      Author: Eloi Benvenuti
 */

#ifndef FW_PERIPHERALS_NAVIGATION_H_
#define FW_PERIPHERALS_NAVIGATION_H_

#include <stdint.h>

#define N_TARGETS_MAX	20
typedef struct _target_list_t{
	float lat[N_TARGETS_MAX];
	float lon[N_TARGETS_MAX];
	uint8_t id[N_TARGETS_MAX];
	enum _target_list_state{
		INVALID = 0,
		VALID,
		DONE,
		CURRENT}
	state[N_TARGETS_MAX];
	// circular buffer index variables
	uint8_t current_index;
	uint8_t last_index;
} target_list_t;


/* Add a new target position to the list. Returns 1 on success and 0
 * if the queue is full.
 */
uint8_t navigation_remove_newest_target();

uint8_t navigation_add_target(float new_lat, float new_lon, uint8_t new_id);

uint8_t navigation_add_target_from_string(char* targetstring, int stringlength);


/* bypass the navigation logic and control the motors individually via CLI or other interface.
 * Commands are single chars:
 * 			- for wheels: f[orward], b[ackward], r[ight], l[eft], x[=stop]; index=0
 * 			- for struts: u[p], d[own], h[alt]; index = 1...4 to select strut
 * Return values:
 * 			1 for success
 * 			0 for invalid command
 */
uint8_t navigation_bypass(char command, uint8_t index);

float navigation_get_angle_to_target();

//Compute angle and distance from the rover to the target
float navigation_dist_to_target(float lat_current, float lon_current, float lat_target, float lon_target);
float navigation_angle_to_target(float lat_current, float lon_current, float lat_target, float lon_target);
//correct angle by taking into acount current heading of the rover (imu)
float navigation_angle_for_rover(float lat1, float lon1, float lat2, float lon2, float headX);
float navigation_degree_to_rad(float degree);

void navigation_change_gain(char pid, char type, float gain);

//"Main" task of the file
void navigation_task();

#endif /* FW_PERIPHERALS_NAVIGATION_H_ */
