/*
 * navigation.h
 *
 *  Created on: Oct 24, 2015
 *      Author: Eloi Benvenuti
 */

#ifndef FW_PERIPHERALS_NAVIGATION_H_
#define FW_PERIPHERALS_NAVIGATION_H_

#include <stdint.h>
#include "comm.h"

#define N_TARGETS_MAX	20
#define OBSTACLE_MAX_DIST 300


/******** NEW types and functions to handle mavlink mission items **********/

typedef struct _mission_item_list_t{
	mavlink_mission_item_t item[N_TARGETS_MAX+1]; //+1 for security; needed if current_index>=count
	uint16_t current_index; //if higher than count-1, it means we are done with all mission items
	uint16_t count; //total number of mission items stored
} mission_item_list_t;

COMM_MAV_RESULT navigation_rx_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg);
COMM_MAV_RESULT navigation_rx_mission_items_start(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg);

COMM_MAV_RESULT navigation_tx_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg);
COMM_MAV_RESULT navigation_tx_mission_items_start(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg);

COMM_MAV_RESULT navigation_next_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg);


/* bypass the navigation logic and control the motors individually via CLI or other interface.
 * Commands are single chars:
 * 			- for wheels: f[orward], b[ackward], r[ight], l[eft], x[=stop]; index=0
 * 			- for struts: u[p], d[own], h[alt]; index = 1...4 to select strut
 * Return values:
 * 			1 for success
 * 			0 for invalid command
 */
uint8_t navigation_bypass(char command, uint8_t index);

mavlink_mission_item_t * navigation_mavlink_get_item_list();
uint16_t navigation_mavlink_get_current_index();
uint16_t navigation_mavlink_get_count();

float navigation_get_lat_rover();
float navigation_get_lon_rover();
float navigation_get_heading_rover();
float navigation_get_lat_target();
float navigation_get_lon_target();
float navigation_get_distance_to_target();
float navigation_get_angle_to_target();
float navigation_get_max_dist_obs();
uint8_t navigation_get_angle_valid();
enum _current_state navigation_get_current_state();

void navigation_set_max_dist(float max_dist);

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
