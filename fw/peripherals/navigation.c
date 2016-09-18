/*
 * navigation.c
 *
 *  Created on: Oct 24, 2015
 *      Author: Eloi Benvenuti
 *
 *  File description: Module for computing the path from position to target and sending command to motors.
 *  distances are in meters and angles in radian.
 */
#if defined(NAVIGATION_TEST)
#include "navigation.h"
#include <math.h>
void Task_sleep(int a);



#else

#include "../../Board.h"
#include "gps.h"
#include "imu.h"
#include "navigation.h"
#include "hal/motors.h" //also contains all sorts of geometries (wheel radius etc)
#include "hal/adc.h"
#include "hal/time_since_boot.h"
#include "hal/ultrasonic.h"
#include <math.h>
#include "../lib/printf.h"
#include "pid.h"
#include "../lib/serial_printf.h"
#include <string.h>

//Logging includes:
#include "../core/log.h"
#include "../core/log_entries.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include "flash.h"

#endif
#define M_PI 3.14159265358979323846
#define EARTH_RADIUS 6356752.3
#define INIT_LAT 0
#define INIT_LON 0
#define TARGET_LAT 1
#define TARGET_LON 1
#define TARGET_REACHED_DISTANCE 1 //meters

#define PGAIN_A 1
#define IGAIN_A 0
#define DGAIN_A 0
#define MOTOR_IMAX 40
#define MOTOR_IMIN 0
#define BACKWARD_THRESHOLD 200 //about 1m

#define MIN_IMU_CALIB_STATUS	6 //to have angle valid = true

typedef struct _navigation_status{
	double lat_rover;
	double lon_rover;
	double heading_rover;
	double lat_target;
	double lon_target;
	double distance_to_target;
	double angle_to_target; // [-180, 180]. Positive means target is located to the right
	double max_dist_obs;
	int32_t motor_values[2]; // current motor speed
	bool position_valid; // defines if GPS and heading angle are valid. if false, the rover shall not drive to target
	enum _current_state{
		STOP=0,
		BYPASS,
		GO_TO_TARGET,
		AVOID_OBSTACLE,
		AVOID_WALL,
		SPACE_NEEDED,
	} current_state;
	char halt;
	char cmd_armed_disarmed; // received command to arm (1) or disarm (0)
} navigation_status_t;

static navigation_status_t navigation_status;
static pid_controler_t pid_a;

static mission_item_list_t mission_items;

static uint8_t pos_counter;

/** public functions **/
int navigation_rover_moving()
{
	return ((navigation_status.motor_values[0]!=0) || (navigation_status.motor_values[1] != 0));
}

void navigation_update_current_target();

mavlink_mission_item_t * navigation_mavlink_get_item_list()
{
	return mission_items.item;
}

uint16_t navigation_mavlink_get_current_index()
{
	return mission_items.current_index;
}

uint16_t navigation_mavlink_get_count()
{
	return mission_items.count;
}


/** helper functions **/

void navigation_update_current_target();

double navigation_degree_to_rad(double degree)
{
    double rad = degree/180*M_PI;
    return rad;
}

double navigation_dist_to_target(double lat_current, double lon_current, double lat_target, double lon_target){

	//The gps gives the coordinate in degree
    lat_current = navigation_degree_to_rad(lat_current);
    lat_target = navigation_degree_to_rad(lat_target);
    lon_current = navigation_degree_to_rad(lon_current);
    lon_target = navigation_degree_to_rad(lon_target);

    double delta_lat = lat_target - lat_current;
    double delta_lon = lon_target - lon_current;

    double a = sin(delta_lat/2)*sin(delta_lat/2) + cos(lat_current)*cos(lat_target) * sin(delta_lon/2)*sin(delta_lon/2);
    
    double distance = 2*EARTH_RADIUS*asin(sqrt(a));
    return distance;
}

/**
 * Params: lat1, long1 => Latitude and Longitude of current point
 *         lat2, long2 => Latitude and Longitude of target  point
 *
 * Returns the degree of a direction from current point to target point (between -180, 180).
 * Negative value are toward West, positive East
 */
double navigation_angle_to_target(double lat1, double lon1, double lat2, double lon2) {
    lat1 = navigation_degree_to_rad(lat1);
    lat2 = navigation_degree_to_rad(lat2);
    lon1 = navigation_degree_to_rad(lon1);
    lon2 = navigation_degree_to_rad(lon2);
    
    double dLon = lon2-lon1;
    double dLat = lat2-lat1;

    double y = sin(dLon) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1)*cos(lat2)*cos(dLat);
    double brng = 180/M_PI*(atan2(y, x));

    return brng;
}

// returns a negative value if target is to the left of the rover, a positive value if target is to the right
double navigation_angle_for_rover(double lat1, double lon1, double lat2, double lon2, double headX) {
    double bearing = navigation_angle_to_target(lat1, lon1, lat2, lon2);
    bearing = bearing - headX;
    // bound value between -180 and 180°
    while(bearing < -180.0)
    {
    		bearing = bearing + 360.0;
    }
    while(bearing > 180.0)
    {
    		bearing = bearing - 360.0;
    }
    // if value is close to ±180°, we keep the same sign as previously to turn in just one direction:
    if(fabs(bearing) > 170.0)
    {
    		bearing=copysign(bearing, navigation_status.angle_to_target);
    }

    return bearing;
}

/******** functions to control driving/armed state via MAVLINK **********/

MAV_RESULT navigation_halt_resume(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg)
{
	float hold_resume = mavlink_msg_command_long_get_param1(msg);
	if(hold_resume == MAV_GOTO_DO_HOLD)
	{
		navigation_status.halt = 1;
		navigation_status.current_state = STOP;
	}
	else if(hold_resume == MAV_GOTO_DO_CONTINUE)
	{
		navigation_status.halt = 0;
	}
	else
	{
		return MAV_RESULT_DENIED;
	}
	return MAV_RESULT_ACCEPTED;
}

/* This function decides if the SBC should be armed or disarmed, depending on the current waypoint and state of this system.
 * Shall be in the navigation task
 */
void navigation_arm_disarm()
{
#ifndef ARM_IMMEDIATELY
	if(navigation_status.position_valid)
#endif
	{
		// condition to arm mainboard: know a valid position. TODO: add more conditions (EPS motor on state, etc...)
		if(navigation_status.cmd_armed_disarmed == 1)
		{
			comm_arm_mainboard();
			//condition to arm SBC:
			if(comm_sbc_running()==1 && mission_items.current_index > 0)
			{
				if(comm_sbc_armed() == 0)
					comm_arm_disarm_subsystems(1);
			}
		}
		else
		{
			if(comm_sbc_armed())
				comm_arm_disarm_subsystems(0);
			comm_disarm_mainboard();
		}
	}
#ifndef ARM_IMMEDIATELY
	else
	{
		if(comm_sbc_armed())
			comm_arm_disarm_subsystems(0);
		comm_disarm_mainboard();
	}
#endif
}


// This function stores the information that "ARM/DISARM" has been received.
void navigation_rxcmd_arm_disarm(float arm_disarm)
{
	navigation_status.cmd_armed_disarmed = (arm_disarm==1);
	if(arm_disarm==1) // also unhalt rover to start driving immediately.
		navigation_status.halt = 0;
}


/******** functions to handle mavlink mission items **********/
static uint16_t mission_item_rx_count;

COMM_MAV_RESULT navigation_rx_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	uint16_t next_index = mavlink_msg_mission_item_get_seq(msg);
	if(next_index < N_TARGETS_MAX)
	{ // buffer has space --> add to list
		mavlink_msg_mission_item_decode(msg, &(mission_items.item[next_index]));
		if(mavlink_msg_mission_item_get_current(msg))
		{
			mission_items.current_index = next_index;
		}

		if(mission_item_rx_count > next_index + 1) // request next item
		{
			mavlink_msg_mission_request_pack(mavlink_system.sysid, target->component, answer_msg,
					msg->sysid, target->component, next_index + 1);
		}
		else // transfer done
		{
			mavlink_msg_mission_ack_pack(mavlink_system.sysid, target->component, answer_msg,
					msg->sysid, target->component, 	MAV_MISSION_ACCEPTED);
			if(mission_item_rx_count == 0)
			{// If a waypoint planner component receives MISSION_ITEM messages outside of transactions it answers with a MISSION_ACK message.
				if(mission_items.count == next_index + 1)
					mission_items.count++; //an item was appended to the end of the list list
			}
			else
			{
				mission_items.count = next_index + 1;
				mission_item_rx_count = 0;
			}
#ifdef FLASH_ENABLED
			log_write_mavlink_item_list(false, &pos_counter);
#endif
		}
		return REPLY_TO_SENDER;
	}
	else
	{ //buffer overflow...
		//TODO: reply with MAV_CMD_ACK_ERR_FAIL because buffer is full.
		return NO_ANSWER; //for now we just don't answer -->timeout.
	}
}

// function to call at the beginning of a mission item write transaction (i.e. after receiving MISSION_COUNT)
COMM_MAV_RESULT navigation_rx_mission_items_start(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	mission_item_rx_count = mavlink_msg_mission_count_get_count(msg);

	/* send mission acknowledge */
	mavlink_msg_mission_request_pack(mavlink_system.sysid, target->component, answer_msg,
			msg->sysid, target->component, 0);

	return REPLY_TO_SENDER;
}


// function to call after a MISSION_REQUEST_LIST command
COMM_MAV_RESULT navigation_tx_mission_items_start(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	mavlink_msg_mission_count_pack(mavlink_system.sysid, target->component, answer_msg,
			msg->sysid, target->component, mission_items.count);

	return REPLY_TO_SENDER;
}

COMM_MAV_RESULT navigation_tx_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	uint16_t seq_id = mavlink_msg_mission_request_get_seq(msg);

	if(seq_id < N_TARGETS_MAX)
	{
		// this function packs the already available struct.
		mavlink_msg_mission_item_encode(mavlink_system.sysid, target->component, answer_msg, &(mission_items.item[seq_id]));
		return REPLY_TO_SENDER;
	}
	else //requested item outside buffer
	{
		//TODO: send NACK
		return NO_ANSWER;
	}
}

//to be called after MISSION_SET_CURRENT ( #41 ) OR after a mission item was reached by the rover.
// 													In the latter case set *msg = NULL
COMM_MAV_RESULT navigation_next_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	if(msg == NULL) // automatically go to next item in list
	{
		mission_items.current_index++;
		if(mission_items.current_index<mission_items.count) //activate next target if available
		{
			mission_items.item[mission_items.current_index].current = 1;
#ifdef FLASH_ENABLED
			log_write_mavlink_item_list(true, &pos_counter);
#endif
		}
		else
		{
			// pretend to start reaching home again but do not update target
			mavlink_msg_mission_current_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, answer_msg,
						0);
			navigation_status.current_state = STOP;
			// SPC gets disarmed automatically as long as it is sending heartbeats,
			//    because current target is 0 (HOME) again
			return REPLY_TO_SENDER;
		}
	}
	else //received command to change the current item
	{
		mission_items.item[mission_items.current_index].current = 0;

		mission_items.current_index = mavlink_msg_mission_set_current_get_seq(msg);

		if(mission_items.current_index < mission_items.count)
			mission_items.item[mission_items.current_index].current = 1;
		else
			return NO_ANSWER; //TODO: reply NACK
#ifdef FLASH_ENABLED
		log_write_mavlink_item_list(false, &pos_counter);
#endif

	}

	// Set state to reach new waypoints:
	navigation_update_current_target();

	// Inform controller about the new waypoint:
	mavlink_msg_mission_current_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, answer_msg,
			mission_items.current_index);
	return REPLY_TO_SENDER;
}

void navigation_mission_item_reached()
{
	//send MISSION_ITEM_REACHED ( #46 )
	COMM_FRAME msg_frame;

	mavlink_msg_mission_item_reached_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &(msg_frame.mavlink_message),
			mission_items.current_index);
	comm_mavlink_broadcast(&(msg_frame));


	mission_items.item[mission_items.current_index].current = 0;
	if(mission_items.item[mission_items.current_index].autocontinue)
	{
		if(navigation_next_mission_item(NULL, NULL, &(msg_frame.mavlink_message)) == REPLY_TO_SENDER)
		{
			comm_mavlink_broadcast(&(msg_frame));
		}
	}
}

COMM_FRAME* navigation_pack_rc_channels_scaled()
{
	static COMM_FRAME frame;

	uint8_t port             = 0;
	int16_t chan7_scaled = 0;
	int16_t chan8_scaled = 0;
	uint8_t rssi             = 0;

	int16_t left_side = navigation_status.motor_values[0];
	int16_t right_side = navigation_status.motor_values[1];

	//get currents
	static int32_t sensor_values[N_WHEELS];

	/* initialize to 0 to reset the running average inside the adc readout function */
	sensor_values[0] = 0;
	sensor_values[1] = 0;
	sensor_values[2] = 0;
	sensor_values[3] = 0;

	adc_read_motor_sensors(sensor_values);

	uint32_t msec = ms_since_boot();

	mavlink_msg_rc_channels_scaled_pack(mavlink_system.sysid, MAV_COMP_ID_PATHPLANNER, &(frame.mavlink_message),
				   msec, port, sensor_values[0]/100,sensor_values[1]/100, sensor_values[2]/100, sensor_values[3]/100,
				   chan7_scaled, chan8_scaled, left_side, right_side,  rssi);

	return &frame;
}

// message that is continuously at each task call
COMM_FRAME* navigation_pack_mavlink_hud()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	float airspeed = 0.; //TODO
	float groundspeed = 0.; //TODO
	int32_t heading = gps_get_gps_heading();
	if(heading < 0)
		heading = heading + 36000;
	heading = heading / 100; //from 0 to 360 deg
	uint16_t throttle = (navigation_status.motor_values[0] + navigation_status.motor_values[1]);
	float alt = (1000.0 * gps_get_int_altitude());//Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	float climb = 0.; //TODO

	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message

	mavlink_msg_vfr_hud_pack(mavlink_system.sysid, MAV_COMP_ID_PATHPLANNER, &(frame.mavlink_message),
			airspeed, groundspeed, (int16_t)heading, throttle, alt, climb);

	return &frame;
}


#if defined (NAVIGATION_TEST)
uint8_t navigation_bypass(char command, uint8_t index);
#else
uint8_t navigation_bypass(char command, uint8_t index)
{
	static int8_t strut_speed[4]; //initialized as zero at the first time

	if(command == 'n')
	{
		navigation_status.current_state = STOP;
		return 1;
	}
	else if(command == 'u' || command== 'd' || command == 'h')
	{
		if(index>0 && index<5)
		{
			if(command == 'h')
				strut_speed[index-1] = 0;
			else if(command == 'u')
				strut_speed[index-1] = 1;
			else if(command == 'd')
				strut_speed[index-1] = -1;
		}
		else
		{
			uint8_t i=0;
			for(i=0;i<4;i++)
			{
				if(command == 'h')
					strut_speed[i] = 0;
				else if(command == 'u')
					strut_speed[i] = 1;
				else if(command == 'd')
					strut_speed[i] = -1;
			}
		}
		motors_struts_move(strut_speed[0],strut_speed[1],strut_speed[2],strut_speed[3]);
	}
	else
	{
		if(command == 'x')
		{
			motors_wheels_stop();
			navigation_status.current_state = STOP;
			navigation_status.motor_values[0] = 0; navigation_status.motor_values[1] = 0;
			return 1;
		}
		switch(command)
		{
		case 'f': navigation_status.motor_values[0] = PWM_SPEED_100; navigation_status.motor_values[1] = PWM_SPEED_100; break;
		case 'b': navigation_status.motor_values[0] = -PWM_SPEED_100; navigation_status.motor_values[1] = -PWM_SPEED_100; break;
		case 'l': navigation_status.motor_values[0] = PWM_SPEED_80; navigation_status.motor_values[1] = PWM_SPEED_100; break;
		case 'r': navigation_status.motor_values[0] = PWM_SPEED_100; navigation_status.motor_values[1] = PWM_SPEED_80; break;
		default: return 0;
		}
		motors_wheels_move(navigation_status.motor_values[0], navigation_status.motor_values[1],
				navigation_status.motor_values[0], navigation_status.motor_values[1]);
	}

	navigation_status.current_state = BYPASS;
	return 1;
}
#endif


#if defined (NAVIGATION_TEST)
void navigation_update_position();
#else
void navigation_update_position()
{
	gps_update_new_position(&(navigation_status.lat_rover), &(navigation_status.lon_rover));

//	if())
//	{
//		//we get a new gps position
//	}
//	else
//	{
//		//we didn't get a new gps position --> update position using odometry.
//		//TODO
//		motors_wheels_update_distance();
//
//		motors_struts_get_position();
//	}

	navigation_status.position_valid = gps_valid();

	// recalculate heading angle
#ifndef USE_GPS_HEADING
	float hdg = imu_get_fheading();
	if(hdg > 180.0)
		hdg = hdg - 360.0;
	navigation_status.heading_rover = hdg;
#else
	navigation_status.heading_rover = gps_get_gps_fheading(); 
#endif
	navigation_status.angle_to_target = navigation_angle_for_rover(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target, navigation_status.heading_rover);
	// recalculate distance to target
	navigation_status.distance_to_target = navigation_dist_to_target(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target);
}
#endif


void navigation_update_current_target()
{
	if(mission_items.current_index < mission_items.count && navigation_status.current_state != BYPASS)
	{
		//TODO: check for frame and command variable of the mission_item.
		navigation_status.lat_target = mission_items.item[mission_items.current_index].x;
		navigation_status.lon_target = mission_items.item[mission_items.current_index].y;
		navigation_status.current_state = GO_TO_TARGET;
	}
	//TODO: do we have to switch state here?
}

void navigation_update_state()
{
    int32_t distance_values[N_ULTRASONIC_SENSORS];
    int32_t smallest = 0;
    int32_t left_s = 0;
    int32_t right_s = 0;
    int32_t front1_s = 0;
    int32_t front2_s = 0;

    ultrasonic_get_distance(distance_values);
    smallest = ultrasonic_get_smallest(distance_values, N_ULTRASONIC_SENSORS);
    left_s = distance_values[US_LEFT];
    right_s = distance_values[US_RIGHT];
    front1_s = distance_values[US_FRONT_1];
    front2_s = distance_values[US_FRONT_2];

    if(navigation_status.current_state == BYPASS){
		//motor command is done directly in CLI
    }
    else if(navigation_status.current_state == AVOID_OBSTACLE){

        if (smallest > navigation_status.max_dist_obs){
        	navigation_status.current_state = GO_TO_TARGET;
        	GPIO_write(Board_OBS_A_EN, 0);
        }else if ((left_s < BACKWARD_THRESHOLD/2) || (right_s < BACKWARD_THRESHOLD/2)){
        	navigation_status.current_state = SPACE_NEEDED;
        }else if (front1_s < BACKWARD_THRESHOLD && front2_s < BACKWARD_THRESHOLD){
        	serial_printf(cli_stdout, "Wall ahead!\n");
        	navigation_status.current_state = AVOID_WALL;
        }

    }
    else if (navigation_status.current_state == AVOID_WALL){
		//We check if we're not facing the wall anymore
			if ((left_s < BACKWARD_THRESHOLD/2) || (right_s < BACKWARD_THRESHOLD/2)){
				navigation_status.current_state = SPACE_NEEDED;
			}
			if (!((front1_s < BACKWARD_THRESHOLD) && (front2_s < BACKWARD_THRESHOLD))){
				if (smallest > navigation_status.max_dist_obs){
					navigation_status.current_state = GO_TO_TARGET;
					GPIO_write(Board_OBS_A_EN, 0);
				}else{
					navigation_status.current_state = AVOID_OBSTACLE;
				}
			}
    }
    else if (navigation_status.current_state == SPACE_NEEDED){
		if (!((left_s < BACKWARD_THRESHOLD/2) || (right_s < BACKWARD_THRESHOLD/2))){
			//Check if we face another type of obstacle or if path is free
			if (smallest < navigation_status.max_dist_obs){
				//We use a different strategy depending on the type of obstacle
				if (front1_s < BACKWARD_THRESHOLD && front2_s < BACKWARD_THRESHOLD){
					serial_printf(cli_stdout, "Wall ahead!\n");
					navigation_status.current_state = AVOID_WALL;
				}else
					navigation_status.current_state = AVOID_OBSTACLE;
			}else{
				navigation_status.current_state = GO_TO_TARGET;
				GPIO_write(Board_OBS_A_EN, 0);
			}
		}
    }
    else
    {
        //normal operation: continue on mission items
		if(mission_items.item[mission_items.current_index].current)
		{
			// Only go to target if mb is armed and rover must go to home OR sbc is armed and ready to record
#ifdef CONTINUE_WAYPOINTS_IMMEDIATELY
			if(comm_mainboard_armed() == 1){
#else
			if(comm_mainboard_armed() == 1 && (mission_items.current_index == 0 || comm_sbc_armed() == 1)){
#endif
				navigation_status.current_state = GO_TO_TARGET;
			}
			else
			{
				navigation_status.current_state = STOP;
			}
		}

		if(navigation_status.current_state == GO_TO_TARGET)
		{
			ultrasonic_get_distance(distance_values);
			if(navigation_status.distance_to_target < TARGET_REACHED_DISTANCE)
			{
				navigation_mission_item_reached();
			}
			else if (smallest < navigation_status.max_dist_obs)
			{
				GPIO_write(Board_OBS_A_EN, 1);

				//We use a different strategy depending on the type of obstacle
				if (front1_s < BACKWARD_THRESHOLD && front2_s < BACKWARD_THRESHOLD)
				{
					serial_printf(cli_stdout, "Wall ahead!\n");
					navigation_status.current_state = AVOID_WALL;
				}
				else if ((left_s < BACKWARD_THRESHOLD/2) || (right_s < BACKWARD_THRESHOLD/2))
				{
					navigation_status.current_state = SPACE_NEEDED;
				}
				else
					navigation_status.current_state = AVOID_OBSTACLE;
			}
		}
    }
}



#if defined (NAVIGATION_TEST)
void navigation_move();
#else
void navigation_move()
{
	static int32_t lspeed, rspeed;
	double angular = 0;

	//only move if not on halt AND state = go_to_target
	if(navigation_status.halt == 0)
	{
		if(navigation_status.current_state == GO_TO_TARGET)
		{
			angular = pid_update(&pid_a, navigation_status.angle_to_target) * PID_SCALING_FACTOR;

			if (angular > 0){
				lspeed = PWM_SPEED_100;
				rspeed = PWM_SPEED_100 - (int32_t)(angular);
				if (rspeed < PWM_MIN_CURVE_SPEED)
					rspeed = PWM_MIN_CURVE_SPEED;
			}else if (angular <= 0){
				rspeed = PWM_SPEED_100;
				lspeed = PWM_SPEED_100 + (int32_t)(angular);
				if (lspeed < PWM_MIN_CURVE_SPEED)
					lspeed = PWM_MIN_CURVE_SPEED;

			}
			motors_wheels_move((int32_t)lspeed, (int32_t)rspeed, (int32_t)lspeed, (int32_t)rspeed);
			navigation_status.motor_values[0] = lspeed; //backup
			navigation_status.motor_values[1] = rspeed;
		}
		else if(navigation_status.current_state == STOP)
		{
			motors_wheels_stop();
			navigation_status.motor_values[0] = 0;
			navigation_status.motor_values[1] = 0;
		}
//		else if(navigation_status.current_state == AVOID_OBSTACLE)
//		{
//			int32_t distance_values[N_ULTRASONIC_SENSORS];
//			ultrasonic_get_distance(distance_values);
//			ultrasonic_check_distance(distance_values, navigation_status.motor_values, PWM_SPEED_80); //80% of speed to move slower than in normal mode
//
//			motors_wheels_move(navigation_status.motor_values[0], navigation_status.motor_values[1],
//					navigation_status.motor_values[0], navigation_status.motor_values[1]);

//		}else if (navigation_status.current_state == AVOID_WALL){
//			// move backwards in opposite curving direction (TODO: to be tested)
//			lspeed = -navigation_status.motor_values[1];
//			rspeed = -navigation_status.motor_values[0];
//			motors_wheels_move((int32_t)lspeed, (int32_t)rspeed, (int32_t)lspeed, (int32_t)rspeed);
//		}else if (navigation_status.current_state == SPACE_NEEDED){
//			navigation_status.motor_values[0] = -PWM_SPEED_100;
//			navigation_status.motor_values[1] = -PWM_SPEED_100;
//			motors_wheels_move(navigation_status.motor_values[0], navigation_status.motor_values[1],
//					navigation_status.motor_values[0], navigation_status.motor_values[1]);
//		}
	}
	else
	{
		navigation_status.motor_values[0] = 0;
		navigation_status.motor_values[1] = 0;
		motors_wheels_stop();
	}

		//		serial_printf(cli_stdout, "speed l=%d, r=%d \n", motor_values[0], motor_values[1]);

}
#endif

void navigation_change_gain(char pid, char type, float gain)
{
	if (pid == 'a'){
		if(type == 'p')
			pid_a.pGain = gain;
		else if (type == 'i')
			pid_a.iGain = gain;
		else if (type == 'd')
			pid_a.dGain = gain;
	}
}

#if defined (NAVIGATION_TEST)
void navigation_init();
#else
void navigation_init()
{
	cli_init();
	motors_init();
	ultrasonic_init();
	pid_init(&pid_a, PGAIN_A, IGAIN_A, DGAIN_A, MOTOR_IMAX, MOTOR_IMIN);
	navigation_status.lat_rover = INIT_LAT;
	navigation_status.lon_rover = INIT_LON;
	navigation_status.heading_rover = 0.0;
	navigation_status.lat_target = TARGET_LAT;
	navigation_status.lon_target = TARGET_LON;
	navigation_status.distance_to_target = 0.0;
	navigation_status.angle_to_target = 0.0;
	navigation_status.current_state = STOP;
	navigation_status.max_dist_obs = OBSTACLE_MAX_DIST;
	navigation_status.halt = 0;
	navigation_status.position_valid = false;
	GPIO_write(Board_OBS_A_EN, 1);

	mission_items.count = 0;
}
#endif

void navigation_restore_mission_items(mission_item_list_t item_list)
{
	int i = 0;
	mission_items.count = item_list.count;
	mission_items.current_index = item_list.current_index;
	for(i=0;i<item_list.count;i++){
		mission_items.item[i].param1 = item_list.item[i].param1;
		mission_items.item[i].param2 = item_list.item[i].param2;
		mission_items.item[i].param3 = item_list.item[i].param3;
		mission_items.item[i].param4 = item_list.item[i].param4;
		mission_items.item[i].x = item_list.item[i].x;
		mission_items.item[i].y = item_list.item[i].y;
		mission_items.item[i].z = item_list.item[i].z;
		mission_items.item[i].seq = item_list.item[i].seq;
		mission_items.item[i].command = item_list.item[i].command;
		mission_items.item[i].target_system = item_list.item[i].target_system;
		mission_items.item[i].target_component = item_list.item[i].target_component;
		mission_items.item[i].frame = item_list.item[i].frame;
		mission_items.item[i].current = item_list.item[i].current;
		mission_items.item[i].autocontinue = item_list.item[i].autocontinue;

	}
}

void navigation_task()
{
	bool logging_enabled = false;
	navigation_init();

#ifdef FLASH_ENABLED
	if (flash_init() == 0) {
		logging_enabled = true;
	}

    if (logging_enabled) {
        if (!log_init()) {
            serial_printf(cli_stdout, "log_init failed\n");
            log_reset();
        }
    }

    //We look if we have mavlink mission item logged, in case we just suffered a crash

    uint32_t time = 0;
    char name[4]; //Name is a 3 characters long string
    mission_item_list_t item_list;
    if(log_read_mavlink_item_list(&item_list, &time, &name, &pos_counter))
    	navigation_restore_mission_items(item_list);

#endif

    unsigned int loops_since_stop = 0;
	while(1){

//		navigation_update_target();
		navigation_update_current_target();
		navigation_update_position();
		navigation_update_state();
		navigation_arm_disarm();
		navigation_move();

		if(navigation_rover_moving() || loops_since_stop < 62) // trasmit messages after stop for a duration such that
																//at least one msg with speed zero is tx'd via LoRa
		{
			comm_mavlink_broadcast(navigation_pack_mavlink_hud());
			comm_mavlink_broadcast(navigation_pack_rc_channels_scaled());
		}

		if(!navigation_rover_moving())
			loops_since_stop++;
		else
			loops_since_stop = 0;


		Task_sleep(500);

	}
}

