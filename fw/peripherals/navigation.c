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
#include "hal/ultrasonic.h"
#include <math.h>
#include "../lib/printf.h"
#include "pid.h"
#include "../lib/serial_printf.h"

//Logging includes:
#include "../core/log.h"
#include "../core/log_entries.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include "flash.h"
#include "hal/spi_helper.h"

#endif
#define M_PI 3.14159265358979323846
#define EARTH_RADIUS 6356752.3
#define INIT_LAT 0
#define INIT_LON 0
#define TARGET_LAT 1
#define TARGET_LON 1
#define TARGET_REACHED_DISTANCE 2 //meters

#define PGAIN_A 1
#define IGAIN_A 0
#define DGAIN_A 0
#define MOTOR_IMAX 40
#define MOTOR_IMIN 0
#define BACKWARD_THRESHOLD 200 //about 1m

#define MIN_IMU_CALIB_STATUS	6 //to have angle valid = true

typedef struct _navigation_status{
	float lat_rover;
	float lon_rover;
	float heading_rover;
	float lat_target;
	float lon_target;
	float distance_to_target;
	float angle_to_target; // [-180, 180]. Positive means target is located to the right
	float max_dist_obs;
	int32_t motor_values[2]; // current motor speed
	uint8_t angle_valid;
	enum _current_state{
		STOP=0,
		BYPASS,
		GO_TO_TARGET,
		AVOID_OBSTACLE,
		AVOID_WALL,
		SPACE_NEEDED,
	} current_state;
} navigation_status_t;

static navigation_status_t navigation_status;
static pid_controler_t pid_a;

static mission_item_list_t mission_items;

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

float navigation_get_lat_rover()
{
	return navigation_status.lat_rover;
}

float navigation_get_lon_rover()
{
	return navigation_status.lon_rover;
}

float navigation_get_heading_rover()
{
	return navigation_status.heading_rover;
}

float navigation_get_lat_target()
{
	return navigation_status.lat_target;
}

float navigation_get_lon_target()
{
	return navigation_status.lon_target;
}

float navigation_get_distance_to_target()
{
	return navigation_status.distance_to_target;
}

float navigation_get_angle_to_target()
{
	return navigation_status.angle_to_target;
}

float navigation_get_max_dist_obs()
{
	return navigation_status.max_dist_obs;
}

uint8_t navigation_get_angle_valid()
{
	return navigation_status.angle_valid;
}

enum _current_state navigation_get_current_state()
{
	return navigation_status.current_state;
}

float navigation_degree_to_rad(float degree)
{
    float rad = degree/180*M_PI;
    return rad;
}

void navigation_set_max_dist(float max_dist)
{
	navigation_status.max_dist_obs = max_dist;
}

float navigation_dist_to_target(float lat_current, float lon_current, float lat_target, float lon_target){

	//The gps gives the coordinate in degree
    lat_current = navigation_degree_to_rad(lat_current);
    lat_target = navigation_degree_to_rad(lat_target);
    lon_current = navigation_degree_to_rad(lon_current);
    lon_target = navigation_degree_to_rad(lon_target);

    float delta_lat = lat_target - lat_current;
    float delta_lon = lon_target - lon_current;

    float a = sinf(delta_lat/2)*sinf(delta_lat/2) + cosf(lat_current)*cosf(lat_target) * sinf(delta_lon/2)*sinf(delta_lon/2);
    
    float distance = 2*EARTH_RADIUS*asinf(sqrtf(a));
    return distance;
}

/**
 * Params: lat1, long1 => Latitude and Longitude of current point
 *         lat2, long2 => Latitude and Longitude of target  point
 *
 * Returns the degree of a direction from current point to target point (between -180, 180).
 * Negative value are toward East, positive West
 */
float navigation_angle_to_target(float lat1, float lon1, float lat2, float lon2) {
    float dLon = navigation_degree_to_rad(lon2-lon1);

    lat1 = navigation_degree_to_rad(lat1);
    lat2 = navigation_degree_to_rad(lat2);
    
    float y = sinf(dLon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1)*cosf(lat2)*cosf(dLon);
    float brng = 180/M_PI*(atan2f(y, x));

    return brng;
}

float navigation_angle_for_rover(float lat1, float lon1, float lat2, float lon2, float headX) {
    float bearing = navigation_angle_to_target(lat1, lon1, lat2, lon2);
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
    if(fabsf(bearing) > 170.0)
    {
    		bearing=copysignf(bearing, navigation_status.angle_to_target);
    }

    return bearing;
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
		}
		else
		{
			navigation_status.current_state = STOP;
			return NO_ANSWER;
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
	comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_MISSIONPLANNER);
	comm_mavlink_broadcast(&(msg_frame));


	mission_items.item[mission_items.current_index].current = 0;
	if(mission_items.item[mission_items.current_index].autocontinue)
	{
		if(navigation_next_mission_item(NULL, NULL, &(msg_frame.mavlink_message)) == REPLY_TO_SENDER)
		{
			comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_MISSIONPLANNER);
			comm_mavlink_broadcast(&(msg_frame));
		}
	}
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
		switch(command)
		{
		case 'f': motors_wheels_move(PWM_SPEED_100, PWM_SPEED_100, PWM_SPEED_100, PWM_SPEED_100);break;
		case 'b': motors_wheels_move(-PWM_SPEED_100, -PWM_SPEED_100, -PWM_SPEED_100, -PWM_SPEED_100);break;
		case 'l': motors_wheels_move(PWM_SPEED_80, PWM_SPEED_100, PWM_SPEED_80, PWM_SPEED_100);break;
		case 'r': motors_wheels_move(PWM_SPEED_100, PWM_SPEED_80, PWM_SPEED_100, PWM_SPEED_80);break;
		case 'x': motors_wheels_stop();break;
		default: return 0;
		}
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
	if(gps_update_new_position(&(navigation_status.lat_rover), &(navigation_status.lon_rover)))
	{
		//we get a new gps position
	}
	else
	{
		//we didn't get a new gps position --> update position using odometry.
		//TODO
		motors_wheels_update_distance();

		motors_struts_get_position();
	}

	navigation_status.angle_valid = (imu_get_calib_status() >= MIN_IMU_CALIB_STATUS);

	// recalculate heading angle
	navigation_status.heading_rover = imu_get_fheading();
	navigation_status.angle_to_target = navigation_angle_for_rover(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target, navigation_status.heading_rover);
	// recalculate distance to target
	navigation_status.distance_to_target = navigation_dist_to_target(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target);
}
#endif


void navigation_update_current_target()
{
	if(mission_items.current_index < mission_items.count)
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
    int i = 0;

    ultrasonic_get_distance(distance_values);
    smallest = ultrasonic_get_smallest(distance_values, N_ULTRASONIC_SENSORS);
    left_s = distance_values[US_LEFT];
    right_s = distance_values[US_RIGHT];
    front1_s = distance_values[US_FRONT_1];
    front2_s = distance_values[US_FRONT_2];

    if(navigation_status.current_state == BYPASS){
		//motor command is done directly in CLI
    }else if(navigation_status.current_state == AVOID_OBSTACLE){

        if (smallest > navigation_status.max_dist_obs){
        	navigation_status.current_state = GO_TO_TARGET;
        	GPIO_write(Board_OBS_A_EN, 0);
        }else if ((left_s < BACKWARD_THRESHOLD/2) || (right_s < BACKWARD_THRESHOLD/2)){
        	navigation_status.current_state = SPACE_NEEDED;
        }else if (front1_s < BACKWARD_THRESHOLD && front2_s < BACKWARD_THRESHOLD){
        	serial_printf(cli_stdout, "Wall ahead!\n");
        	navigation_status.current_state = AVOID_WALL;
        }

    }else if (navigation_status.current_state == AVOID_WALL){

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

    }else if (navigation_status.current_state == SPACE_NEEDED){
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
    }else{ 
        //normal operation: continue on mission items
    	if(mission_items.item[mission_items.current_index].current){
    		//TODO: add conditions that may stop from changing state, for example low battery.

    		if(navigation_status.angle_valid){
    			navigation_status.current_state = GO_TO_TARGET;
    		}else{
                //serial_printf(cli_stdout, "Calibrate IMU (status: %d/9)\n",imu_get_calib_status());
    			for (i = 0; i<(7-imu_get_calib_status());i++) //blink shorter for better calibration
    			{
    				GPIO_toggle(Board_LED_RED);
    				Task_sleep(50);
    				GPIO_toggle(Board_LED_RED);
    				Task_sleep(50);
    			}
    			GPIO_write(Board_LED_RED,0);
    		}
    	}

    	if(navigation_status.current_state == GO_TO_TARGET){
    		ultrasonic_get_distance(distance_values);
    		if(navigation_status.distance_to_target < TARGET_REACHED_DISTANCE){
    			navigation_mission_item_reached();
    		}else if (smallest < navigation_status.max_dist_obs){
    			GPIO_write(Board_OBS_A_EN, 1);

    			//We use a different strategy depending on the type of obstacle
    			if (front1_s < BACKWARD_THRESHOLD && front2_s < BACKWARD_THRESHOLD){
    				serial_printf(cli_stdout, "Wall ahead!\n");
    				navigation_status.current_state = AVOID_WALL;
    			}else if ((left_s < BACKWARD_THRESHOLD/2) || (right_s < BACKWARD_THRESHOLD/2)){
    				navigation_status.current_state = SPACE_NEEDED;
    			}else
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
	float angular = 0;

	if(navigation_status.current_state == GO_TO_TARGET)
	{
		angular = pid_update(&pid_a, navigation_status.angle_to_target) * PID_SCALING_FACTOR;

		if (angular > 0){
			lspeed = PWM_SPEED_100;
			rspeed = PWM_SPEED_100 - (int32_t)(angular);
			if (rspeed < PWM_SPEED_80)
				rspeed = PWM_SPEED_80;
		}else if (angular <= 0){
			rspeed = PWM_SPEED_100;
			lspeed = PWM_SPEED_100 + (int32_t)(angular);
			if (lspeed < PWM_SPEED_80)
				lspeed = PWM_SPEED_80;

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
	else if(navigation_status.current_state == AVOID_OBSTACLE)
	{
		int32_t distance_values[N_ULTRASONIC_SENSORS];
		ultrasonic_get_distance(distance_values);
		ultrasonic_check_distance(distance_values, navigation_status.motor_values, PWM_SPEED_80); //80% of speed to move slower than in normal mode

		motors_wheels_move(navigation_status.motor_values[0], navigation_status.motor_values[1],
				navigation_status.motor_values[0], navigation_status.motor_values[1]);

	}else if (navigation_status.current_state == AVOID_WALL){
		// move backwards in opposite curving direction (TODO: to be tested)
		lspeed = -navigation_status.motor_values[1];
		rspeed = -navigation_status.motor_values[0];
		motors_wheels_move((int32_t)lspeed, (int32_t)rspeed, (int32_t)lspeed, (int32_t)rspeed);
	}else if (navigation_status.current_state == SPACE_NEEDED){
		navigation_status.motor_values[0] = -PWM_SPEED_100;
		navigation_status.motor_values[1] = -PWM_SPEED_100;
		motors_wheels_move(navigation_status.motor_values[0], navigation_status.motor_values[1],
				navigation_status.motor_values[0], navigation_status.motor_values[1]);
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
	GPIO_write(Board_OBS_A_EN, 1);

	mission_items.count = 0;
}
#endif

uint8_t item_compare (mavlink_mission_item_t item1, mavlink_mission_item_t item2)
{
	uint8_t ret = 1;
	if (item1.param1 != item2.param1)
		ret = 0;
	if (item1.param2 != item2.param2)
			ret = 0;
	if (item1.param3 != item2.param3)
			ret = 0;
	if (item1.param4 != item2.param4)
			ret = 0;
	if (item1.x != item2.x)
			ret = 0;
	if (item1.y != item2.y)
			ret = 0;
	if (item1.z != item2.z)
			ret = 0;
	if (item1.command != item2.command)
			ret = 0;
	if (item1.seq != item2.seq)
			ret = 0;
	if (item1.current != item2.current)
			ret = 0;
	if (item1.autocontinue != item2.autocontinue)
			ret = 0;
	if (item1.frame != item2.frame)
			ret = 0;
	if (item1.target_component != item2.target_component)
			ret = 0;
	if (item1.target_system != item2.target_system)
			ret = 0;
	return ret;
}

mavlink_mission_item_t item_init(uint8_t preset)
{
	mavlink_mission_item_t item;

	if (preset == 1)
	{
		item.param1 = 1.1;
		item.param1 = 2.2;
		item.param1 = 3.3;
		item.param1 = 4.4;
		item.x = 5.5;
		item.y = 6.6;
		item.z = 7.7;
		item.command = 8;
		item.seq = 9;
		item.current = 10;
		item.autocontinue = 11;
		item.frame = 12;
		item.target_component = 13;
		item.target_system = 14;
	}
	else if (preset ==2)
	{
		item.param1 = 2.1;
		item.param1 = 3.2;
		item.param1 = 4.3;
		item.param1 = 5.4;
		item.x = 6.5;
		item.y = 7.6;
		item.z = 8.7;
		item.command = 9;
		item.seq = 10;
		item.current = 11;
		item.autocontinue = 12;
		item.frame = 13;
		item.target_component = 14;
		item.target_system = 15;

	}
	else
	{
		item.param1 = 0;
		item.param1 = 0;
		item.param1 = 0;
		item.param1 = 0;
		item.x = 0;
		item.y = 0;
		item.z = 0;
		item.command = 0;
		item.seq = 0;
		item.current = 0;
		item.autocontinue = 0;
		item.frame = 0;
		item.target_component = 0;
		item.target_system = 0;
	}
	return item;
}

void navigation_task()
{
	navigation_init();

	/************* flash test START ****************/
	spi_helper_init_handle();

	// force enable logging
	bool logging_enabled = true;


	static uint8_t buf[250];
	flash_id_read(buf);
	const uint8_t flash_id[] = {0x01,0x20,0x18}; // S25FL127S ID
	if (memcmp(buf, flash_id, sizeof(flash_id)) == 0) {
		// flash answers with correct ID
		serial_printf(cli_stdout, "Flash ID OK\n");
	} else {
		serial_printf(cli_stdout, "Flash ID ERROR\n");
	}

    if (logging_enabled) {
        if (!log_init()) {
            serial_printf(cli_stdout, "log_init failed\n");
            log_reset();
        }
    }
    //serial_printf(cli_stdout, "log position 0x%x\n", log_write_pos());
	/************* flash test END ****************/
    char name[4];
    uint32_t time = 0;
    mission_item_list_t debug_list;
	while(1){

//		navigation_update_target();
//		navigation_update_current_target();
//		navigation_update_position();
//		navigation_update_state();
//		navigation_move();

    	mission_items.count = 1;
    	mission_items.current_index = 0;
    	mission_items.item[0] = item_init(1);
    	mission_items.item[1] = item_init(2);
    	log_write_mavlink_item_list();
    	log_read_mavlink_item_list(&debug_list, &time, name);
    	if (item_compare(mission_items.item[0], debug_list.item[0]))
    		serial_printf(cli_stdout, "item logging success\n");
    	else
    		serial_printf(cli_stdout, "item logging failed\n");
		Task_sleep(500);

	}
}

