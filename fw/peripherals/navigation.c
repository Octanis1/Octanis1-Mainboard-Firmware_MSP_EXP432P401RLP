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

#endif
#define M_PI 3.14159265358979323846
#define EARTH_RADIUS 6356752.3
#define INIT_LAT 45.531993
#define INIT_LON 6.591617
#define TARGET_LAT 0
#define TARGET_LON 6.591798
#define TARGET_REACHED_DISTANCE 1

#define PGAIN_A 1
#define IGAIN_A 0
#define DGAIN_A 0
#define MOTOR_IMAX 40
#define MOTOR_IMIN 0
#define BACKWARD_THRESHOLD 200 //about 1m

typedef struct _navigation_status{
	float lat_rover;
	float lon_rover;
	float heading_rover;
	float lat_target;
	float lon_target;
	float distance_to_target;
	float angle_to_target;
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
static target_list_t navigation_targets; //DEPRECATED!
static pid_controler_t pid_a;

static mission_item_list_t mission_items;


float navigation_get_angle_to_target()
{
	return navigation_status.angle_to_target;
}

float navigation_degree_to_rad(float degree)
{
    float rad = degree/360 *2*M_PI;
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
    float angle = navigation_angle_to_target(lat1, lon1, lat2, lon2);
    angle = angle - headX;
    return angle;
}

/******** NEW functions to handle mavlink mission items **********/

static uint16_t mission_item_rx_seq_id; // temporary sequence number during MISSION_ITEM exchange
static uint16_t mission_item_rx_count;

COMM_MAV_RESULT navigation_rx_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	//note: If a waypoint planner component receives WAYPOINT messages outside of transactions it answers with a WAYPOINT_ACK message.

	uint8_t next_index = (mission_items.last_index + 1) % N_TARGETS_MAX;
	if(next_index == mission_items.current_index &&
			(mission_items.state[mission_items.current_index] == ITEM_CURRENT ||
			mission_items.state[mission_items.current_index] == ITEM_VALID))
	{ //do not overwrite valid mission item!
		//TODO: reply with MAV_CMD_ACK_ERR_FAIL because buffer is full.
		return NO_ANSWER; //for now we just don't answer -->timeout. BUT: list is still stored!!
	}
	else
	{ // buffer has space --> add to list
		mavlink_msg_mission_item_decode(msg, &(mission_items.item[next_index]));
		mission_items.last_index = next_index;
		mission_items.state[next_index] = ITEM_VALID;

		mission_item_rx_seq_id++; //increase sequence id
		if(mission_item_rx_count > mission_item_rx_seq_id) // request next item
		{
			mavlink_msg_mission_request_pack(mavlink_system.sysid, target->component, answer_msg,
					msg->sysid, target->component, mission_item_rx_seq_id);
		}
		else
		{
			mavlink_msg_mission_ack_pack(mavlink_system.sysid, target->component, answer_msg,
					msg->sysid, target->component, 	MAV_MISSION_ACCEPTED);
		}

		return REPLY_TO_SENDER;

	}

}

// function to call at the beginning of a mission item write transaction (i.e. after receiving MISSION_COUNT)
COMM_MAV_RESULT navigation_rx_mission_items_start(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	mission_item_rx_seq_id = 0;
	mission_item_rx_count = mavlink_msg_mission_count_get_count(msg);

	/* send mission acknowledge */
	mavlink_msg_mission_request_pack(mavlink_system.sysid, target->component, answer_msg,
			msg->sysid, target->component, mission_item_rx_seq_id);

	return REPLY_TO_SENDER;
}


static uint16_t mission_item_tx_seq_id; // temporary sequence number during MISSION_ITEM exchange
static uint16_t mission_item_tx_count;

// function to call after a MISSION_REQUEST_LIST command
COMM_MAV_RESULT navigation_tx_mission_items_start(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	//compute number of mission items in buffer:
	mission_item_tx_count = (mission_items.last_index + 1 + N_TARGETS_MAX - mission_items.current_index) % N_TARGETS_MAX;

	mavlink_msg_mission_count_pack(mavlink_system.sysid, target->component, answer_msg,
			msg->sysid, target->component, mission_item_tx_count);

	mission_item_tx_seq_id = 0;

	return REPLY_TO_SENDER;
}

COMM_MAV_RESULT navigation_tx_mission_item(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg, mavlink_message_t *answer_msg)
{
	mission_item_tx_seq_id = mavlink_msg_mission_request_get_seq(msg);
	uint16_t item_index = (mission_items.current_index + mission_item_tx_seq_id) % N_TARGETS_MAX;

	// this function packs the already available struct.
	mavlink_msg_mission_item_encode(mavlink_system.sysid, target->component, answer_msg, &(mission_items.item[item_index]));

	return REPLY_TO_SENDER;
}


/******** DEPRECATED FUNCTIONS FOR MANAGING GPS TARGETS VIA SERIAL *******/
//TODO: REMOVE
uint8_t navigation_remove_newest_target()
{
	uint8_t previous_index = (navigation_targets.last_index + N_TARGETS_MAX -1) % N_TARGETS_MAX;

	if(navigation_targets.last_index == navigation_targets.current_index)
	{
		navigation_status.current_state = STOP; //as we delete the current target
		previous_index = navigation_targets.last_index;
	}
	navigation_targets.state[navigation_targets.last_index] = INVALID;
	navigation_targets.last_index = previous_index;

	return 1;

}

uint8_t navigation_add_target(float new_lat, float new_lon, uint8_t new_id)
{
	uint8_t next_index = (navigation_targets.last_index + 1) % N_TARGETS_MAX;
	if(next_index == navigation_targets.current_index)
	{
		return 0;
	}
	else
	{
		navigation_targets.last_index = next_index;
		navigation_targets.lat[next_index] = new_lat;
		navigation_targets.lon[next_index] = new_lon;
		navigation_targets.id[next_index] = new_id;
		navigation_targets.state[next_index] = VALID;

		return 1;
	}
}

uint8_t navigation_add_target_from_string(char* targetstring, int stringlength)
{
	char ch=targetstring[0];
	char * p = &targetstring[1];

	float lat = a2f(ch, &p ,10);

	ch=*p++;

	float lon = a2f(ch, &p ,10);

	ch=*p++;

	int id;
	a2i(ch, &p, 10,&id);

	return navigation_add_target(lat, lon, id);
}

/******** END DEPRECATED FUNCTIONS FOR MANAGING GPS TARGETS VIA SERIAL *******/


#if defined (NAVIGATION_TEST)
uint8_t navigation_bypass(char command, uint8_t index);
#else
uint8_t navigation_bypass(char command, uint8_t index)
{
	if(command == 'n')
	{
		navigation_status.current_state = STOP;
		return 1;
	}
	else if(index>0 && index<5)
	{
		static int8_t strut_speed[4]; //initialized as zero at the first time
		if(command == 'h')
			strut_speed[index-1] = 0;
		else if(command == 'u')
			strut_speed[index-1] = 1;
		else if(command == 'd')
			strut_speed[index-1] = -1;
		else
			return 0;

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

/* fetch the next target to move to from the queue */
void navigation_update_target()
{
	if(navigation_targets.state[navigation_targets.current_index] == DONE)
	{
		navigation_targets.current_index = (navigation_targets.current_index + 1) % N_TARGETS_MAX;
	}

	if(navigation_targets.state[navigation_targets.current_index] == VALID)
	{
		navigation_status.lon_target = navigation_targets.lon[navigation_targets.current_index];
		navigation_status.lat_target = navigation_targets.lat[navigation_targets.current_index];
		navigation_targets.state[navigation_targets.current_index] = CURRENT;
	}
	else if(navigation_targets.state[navigation_targets.current_index] == INVALID &&
			navigation_targets.current_index != navigation_targets.last_index)
	{//search queue for valid goals, until whole buffer is cycled
		navigation_targets.current_index = (navigation_targets.current_index + 1) % N_TARGETS_MAX;
	}
}

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

	navigation_status.angle_valid = (imu_get_calib_status()>=6);

	// recalculate heading angle
	navigation_status.heading_rover = imu_get_fheading();
	navigation_status.angle_to_target = navigation_angle_for_rover(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target, navigation_status.heading_rover);
	// recalculate distance to target
	navigation_status.distance_to_target = navigation_dist_to_target(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target);
}
#endif

void navigation_update_state()
{
	int32_t distance_values[N_ULTRASONIC_SENSORS];
	int i = 0;
	if(navigation_status.current_state == BYPASS){
		//motor command is done directly in CLI
	}else if(navigation_status.current_state == AVOID_OBSTACLE){
		ultrasonic_get_distance(distance_values);

//		for (i=0;i<N_ULTRASONIC_SENSORS;i++)
//			serial_printf(stdout, "US %d val : %d \n", i, distance_values[i]);

		if (ultrasonic_get_smallest (distance_values, N_ULTRASONIC_SENSORS) > navigation_status.max_dist_obs){
			navigation_status.current_state = GO_TO_TARGET;
			GPIO_write(Board_OBS_A_EN, 0);
		}else if ((distance_values[US_LEFT] < BACKWARD_THRESHOLD/2) || (distance_values[US_RIGHT] < BACKWARD_THRESHOLD/2)){
			navigation_status.current_state = SPACE_NEEDED;
		}else if (distance_values[US_FRONT_1] < BACKWARD_THRESHOLD && distance_values[US_FRONT_2] < BACKWARD_THRESHOLD){
			serial_printf(stdout, "Wall ahead!\n");
			navigation_status.current_state = AVOID_WALL;
		}

	}else if (navigation_status.current_state == AVOID_WALL){
		ultrasonic_get_distance(distance_values);

		//We check if we're not facing the wall anymore
//		for (i=0;i<N_ULTRASONIC_SENSORS;i++)
//			serial_printf(stdout, "US %d val : %d \n", i, distance_values[i]);
		if ((distance_values[US_LEFT] < BACKWARD_THRESHOLD/2) || (distance_values[US_RIGHT] < BACKWARD_THRESHOLD/2)){
			navigation_status.current_state = SPACE_NEEDED;
		}
		if (!((distance_values[US_FRONT_1] < BACKWARD_THRESHOLD) && (distance_values[US_FRONT_2] < BACKWARD_THRESHOLD))){

			if (ultrasonic_get_smallest (distance_values, N_ULTRASONIC_SENSORS) > navigation_status.max_dist_obs){
				navigation_status.current_state = GO_TO_TARGET;
				GPIO_write(Board_OBS_A_EN, 0);

			}else{
				navigation_status.current_state = AVOID_OBSTACLE;
			}
		}

	}else if (navigation_status.current_state == SPACE_NEEDED){
		ultrasonic_get_distance(distance_values);
		if (!((distance_values[US_LEFT] < BACKWARD_THRESHOLD/2) || (distance_values[US_RIGHT] < BACKWARD_THRESHOLD/2))){
			//Check if we face another type of obstacle or if path is free
			if (ultrasonic_get_smallest (distance_values, N_ULTRASONIC_SENSORS) < navigation_status.max_dist_obs){
				//We use a different strategy depending on the type of obstacle
				if (distance_values[US_FRONT_1] < BACKWARD_THRESHOLD && distance_values[US_FRONT_2] < BACKWARD_THRESHOLD){
					serial_printf(stdout, "Wall ahead!\n");
					navigation_status.current_state = AVOID_WALL;
				}else
					navigation_status.current_state = AVOID_OBSTACLE;
			}else{
				navigation_status.current_state = GO_TO_TARGET;
				GPIO_write(Board_OBS_A_EN, 0);
			}
		}
	}else{
		if(navigation_targets.state[navigation_targets.current_index] == CURRENT)
		{ //TODO: add conditions that may stop from changing state, for example low battery.

			if(navigation_status.angle_valid)
			{
				navigation_status.current_state = GO_TO_TARGET;
			}
			else
			{
//				serial_printf(stdout, "Calibrate IMU (status: %d/9)\n",imu_get_calib_status());
				int i;
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

		if(navigation_status.current_state == GO_TO_TARGET)
		{
			ultrasonic_get_distance(distance_values);
//			for (i=0;i<N_ULTRASONIC_SENSORS;i++)
//				serial_printf(stdout, "US %d val : %d \n", i, distance_values[i]);
			if(navigation_status.distance_to_target < TARGET_REACHED_DISTANCE)
			{
				navigation_targets.state[navigation_targets.current_index] = DONE;
				navigation_status.current_state = STOP;
			}else if (ultrasonic_get_smallest (distance_values, N_ULTRASONIC_SENSORS) < navigation_status.max_dist_obs){
				GPIO_write(Board_OBS_A_EN, 1);

				//We use a different strategy depending on the type of obstacle
				if (distance_values[US_FRONT_1] < BACKWARD_THRESHOLD && distance_values[US_FRONT_2] < BACKWARD_THRESHOLD){
					serial_printf(stdout, "Wall ahead!\n");
					navigation_status.current_state = AVOID_WALL;
				}else if ((distance_values[US_LEFT] < BACKWARD_THRESHOLD/2) || (distance_values[US_RIGHT] < BACKWARD_THRESHOLD/2)){
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
		angular = pid_update(&pid_a, navigation_status.angle_to_target);

		if (angular > 0){
			lspeed = PWM_SPEED_100;
			rspeed = PWM_SPEED_100 - angular;
			if (rspeed < PWM_SPEED_70)
				rspeed = PWM_SPEED_70;
		}else if (angular <= 0){
			rspeed = PWM_SPEED_100;
			lspeed = PWM_SPEED_100 + angular;
			if (lspeed < PWM_SPEED_70)
				lspeed = PWM_SPEED_70;

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

	//		serial_printf(stdout, "speed l=%d, r=%d \n", motor_values[0], motor_values[1]);

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
	navigation_status.current_state = AVOID_OBSTACLE;
	navigation_status.max_dist_obs = OBSTACLE_MAX_DIST;
	GPIO_write(Board_OBS_A_EN, 1);

	int i=navigation_add_target(TARGET_LAT, TARGET_LON, 0); //fill initial target to list (TODO:remove deprecated)
	mission_items.last_index = N_TARGETS_MAX - 1; //put last item index to -1 such that the first written element will be at index 0

}
#endif

void navigation_task()
{
	navigation_init();

	while(1){

		navigation_update_target();
		navigation_update_position();
		navigation_update_state();
		navigation_move();


		Task_sleep(500);

	}
}

