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
#include <string.h>
#include "hal/time_since_boot.h"
#include "hal/adc.h"

//Logging includes:
#include "../core/log.h"
#include "../core/log_entries.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include "flash.h"
#include "hal/spi_helper.h"

#endif
#define M_PI 3.14159265358979323846
#define PERIOD 6.28318530717958647693
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

//odometry definitions
#define VMOT_TO_RPS  0.000683333  //initialization constants revised by system after each GPS update
#define INITIAL_TURNING_CONSTANT 1
#define INITIAL_DISTANCE_CONST	 1
#define INITIAL_ANGLE_CONSTANT   1
#define TRUE					 1
#define FALSE					 0

#define BLOCKED_THRESHOLD		 10000000000000  //defined by user //useless values for testing
#define AIR_THRESHOLD			 0
#define MISSION_LATITUDE		 0

#define Y_TO_LATITUDE			 110946.257352 //true constants
#define KILO					 1000
#define E_FOUR					 10000
#define MEGA					 1000000
#define E_SEVEN					 10000000
#define PERIOD_DEGREES			 360		   //degrees in full period
#define WHEEL_DISTANCE			 440 		   //in mm

#define SENSOR_VALUES_UNAVAILABLE

struct odo{
	int32_t first_third_x;			//in mm
	int32_t second_third_x;			//in mm
	int32_t third_third_x;			//in mm
	int32_t first_third_y;			//in mm
	int32_t second_third_y;			//in mm
	int32_t third_third_y;			//in mm
	int32_t radius;					//in mm
	int32_t angle;					//in m�
	int32_t first_third_heading;	//in m�
	int32_t second_third_heading;	//in m�
	int32_t third_third_heading;	//in m�
	int32_t latitude;					//in degrees
	int32_t longitude; 				//in degrees
	float checked_lat; 				//in degrees
	float checked_lon; 				//in degrees
	uint32_t odo_time;				//in msec
	int32_t velocity; 				//in mm/s
	float heading; 					//in rad
	uint8_t straight;				//TRUE or FALSE
} odo;

static float vmot2rps_factor = VMOT_TO_RPS;
static float angle_constant = INITIAL_ANGLE_CONSTANT;

void navigation_distance_odometer(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_WHEELS], uint8_t position_i);
void navigation_update_xy(uint8_t position_i);
void navigation_get_radius_and_angle(float speed[N_WHEELS], uint32_t delta_time, uint8_t position_i);
int navigation_navigation_error(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_WHEELS]);

typedef struct _navigation_status{
	int32_t lat_rover;
	int32_t lon_rover;
	float old_lat;  		 //used for recalibrating factors
	float old_lon;			 //used for recalibrating factors
	uint8_t position_i; 	 //variable for timing of gps / odometry data
	uint32_t heading_rover;	 //in m�
	float old_gps_heading;
	float lat_target;
	float lon_target;
	float distance_to_target;
	float angle_to_target; // [-180, 180]. Positive means target is located to the right
	float max_dist_obs;
	int32_t motor_values[2]; // current voltage to controll motor speed
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

/******** functions to control driving/armed state via MAVLINK **********/

MAV_RESULT navigation_halt_resume(COMM_MAV_MSG_TARGET *target, mavlink_message_t *msg)
{
	float hold_resume = mavlink_msg_command_long_get_param1(msg);
	if(hold_resume == MAV_GOTO_DO_HOLD)
	{
		navigation_status.halt = 1;
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
	if(navigation_status.position_valid)
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
	else
	{
		if(comm_sbc_armed())
			comm_arm_disarm_subsystems(0);
		comm_disarm_mainboard();
	}
}


// This function stores the information that "ARM/DISARM" has been received.
void navigation_rxcmd_arm_disarm(float arm_disarm)
{
	navigation_status.cmd_armed_disarmed = (arm_disarm==1);
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
			log_write_mavlink_item_list();
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

COMM_FRAME* navigation_pack_rc_channels_scaled()
{
	uint32_t msec = ms_since_boot();

	static COMM_FRAME frame;

	uint8_t port 		 = 0;
	int16_t chan7_scaled = 0;
	int16_t chan8_scaled = 0;
	uint8_t rssi 		 = 0;

	int16_t left_side = navigation_status.motor_values[0];
	int16_t right_side = navigation_status.motor_values[1];

	//get currents
	static uint16_t sensor_values[N_WHEELS];

	/* initialize to 0 to reset the running average inside the adc readout function */
	sensor_values[0] = 0;
	sensor_values[1] = 0;
	sensor_values[2] = 0;
	sensor_values[3] = 0;

	adc_read_motor_sensors(sensor_values);

	mavlink_msg_rc_channels_scaled_pack(mavlink_system.sysid, MAV_COMP_ID_PATHPLANNER, &(frame.mavlink_message),
			msec, port, left_side, right_side, sensor_values[0],sensor_values[1], sensor_values[2], sensor_values[3],
			chan7_scaled, chan8_scaled, rssi);

	return &frame;
}

// message that is continuously at each task call
COMM_FRAME* navigation_pack_mavlink_hud()
{
	// Mavlink heartbeat
	// Define the system type, in this case an airplane
	float airspeed = 0.; //TODO
	float groundspeed = odo.velocity;
	int16_t heading = imu_get_heading();
	uint16_t throttle = (navigation_status.motor_values[0] + navigation_status.motor_values[1]);
	float alt = (1000.0 * gps_get_int_altitude());//Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude.
	float climb = 0.; //TODO

	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message

	mavlink_msg_vfr_hud_pack(mavlink_system.sysid, MAV_COMP_ID_PATHPLANNER, &(frame.mavlink_message),
			airspeed, groundspeed, heading, throttle, alt, climb);

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
	float gps_latitude, gps_longitude;
	int32_t gps_heading, heading_rover;

	navigation_status.motor_values[0] = 50; //manually changing voltages
	navigation_status.motor_values[1] = 50;

	gps_heading = (gps_get_cog()/360) * PERIOD * E_SEVEN;
	//delta_heading = gps_heading - navigation_status.old_gps_heading;
	//navigation_status.old_gps_heading = gps_heading;

	//if first_time, initialize all structures
	/*
	if(first_time)
	{
		navigation_initialize();
		navigation_initialize_odometer();
		gps_initialize();
		first_time = 0;
	}
	*/

	gps_latitude = gps_get_latitude();
	gps_longitude = gps_get_longitude();

	//delta_lon = gps_longitude - navigation_status.old_lon;
	//delta_lat = gps_latitude - navigation_status.old_lat;

	gps_run_gps(navigation_status.position_i);  //saves a gps point

	if (navigation_status.position_i == (MAX_RECENT_VALUES-1))
	{
		//navigation_recalibrate_odometer(delta_lat, delta_lon, delta_heading);
		navigation_reinitialize_odometer(gps_heading);
		gps_calculate_position();	//calculates a gps position from a number of gps points
		gps_reset_gps();
		navigation_status.position_i = VALUES_AFTER_GPS_RESET;
	}
	else
	{
		navigation_status.position_i++;
	}

	navigation_run_odometer(navigation_status.motor_values, navigation_status.position_i);     //saves an odometer position

	navigation_status.lat_rover = gps_latitude + odo.latitude;
	navigation_status.lon_rover = gps_longitude + odo.longitude;

	gps_receive_lat_rover(navigation_status.lat_rover);
	gps_receive_lon_rover(navigation_status.lon_rover);

	/*motors_struts_get_position();*/

	//checks if position is valid
	navigation_status.position_valid = imu_valid() && gps_valid();

	// recalculate heading angle
#ifdef IMU_AVALABLE
	navigation_status.heading_rover = imu_get_fheading();
#endif
#ifndef IMU_AVALABLE
	gps_heading = 0;
	heading_rover = gps_heading + odo.heading;
	while (heading_rover > (PERIOD_DEGREES * KILO)){
		heading_rover -= (PERIOD_DEGREES * KILO);
	}
	while (heading_rover < 0) {
		heading_rover += (PERIOD_DEGREES * KILO);
	}
	navigation_status.heading_rover = heading_rover;
#endif
	navigation_status.angle_to_target = navigation_angle_for_rover(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target, navigation_status.heading_rover);
	// recalculate distance to target
	navigation_status.distance_to_target = navigation_dist_to_target(navigation_status.lat_rover,navigation_status.lon_rover,
			navigation_status.lat_target, navigation_status.lon_target);
}
#endif

void navigation_initialize()
{
	navigation_status.lon_rover = 0;
	navigation_status.lat_rover = 0;
	navigation_status.position_i = 1;
	navigation_status.old_lat = 0;
	navigation_status.old_lon = 0;
	navigation_status.heading_rover = 0;
}


int navigation_run_odometer(int32_t voltage[N_SIDES], uint8_t position_i)
{
	static uint16_t sensor_values[N_WHEELS];

	/* initialize to 0 to reset the running average inside the adc readout function */
	sensor_values[0] = 0;
	sensor_values[1] = 0;
	sensor_values[2] = 0;
	sensor_values[3] = 0;

	int i;

	for (i = 0; i < N_WHEELS; i++){
		sensor_values[i] = motors_get_sensor_values(i);
	}

	//check to see if a wheel is blocked or off the ground
	if(navigation_navigation_error(sensor_values, voltage)) {
		return 0;
	}
	else {
		//function to estimate the distance needs curvature.
		navigation_distance_odometer(sensor_values, voltage, position_i);
		return 1;
	}
}

int navigation_navigation_error(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_SIDES])
{
	int error = 0;
	int ratio[N_WHEELS];
	int i = 0;
	for (i=0; i<N_WHEELS; i++) {
		ratio[i] = sensor_values[i] / voltage[i%N_SIDES];
		if (ratio[i]>BLOCKED_THRESHOLD || ratio[i]<AIR_THRESHOLD) {
			error++;
		}
	}
	if (!error)
		return 0;
	else
		return 1;
}

void navigation_distance_odometer(uint16_t sensor_values[N_WHEELS], int32_t voltage[N_SIDES], uint8_t position_i)
{
	float x_to_longitude;
	uint32_t circumference, velocity, msec, delta_time = 0; //in mm and in um/s
	float speed[N_WHEELS], rps[N_WHEELS];
	uint8_t i = 0;

	circumference = WHEEL_RADIUS * 2 * M_PI;

#ifdef SENSOR_VALUES_AVAILABLE
	for (i=0; i<N_WHEELS; i++) {
		rps[i] = vmot2rps_factor * voltage[i%N_SIDES] / sensor_values[i];
		speed[i] = rps[i] * circumference;
		velocity += speed[i];
	}
#endif
#ifndef SENSOR_VALUES_AVAILABLE
	for (i=0; i<N_WHEELS; i++) {
		rps[i] = vmot2rps_factor * voltage[i%N_SIDES]; 		//rounds per second
		speed[i] = rps[i] * circumference;					//makes mm / s
		velocity += speed[i];
	}
#endif

	velocity = velocity / N_WHEELS;
	odo.velocity = velocity;
	msec = ms_since_boot();
	delta_time = msec - odo.odo_time;
	odo.odo_time = msec;
	navigation_get_radius_and_angle(speed, delta_time, position_i);
	navigation_update_xy(position_i);

	//convert distance to polar coordinates
	x_to_longitude = Y_TO_LATITUDE * cos(MISSION_LATITUDE);
	odo.longitude = (odo.first_third_x + odo.second_third_x + odo.third_third_x) * 10000 / (x_to_longitude);
	odo.latitude = (odo.first_third_y + odo.second_third_y + odo.third_third_y) * 10000 / (Y_TO_LATITUDE); //10^7 degrees for gps message and precision
}

void navigation_get_radius_and_angle(float speed[N_WHEELS], uint32_t delta_time, uint8_t position_i)
{
	float ratio;
	int32_t radius, angle; //radius in mm and angle in m�

	ratio = ((1000 * speed[0]) / speed[1])/1000;

	if (speed[0] != speed[1]){
		radius = (WHEEL_DISTANCE / 2) * (ratio + 1)/(ratio - 1);					    //[mm]
		angle = odo.velocity * delta_time * PERIOD_DEGREES / (2 * radius * M_PI);		//mm/s * msec / mm = [m�]
		odo.straight = FALSE;
	}
	else {
		angle = 0;
		radius = speed[0] * delta_time / KILO;		//[mm] int * uint
		odo.straight = TRUE;
	}

	odo.radius = radius;
	odo.angle = angle * angle_constant; //check order of magnitude of angle constant to make sure we have m�
	if (position_i < VALUES_AFTER_GPS_RESET){
		odo.first_third_heading += angle;
	}
	else if (position_i < (MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET)){
		odo.second_third_heading += angle;
	}
	else {
		odo.third_third_heading += angle;
	}
	odo.heading = odo.first_third_heading + odo.second_third_heading + odo.third_third_heading;
	//odo.heading = 6000;
}

void navigation_update_xy(uint8_t position_i)
{
	float alpha, angle; 																//in rad
	alpha = (navigation_status.heading_rover - odo.angle) * 2 * M_PI / (PERIOD_DEGREES * KILO); //cos and sin require rad
	angle = odo.angle * 2 * M_PI / (PERIOD_DEGREES * KILO);										//cos and sin require rad
	if (position_i < VALUES_AFTER_GPS_RESET){
		if (odo.straight){
			odo.first_third_x += sin(alpha) * odo.radius;								//radius, x, y in mm
			odo.first_third_y += cos(alpha) * odo.radius;
		}
		else{
			odo.first_third_x += (odo.radius - (float)odo.radius * cos(angle)) * cos(alpha) + odo.radius * sin(angle) * sin(alpha);
			odo.first_third_y += (odo.radius * cos(angle) - odo.radius) * sin(alpha) + odo.radius * sin(angle) * cos(alpha);
		}
	}
	else if (position_i < (MAX_RECENT_VALUES - VALUES_AFTER_GPS_RESET)){
		if (odo.straight){
			odo.second_third_x += sin(alpha) * odo.radius;
			odo.second_third_y += cos(alpha) * odo.radius;
		}
		else{
			odo.second_third_x += (odo.radius - odo.radius * cos(angle)) * cos(alpha) + odo.radius * sin(angle) * sin(alpha);
			odo.second_third_y += (odo.radius * cos(angle) - odo.radius) * sin(alpha) + odo.radius * sin(angle) * cos(alpha);
		}
	}
	else {
		if (odo.straight){
			odo.third_third_x += sin(alpha) * odo.radius;
			odo.third_third_y += cos(alpha) * odo.radius;
		}
		else{
			odo.third_third_x += (odo.radius - odo.radius * cos(angle)) * cos(alpha) + odo.radius * sin(angle) * sin(alpha);
			odo.third_third_y += (odo.radius * cos(angle) - odo.radius) * sin(alpha) + odo.radius * sin(angle) * cos(alpha);
		}
	}
}

void navigation_recalibrate_odometer(float delta_lat, float delta_lon, float delta_heading)
{
	//recalibrate vmot2rps_factor
	float update_friction, x_sum, y_sum, angle_sum = 0;
	x_sum = odo.first_third_x + odo.second_third_x;
	y_sum = odo.first_third_y + odo.second_third_y;
	odo.checked_lat = x_sum / (Y_TO_LATITUDE * cos(MISSION_LATITUDE));
	odo.checked_lon = y_sum / Y_TO_LATITUDE;
	update_friction = sqrt(odo.checked_lat * odo.checked_lat + odo.checked_lon * odo.checked_lon)/sqrt(delta_lat * delta_lat + delta_lon * delta_lon); //look at units of delta_lat and delta_lon!!!
	vmot2rps_factor = vmot2rps_factor / update_friction;

	//recalibrate angle_constant
	angle_sum = odo.first_third_heading + odo.second_third_heading;
	angle_constant = delta_heading / angle_sum;
}

void navigation_reinitialize_odometer(int32_t gps_heading)
{
	odo.first_third_x = odo.third_third_x;
	odo.first_third_y = odo.third_third_y;
	odo.second_third_x = 0;
	odo.second_third_y = 0;
	odo.third_third_x = 0;
	odo.third_third_y = 0;
	odo.radius = 0;
	odo.angle = 0;
	odo.first_third_heading = odo.third_third_heading;
	odo.second_third_heading = 0;
	odo.third_third_heading = 0;
	odo.heading = odo.third_third_heading * E_FOUR;
	odo.longitude = (odo.first_third_x + odo.second_third_x + odo.third_third_x) * E_FOUR / (Y_TO_LATITUDE * cos(MISSION_LATITUDE));
	odo.latitude = (odo.first_third_y + odo.second_third_y + odo.third_third_y) * E_FOUR / Y_TO_LATITUDE;
	odo.heading = odo.heading + gps_heading;
}

void navigation_initialize_odometer()
{
	odo.latitude = 0;
	odo.longitude = 0;
	odo.checked_lat = 0;
	odo.checked_lon = 0;
	odo.odo_time = 0;
	odo.velocity = 0;
	odo.first_third_heading = 0;
	odo.second_third_heading = 0;
	odo.third_third_heading = 0;
	odo.first_third_x = 0;
	odo.first_third_y = 0;
	odo.second_third_x = 0;
	odo.second_third_y = 0;
	odo.third_third_x = 0;
	odo.third_third_y = 0;
	odo.radius = 0;
}

int32_t navigation_get_angle()
{
	return odo.angle;
}

int16_t navigation_get_radius()
{
	int32_t radius;
	radius = odo.radius;
	radius = (int16_t)radius;
	return radius;
}

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
			if(comm_mainboard_armed() == 1 && (mission_items.current_index == 0 || comm_sbc_armed() == 1)){
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
	float angular = 0;

	//only move if not on halt AND state = go_to_target
	if(navigation_status.halt == 0)
	{
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

uint8_t navigation_get_position_i()
{
	uint8_t position_i;
	position_i = navigation_status.position_i;
	return position_i;
}

void navigation_task()
{
	navigation_init();
#ifdef FLASH_ENABLED
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

    //We look if we have mavlink mission item logged, in case we just suffered a crash

    uint32_t time = 0;
    char name[4]; //Name is a 3 characters long string
    mission_item_list_t item_list;
    if(log_read_mavlink_item_list(&item_list, &time, &name))
    	navigation_restore_mission_items(item_list);

#endif

	while(1){

//		navigation_update_target();
		navigation_update_current_target();
		navigation_update_position();
		navigation_update_state();
		navigation_arm_disarm();
		navigation_move();


#ifdef MAVLINK_ON_LORA_ENABLED
		comm_set_tx_flag(CHANNEL_LORA, MAV_COMP_ID_PATHPLANNER);
#endif

#ifdef MAVLINK_ON_UART0_ENABLED
		comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_PATHPLANNER);
#endif
		comm_mavlink_broadcast(navigation_pack_rc_channels_scaled());

#ifdef MAVLINK_ON_UART0_ENABLED
		comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_PATHPLANNER);
#endif
		comm_mavlink_broadcast(navigation_pack_mavlink_hud());

		Task_sleep(500);
	}
}

