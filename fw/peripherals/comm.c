/*
 * comm.c
 *
 *  Created on: 10.08.2015
 *      Author: Sam
 */

#include "../../Board.h"
#include "comm.h"
#include <string.h>

// comm modules to RX and TX data
#include "hal/ultrasonic.h"
#include "hal/rockblock.h"
#include "hal/rn2483.h"
#include "hal/sim800.h"
#include "hal/hm10.h"
#include "imu.h"
#define COMMAND_BUFFER_LENGTH	HM10_RXBUFFER_SIZE 	//how many bytes can be sent back as response to a command
#include "hal/vc0706.h"
#include "../lib/printf.h"

// peripheral includes to execute commands
#include "navigation.h"
#include "../core/system.h"
#include "../core/log.h"


/* libraries to get status info from other peripherals */
#include "gps.h"
#include "imu.h"
#include "weather.h"
#include "../core/eps.h"


#define N_USER_MESSAGES 10
static COMM_LED_CONTROL led_control;
static COMM_MSG_CONTROL msg_control[N_USER_MESSAGES];
static int n_message_rules = 0;

/* Struct definitions */
typedef struct _rover_status_comm {
	float gps_lat;
	float gps_long;
	uint8_t gps_fix_quality;
	uint32_t system_seconds;
	uint16_t v_bat;
	uint16_t v_solar;
	uint16_t i_in;
	uint16_t i_out;
	uint8_t imu_calib_status;
	int16_t imu_heading; //converted from double
	int16_t imu_roll; //converted from double
	int16_t imu_pitch; //converted from double
	int int_temperature;
	unsigned int int_pressure;
	unsigned int int_humidity;
	int ext_temperature;
	unsigned int ext_pressure;
	unsigned int ext_humidity; //converted from float
	int16_t accel_x;
	int16_t accel_y;
	int16_t accel_z;
	int32_t speed;
	int32_t altitude;
	int16_t angle_target;
} rover_status_comm;


void comm_init(rover_status_comm* stat)
{
	Task_sleep(5000);
	cli_printf("reset occured.\n", 0);

#ifdef LORA_ENABLED
	if(rn2483_begin()){
	#if VERBOSE==1
		cli_printf("rn2483 begin OK.\n", 0);
	#endif
	}else{
		cli_printf("rn2483 begin NOK\n", 0);
		rn2483_end();
	}
#endif


#ifdef GSM_ENABLED
	if(sim800_begin()){
	#if VERBOSE==1
		cli_printf("sim800 begin OK.\n", 0);
	#endif
	}else{
		cli_printf("sim800 begin NOK\n", 0);
		sim800_end();
	}
#endif


#ifdef BLE_ENABLED
	if(hm10_begin()){
	#if VERBOSE==1
		cli_printf("hm10 begin OK.\n", 0);
	#endif
	}else{
		cli_printf("hm10 begin NOK\n", 0);
		hm10_end();
	}
#endif


	stat->gps_lat = -1.0;
	stat->gps_long = -1.0;
	stat->gps_fix_quality = -1;
	stat->system_seconds = -1;
	stat->v_bat = 0;
	stat->v_solar = 0;
	stat->i_in = 0;
	stat->i_out = 0;
	stat->imu_calib_status = 0;
	stat->imu_heading = -1;
	stat->imu_roll = -1;
	stat->imu_pitch = -1;
	stat->int_temperature = -274;
	stat->int_pressure = -1;
	stat->int_humidity = -1;
	stat->ext_temperature = -274;
	stat->ext_pressure = -1;
	stat->ext_humidity = -1;
	stat->accel_x = -1;
	stat->accel_y = -1;
	stat->accel_z = -1;
	stat->speed = -1;
	stat->altitude = -1;
}


void comm_poll_status(rover_status_comm* stat)
{
	/*Fill in struct with status information */
	stat->gps_lat = gps_get_lat();
	stat->gps_long = gps_get_lon();
	stat->gps_fix_quality = gps_get_fix_quality();
	stat->system_seconds = Seconds_get();
	stat->v_bat = eps_get_vbat();
	stat->v_solar = eps_get_vsolar();
	stat->i_in = eps_get_iin();
	stat->i_out = eps_get_iout();
	stat->imu_calib_status = imu_get_calib_status();
	stat->imu_heading = imu_get_heading();
	stat->imu_roll = imu_get_roll();
	stat->imu_pitch = imu_get_pitch();
	stat->int_temperature = weather_get_int_temp();
	stat->int_pressure = weather_get_int_press();
	stat->int_humidity = weather_get_int_humid();
	stat->ext_temperature = weather_get_ext_temp();
	stat->ext_pressure = weather_get_ext_press();
	stat->ext_humidity = weather_get_ext_humid();
	stat->accel_x = imu_get_accel_x();
	stat->accel_y = imu_get_accel_y();
	stat->accel_z = imu_get_accel_z();
	stat->speed = gps_get_int_speed();
	stat->altitude = gps_get_int_altitude();
	stat->angle_target = (int16_t)(100.0 * navigation_get_angle_to_target());
}

static int threshold_reached(COMM_CONDITION cond, rover_status_comm* stat)
{
	int vari = 0;
	switch(cond.variable){
		case IMUX: vari = stat->accel_x;break; //100m/s^2
		case IMUY: vari = stat->accel_y;break;
		case IMUZ: vari = stat->accel_z;break;
		case IMUR: vari = (stat->imu_roll)/100;break; //degrees
		case IMUP: vari = (stat->imu_pitch)/100;break;
		case IMUH: vari = (stat->imu_heading)/100;break;
		case TEMP: vari = (stat->int_temperature)/100;break; //°C
		case PRES: vari = (stat->int_pressure)/100;break; // hPa
		case HUMI: vari = (stat->int_humidity)/1000;break; // %rel humid
		default: return 0; break;
	}

	switch(cond.op){
		case SMALLER: return (vari < cond.threshold);
		case GREATER: return (vari > cond.threshold);
		case EQUAL: return (vari == cond.threshold);
		case NEQUAL: return (vari != cond.threshold);
		default: return 0;
	}
}

#define MAX_LED_FREQ 10
void comm_execute_thresholds(rover_status_comm* stat)
{
	static int led_off = 1; //makes sure we turn off the led only once, as not to disturb other uses of the red LED.
	/* Execute LED thresholds */
	static int led_cycles = 0;
	if(threshold_reached(led_control.cond, stat))
	{
		led_cycles += led_control.frequency;

		if(led_cycles > 3*MAX_LED_FREQ)
			GPIO_write(Board_LED_RED,1);
		else
			GPIO_write(Board_LED_RED,0);

		if(led_cycles > 6*MAX_LED_FREQ)
		{
			led_cycles = 0;
		}

		led_off = 0;
	}
	else
	{
		if(!led_off)
		{
			GPIO_write(Board_LED_RED,0);
			led_off = 1;
		}
	}

	int i = 0;

	for(i=0;i<n_message_rules;i++)
	{
		if(threshold_reached(msg_control[i].cond, stat))
		{
			//only send a user message after threshold crossing
			if(msg_control[i].msg_sent_since_last_threshold_crossing == 0)
			{
				comm_tx_data(msg_control[i].message, msg_control[i].msglength, msg_control[i].destination);
				msg_control[i].msg_sent_since_last_threshold_crossing = 1;
			}
		}
		else
		{
			msg_control[i].msg_sent_since_last_threshold_crossing = 0; //reset state to send msg again next time.
		}
	}

}

void comm_receive_command(COMM_DESTINATION destination)
{
	static int rx_stringlength,tx_stringlength = 0;
	static int received_command = 0;
	static int answer_required = 0;

	static char	rxdata[COMMAND_BUFFER_LENGTH];
	//answer buffer
	static char txdata[COMMAND_BUFFER_LENGTH];
	//clears buffer
	memset(&txdata[0], 0, sizeof(txdata));

	// check for rx'ed data:
	switch(destination)
	{
	   case DESTINATION_BLE:
		  received_command = hm10_receive(rxdata, &rx_stringlength);
		  break;
	   default:
	   	   cli_printf("comm Task: Command RX source not supported\n");
	}

	if(received_command)
	{
		answer_required = comm_process_command(rxdata, rx_stringlength, txdata, &tx_stringlength, destination);

		if(answer_required)
		{
			comm_tx_data(txdata, tx_stringlength, destination);
		}
	}
}

int comm_process_command(char* command, int commandlength, char* txbuffer, int* answerlength, COMM_DESTINATION destination)
{
	//return value (default yes):
	int require_answer = 1;
	int command_valid = 1;
	int i = 0;
	COMM_CONDITION condition;
	char ** endptr = NULL;
	int8_t br[8];
	int8_t bl[8];


	if(strncmp ("thr ", command, 4) == 0){ //new threshold to be set
		/*extract the variable*/
		if(strncmp ("temp", &command[4], 4) == 0)
			condition.variable = TEMP;
		else if(strncmp ("pres", &command[4], 4) == 0)
			condition.variable = PRES;
		else if(strncmp ("humi", &command[4], 4) == 0)
			condition.variable = HUMI;
		else if(strncmp ("imux", &command[4], 4) == 0)
			condition.variable = IMUX;
		else if(strncmp ("imuy", &command[4], 4) == 0)
			condition.variable = IMUY;
		else if(strncmp ("imuz", &command[4], 4) == 0)
			condition.variable = IMUZ;
		else if(strncmp ("imup", &command[4], 4) == 0)
			condition.variable = IMUP;
		else if(strncmp ("imur", &command[4], 4) == 0)
			condition.variable = IMUR;
		else if(strncmp ("imuh", &command[4], 4) == 0)
			condition.variable = IMUH;
		else
			command_valid = 0;

		/* extract operator */
		if(command[8] == '<')
			condition.op = SMALLER;
		else if(command[8] == '>')
			condition.op = GREATER;
		else if(command[8] == '=')
			condition.op = EQUAL;
		else if(command[8] == '!')
			condition.op = NEQUAL;
		else
			command_valid = 0;

		/* extract threshold value */
		int compos = strcspn(command, ",");
		char *numptr = &command[10];
		a2i(command[9], &numptr ,10,&(condition.threshold));

		if(compos < commandlength)
		{
			if(strncmp ("led", &command[compos+1], 3) == 0){
				if(strncmp ("off", &command[compos+4], 3) == 0){
					led_control.frequency = 0;}
				else if(strncmp ("on", &command[compos+4], 2) == 0){
					led_control.frequency = 1000;}// freq bigger than 2*MAX_UINT8--> led will never toggle
				else{
					led_control.frequency = command[compos+4] - '0';}
				led_control.cond = condition;
			}
			else if(command[compos+1] == 'b' || command[compos+1] == 'l'){ //send BLE or lora message
				int j=0;
				int k=0;
				for(k=compos+3;k<commandlength-1;k++)
				{
					msg_control[n_message_rules].message[j] = command[k];
					j++;
				}
				msg_control[n_message_rules].msglength = j;
				msg_control[n_message_rules].cond = condition;
				if(command[compos+1] == 'b')
					msg_control[n_message_rules].destination = DESTINATION_BLE;
				else if(command[compos+1] == 'l')
					msg_control[n_message_rules].destination = DESTINATION_LORA_TTN;
				else
					msg_control[n_message_rules].destination = DESTINATION_DEBUG_UART;

				msg_control[n_message_rules].msg_sent_since_last_threshold_crossing = 0;
				n_message_rules = (n_message_rules+1) % N_USER_MESSAGES;
			}
			else
			{
				command_valid = 0;
			}
		}else command_valid = 0;

		//prepare response
		if(command_valid)
			tfp_sprintf(txbuffer, "threshold set.");
		else
			tfp_sprintf(txbuffer, "invalid threshold command.");

	}
	else if(strncmp ("rst", command, 3) == 0){ //reset all thresholds
		n_message_rules = 0;
		led_control.frequency = 0;
		tfp_sprintf(txbuffer, "thresholds reset.");
	}
	else if(strncmp ("mot", command, 3) == 0){ //motor command was sent
		if(navigation_bypass(command[3],(command[4]-'0')))
			tfp_sprintf(txbuffer, "okm");
		else
			tfp_sprintf(txbuffer, "inv");
	}
	else if(strncmp("targ", command, 4) == 0){
		if(strncmp("rm", &command[5], 2) == 0) //remove newest target from list
		{
			if(navigation_remove_newest_target())
				tfp_sprintf(txbuffer, "newest target removed");
		}
		else
		{
			if(navigation_add_target_from_string(&command[5], commandlength - 5))
				tfp_sprintf(txbuffer, "target added");
			else
				tfp_sprintf(txbuffer, "target list full");
		}
	}
	else if(strcmp("gps\n", command) == 0){
		tfp_sprintf(txbuffer, "fq %d", gps_get_fix_quality());}
	else if(strcmp("lat\n", command) == 0){
		ftoa(gps_get_lat(), txbuffer, 4);
	}else if(strcmp("lon\n", command) == 0){
		ftoa(gps_get_lon(), txbuffer, 4);
	}else if(strcmp("sat\n", command) == 0){
		tfp_sprintf(txbuffer, "sat %d", gps_get_satellites_tracked());
	}else if(strcmp("valid\n", command) == 0){
		tfp_sprintf(txbuffer, "valid %d", gps_get_validity());
	}else if(strcmp("hdop\n", command) == 0){
		tfp_sprintf(txbuffer, "hdop %d", gps_get_hdop());
	}else if(strcmp("lastgps\n", command) == 0){
		tfp_sprintf(txbuffer, "lu %d", gps_get_last_update_time());
	}else if(strcmp("tasks\n", command) == 0){
	   system_listTasks();
	   require_answer = 0; //printf is already done in syste_listTasks() --> only answers in DEBUG_UART
	}else if(strcmp("rbs\n", command) == 0){
	   tfp_sprintf(txbuffer, "rb sleep? %d", rockblock_get_sleep_status());
	}else if(strcmp("rbn\n", command) == 0){
	   tfp_sprintf(txbuffer, "rb net? %d", rockblock_get_net_availability());
	}else if (strcmp("logrst", command) == 0){
	   log_reset();
	   tfp_sprintf(txbuffer, "ok");
	}else if (strcmp("logpos\n", command) == 0){
	   tfp_sprintf(txbuffer, "logpos %u", log_write_pos());
	}else if (strncmp("max_dist", command, 8) == 0){
		endptr = NULL;
		navigation_set_max_dist(strtof(&command[9], endptr));
	}else if(strncmp("pid", command, 3) == 0){
		//command = "pid <a> <p/i/d> <value in floating point>"
		endptr = NULL;
		navigation_change_gain(command[4], command[6], strtof(&command[8], endptr));
	}else if(strncmp("bl", command, 2) == 0){
		char *delim = " ";
		char *token = NULL;
		i = 0;
		for(token = strtok(&command[3], delim); token != NULL; strtok (NULL,delim)){
			char *unconverted;
			bl[i] = strtof(token, &unconverted);
			if (!isspace(*unconverted) && *unconverted != 0){
				tfp_sprintf(txbuffer, "invalid number of arguments.\n");
				break;
			}
			i++;
		}
		ultrasonic_set_bl(bl[0], bl[1], bl[2], bl[3], bl[4], bl[5], bl[6], bl[7]);
	}else if(strncmp("br", command, 2) == 0){
		char *delim = " ";
		char *token = NULL;
		i = 0;
		for(token = strtok(&command[3], delim); token != NULL; strtok (NULL,delim)){
			char *unconverted;
			br[i] = strtof(token, &unconverted);
			if (!isspace(*unconverted) && *unconverted != 0){
				tfp_sprintf(txbuffer, "invalid number of arguments.\n");
				break;
			}
			i++;
		}
		ultrasonic_set_br(br[0], br[1], br[2], br[3], br[4], br[5], br[6], br[7]);
	}
	else{ // received command not valid
		tfp_sprintf(txbuffer, "invalid command");
	}


	*answerlength = strlen(txbuffer);

	return require_answer;
}

void comm_tx_data(char* txdata, int stringlength, COMM_DESTINATION destination)
{

	/** Prepare hex string for LoRa **/
	char hex_string_byte[2];
	char hex_string[COMM_FRAME_SIZE]; //TODO: ATTENTION: this is too small! need to change this
	memset(&hex_string, 0, sizeof(hex_string));

	int i;
	for(i=0; i<stringlength; i++){
		memset(&hex_string_byte, 0, sizeof(hex_string_byte));
		tfp_sprintf(hex_string_byte, "%02x", txdata[i]);
		strcat(hex_string, hex_string_byte);
	}
	/** end hex string **/

	switch(destination) {
	   case DESTINATION_LORA_TTN:
		  rn2483_send_receive(hex_string, 2*stringlength);
		  break;

	   case DESTINATION_GSM:
		  sim800_send_http(hex_string, strlen(hex_string), MIME_TEXT_PLAIN);
		  break;

	   case DESTINATION_GSM_SMS:
		  sim800_send_sms(txdata, strlen(txdata));
		  break;

	   case DESTINATION_BLE:
		  hm10_send(txdata, strlen(txdata));
		  break;

	   case DESTINATION_DEBUG_UART:
		   cli_printf("%s\n",txdata);
		   break;

	   default:
		   cli_printf("comm Task: Status TX destination not supported\n");
	}

}


void comm_send_status(rover_status_comm* stat, COMM_DESTINATION destination)
{
	/* create Hexstring buffer from struct */
	int stringlength=5;
	char txdata[COMM_STRING_SIZE] = "stat:";

	stringlength += ftoa(stat->gps_lat, &txdata[stringlength], 7); //convert gps latitude to string with sign and 7 afterpoint
	txdata[stringlength++] = ','; 					//plus a comma


	stringlength += ftoa(stat->gps_long, &txdata[stringlength], 7); //convert gps long to string with sign and 7 afterpoint
	txdata[stringlength++] = ','; 					//plus a comma

	stringlength += tfp_sprintf(&(txdata[stringlength]), "%d,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%u,%u,%d,%d,%d,%d,%u,%d\n",
											stat->gps_fix_quality,
											stat->system_seconds,
											stat->v_bat,
											stat->v_solar,
											stat->i_in,
											stat->i_out,
											stat->imu_calib_status,
											stat->imu_heading,
											stat->imu_roll,
											stat->imu_pitch,
											stat->int_temperature,
											stat->int_pressure,
											stat->int_humidity,
										/*	stat->ext_temperature,
											stat->ext_pressure,
											stat->ext_humidity,*/
											stat->accel_x,
											stat->accel_y,
											stat->accel_z,
											stat->speed,
											stat->altitude,
											stat->angle_target);

	if(stringlength > COMM_FRAME_SIZE) //should never happen! corrupt memory will be the result!
	{
		cli_printf("status string overflow! %u",stringlength);
		stringlength = COMM_FRAME_SIZE;
	}

	comm_tx_data(txdata, stringlength, destination);
}



void comm_task(){

#ifdef LORA_ENABLED
	#ifdef CONFIG_MODE
		int comm_result=rn2483_config();
		if(comm_result)
			cli_printf("LoRa config failed: %d", comm_result);
		else
			cli_printf("LoRa config success: %d", comm_result);
	#endif
#endif

	rover_status_comm my_rover_status;
    comm_init(&my_rover_status);

    int i = 11;
    int rx_counter = RX_TO_TX_RATIO;
    int lora_tx_counter = LORA_TX_RATIO;
    while(1){

    		// Poll for received commands
		#ifdef BLE_ENABLED
    			comm_receive_command(DESTINATION_BLE);
		#endif

    		rx_counter++;
		Task_sleep(50);

		// Status TX part:
		comm_poll_status(&my_rover_status);
		comm_execute_thresholds(&my_rover_status);

    		if(rx_counter > RX_TO_TX_RATIO)
    		{
    			rx_counter=0;

			#ifdef GSM_ENABLED
			comm_send_status(&my_rover_status, DESTINATION_GSM);
//			if(i > 10){
//				Task_sleep(2000);
//				comm_send_status(&my_rover_status, DESTINATION_GSM_SMS);
//				i=0;
//			}
			i++;
			Task_sleep(5000);
			#endif


			#ifdef LORA_ENABLED
			lora_tx_counter++;
			if(lora_tx_counter > LORA_TX_RATIO)
			{
				comm_send_status(&my_rover_status, DESTINATION_LORA_TTN);
				lora_tx_counter = 0;
			}
			#endif


			#ifdef UARTCAM_ENABLED
			if(vc0706_begin()){
				vc0706_gprs_upload_jpeg();
			}
			vc0706_end();
			#endif


			#ifdef BLE_ENABLED
			comm_send_status(&my_rover_status, DESTINATION_BLE);
			#endif

			comm_send_status(&my_rover_status, DESTINATION_DEBUG_UART);

    		}
    }

}
