/*
 *  File: weather.c
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */
#include "../../Board.h"

#include "weather.h"
#include "hal/AS3935.h"
#include "hal/bme280i2c.h"
#include "hal/bmp280_support.h"
#include "hal/SI1133.h"
#include "hal/SHT2x.h"
#include "hal/windsensor.h"
#include "hal/mcp3425.h"
#include "hal/i2c_helper.h"
#include "../lib/printf.h"
#include "geiger.h"
#include "driverlib.h"
#include <string.h>

//Logging includes:
#include "../core/log.h"
#include "../core/log_entries.h"
#include "../lib/cmp/cmp.h"
#include "../lib/cmp_mem_access/cmp_mem_access.h"
#include "../hardware_test/mailbox_test.h"
#include "flash.h"
#include "hal/spi_helper.h"

//mavlink includes:
#include "comm.h"
#include "hal/time_since_boot.h"
#include "gps.h"


#define CPM_CONVERSION_FACTOR 	0.0224 //see wiki for origin of factor
#define SIGNAL_STRENGTH		  	255
#define KELVIN				  	27315

#include <xdc/runtime/Types.h>

static uint8_t external_board_connected = 0;

#define UPDATE_PERIOD			500 //task sleep ms

static struct _weather_data {
	s32 int_temp; 			// in 0.01 degree C
	u32 int_press; 			// in Pa
	u32 int_humid;			// in 100th percentage
	s32 ext_temp_bmp280; 	// in 0.01 degree C
	float ext_temp_sht21; 	// in degree C
	int ext_temp; 			// average of both values in 0.01 degree Centigrade
	u32 ext_press; 			// pressure in Pa
	float ext_humid;			// in %
	u16 uv_light; 			// scaled (x100) UV factor from 0 to 11+
	u16 ir_light;
	u16 vis_light;
	u16 deep_uv_light; 		// scaled (x100) UV factor from 0 to 11+
	u16 activity; 			//in counts per minute
	float health_effect; 	//in uSv/a
} weather_data;

//in 0.01 degree Centigrade
int weather_get_int_temp(){
	return weather_data.int_temp;
}

//in Pa
unsigned int weather_get_int_press(){
	return weather_data.int_press;
}

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_int_humid(){
	return weather_data.int_humid;
}

//in 0.01 degree Centigrade
int weather_get_ext_temp(){
	return weather_data.ext_temp;
}

//in Pa
unsigned int weather_get_ext_press(){
	return weather_data.ext_press;
}

//value of 42313 represents 42313 / 1024 = 41.321 %rH
unsigned int weather_get_ext_humid(){
	return (int)(1024*weather_data.ext_humid);
}

//units?
uint32_t weather_get_uv_index(){
	return weather_data.uv_light/100;
}

//units?
uint32_t weather_get_ir_light(){
	return weather_data.ir_light;
}

/* function calculating averages etc. of measured sensor data */
void weather_aggregate_data()
{
	// average SHT21 and BMP280 external temperature data and scale the result to 0.01 degree Celsius
	weather_data.ext_temp = (int)(50*weather_data.ext_temp_sht21) + weather_data.ext_temp_bmp280 / 2;
}


COMM_FRAME* weather_pack_mavlink_pressure()
{
	uint32_t msec = ms_since_boot(); //1000 * (uint32_t)Seconds_get();
	float press_diff = 0;
	float press_abs;
	int16_t temp;

	if(external_board_connected)
	{
		temp = weather_data.int_temp;
		press_abs = ((float)weather_data.ext_press)/100; //Absolute pressure (hectopascal)
		press_diff = ((float)weather_data.int_press)/100 - press_abs;
	}
	else
	{
		temp = weather_data.int_temp;
		press_abs = ((float)weather_data.int_press)/100; //Absolute pressure (hectopascal)
	}


	// Initialize the message buffer
	static COMM_FRAME frame;

	// Pack the message
	mavlink_msg_scaled_pressure_pack(mavlink_system.sysid, MAV_COMP_ID_PERIPHERAL, &(frame.mavlink_message),
			msec, press_abs, press_diff, temp);

	return &frame;
}


/*Order: internal humidiy in k%, external temperature un hundredths of Kelvins as measured by bmp280,
* external temperature un hundredths of Kelvins as measured by sht21, external humidity in k%, uv light,
* infrared light, visible light, irradiance, activity in counts per minute, health effect in uSv per annum
*/
COMM_FRAME* weather_pack_mavlink_rc_channels()
{
	uint32_t msec = ms_since_boot(); //1000 * (uint32_t)Seconds_get();
	int ext_temp_bmp280 = weather_data.ext_temp_bmp280 + KELVIN;   //in hundredths of Kelvins
	int ext_temp_sht21 = weather_data.ext_temp_sht21*100 + KELVIN;	   //in hundredths of Kelvins
	float ext_humidity = weather_data.ext_humid*100; //in k%

	uint16_t n_channels = 10;

	static COMM_FRAME frame;

	mavlink_msg_rc_channels_pack(mavlink_system.sysid, MAV_COMP_ID_PERIPHERAL, &(frame.mavlink_message),msec, n_channels,
	/*ch1*/		weather_data.int_humid,
	/*ch2*/		ext_humidity,
	/*ch3*/		ext_temp_bmp280,
	/*ch4*/		ext_temp_sht21,
	/*ch5*/		weather_data.uv_light,
	/*ch6*/		weather_data.ir_light,
	/*ch7*/		weather_data.vis_light,
	/*ch8*/		weather_data.deep_uv_light,
	/*ch9*/		weather_data.activity,
	/*ch10*/		weather_data.health_effect,
	/*ch11*/		gps_get_fix_quality(),
	/*ch12*/		UINT16_MAX,
	/*ch13*/		UINT16_MAX,
	/*ch14*/		UINT16_MAX,
	/*ch15*/		UINT16_MAX,
	/*ch16*/		UINT16_MAX,
	/*ch17*/		UINT16_MAX,
	/*ch18*/		UINT16_MAX,
				SIGNAL_STRENGTH);

	return &frame;
}

uint8_t weather_check_external_connected()
{
	uint8_t pinval = GPIO_getInputPinValue(Board_UV_INT_PORT, Board_UV_INT_PIN); //if the pin is pulled up, the external weather monitor is connected
	GPIO_setAsInputPinWithPullUpResistor(Board_UV_INT_PORT, Board_UV_INT_PIN);
	return pinval;
}

void weather_task(){
	bool logging_enabled = false;

	time_since_boot_init();
	cli_init();

	Task_sleep(500);
	external_board_connected = weather_check_external_connected();

	i2c_helper_init_handle();

	// Initialize external sensors
	if(external_board_connected)
	{
		bmp280_init(UPDATE_PERIOD);
		si1133_begin();
		lightning_reset();
		lightning_calibrate();
	}

	// Initialize on-board sensors
	bme280_init();
	geiger_turn_on_off(GEIGER_ON);

/*//Uncomment if flash logging required
	// Initialize flash
	if (flash_init() == 0) {
		logging_enabled = true;
	}

	if (logging_enabled) {
		if (!log_init()) {
			serial_printf(cli_stdout, "log_init failed\n");
			log_reset();
		}
	}
*/
    //serial_printf(cli_stdout, "log position 0x%x\n", log_write_pos());


    while(1){
		if(external_board_connected)
		{
			bmp280_data_readout(&(weather_data.ext_temp_bmp280),&(weather_data.ext_press));

			weather_data.ext_temp_sht21 = sht2x_get_temp(); //TODO: fix the fact that program stops here if sensor is not connected.
			weather_data.ext_humid = sht2x_get_humidity();

			weather_data.uv_light=si1133_readUV();
			weather_data.ir_light=si1133_readIR();
			weather_data.vis_light=si1133_readVIS();
			weather_data.deep_uv_light=si1133_readDEEP_UV();

//			serial_printf(cli_stdout, "uv: %u, ir: %u \n","vis: %u, deep_uv: %u \n", weather_data.uv_light, weather_data.ir_light,weather_data.vis_light, weather_data.deep_uv_light);
		}

		// bme280 can give pressure, humidity and temperature
		bme280_data_readout(&(weather_data.int_temp),&(weather_data.int_press),&(weather_data.int_humid));

		// radioactivity
		weather_data.activity = get_last_minute_count();
		weather_data.health_effect = weather_data.activity/CPM_CONVERSION_FACTOR;


		// make conversions and averages from individual sensors.
		weather_aggregate_data();

		comm_mavlink_broadcast(weather_pack_mavlink_pressure());

		comm_mavlink_broadcast(weather_pack_mavlink_rc_channels());

		Task_sleep(4*UPDATE_PERIOD); //note: inside bmp280_init, the standby time of the sensor is set according to this value
	}

}
