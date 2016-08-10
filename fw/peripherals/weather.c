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

// NOTE: set to 0 if it should not be logged
#define LOG_GPS_TIMESTEP      5
#define LOG_IMU_TIMESTEP      1
#define LOG_WEATHER_TIMESTEP  5
#define LOG_BACKUP_TIMESTEP   60
#define HUMIDITY_CONSTANT 	  1.024
#define CPM_CONVERSION_FACTOR 0.0224 //see wiki for origin of factor
#define RC_CHANNELS			  18
#define SIGNAL_STRENGTH		  255
#define KELVIN				  27315

#include <xdc/runtime/Types.h>

static uint8_t external_board_connected = 0;

#define UPDATE_PERIOD			2000 //task sleep ms

static struct _weather_data {
	int int_temp; //in 0.01 degree Centigrade
	unsigned int int_press; //in Pa
	unsigned int int_humid; //value of 42313 represents 42313 / 1024 = 41.321 %rH
	int ext_temp_bmp280; //in 0.01 degree Centigrade
	float ext_temp_sht21;
	int ext_temp; // average of both values in 0.01 degree Centigrade
	unsigned int ext_press; //pressure in steps of 1.0 Pa
	float ext_humid;
	u16 uv_light;
	u16 ir_light;
	u16 vis_light;
	u16 deep_uv_light;
	float vis_lux;
	float irradiance;
	u16 activity; //in counts per minute
	float health_effect; //in uSv/a
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
uint32_t weather_get_uv_light(){
	return weather_data.uv_light;
}

//units?
uint32_t weather_get_ir_light(){
	return weather_data.ir_light;
}

//units?
float weather_get_vis_lux(){
	return weather_data.vis_lux;
}

//units?
float weather_get_irradiance(){
	return weather_data.irradiance;
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
	int ext_temp_sht21 = weather_data.ext_temp_sht21 + KELVIN;	   //in hundredths of Kelvins
	float int_humidity = weather_data.int_humid/HUMIDITY_CONSTANT; //in k%
	float ext_humidity = weather_data.ext_humid/HUMIDITY_CONSTANT; //in k%

	uint16_t chan11_raw = 0;
	uint16_t chan12_raw = 0;
	uint16_t chan13_raw = 0;
	uint16_t chan14_raw = 0;
	uint16_t chan15_raw = 0;
	uint16_t chan16_raw = 0;
	uint16_t chan17_raw = 0;
	uint16_t chan18_raw = 0;

	static COMM_FRAME frame;

	mavlink_msg_rc_channels_pack(mavlink_system.sysid, MAV_COMP_ID_PERIPHERAL, &(frame.mavlink_message),
			msec, RC_CHANNELS, int_humidity, ext_temp_bmp280, ext_temp_sht21, ext_humidity, weather_data.uv_light,
			weather_data.ir_light, weather_data.vis_lux, weather_data.irradiance, weather_data.activity,
			weather_data.health_effect, chan11_raw, chan12_raw, chan13_raw, chan14_raw, chan15_raw,chan16_raw,
			chan17_raw, chan18_raw, SIGNAL_STRENGTH);

	return &frame;
}

uint8_t weather_check_external_connected()
{
	uint8_t pinval = GPIO_getInputPinValue(Board_UV_INT_PORT, Board_UV_INT_PIN); //if the pin is pulled up, the external weather monitor is connected
	GPIO_setAsInputPinWithPullUpResistor(Board_UV_INT_PORT, Board_UV_INT_PIN);
	return pinval;
}


void log_weather(struct _weather_data *d)
{
/*    struct logger *l = logger_get("weather");
    cmp_write_array(l->ctx, 8);
    cmp_write_integer(l->ctx, d->int_temp);
    cmp_write_uinteger(l->ctx, d->int_press);
    // ...
    logger_finish(l);
    */
}


void weather_task(){
	time_since_boot_init();
	cli_init();

	Task_sleep(500);

	external_board_connected = weather_check_external_connected();

	i2c_helper_init_handle();
	windsensor_init();
//	mcp_init();
	geiger_turn_on_off(GEIGER_ON);

	// Initialize external sensors
	if(external_board_connected)
	{
		bmp280_init(UPDATE_PERIOD);
		si1133_begin();
		//	mcp_init();
		lightning_reset();
		lightning_calibrate();
	}

	// Initialize on-board sensors
	bme280_init();


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
#endif


    while(1){
    	weather_data.activity = get_last_minute_count();
    	float activity = weather_data.activity;
    	weather_data.health_effect = activity/CPM_CONVERSION_FACTOR;
    	float health_effect = weather_data.health_effect;
    	serial_printf(cli_stdout, "mSv/a, %f\n", health_effect);

#ifdef FLASH_ENABLED

        log_counter++;


        if (logging_enabled) {
            if (LOG_GPS_TIMESTEP > 0 && log_counter % LOG_GPS_TIMESTEP == 0) {
                log_write_gps();
                // serial_printf(cli_stdout, "log gps, %x\n", log_write_pos());
            }
            if (LOG_IMU_TIMESTEP > 0 && log_counter % LOG_IMU_TIMESTEP == 0) {
                log_write_imu();
                // serial_printf(cli_stdout, "log imu, %x\n", log_write_pos());
            }
            if (LOG_WEATHER_TIMESTEP > 0 && log_counter % LOG_WEATHER_TIMESTEP == 0) {
                log_write_weather();
                // serial_printf(cli_stdout, "log weather, %x\n", log_write_pos());
            }
            if (LOG_BACKUP_TIMESTEP > 0 && log_counter % LOG_BACKUP_TIMESTEP == 0) {
                log_position_backup();
                // serial_printf(cli_stdout, "log backup, %x\n", log_write_pos());
            }
        }

#endif
		if(external_board_connected)
		{
			bmp280_data_readout(&(weather_data.ext_temp_bmp280),&(weather_data.ext_press));

			//weather_data.ext_temp_sht21 = sht2x_get_temp(); //TODO: fix the fact that program stops here if sensor is not connected.
			//weather_data.ext_humid = sht2x_get_humidity();

			weather_data.uv_light=si1133_readUV();
			weather_data.ir_light=si1133_readIR();
			weather_data.vis_light=si1133_readVIS();
			weather_data.deep_uv_light=si1133_readDEEP_UV();

			serial_printf(cli_stdout, "uv: %u, ir: %u \n","vis: %u, deep_uv: %u \n", weather_data.uv_light, weather_data.ir_light,weather_data.vis_light, weather_data.deep_uv_light);
		}

		bme280_data_readout(&(weather_data.int_temp),&(weather_data.int_press),&(weather_data.int_humid));
		//note: bme280 can give pressure, humidity and temperature


		// Logging:

		uint32_t time = Seconds_get();
//		Types_FreqHz freq1;
//		Timestamp_getFreq(&freq1);

#ifdef MAVLINK_ON_UART0_ENABLED
		comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_PERIPHERAL);
#endif
		comm_mavlink_broadcast(weather_pack_mavlink_pressure());

#ifdef MAVLINK_ON_UART0_ENABLED
		comm_set_tx_flag(CHANNEL_APP_UART, MAV_COMP_ID_PERIPHERAL);
#endif
		comm_mavlink_broadcast(weather_pack_mavlink_rc_channels());

		//	windsensor_getvalue();

		weather_aggregate_data();

		Task_sleep(UPDATE_PERIOD); //note: inside bmp280_init, the standby time of the sensor is set according to this value


#ifdef FLASH_ENABLED
        log_weather(&weather_data);
#endif
	}

}
