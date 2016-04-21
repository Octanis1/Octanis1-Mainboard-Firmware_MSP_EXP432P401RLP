/*
 *  File: weather.c
 *  Description: High level model for accessing calibrated temperature, pressure and moisture
 *  Author:
 */
#include "../../Board.h"

#include "weather.h"
#include "hal/AS3935.h"
#include "hal/bme280i2c.h"
#include "hal/bmp180i2c.h"
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

// NOTE: set to 0 if it should not be logged
#define LOG_GPS_TIMESTEP     5
#define LOG_IMU_TIMESTEP     1
#define LOG_WEATHER_TIMESTEP 5
#define LOG_BACKUP_TIMESTEP  60

typedef struct Types_FreqHz {
    Bits32 hi;
    // most significant 32-bits of frequency
    Bits32 lo;
    // least significant 32-bits of frequency
} Types_FreqHz;

static struct _weather_data {
	int int_temp; //in 0.01 degree Centigrade
	unsigned int int_press; //in Pa
	unsigned int int_humid; //value of 42313 represents 42313 / 1024 = 41.321 %rH
	int ext_temp_bmp180; //in 0.1 degree Centigrade
	float ext_temp_sht21;
	int ext_temp; // average of both values in 0.01 degree Centigrade
	unsigned int ext_press; //pressure in steps of 1.0 Pa
	float ext_humid;
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

/* function calculating averages etc. of measured sensor data */
void weather_aggregate_data()
{
	// average SHT21 and BMP180 external temperature data and scale the result to 0.01 degree Celsius
	weather_data.ext_temp = (int)((10*weather_data.ext_temp_sht21+(float)weather_data.ext_temp_bmp180)*5);
}

uint8_t weather_check_external_connected()
{
	uint8_t j, is_connected = 0;
	P1->OUT |= BIT5; //pre-condition the pin to HIGH
	P1->DIR &= ~BIT5; //make UV sensor enable pin an input
	for(j=0; j++; j<20); //let pin fall to LOW if external board is connected.
	is_connected = (((P1->IN) & BIT5) == 0); //if the pin is pulled down, the external weather monitor is connected
	P1->DIR |= BIT5;
	P1->OUT &= ~BIT5; //pre-condition the pin to HIGH
	return is_connected;
}


/*************** IMU stuff moved here ******************/
#include "imu.h"
#include "hal/motors.h"
#include "hal/bno055_support.h"
#include "hal/i2c_helper.h"

	static struct _imu_data {
		int16_t accel_x;
		int16_t accel_y;
		int16_t accel_z;
		double d_euler_data_p;
		double d_euler_data_h;
		double d_euler_data_r;
		unsigned char calib_status;
	} imu_data;

	// pitch Euler data in 100 degrees
	int16_t imu_get_pitch(){
		return (int16_t)(100*imu_data.d_euler_data_p);
	}

	// heading Euler data in 100 degrees
	int16_t imu_get_heading(){
		return (int16_t)(100*imu_data.d_euler_data_h);
	}

	float imu_get_fheading(){
		return (float)imu_data.d_euler_data_h;
	}

	// roll Euler data in 100 degrees
	int16_t imu_get_roll(){
		return (int16_t)(100*imu_data.d_euler_data_r);
	}

	// return IMU calib status
	uint8_t imu_get_calib_status(){
		return (uint8_t)(imu_data.calib_status);
	}

	// linear acceleration data in 100 m/s^2
	int16_t imu_get_accel_x(){
		return (imu_data.accel_x);
	}

	// linear acceleration data in 100 m/s^2
	int16_t imu_get_accel_y(){
		return (imu_data.accel_y);
	}

	// linear acceleration data in 100 m/s^2
	int16_t imu_get_accel_z(){
		return (imu_data.accel_z);
	}


/************** END IMU stuff *******************/

void weather_task(){
    while(1) {
        Task_sleep(3000);
    }

	static uint8_t external_board_connected = 0;
	external_board_connected = weather_check_external_connected();

	i2c_helper_init_handle();
	windsensor_init();
//	mcp_init();
	geiger_turn_on_off(GEIGER_ON);

	// Initialize external sensors
	if(external_board_connected)
	{
		bmp180_init(); //TODO: change to BMP280
		//	mcp_init();
		lightning_reset();
		lightning_calibrate();
	}

	// Initialize on-board sensors
	bme280_init();


/************* IMU STUFF moved here *************/

	imu_init();

    uint32_t log_counter = 0;
	while(1){


		imu_data.calib_status=bno055_check_calibration_status();
		bno055_get_heading(&(imu_data.d_euler_data_h), &(imu_data.d_euler_data_p), &(imu_data.d_euler_data_r));
		bno055_get_accel(&(imu_data.accel_x), &(imu_data.accel_y), &(imu_data.accel_z));


/************** END IMU stuff *******************/

		Task_sleep(3000);

		if(external_board_connected)
		{
			bmp180_data_readout(&(weather_data.ext_temp_bmp180),&(weather_data.ext_press));
			//		float uv = mcp_get_data();

			weather_data.ext_temp_sht21 = sht2x_get_temp(); //TODO: fix the fact that program stops here if sensor is not connected.
			weather_data.ext_humid = sht2x_get_humidity();

		}

		bme280_data_readout(&(weather_data.int_temp),&(weather_data.int_press),&(weather_data.int_humid));
		//note: bme280 can give pressure, humidity and temperature

		// Logging:

		uint32_t time = Seconds_get();
//		Types_FreqHz freq1;
//		Timestamp_getFreq(&freq1);




		//	windsensor_getvalue();

		weather_aggregate_data();
		cli_printf("W ok. T= %u, He=%u \n", weather_data.int_temp, weather_get_ext_humid());
	}

}
