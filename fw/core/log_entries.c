#include "log.h"
#include "log_internal.h"
#include <string.h>
#include <stdbool.h>
#include "../peripherals/weather.h"
#include "../peripherals/gps.h"
#include "../peripherals/imu.h"
#include "../peripherals/navigation.h"


/* proposed alternative API:

void imu_log(struct imu_data* imu)
{
    cmp_ctx_t *ctx = log_entry_create("imu");
    cmp_write_array(ctx, 3);
    cmp_write_integer(ctx, imu->head);
    cmp_write_integer(ctx, imu->roll);
    cmp_write_integer(ctx, imu->pitch);
    log_entry_write_to_flash();
}
*/

bool log_read_mavlink_item(cmp_ctx_t * ctx, mavlink_mission_item_t * item)
{
	bool ret = 0;
	uint32_t array_l = 0;

	ret = cmp_read_array(ctx, &array_l);
	if (ret == false || array_l != 14)
		return false;
	ret = cmp_read_float(ctx, &(item->param1));
	ret = cmp_read_float(ctx, &(item->param2));
	ret = cmp_read_float(ctx, &(item->param3));
	ret = cmp_read_float(ctx, &(item->param4));
	ret = cmp_read_float(ctx, &(item->x));
	ret = cmp_read_float(ctx, &(item->y));
	ret = cmp_read_float(ctx, &(item->z));
	ret = cmp_read_ushort(ctx, &(item->seq));
	ret = cmp_read_ushort(ctx, &(item->command));
	ret = cmp_read_uchar(ctx, &(item->target_system));
	ret = cmp_read_uchar(ctx, &(item->target_component));
	ret = cmp_read_uchar(ctx, &(item->frame));
	ret = cmp_read_uchar(ctx, &(item->current));
	ret = cmp_read_uchar(ctx, &(item->autocontinue));

	return ret;
}

bool log_read_mavlink_item_list(mission_item_list_t * item_list, uint32_t * time, char *name, uint8_t *pos_counter)
{
	cmp_mem_access_t cma;
	cmp_ctx_t ctx;

	uint16_t i = 0;
	size_t entry_len = 0;
	uint32_t next_entry = 0;
	uint8_t buf[LOG_ENTRY_DATA_LEN];
	bool ret = 0;
	uint32_t array_l = 0;
	uint32_t name_sz = sizeof(name);

	bool found_current = false;

	ret = log_read_last_mav_entry(buf, &entry_len, &next_entry, pos_counter);
	if (ret == false)
		return ret;

	cmp_mem_access_ro_init(&ctx, &cma, buf, LOG_ENTRY_DATA_LEN);

	ret = cmp_read_array(&ctx, &array_l);
	if (ret == false || array_l != 3)
		return false;

	ret = cmp_read_uint(&ctx, time);
	ret = cmp_read_str(&ctx, name, &name_sz);
	if (strcmp(name, "mav") != 0)
		return false;

	ret = cmp_read_array(&ctx, &array_l);
	if (ret == false || array_l != 3)
		return false;
	ret = cmp_read_ushort(&ctx, &(item_list->current_index));
	ret = cmp_read_ushort(&ctx, &(item_list->count));
	ret = cmp_read_array(&ctx, &array_l);
	if (ret == false || array_l != item_list->count)
		return false;

	for (i=0; i<item_list->count && i < (N_TARGETS_MAX +1); i++){
		ret = log_read_mavlink_item(&ctx, &(item_list->item[i]));
		if(!found_current && item_list->item[i].current == 0xff && i > 0)
		{
			found_current = true;
			item_list->item[i - 1].current = 1;
			item_list->item[i].current = 0;
		}
		else
		{
			item_list->item[i].current = 0;
		}
		if (ret == false)
			return false;
	}

	if(!found_current) //the last waypoint must be the current WP
	{
		item_list->item[item_list->count-1].current = 1;
	}

	return ret;
}

void log_serialize_mavlink_item(cmp_ctx_t *ctx, mavlink_mission_item_t item)
{
    cmp_write_array(ctx, 14);
    cmp_write_float(ctx, item.param1);
    cmp_write_float(ctx, item.param2);
    cmp_write_float(ctx, item.param3);
    cmp_write_float(ctx, item.param4);
    cmp_write_float(ctx, item.x);
    cmp_write_float(ctx, item.y);
    cmp_write_float(ctx, item.z);
    cmp_write_uinteger(ctx, item.seq);
    cmp_write_uinteger(ctx, item.command);
    cmp_write_uinteger(ctx, item.target_system);
    cmp_write_uinteger(ctx, item.target_component);
    cmp_write_uinteger(ctx, item.frame);
    cmp_write_uinteger(ctx, item.current);
    cmp_write_uinteger(ctx, item.autocontinue);

}

void log_write_mavlink_item_list(bool overwrite, uint8_t *pos_counter)
{
    int i = 0;
    uint8_t temp;
    mavlink_mission_item_t * mav_list = navigation_mavlink_get_item_list();
    uint16_t current_index = navigation_mavlink_get_current_index();
    uint16_t count = navigation_mavlink_get_count();

    cmp_ctx_t *ctx = log_entry_create("mav");

    cmp_write_array(ctx, 3);
    cmp_write_uinteger(ctx, current_index);
    cmp_write_uinteger(ctx, count);
    cmp_write_array(ctx, count);
    for(i=0; i<count; i++)
    {
    	if(i > current_index)
    	{
    		mav_list[i].current = 0xff;
    	}
        log_serialize_mavlink_item(ctx, mav_list[i]);
        mav_list[i].current = 0x00;
    }
    mav_list[current_index].current = 1;

    if(overwrite)
    {
        log_mav_overwrite_flash();
    }
    else
    {
    	log_mav_write_to_flash(pos_counter);
    }
}

//void log_write_navigation_status(void)
//{
//    float lat_rover = navigation_get_lat_rover();
//    float lon_rover = navigation_get_lon_rover();
//    float heading_rover = navigation_get_heading_rover();
//    float lat_target = navigation_get_lat_target();
//    float lon_target = navigation_get_lon_target();
//    float distance_to_target = navigation_get_distance_to_target();
//    float angle_to_target = navigation_get_angle_to_target();
//    float max_dist_obs = navigation_get_max_dist_obs();
//    /*Strange enum related bug, not loging it currently*/
//    //int8_t current_state = (int8_t) navigation_get_current_state();
//
//    cmp_ctx_t *ctx = log_entry_create("nav");
//
//    cmp_write_array(ctx, 8);
//    cmp_write_float(ctx, lat_rover);
//    cmp_write_float(ctx, lon_rover);
//    cmp_write_float(ctx, heading_rover);
//    cmp_write_float(ctx, lat_target);
//    cmp_write_float(ctx, lon_target);
//    cmp_write_float(ctx, distance_to_target);
//    cmp_write_float(ctx, angle_to_target);
//    cmp_write_float(ctx, max_dist_obs);
//    //cmp_write_integer(ctx, current_state);
//
//    log_entry_write_to_flash();
//}


void log_write_gps(void)
{
    float lat = gps_get_lat();
    float lon = gps_get_lon();
    uint8_t fix_qual = gps_get_fix_quality();

    cmp_ctx_t *ctx = log_entry_create("gps");

    cmp_write_array(ctx, 3);
    cmp_write_float(ctx, lat);
    cmp_write_float(ctx, lon);
    cmp_write_uinteger(ctx, fix_qual);

    log_entry_write_to_flash();
}

void log_write_imu(void)
{
    uint8_t imu_calib = imu_get_calib_status();
    int16_t imu_head = imu_get_heading();
    int16_t imu_roll = imu_get_roll();
    int16_t imu_pitch = imu_get_pitch();

    cmp_ctx_t *ctx = log_entry_create("imu");

    cmp_write_array(ctx, 4);
    cmp_write_uinteger(ctx, imu_calib);
    cmp_write_integer(ctx, imu_head);
    cmp_write_integer(ctx, imu_roll);
    cmp_write_integer(ctx, imu_pitch);

    log_entry_write_to_flash();
}

void log_write_weather(void)
{
    unsigned int int_press = weather_get_int_press();
    int int_temp = weather_get_int_temp();
    unsigned int int_humi = weather_get_int_humid();
    unsigned int ex_press = weather_get_ext_press();
    int ex_temp = weather_get_ext_temp();
    unsigned int ex_humi = weather_get_ext_humid();
    uint32_t  uv_light = weather_get_uv_light();
	uint32_t  ir_light = weather_get_ir_light();
	float vis_lux = weather_get_vis_lux();
	float irradiance = weather_get_irradiance();


    cmp_ctx_t *ctx = log_entry_create("wea");

    cmp_write_array(ctx, 6);
    cmp_write_uinteger(ctx, int_press);
    cmp_write_integer(ctx, int_temp);
    cmp_write_uinteger(ctx, int_humi);
    cmp_write_uinteger(ctx, ex_press);
    cmp_write_integer(ctx, ex_temp);
    cmp_write_uinteger(ctx, ex_humi);


    log_entry_write_to_flash();
}

#if 0
// todo:
// - check if correctly read (eg. if (!cmp_read_float(ctx, &f)) {error} )
// - read int a struct pointer (eg. struct imu_data *, struct gps_data *, ...)

/* This struct is used to pass
 * the decoded info of a MessagePack buffer
 * It either contain the weather info, the imu
 * info OR the gps info, depending of the value of
 * data_type. */
typedef struct log_data_t {
    uint8_t data_type;
    uint32_t timestamp;
    float lat;
    float lon;
    uint8_t fix_qual;
    uint8_t imu_calib;
    uint16_t imu_head;
    uint16_t imu_roll;
    uint16_t imu_pitch;
    unsigned int int_press;
    int int_temp;
    unsigned int int_humi;
    unsigned int ex_press;
    int ex_temp;
    unsigned int ex_humi;
}log_data_t;

void logging_parse_gps(struct logger * l, log_data_t * decoded)
{
    cmp_read_float(&l->ctx, &decoded->lat);
    cmp_read_float(&l->ctx, &decoded->lon);
    cmp_read_u8(&l->ctx, &decoded->fix_qual);
}

void logging_parse_weather(struct logger *l, log_data_t * decoded)
{

    cmp_read_uint(&l->ctx, &decoded->int_press);;
    cmp_read_int(&l->ctx, &decoded->int_temp);
    cmp_read_uint(&l->ctx, &decoded->int_humi);
    cmp_read_uint(&l->ctx, &decoded->ex_press);
    cmp_read_int(&l->ctx, &decoded->ex_temp);
    cmp_read_uint(&l->ctx, &decoded->ex_humi);
}

void logging_parse_imu(struct logger *l, log_data_t * decoded)
{
    cmp_read_u8(&l->ctx, &decoded->imu_calib);
    cmp_read_u16(&l->ctx, &decoded->imu_head);
    cmp_read_u16(&l->ctx, &decoded->imu_roll);
    cmp_read_u16(&l->ctx, &decoded->imu_pitch);
}

#endif
