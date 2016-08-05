#include "log.h"
#include "../peripherals/weather.h"
#include "../peripherals/gps.h"
#include "../peripherals/imu.h"

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

    cmp_write_array(ctx, 10);
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
