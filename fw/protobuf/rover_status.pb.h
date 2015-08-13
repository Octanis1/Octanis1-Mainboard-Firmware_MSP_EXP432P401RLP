/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.4-dev at Thu Aug 13 19:13:27 2015. */

#ifndef PB_ROVER_STATUS_PB_H_INCLUDED
#define PB_ROVER_STATUS_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Enum definitions */
/* Struct definitions */
typedef struct _rover_status {
    bool has_system_temp;
    int32_t system_temp;
    bool has_bv;
    float bv;
    bool has_pv;
    float pv;
    bool has_cpu_load;
    int32_t cpu_load;
    bool has_gps_health;
    int32_t gps_health;
    bool has_hx1_health;
    int32_t hx1_health;
    bool has_rockblock_health;
    int32_t rockblock_health;
    bool has_watchdog_last_fired;
    int32_t watchdog_last_fired;
} rover_status;

/* Default values for struct fields */

/* Initializer values for message structs */
#define rover_status_init_default                {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}
#define rover_status_init_zero                   {false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0}

/* Field tags (for use in manual encoding/decoding) */
#define rover_status_system_temp_tag             1
#define rover_status_bv_tag                      2
#define rover_status_pv_tag                      3
#define rover_status_cpu_load_tag                4
#define rover_status_gps_health_tag              5
#define rover_status_hx1_health_tag              6
#define rover_status_rockblock_health_tag        7
#define rover_status_watchdog_last_fired_tag     8

/* Struct field encoding specification for nanopb */
extern const pb_field_t rover_status_fields[9];

/* Maximum encoded size of messages (where known) */
#define rover_status_size                        76

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define ROVER_STATUS_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
