/*
 *  Do not modify this file; it is automatically 
 *  generated and any modifications will be overwritten.
 *
 * @(#) xdc-A65
 */

#include <xdc/std.h>

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle cli_task_xdc;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle cli_print_task_xdc;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle gps_task_xdc;

#include <ti/sysbios/knl/Task.h>
extern const ti_sysbios_knl_Task_Handle weather_task_xdc;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle cron_quick;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle cron_hourly;

#include <ti/sysbios/knl/Mailbox.h>
extern const ti_sysbios_knl_Mailbox_Handle cli_print_mailbox;

#define TI_DRIVERS_WIFI_INCLUDED 0

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

#ifndef ti_sysbios_knl_Task__include
#ifndef __nested__
#define __nested__
#include <ti/sysbios/knl/Task.h>
#undef __nested__
#else
#include <ti/sysbios/knl/Task.h>
#endif
#endif

extern ti_sysbios_knl_Task_Struct TSK_idle;

