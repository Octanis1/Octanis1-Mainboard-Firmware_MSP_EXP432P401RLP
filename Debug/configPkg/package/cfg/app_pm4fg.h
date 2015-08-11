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
extern const ti_sysbios_knl_Task_Handle comm_task_xdc;

#include <ti/sysbios/knl/Event.h>
extern const ti_sysbios_knl_Event_Handle navEvents;

#include <ti/sysbios/knl/Event.h>
extern const ti_sysbios_knl_Event_Handle commEvents;

#include <ti/sysbios/knl/Clock.h>
extern const ti_sysbios_knl_Clock_Handle clockInst0;

#include <ti/sysbios/knl/Mailbox.h>
extern const ti_sysbios_knl_Mailbox_Handle cli_print_mailbox;

#define TI_DRIVERS_WIFI_INCLUDED 0

extern int xdc_runtime_Startup__EXECFXN__C;

extern int xdc_runtime_Startup__RESETFXN__C;

