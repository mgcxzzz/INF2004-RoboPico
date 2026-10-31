#ifndef BUDDY_NAV_API_H
#define BUDDY_NAV_API_H
#include "include/interfaces/motion_api.h"
/* Preferred buddy-facing names. The motion_* names are compatibility facade
 * calls to the SAME coordinator, never direct motor access. */
#define nav_request_takeover motion_pause
#define nav_request_stop motion_stop
#define nav_drive_distance motion_drive_distance
#define nav_turn_relative motion_turn_relative
#define nav_resume_line motion_resume_line
#define nav_get_status motion_get_status
#define nav_get_command motion_get_command
#define nav_submit_barcode motion_submit_barcode
#define nav_submit_navigation motion_submit_navigation
#define nav_report_terrain motion_report_terrain
#define nav_request_speed_assist motion_request_speed_assist
#define nav_request_ramp_assist motion_request_ramp_assist
#define nav_get_ramp_assist motion_get_ramp_assist
#define nav_accept_barcode motion_accept_barcode
#define nav_report_fault motion_report_fault
#define nav_result_name motion_result_name
#endif
