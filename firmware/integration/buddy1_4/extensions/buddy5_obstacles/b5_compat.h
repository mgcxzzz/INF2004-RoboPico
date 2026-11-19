#ifndef B5_COMPAT_H
#define B5_COMPAT_H
/* rtos_buddy1_5: the ported Buddy5 SDK modules (width_probe.c, edge_probe.c,
 * manual_bypass.c, automatic_bypass.c) are kept textually close to their
 * frozen SDK sources, which use the pre-BARR-C coordinator type names. These
 * aliases name the SAME barr-c-cleanup coordinator types; no wrapper, no
 * second motion path. */
#include "include/interfaces/nav_api.h"
typedef motion_status_t MotionStatus;
typedef motion_command_status_t MotionCommandStatus;
typedef motion_result_t MotionResult;
typedef motion_owner_t MotionOwner;
#endif
