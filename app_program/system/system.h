/*
 *	system.h
 *	Shared system tasks (team-owned, TEAM.md section 1):
 *	  Safety Supervisor       safety_supervisor.c   priority 6
 *	  Navigation Coordinator  nav_coordinator.c     priority 10
 *	  bench helpers           sys_bench.c           priority 11
 *	plus the shared hardware helpers in sys_hal.c (1 us clock, the one
 *	GPIO interrupt shared by every subsystem).
 *
 *	usermain() calls system_init() before every subsystemN_init(), and
 *	system_start() before every subsystemN_start().  See README.md.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include "../app.h"
#include "sys_config.h"
#include "sys_hal.h"
#include "safety_iface.h"
#include "nav_iface.h"

IMPORT ER system_init(void);
IMPORT ER system_start(void);

#endif /* SYSTEM_H */
