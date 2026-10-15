/*
 *	subsystem5.h
 *	Subsystem 5 -- Adaptive Ultrasonic Scanning and Obstacle Profiling.
 *	Task: Obstacle Scan and Recovery.  Owner: Buddy 5.
 *
 *	Public interface for the other subsystems.  Everything Buddy 5
 *	exchanges with the rest of the car is declared in b5_iface.h
 *	(events in, requests out); this header only adds the template's
 *	init/start/test entry points and the priority band.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include "../app.h"
#include "b5_iface.h"

#define S5_PRI_MAIN	20		/* priority band 20-21 (TEAM.md) */

#endif /* SUBSYSTEM5_H */
