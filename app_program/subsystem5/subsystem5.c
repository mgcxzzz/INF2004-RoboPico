/*
 *	subsystem5.c
 *	Subsystem 5 -- Adaptive Ultrasonic Scanning and Obstacle Profiling.
 *
 *	One task, Obstacle Scan and Recovery (task_obstacle.c), which owns the
 *	HC-SR04+ on Grove 4 (TRIG GP16, ECHO GP17) and the SG90 scanning servo
 *	on servo port 1 (GP12, PWM slice 6).  It never drives the motors: it
 *	asks the Navigation Coordinator (app_program/system/) instead.
 *
 *	Files in this folder:
 *	  task_obstacle.c        the task: statechart 6.5, one function per state
 *	  obstacle_profile.c/.h  obstacle width, beam-spread correction, side choice
 *	  drv_ultrasonic.c/.h    HC-SR04+ driver (echo timed by GPIO interrupt)
 *	  drv_servo.c/.h         SG90 driver (50 Hz PWM)
 *	  b5_config.h            pins, timing, every tuning value
 *	  b5_iface.h             interface with the other subsystems
 *	  b5_stubs.c             weak placeholders for functions others provide
 *	  subsystem5_test.c      on-target tests (make TEST=1)
 *	See README.md in this folder.
 */

#include "subsystem5.h"

EXPORT ER subsystem5_init(void)
{
	return b5_init();		/* drivers, interrupt, event flag */
}

EXPORT ER subsystem5_start(void)
{
	return b5_start();		/* the Obstacle Scan and Recovery task */
}
