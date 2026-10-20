/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU-Based Motion and Terrain Monitoring (Buddy 4),
 *	task "IMU Terrain".  Student ID: 2503753.
 *
 *	Public interface of subsystem 4.  Other subsystems include it as
 *	"../subsystem4/subsystem4.h".  Everything the IMU publishes (imu_event_t,
 *	motion classes, hump results, telemetry frame) is in imu_iface.h,
 *	included below; see README.md in this folder.
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "imu_iface.h"

/* -------------------------------------------------------------------------
 *  Template demo (IF-03).  The template consumer of subsystem 3 still sends
 *  its summary lines to these two objects, so they are kept (nobody reads
 *  them any more; the consumer drops a line when the buffer is full).
 *  Delete this block and the matching lines in subsystem4.c once the
 *  template consumer is gone from subsystem 3.
 * ------------------------------------------------------------------------- */
#define S4_SUMMARY_MAX		48
#define FLG_BATCH		0x0002	/* set by subsystem 3 */

IMPORT ID	mbf_summary;
IMPORT ID	flg_events;

/* Private to subsystem 4 (task_imu.c) */
IMPORT ER	imu_task_init(void);
IMPORT ER	imu_task_start(void);
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT UW	imu_periods(void);		/* 20 ms periods run (tests)  */
IMPORT UW	imu_frames_missed(void);	/* NFR-IMU-01 evidence        */

#endif /* SUBSYSTEM4_H */
