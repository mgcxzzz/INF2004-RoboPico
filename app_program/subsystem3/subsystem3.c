/*
 *	subsystem3.c
 *	Subsystem 3 -- Line Following and Barcode Navigation (Buddy 3).
 *
 *	One task, Line and Barcode (task_line.c), which owns the two front IR
 *	sensors (GP6, GP7) and the side-offset barcode sensor (GP19).  It
 *	never drives the motors: it steers through the Navigation Coordinator
 *	while that grants line control, and sends it line and barcode events.
 *
 *	Files in this folder:
 *	  task_line.c         the task: statechart 6.3
 *	  s3_line.c/.h        line position, steering, lost / junction
 *	  s3_reader.c/.h      barcode edges -> bar widths by distance
 *	  s3_barcode.c/.h     Code 39 decoding
 *	  s3_ir.c             the IR sensors and the GP19 edge interrupt
 *	  s3_pack.c           telemetry frame (schema: ../system/sys_tlm.h)
 *	  s3_config.h         pins, timing, every tuning value, command table
 *	  s3_iface.h          interface with the other subsystems
 *	  s3_stubs.c          weak placeholders for functions others provide
 *	  subsystem3_test.c   on-target tests (make TEST=1)
 *	  task_consumer.c     template demo, see subsystem3.h
 *	See README.md in this folder.
 */

#include "subsystem3.h"
#include "../system/sys_config.h"

EXPORT ID		mbx_samples;	/* IF-01 (template demo) */
EXPORT ID		mtx_stats;	/* IF-04 (template demo) */
EXPORT APP_STATS	app_stats;

EXPORT ER subsystem3_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	/* Template demo objects (see subsystem3.h). */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	if (mtx_stats < E_OK) {
		return mtx_stats;
	}

#if SYS_SIM_LINE
	return E_OK;		/* the bench simulates Line and Barcode */
#else
	return s3_init();	/* pins, GP19 interrupt, event flag */
#endif
}

EXPORT ER subsystem3_start(void)
{
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	if (tid < E_OK) {
		return tid;
	}

#if SYS_SIM_LINE
	return E_OK;
#else
	return s3_start();	/* the Line and Barcode task */
#endif
}
