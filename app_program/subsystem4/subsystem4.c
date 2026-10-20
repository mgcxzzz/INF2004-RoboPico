/*
 *	subsystem4.c
 *	Subsystem 4 -- IMU Terrain: creates its objects and starts its task.
 *	The task itself is in task_imu.c, the statechart in imu_proc.c.
 */

#include "subsystem4.h"

EXPORT ID	mbf_summary;	/* IF-03, template demo (see subsystem4.h) */
EXPORT ID	flg_events;	/* IF-03, template demo */

EXPORT ER subsystem4_init(void)
{
	T_CFLG	cflg = {0};
	T_CMBF	cmbf = {0};

	/* Template demo objects, kept for subsystem 3's template consumer */
	cflg.flgatr  = TA_TFIFO | TA_WMUL;
	cflg.iflgptn = 0;
	flg_events = tk_cre_flg(&cflg);
	if (flg_events < E_OK) {
		return flg_events;
	}

	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz  = 4 * S4_SUMMARY_MAX;
	cmbf.maxmsz = S4_SUMMARY_MAX;
	mbf_summary = tk_cre_mbf(&cmbf);
	if (mbf_summary < E_OK) {
		return mbf_summary;
	}

	/* IMU Terrain: operating-state queue and 20 ms cyclic handler */
	return imu_task_init();
}

EXPORT ER subsystem4_start(void)
{
	return imu_task_start();
}
