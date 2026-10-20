/*
 *	subsystem3.h
 *	Subsystem 3 -- Line Following and Barcode Navigation (Buddy 3).
 *	Student ID: 2501910.
 *
 *	Public interface of subsystem 3.  Other subsystems include it as
 *	"../subsystem3/subsystem3.h".  The Line and Barcode interface itself
 *	is in s3_iface.h (included below); see README.md in this folder.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "s3_iface.h"

/* -------------------------------------------------------------------------
 *  Template demo (kept only because the template tasks of subsystems 2
 *  and 4 still use it: IF-01 mailbox and IF-04 statistics).  Delete this
 *  block, task_consumer.c and the matching lines in subsystem3.c once
 *  Buddies 2 and 4 have replaced their template tasks.
 * ------------------------------------------------------------------------- */
#define S3_PRI_CONSUMER		17	/* below the line task (16), same band */
#define S3_BATCH_SIZE		5	/* report to subsystem 4 every N samples */

/* IF-01: send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx */
IMPORT ID	mbx_samples;

/* IF-04: statistics.  Read them ONLY while holding mtx_stats:
 *	tk_loc_mtx(mtx_stats, TMO_FEVR);  copy = app_stats;  tk_unl_mtx(mtx_stats);
 */
typedef struct {
	UW	received;	/* samples received so far */
	INT	min;
	INT	max;
	W	sum;
} APP_STATS;

IMPORT ID		mtx_stats;
IMPORT APP_STATS	app_stats;

/* Private to subsystem 3 */
IMPORT void	consumer_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM3_H */
