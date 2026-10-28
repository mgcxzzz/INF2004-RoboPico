/*
 *	system.c
 *	Start-up of the shared system tasks.  See system.h and README.md.
 */

#include "system.h"

IMPORT ER sys_bench_init(void);		/* sys_bench.c */
IMPORT ER sys_bench_start(void);

/* Phase 1: shared hardware, then the objects other subsystems post to.
   Safety first, so anything that fails later can already report to it. */
EXPORT ER system_init(void)
{
	ER er;

	er = sys_hal_init();		/* 1 us clock; before any driver */
	if (er == E_OK) {
		er = sys_fault_init();	/* shared fault bitmask */
	}
	if (er == E_OK) {
		er = safety_init();
	}
	if (er == E_OK) {
		er = nav_init();
	}
	if (er == E_OK) {
		er = sys_bench_init();
	}
	return er;
}

/* Phase 2: the tasks, highest priority first. */
EXPORT ER system_start(void)
{
	ER er;

	er = safety_start();
	if (er == E_OK) {
		er = nav_start();
	}
	if (er == E_OK) {
		er = sys_bench_start();
	}
	return er;
}
