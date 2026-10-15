/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	Results are recorded in testing/subsystem5/TEST_LOG.md.
 *
 *	The HC-SR04+ and the servo belong to the Obstacle Scan and Recovery
 *	task, so these tests never drive them directly: they check the pure
 *	logic, then watch what the running task reports.
 */

#include "subsystem5.h"
#include "../test_report.h"
#include "obstacle_profile.h"
#include "b5_config.h"

#if APP_TEST

/* A scan recorded from the host simulation (testing/host): a 100 mm wide
   flat obstacle 300 mm straight ahead, 7.5 deg beam half-angle. */
LOCAL const struct { INT angle_dd; INT range_mm; } s5_scan_100mm[] = {
	{  300, 4000 }, {  600, 4000 }, {  675, 4000 }, {  712, 4000 },
	{  721, 4000 }, {  731,  304 }, {  750,  303 }, {  900,  300 },
	{ 1050,  303 }, { 1068,  304 }, { 1077, 4000 }, { 1087, 4000 },
	{ 1125, 4000 }, { 1200, 4000 }, { 1500, 4000 },
};

LOCAL b5_params_t s5_params(void)
{
	b5_params_t p;

	p.near_mm           = B5_NEAR_MM;
	p.alpha_dd          = B5_ALPHA_DD;
	p.scan_min_dd       = B5_SCAN_START_DD;
	p.scan_max_dd       = B5_SCAN_END_DD;
	p.car_width_mm      = B5_CAR_WIDTH_MM;
	p.car_length_mm     = B5_CAR_LENGTH_MM;
	p.margin_mm         = B5_MARGIN_MM;
	p.obst_depth_mm     = B5_OBST_DEPTH_MM;
	p.open_clearance_mm = B5_OPEN_CLEARANCE_MM;
	p.b_prefer_right    = (B5_PREFER_RIGHT != 0);
	return p;
}

EXPORT void subsystem5_test(void)
{
	b5_scan_t	scan;
	b5_profile_t	prof;
	b5_params_t	par = s5_params();
	b5_status_t	st0, st1;
	UINT		i;
	UW		pings;

	TEST_INIT();

	/* --- pure logic ------------------------------------------------- */
	TEST_CHECK("S5-U01", (b5_cos_q15(600) > 16380) && (b5_cos_q15(600) < 16390)
			  && (b5_median3(5, 1, 9) == 5),
		   "Q15 trig and median-of-three");

	b5_scan_clear(&scan);
	for (i = 0; i < sizeof(s5_scan_100mm) / sizeof(s5_scan_100mm[0]); i++) {
		(void)b5_scan_add(&scan, (int16_t)s5_scan_100mm[i].angle_dd,
				  s5_scan_100mm[i].range_mm, true);
	}
	b5_build_profile(&scan, &par, &prof);
	TEST_CHECK("S5-U02", prof.b_found && (prof.width_mm >= 90)
			  && (prof.width_mm <= 110),
		   "100 mm obstacle measured within 10 % (NFR-USO-02)");
	TEST_CHECK("S5-U03", prof.side != B5_SIDE_NONE,
		   "open both sides: a bypass side is chosen");

	/* --- the running task ------------------------------------------- */
	b5_get_status(&st0);
	TEST_CHECK("S5-U04", st0.state < B5_ST_COUNT,
		   "Obstacle Scan and Recovery task is running");

	/* Wait up to 5 s for a run to start (the bench simulation starts
	   one by itself; on the real car, start it from the supervisor). */
	for (i = 0; (i < 50U) && (b5_get_state() == B5_ST_IDLE); i++) {
		tk_dly_tsk(100);
	}
	b5_get_status(&st0);

	/* Give the task 1.2 s.  If the car is running it is in Guard and
	   pings; the count shows the sensor answers and the spacing holds. */
	tk_dly_tsk(1200);
	b5_get_status(&st1);
	pings = st1.pings - st0.pings;

	if (pings == 0U) {
		tm_printf((UB*)"[TEST] S5-U05 SKIP  task not pinging (state %s)"
			  " - start a run to test the sensor\n",
			  b5_state_name(st1.state));
	} else {
		TEST_CHECK("S5-U05", (st1.no_echo - st0.no_echo) < pings,
			   "HC-SR04+ answers pings");
		TEST_CHECK("S5-U06", pings <= (1200U / B5_PING_PERIOD_MS) + 1U,
			   "pings at least 60 ms apart (NFR-USO-06)");
	}

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
