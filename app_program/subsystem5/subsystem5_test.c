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
#include "b5_pack.h"

#if APP_TEST

/* A scan recorded from the host simulation (testing/host): a 100 mm wide
   flat obstacle 300 mm straight ahead, 7.5 deg beam half-angle.
   Angles are signed, -900 = left, 0 = ahead, +900 = right. */
LOCAL const struct { INT angle_dd; INT range_mm; } s5_scan_100mm[] = {
	{ -600, 4000 }, { -300, 4000 }, { -225, 4000 }, { -187, 4000 },
	{ -177, 4000 }, { -168,  304 }, { -150,  303 }, {    0,  300 },
	{  150,  303 }, {  169,  304 }, {  179, 4000 }, {  188, 4000 },
	{  225, 4000 }, {  300, 4000 }, {  600, 4000 },
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
	UW		misses;
	b5_tlm_t	tin, tout;
	uint8_t		frame[B5_TLM_FRAME_BYTES];

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

	/* --- minimum-bit representation (Tribunal 1) ---------------------- */
	tin = (b5_tlm_t){ 0 };
	tin.state = B5_ST_BYPASSING;  tin.fault = B5_FAULT_MOTION;
	tin.side = B5_SIDE_LEFT;      tin.servo_dd = -600;
	tin.range_mm = 5000;          tin.width_mm = 104;
	tin.pings = 65535U;           tin.b_found = true;
	b5_tlm_pack(&tin, frame);
	TEST_CHECK("S5-U13", b5_tlm_unpack(frame, B5_TLM_FRAME_BYTES, &tout)
			  && (tout.state == B5_ST_BYPASSING)
			  && (tout.servo_dd == -600) && (tout.side == B5_SIDE_LEFT)
			  && (tout.range_mm == B5_MM_MAX) && (tout.width_mm == 104)
			  && (tout.pings == 65535U) && tout.b_found,
		   "telemetry frame round trip: negative angle, clamped range");

	b5_scan_clear(&scan);
	(void)b5_scan_add(&scan, -900, 4396, true);
	TEST_CHECK("S5-U14", (sizeof(b5_sample_t) == 4U)
			  && (scan.sample[0].range_mm == B5_MM_MAX)
			  && (scan.sample[0].angle_dd == -900),
		   "scan sample in one word; 4396 mm clamps, never wraps to 300");

	TEST_CHECK("S5-U15", (b5_wrap_diff16((int16_t)-32700, 32700) == 136)
			  && (b5_wrap_diff16(100, 250) == -150),
		   "16-bit distance difference correct across a wrap");

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
	pings  = (UW)(uint16_t)(st1.pings - st0.pings);	  /* 16-bit, wraps */
	misses = (UW)(uint16_t)(st1.no_echo - st0.no_echo);

	if (pings == 0U) {
		tm_printf((UB*)"[TEST] S5-U05 SKIP  task not pinging (state %s)"
			  " - start a run to test the sensor\n",
			  b5_state_name(st1.state));
	} else {
		TEST_CHECK("S5-U05", misses < pings,
			   "HC-SR04+ answers pings");
		TEST_CHECK("S5-U06", pings <= (1200U / B5_PING_PERIOD_MS) + 1U,
			   "pings at least 60 ms apart (NFR-USO-06)");
	}

	TEST_SUMMARY("subsystem5");
}

#endif /* APP_TEST */
