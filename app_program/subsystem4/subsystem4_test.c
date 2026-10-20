/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4 (IMU Terrain).  Built only with:
 *	make TEST=1.  Results are recorded in testing/subsystem4/TEST_LOG.md.
 *
 *	The GY-511 and I2C1 belong to the IMU task, so these tests never touch
 *	the bus: they check the pure logic on the Pico itself, then watch what
 *	the running task reports.  Keep the car STILL while they run (the
 *	task calibrates in the first 2 s).  The host tests in
 *	testing/subsystem4/host cover the statechart in much more detail.
 */

#include "subsystem4.h"
#include "../test_report.h"
#include "imu_proc.h"

#if APP_TEST

/* Feed one 20 ms period: 8 samples (car frame, mg). */
LOCAL void s4_feed(b4_ctx_t *c, b4_out_t *o, UW *t_us, INT fwd, INT up,
		   INT dist_mm, BOOL spike)
{
	static b4_in_t const	zero_in;
	static b4_out_t const	zero_out;
	static UINT		k = 0U;	/* 0..2 mg dither: a real sensor is
					   never perfectly steady (and the
					   task treats frozen data as a fault) */
	b4_in_t			in = zero_in;
	UINT			i;

	*o = zero_out;
	*t_us += B4_PERIOD_MS * 1000U;
	in.b_read_ok   = true;
	in.n           = 8U;
	in.t_us        = *t_us;
	in.gap_us      = B4_PERIOD_MS * 1000U;
	in.b_motion    = true;
	in.distance_mm = (int16_t)dist_mm;
	k = (k + 1U) % 3U;
	for (i = 0U; i < in.n; i++) {
		in.s[i].fwd_mg = (int16_t)fwd;
		in.s[i].up_mg  = (int16_t)(up + (INT)k);
	}
	if (spike) {
		in.s[5].up_mg = (int16_t)(up + 2 * B4_SHOCK_MG);
	}
	b4_tick(c, &in, o);
}

EXPORT void subsystem4_test(void)
{
	static b4_ctx_t	c;
	static b4_out_t const zero_out;
	b4_out_t	o = zero_out;
	UW		t = 0U;
	INT		i, d = 0;
	BOOL		peak_ok = FALSE, impact = FALSE;
	static imu_tlm_t const zero_tlm;
	imu_tlm_t	tin = zero_tlm, tout;
	uint8_t		frame[IMU_TLM_FRAME_BYTES];
	UW		p0, m0;

	TEST_INIT();

	/* S4-U01: the real task found the GY-511 and calibrated (car still).
	   The test run starts right after boot, so allow 4 s. */
	for (i = 0; (i < 40) && !imu_data_valid(); i++) {
		tk_dly_tsk(100);
	}
	TEST_CHECK("S4-U01", imu_data_valid(),
		   "GY-511 answers and calibration is valid (car still)");

	/* S4-U02: 50 Hz with no missed frames over 5 s (NFR-IMU-01). */
	p0 = imu_periods();
	m0 = imu_frames_missed();
	tk_dly_tsk(5000);
	TEST_CHECK("S4-U02",
		   (imu_periods() - p0 >= 248U) && (imu_periods() - p0 <= 252U) &&
		   (imu_frames_missed() == m0),
		   "250 periods of 20 ms in 5 s, none missed");

	/* S4-U03: arcsine table (pitch from gravity). */
	TEST_CHECK("S4-U03",
		   (b4_asin_dd(0) == 0) && (b4_asin_dd(16384) == 300) &&
		   (b4_asin_dd(-16384) == -300) && (b4_asin_dd(32767) >= 890),
		   "asin(0)=0, asin(0.5)=30.0 deg, asin(-0.5)=-30.0 deg");

	/* S4-U04: a 20 mm hump with a 120 mm wheelbase, profiled from pitch
	   and distance on the Pico itself, is within NFR-IMU-02. */
	b4_reset(&c);
	b4_init_done(&c, true, t, &o);
	for (i = 0; i < 110; i++) {			/* calibrate, 2.2 s */
		s4_feed(&c, &o, &t, 0, 1000, 0, FALSE);
	}
	b4_mission(&c, B4_MISSION_START, t, &o);
	/* front climbs 20 mm over 40 mm, flat 80 mm, down over 40 mm: the
	   body pitch follows the two contacts 120 mm apart */
	for (i = 0; (i < 400) && !peak_ok; i++) {
		INT s = d, sr = d - B4_WHEELBASE_MM, hf, hr, sinq;
		hf = (s < 200) ? 0 : (s < 240) ? (s - 200) / 2 :
		     (s < 320) ? 20 : (s < 360) ? (360 - s) / 2 : 0;
		hr = (sr < 200) ? 0 : (sr < 240) ? (sr - 200) / 2 :
		     (sr < 320) ? 20 : (sr < 360) ? (360 - sr) / 2 : 0;
		sinq = ((hf - hr) * 1000) / B4_WHEELBASE_MM;	/* mg */
		d += 3;						/* 150 mm/s */
		s4_feed(&c, &o, &t, sinq,
			(INT)b4_isqrt((uint32_t)(1000000 - (sinq * sinq))),
			d, FALSE);
		if ((o.n_events > 0U) &&
		    (o.events[0].type == IMU_EVT_HUMP_PEAK)) {
			peak_ok = b4_nfr02_ok((INT)o.events[0].height_mm, 20);
		}
	}
	TEST_CHECK("S4-U04", peak_ok,
		   "20 mm hump measured within max(5 mm, 10 %)");

	/* S4-U05: a shock is flagged in the same 20 ms period (NFR-IMU-03). */
	for (i = 0; i < 20; i++) {
		s4_feed(&c, &o, &t, 0, 1000, d, FALSE);
	}
	s4_feed(&c, &o, &t, 0, 1000, d, TRUE);
	impact = o.b_shock && (c.state == IMU_ST_IMPACT) &&
		 (o.shock_age_us < (B4_SHOCK_LIMIT_MS * 1000U));
	TEST_CHECK("S4-U05", impact, "shock detected in the period it happened");

	/* S4-U06: telemetry frame round trip, min / max / negative values. */
	tin.seq = 255U; tin.state = IMU_ST_FAULT; tin.motion = IMU_MOTION_IMPACT;
	tin.evt = IMU_EVT_RUN_SUMMARY; tin.detail = 15U; tin.b_valid = true;
	tin.pitch_dd = -900; tin.fault = 15U; tin.height_mm = 255U;
	tin.shock_mg = 4095U; tin.shock_ms = 63U; tin.missed = 65535U;
	tin.t_ms = 1234U;
	imu_tlm_pack(&tin, frame);
	TEST_CHECK("S4-U06",
		   imu_tlm_unpack(frame, IMU_TLM_FRAME_BYTES, &tout) &&
		   (tout.pitch_dd == -900) && (tout.height_mm == 255U) &&
		   (tout.shock_mg == 4095U) && (tout.missed == 65535U) &&
		   (tout.state == IMU_ST_FAULT) && (tout.t_ms == 1234U),
		   "IMU frame packs and unpacks (min, max, negative)");

	TEST_SUMMARY("subsystem4");
}

#endif /* APP_TEST */
