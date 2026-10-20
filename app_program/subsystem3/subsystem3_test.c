/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3 (Line and Barcode).  Built only with:
 *	make TEST=1.  Record results in testing/subsystem3/TEST_LOG.md (and
 *	INT- results in testing/integration/TEST_LOG.md).
 */

#include "subsystem3.h"
#include "s3_barcode.h"
#include "s3_config.h"
#include "s3_line.h"
#include "../test_report.h"

#if APP_TEST

/* Widths of the course sample "*A*" as printed (Barcode_Sample.pdf,
 * 0.1 mm units): narrow about 3.0 mm, wide about 9.0 mm, slightly uneven
 * as a real print is. */
LOCAL const UW s3_sample_a[29] = {
	31, 90, 31, 29, 91, 29, 91, 29, 31,	/* '*' */
	29,					/* gap */
	91, 29, 31, 29, 31, 90, 31, 29, 91,	/* 'A' */
	29,					/* gap */
	31, 89, 31, 29, 91, 29, 91, 29, 31	/* '*' */
};

LOCAL BOOL all_patterns_valid(void)
{
	UINT	i, j;

	for (i = 0; i < S3_C39_CHARS; i++) {
		UW p = s3_c39_pattern(s3_c39_at(i));
		UINT bits = 0;

		for (j = 0; j < 9; j++) {
			bits += (p >> j) & 1U;
		}
		if ((bits != 3) || (s3_c39_char(p) != s3_c39_at(i))) {
			return FALSE;
		}
	}
	return TRUE;
}

EXPORT void subsystem3_test(void)
{
	UW		w[S3_BC_MAX_ELEMENTS];
	UW		rev[29];
	char		text[S3_BC_MAX_CHARS + 1];
	UINT		i, n;
	s3_line_t	ln;
	s3_line_out_t	out;
	s3_tlm_t	tin, tout;
	UB		frame[S3_TLM_FRAME_BYTES];
	BOOL		b_junction = FALSE;
	BOOL		b_lost = FALSE;

	TEST_INIT();

	/* --- template demo (until subsystems 2 and 4 replace theirs) ----- */
	{
		APP_STATS before, after;

		tk_loc_mtx(mtx_stats, TMO_FEVR);
		before = app_stats;
		tk_unl_mtx(mtx_stats);
		tk_dly_tsk(2000);
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		after = app_stats;
		tk_unl_mtx(mtx_stats);
		TEST_CHECK("INT-01", after.received >= before.received + 5,
			   "samples flow from subsystem 2 to subsystem 3");
	}

	/* --- pure logic ------------------------------------------------- */
	TEST_CHECK("S3-U01", all_patterns_valid(),
		   "Code 39 table: 44 characters, 3 wide elements each");

	TEST_CHECK("S3-U02",
		   (s3_bc_decode(s3_sample_a, 29, text, sizeof(text)) == S3_BC_OK)
		   && (text[0] == 'A') && (text[1] == '\0'),
		   "course sample *A* (3 mm / 9 mm print) decodes to A");

	for (i = 0; i < 29; i++) {
		rev[i] = s3_sample_a[28 - i];
	}
	TEST_CHECK("S3-U03",
		   s3_bc_decode(rev, 29, text, sizeof(text)) == S3_BC_BACKWARDS,
		   "the same barcode read backwards: no command (team rule)");

	n = s3_bc_encode("Z", 15, 45, w, S3_BC_MAX_ELEMENTS);
	TEST_CHECK("S3-U04",
		   (n == 29) && (s3_bc_decode(w, n, text, sizeof(text)) == S3_BC_OK)
		   && (text[0] == 'Z'),
		   "half-size *Z* (1.5 mm narrow) decodes to Z");

	s3_line_reset(&ln);
	s3_line_step(&ln, S3_SENSE_LEFT, 2, &out);
	i = (out.left_mm_s < out.right_mm_s);
	for (n = 0; n < 10; n++) {
		s3_line_step(&ln, S3_SENSE_LEFT | S3_SENSE_RIGHT, 2, &out);
		b_junction |= ((out.events & S3_LEV_JUNCTION) != 0);
	}
	s3_line_reset(&ln);
	for (n = 0; n < 200; n++) {
		s3_line_step(&ln, 0, 2, &out);
		b_lost |= ((out.events & S3_LEV_LOST) != 0);
	}
	TEST_CHECK("S3-U05", i && b_junction && b_lost,
		   "line under left sensor steers left; junction; line lost");

	/* --- the running task -------------------------------------------- */
	TEST_CHECK("S3-U06",
		   (s3_get_state() != S3_ST_INIT) && (s3_get_state() != S3_ST_FAULT)
		   && (s3_ir_read() <= 7U),
		   "Line and Barcode task running, sensors readable");

	tin = (s3_tlm_t){ 0 };
	tin.seq = 7; tin.state = S3_ST_AWAIT; tin.sensors = 5;
	tin.last_char = s3_c39_index('Z'); tin.last_cmd = 2;
	tin.barcodes_ok = 255; tin.junctions = 3;
	s3_tlm_pack(&tin, frame);
	TEST_CHECK("S3-U07",
		   s3_tlm_unpack(frame, S3_TLM_FRAME_BYTES, &tout)
		   && (tout.seq == 7) && (tout.state == S3_ST_AWAIT)
		   && (tout.sensors == 5) && (s3_c39_at(tout.last_char) == 'Z')
		   && (tout.barcodes_ok == 255) && (tout.junctions == 3),
		   "telemetry frame round trip (team schema, source 3)");

	TEST_SUMMARY("subsystem3");
}

#endif /* APP_TEST */
