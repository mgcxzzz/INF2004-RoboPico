/* Replays real bench scans from 2026-10-31 through the production planner
 * (b5_build_navigation_profile) and checks the 2026-10-31 rules:
 * near-tie -> preferred side, worst-case sideways move, real obstacles
 * still block, and the clearances match what the bench printed.
 *
 * Rule B (2026-10-31 review): a side is chosen only if its whole manoeuvre
 * footprint passes. Expected sides changed for four scans:
 *   cases12_s1  LEFT -> NONE: the rejoin leg (incl. 100 mm search) reaches
 *               behind a measured right-side return (~655 mm at +16.5 deg)
 *   cases12_s4  LEFT -> NONE: grown car on the 45 deg diagonal meets the
 *               321 mm planning envelope's near corner (stop at ~251 mm)
 *   d150_s9/10  RIGHT -> NONE: same, stop at ~167 mm
 * Second pass: the bench's 4000 mm readings were no-target results whose
 * class (completed no-return vs. no fall) was not logged; replayed as weak
 * no-return clears they must raise the WEAK flag and never raise measured
 * confidence. */
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem5/obstacle_profile.h"
#include "../../app_program/subsystem5/b5_config.h"
#include "b5_bench_scans_2026-10-31.h"

static int fails, checks;
#define CHECK(c, msg, name) do { checks++; if (!(c)) { fails++; printf("FAIL %s: %s\n", name, msg); } } while (0)

static const char *sweep_name(unsigned r)
{
    return r == B5_SWEEP_FAIL ? "FAIL" : r == B5_SWEEP_PASS_ASSUMED ? "PASS_ASSUMED" :
           r == B5_SWEEP_PASS ? "PASS" : "not-run";
}

int main(void)
{
    b5_params_t par = { B5_NEAR_MM, B5_ALPHA_DD, B5_SCAN_START_DD, B5_SCAN_END_DD, B5_CAR_WIDTH_MM,
                        B5_CAR_LENGTH_MM, B5_MARGIN_MM, B5_OBST_DEPTH_MM, B5_OPEN_CLEARANCE_MM, B5_PREFER_RIGHT != 0 };
    /* 2026-11-01 measured car geometry (pivot 130 mm behind the front IR
     * bar, sensor 65 mm behind it, width 135): the 45 deg diagonal of the
     * grown car meets the planning envelope from these ~250 mm stops too,
     * except offset_s2_tie on the left (narrower 282 mm envelope). */
    const char *expect[] = { "NONE", "NONE", "NONE", "NONE", "LEFT", "NONE", "NONE", "NONE" };
    for (unsigned k = 0; k < sizeof k_bench / sizeof k_bench[0]; k++) {
        const bench_scan_t *b = &k_bench[k];
        b5_scan_t scan; b5_profile_t p;
        b5_scan_clear(&scan);
        for (unsigned i = 0; i < b->n; i++) (void)b5_scan_add(&scan, b->a[i], b->r[i], b->v[i] != 0U);
        b5_sweep_report_t rl, rr;
        b5_build_navigation_profile_ex(&scan, &par, &p, &rl, &rr);
        const char *side = p.side == B5_SIDE_RIGHT ? "RIGHT" : p.side == B5_SIDE_LEFT ? "LEFT" : "NONE";
        printf("%-26s bench-v0.2 %-5s now %-5s clearance %4u/%-4u (bench %d/%d) lateral %u planning %u span %u\n",
               b->name, b->v02_side, side, (unsigned)p.clear_left_mm, (unsigned)p.clear_right_mm, b->clr_l, b->clr_r,
               (unsigned)p.lateral_mm, (unsigned)p.planning_width_mm, (unsigned)p.width_mm);
        printf("%-26s   sweep L %s(why %u leg %u at %d,%d) R %s(why %u leg %u at %d,%d) evidence 0x%02x\n", "",
               sweep_name(rl.result), rl.why, rl.fail_leg, rl.fail_x_mm, rl.fail_y_mm,
               sweep_name(rr.result), rr.why, rr.fail_leg, rr.fail_x_mm, rr.fail_y_mm, (unsigned)p.evidence_flags);
#if B5_SIDE_COMPARE_SYMMETRIC == 0
        CHECK((int)p.clear_left_mm == b->clr_l && (int)p.clear_right_mm == b->clr_r, "clearances match the bench output", b->name);
#else
        /* 2026-11-01: clearances are now measured over the same angles on
         * both sides (|angle| <= 44 deg), so they differ from the bench's. */
        CHECK(p.clear_left_mm <= (unsigned)b->clr_l || b->clr_l == 0, "left clearance never larger than the full-arc value", b->name);
#endif
        CHECK(strcmp(side, expect[k]) == 0, "side as expected", b->name);
        CHECK(p.planning_width_mm >= B5_EXPECTED_MAX_WIDTH_MM, "planning width >= 200 mm", b->name);
        if (p.side != B5_SIDE_NONE) CHECK((int)p.lateral_mm >= B5_LATERAL_MIN_MM, "sideways move >= worst case", b->name);
        if (p.side == B5_SIDE_LEFT)  CHECK(rl.result == B5_SWEEP_PASS || rl.result == B5_SWEEP_PASS_ASSUMED, "chosen side's footprint passed", b->name);
        if (p.side == B5_SIDE_RIGHT) CHECK(rr.result == B5_SWEEP_PASS || rr.result == B5_SWEEP_PASS_ASSUMED, "chosen side's footprint passed", b->name);

        /* weak replay: 4000 mm readings as clean no-return triplets */
        {
            b5_scan_t w; b5_profile_t pw; unsigned n_weak = 0;
            b5_scan_clear(&w);
            for (unsigned i = 0; i < b->n; i++) {
                b5_sample_t smp = { 0 };
                smp.angle_dd = b->a[i];
                smp.range_mm = (unsigned)b->r[i];
                smp.b_valid = b->v[i] ? 1U : 0U;
                smp.b_stable = smp.b_valid;
                if (b->v[i] && b->r[i] >= 4000) { smp.b_weak = 1U; smp.b_stable = 0U; n_weak++; }
                (void)b5_scan_add_sample(&w, &smp);
            }
            b5_build_navigation_profile(&w, &par, &pw);
            printf("%-26s   weak replay: %u no-return angles, side %s, measured %u%% coverage %u%%, evidence 0x%02x\n", "",
                   n_weak, pw.side == B5_SIDE_RIGHT ? "RIGHT" : pw.side == B5_SIDE_LEFT ? "LEFT" : "NONE",
                   (unsigned)pw.confidence_pct, (unsigned)pw.coverage_pct, (unsigned)pw.evidence_flags);
            if (n_weak > 0) CHECK(pw.confidence_pct < pw.coverage_pct, "weak clears do not raise measured confidence", b->name);
            CHECK(pw.side == B5_SIDE_NONE || pw.side == p.side, "weak replay never picks a side the measured replay rejected", b->name);
        }
    }
    printf("%d checks, %d failed\n", checks, fails);
    return fails != 0;
}
