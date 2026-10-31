/**
 * @file    test_b5_ruleb.c
 * @brief   Rule B (2026-10-31 review) host tests: three ping classes,
 *          per-angle combination, weak clearance, swept-footprint check.
 *
 * Required cases (review conditions): persistent ECHO-high, disconnected
 * sensor, mixed near/no-return triplets, both sides blocked, an enlarged
 * lateral move that exceeds the available clearance. Plus: weak clears do
 * not inflate measured confidence, the tie band applies only among sides
 * whose footprint passed, unscanned footprint is reported as ASSUMED.
 *
 * World: axis-aligned segments seen from the sensor (x right, y ahead).
 * Rays that hit nothing within 4 m give a completed no-return (the pulse
 * falls after > ECHO_MAX), unless a fault mode is injected.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "obstacle_profile.h"
#include "b5_config.h"

static int g_checks, g_fails;
#define CHECK(c, ...) do { g_checks++; if (!(c)) { g_fails++; printf("  FAIL line %d: ", __LINE__); \
                           printf(__VA_ARGS__); printf("\n"); } } while (0)

#define PI (3.14159265358979)

typedef struct { double x1, y1, x2, y2; } seg_t;     /* horizontal or vertical */

enum { F_NONE = 0, F_STUCK_HIGH, F_NO_RISE, F_NO_FALL, F_SHORT };

static seg_t g_seg[8];
static int   g_nseg;
static int   g_fault_all;               /* fault for every ping          */
static int   g_far_as_echo;             /* old behaviour: far = echo 4000 */
/* per-angle ping override: return 1 and fill p if this ping is overridden */
static int (*g_override)(int angle_dd, int k, b5_ping_t * p);

static double
cast (int angle_dd)
{
    double a = (angle_dd / 10.0) * PI / 180.0;
    double s = sin(a), c = cos(a), best = 1e9;
    int i;
    for (i = 0; i < g_nseg; i++) {
        seg_t const * g = &g_seg[i];
        if (g->y1 == g->y2) {                       /* horizontal y = Y */
            if (c <= 1e-9) continue;
            double t = g->y1 / c, x = t * s;
            if (t > 0 && x >= fmin(g->x1, g->x2) && x <= fmax(g->x1, g->x2) && t < best) best = t;
        } else {                                    /* vertical x = X */
            if (fabs(s) <= 1e-9) continue;
            double t = g->x1 / s, y = t * c;
            if (t > 0 && y >= fmin(g->y1, g->y2) && y <= fmax(g->y1, g->y2) && t < best) best = t;
        }
    }
    return best;
}

static b5_ping_t
sim_ping (int angle_dd, int k)
{
    b5_ping_t p;
    double r;
    if (g_override && g_override(angle_dd, k, &p)) return p;
    switch (g_fault_all) {
        case F_STUCK_HIGH: return b5_classify_ping(false, false, false, false, 0U);
        case F_NO_RISE:    return b5_classify_ping(true, false, false, false, 0U);
        case F_NO_FALL:    return b5_classify_ping(true, true, false, false, 0U);
        case F_SHORT:      return b5_classify_ping(true, false, false, true, 150U);
        default: break;
    }
    r = cast(angle_dd);
    if (r >= 3990.0) {
        if (g_far_as_echo) return b5_classify_ping(true, true, true, false, B5_ECHO_MAX_US);
        return b5_classify_ping(true, true, true, false, B5_ECHO_MAX_US + 8000U);
    }
    return b5_classify_ping(true, true, true, false, (uint32_t)lround(r * 2000.0 / 343.0));
}

static void
sample (b5_scan_t * scan, int angle_dd, int n)
{
    b5_ping_t p[3];
    b5_sample_t s;
    int k;
    for (k = 0; k < n; k++) p[k] = sim_ping(angle_dd, k);
    b5_combine_pings((int16_t)angle_dd, p, (uint8_t)n, &s, NULL);
    (void)b5_scan_add_sample(scan, &s);
}

/* Coarse sweep + edge refinement exactly as task_obstacle.c orders them. */
static void
do_scan (b5_scan_t * scan)
{
    int a, n;
    int16_t r;
    b5_scan_clear(scan);
    for (a = B5_SCAN_START_DD; a <= B5_SCAN_END_DD; a += B5_SCAN_STEP_DD) sample(scan, a, B5_SAMPLES_PER_ANGLE);
    if (b5_scan_has_near(scan, B5_NEAR_MM)) {
        for (n = 0; n < (int)B5_MAX_REFINE_SAMPLES; n++) {
            if (!b5_next_refine_angle(scan, B5_NEAR_MM, B5_REFINE_STOP_DD, &r)) break;
            sample(scan, r, B5_REFINE_PINGS);
        }
    }
}

static b5_params_t
params (void)
{
    b5_params_t p;
    p.near_mm = B5_NEAR_MM; p.alpha_dd = B5_ALPHA_DD;
    p.scan_min_dd = B5_SCAN_START_DD; p.scan_max_dd = B5_SCAN_END_DD;
    p.car_width_mm = B5_CAR_WIDTH_MM; p.car_length_mm = B5_CAR_LENGTH_MM;
    p.margin_mm = B5_MARGIN_MM; p.obst_depth_mm = B5_OBST_DEPTH_MM;
    p.open_clearance_mm = B5_OPEN_CLEARANCE_MM; p.b_prefer_right = (B5_PREFER_RIGHT != 0);
    return p;
}

static void
world_reset (void)
{
    g_nseg = 0; g_fault_all = F_NONE; g_far_as_echo = 0; g_override = NULL;
}

static void
add_seg (double x1, double y1, double x2, double y2)
{
    g_seg[g_nseg++] = (seg_t){ x1, y1, x2, y2 };
}

static char const *
side_name (unsigned s)
{
    return s == B5_SIDE_RIGHT ? "RIGHT" : s == B5_SIDE_LEFT ? "LEFT" : "NONE";
}

static char const *
sweep_name (unsigned r)
{
    return r == B5_SWEEP_FAIL ? "FAIL" : r == B5_SWEEP_PASS_ASSUMED ? "PASS_ASSUMED" :
           r == B5_SWEEP_PASS ? "PASS" : "not-run";
}

static void
plan (b5_scan_t * scan, b5_profile_t * p, b5_sweep_report_t * rl, b5_sweep_report_t * rr, char const * name)
{
    b5_params_t par = params();
    do_scan(scan);
    b5_build_navigation_profile_ex(scan, &par, p, rl, rr);
    printf("  %-34s side %-5s found %u measured %3u%% coverage %3u%% clear L/R %4u/%-4u "
           "sweep L %s(why %u) R %s(why %u) ev 0x%02x\n",
           name, side_name(p->side), (unsigned)p->b_found, (unsigned)p->confidence_pct,
           (unsigned)p->coverage_pct, (unsigned)p->clear_left_mm, (unsigned)p->clear_right_mm,
           sweep_name(rl->result), rl->why, sweep_name(rr->result), rr->why, (unsigned)p->evidence_flags);
}

/* ------------------------------------------------------------------ */

static void
test_classifier (void)
{
    b5_ping_t p;
    printf("classifier\n");
    p = b5_classify_ping(false, true, true, false, 1380U);
    CHECK(p.cls == B5_PING_FAULT && p.why == B5_PING_WHY_STUCK_HIGH, "ECHO high before TRIG is a fault even with a later pulse");
    p = b5_classify_ping(true, false, false, false, 0U);
    CHECK(p.cls == B5_PING_FAULT && p.why == B5_PING_WHY_NO_RISE, "no rise (disconnected) is a fault");
    p = b5_classify_ping(true, true, false, false, 0U);
    CHECK(p.cls == B5_PING_FAULT && p.why == B5_PING_WHY_NO_FALL, "rise without fall is a fault, not open space");
    p = b5_classify_ping(true, false, false, true, 150U);
    CHECK(p.cls == B5_PING_FAULT && p.why == B5_PING_WHY_SHORT, "noise-only is a fault");
    p = b5_classify_ping(true, true, true, false, 150U);
    CHECK(p.cls == B5_PING_FAULT && p.why == B5_PING_WHY_SHORT, "short pulse with fall is a fault");
    p = b5_classify_ping(true, true, true, false, 1380U);
    CHECK(p.cls == B5_PING_ECHO && p.range_mm == 236U, "1380 us echo = 236 mm (got %u)", (unsigned)p.range_mm);
    p = b5_classify_ping(true, true, true, false, B5_ECHO_MAX_US);
    CHECK(p.cls == B5_PING_ECHO, "width == ECHO_MAX is still an echo");
    p = b5_classify_ping(true, true, true, false, B5_ECHO_MAX_US + 1U);
    CHECK(p.cls == B5_PING_NO_RETURN && p.range_mm == B5_RANGE_FAR_MM, "completed long pulse = no-return");
    p = b5_classify_ping(true, true, true, false, 200000U);
    CHECK(p.cls == B5_PING_NO_RETURN && p.width_us == 131071U, "width saturates in its field");
}

static b5_ping_t P(int cls, int mm)
{
    if (cls == B5_PING_ECHO) return b5_classify_ping(true, true, true, false, (uint32_t)(mm * 2000 / 343 + 1));
    if (cls == B5_PING_NO_RETURN) return b5_classify_ping(true, true, true, false, 30000U);
    return b5_classify_ping(false, false, false, false, 0U);
}

static void
test_combiner (void)
{
    b5_ping_t t[3];
    b5_sample_t s;
    b5_ping_counts_t c;
    printf("per-angle combination\n");

    t[0] = P(B5_PING_FAULT, 0); t[1] = P(B5_PING_FAULT, 0); t[2] = P(B5_PING_FAULT, 0);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 0 && s.b_weak == 0 && c.n_fault == 3, "persistent ECHO-high triplet = unknown, never clear");

    t[0] = P(B5_PING_NO_RETURN, 0); t[1] = P(B5_PING_NO_RETURN, 0); t[2] = P(B5_PING_NO_RETURN, 0);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 1 && s.b_weak == 1 && s.b_stable == 0 && s.range_mm == B5_RANGE_FAR_MM, "3 clean no-returns = weak clear");

    t[2] = P(B5_PING_FAULT, 0);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 0, "2 no-returns + fault = unknown (incomplete)");
    b5_combine_pings(0, t, 2, &s, &c);
    CHECK(s.b_valid == 0, "only 2 pings of no-return = unknown (needs %u)", B5_WEAK_CLEAR_PINGS);

    t[0] = P(B5_PING_ECHO, 200); t[1] = P(B5_PING_NO_RETURN, 0); t[2] = P(B5_PING_NO_RETURN, 0);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 1 && s.b_weak == 0 && s.b_stable == 0 && s.range_mm <= 201U && s.range_mm >= 199U,
          "mixed near/no-return = occupied at the near echo (%u)", (unsigned)s.range_mm);

    t[0] = P(B5_PING_NO_RETURN, 0); t[1] = P(B5_PING_ECHO, 900); t[2] = P(B5_PING_ECHO, 300);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 1 && s.range_mm >= 299U && s.range_mm <= 301U && s.b_stable == 0, "nearest echo wins (%u)", (unsigned)s.range_mm);

    t[0] = P(B5_PING_ECHO, 300); t[1] = P(B5_PING_ECHO, 310); t[2] = P(B5_PING_ECHO, 305);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 1 && s.b_stable == 1, "3 echoes within spread = stable");
    t[1] = P(B5_PING_ECHO, 340);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 1 && s.b_stable == 0 && s.range_mm <= 301U, "spread > max = unstable, nearest");

    t[0] = P(B5_PING_ECHO, 300); t[1] = P(B5_PING_ECHO, 300); t[2] = P(B5_PING_FAULT, 0);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 1 && s.b_partial == 1 && s.b_block_only == 0, "echo + 1 fault = partial");
    t[1] = P(B5_PING_FAULT, 0);
    b5_combine_pings(0, t, 3, &s, &c);
    CHECK(s.b_valid == 1 && s.b_block_only == 1, "echo + 2 faults = block-only");
}

/* Sensor faults through the planner. */
static void
test_faults (void)
{
    b5_scan_t scan; b5_profile_t p; b5_sweep_report_t rl, rr;
    printf("sensor faults through the planner\n");

    world_reset(); add_seg(-50, 250, 50, 250);
    g_fault_all = F_STUCK_HIGH;
    plan(&scan, &p, &rl, &rr, "persistent ECHO-high");
    CHECK(p.side == B5_SIDE_NONE && p.coverage_pct == 0 && p.confidence_pct == 0, "ECHO-high: no side, 0%% coverage");
    CHECK(p.b_found == 0 && (p.evidence_flags & B5_PROFILE_EVIDENCE_WEAK_CLEAR) == 0, "ECHO-high never yields clearance");

    world_reset(); add_seg(-50, 250, 50, 250);
    g_fault_all = F_NO_FALL;
    plan(&scan, &p, &rl, &rr, "ECHO rises, never falls");
    CHECK(p.side == B5_SIDE_NONE && p.coverage_pct == 0, "no-fall: no side");

    world_reset(); add_seg(-50, 250, 50, 250);
    g_fault_all = F_NO_RISE;
    plan(&scan, &p, &rl, &rr, "disconnected sensor (no rise)");
    CHECK(p.side == B5_SIDE_NONE && p.coverage_pct == 0, "disconnected: no side");

    world_reset(); add_seg(-50, 250, 50, 250);
    g_fault_all = F_SHORT;
    plan(&scan, &p, &rl, &rr, "noise pulses only");
    CHECK(p.side == B5_SIDE_NONE && p.coverage_pct == 0, "noise only: no side");
}

/* mixed near/no-return: right side rays get one near echo among no-returns */
static int ov_mixed_right(int a, int k, b5_ping_t * p)
{
    if (a >= 220 && a <= 450 && k == 1) { *p = b5_classify_ping(true, true, true, false, (uint32_t)(380 * 2000 / 343)); return 1; }
    return 0;
}
/* left side: no-return, no-return, fault (incomplete) */
static int ov_incomplete_left(int a, int k, b5_ping_t * p)
{
    if (a <= -220 && k == 2) { *p = b5_classify_ping(true, true, false, false, 0U); return 1; }
    return 0;
}

static void
test_mixed (void)
{
    b5_scan_t scan; b5_profile_t p, p0; b5_sweep_report_t rl, rr;
    printf("mixed and incomplete triplets\n");

    /* Reference: open both sides at 400 mm (after a reverse), weak clears */
    world_reset(); add_seg(-50, 400, 50, 400);
    plan(&scan, &p0, &rl, &rr, "open both sides (reference)");
    CHECK(p0.side != B5_SIDE_NONE, "reference picks a side");
    CHECK((p0.evidence_flags & B5_PROFILE_EVIDENCE_WEAK_CLEAR) != 0, "reference relies on no-returns: WEAK flag");
    CHECK(p0.confidence_pct < p0.coverage_pct, "weak clears excluded from measured confidence (%u < %u)",
          (unsigned)p0.confidence_pct, (unsigned)p0.coverage_pct);

    world_reset(); add_seg(-50, 400, 50, 400); g_override = ov_mixed_right;
    plan(&scan, &p, &rl, &rr, "right: one near echo among no-returns");
    CHECK(p.side != B5_SIDE_RIGHT, "a credible near echo blocks the right side");

    world_reset(); add_seg(-50, 400, 50, 400); g_override = ov_incomplete_left;
    plan(&scan, &p, &rl, &rr, "left: no-return x2 + fault");
    CHECK(p.side != B5_SIDE_LEFT, "incomplete left triplets stay unknown, never clear");
    CHECK(p.clear_left_mm < (unsigned)(B5_CAR_WIDTH_MM + 2 * B5_MARGIN_MM),
          "unknown rays never give a passable left gap (%u)", (unsigned)p.clear_left_mm);
}

static void
test_both_blocked (void)
{
    b5_scan_t scan; b5_profile_t p; b5_sweep_report_t rl, rr;
    printf("both sides blocked\n");
    world_reset();
    add_seg(-50, 300, 50, 300);
    add_seg(-230, 100, -230, 1500);              /* wall left  */
    add_seg(230, 100, 230, 1500);                /* wall right */
    plan(&scan, &p, &rl, &rr, "walls 230 mm both sides");
    CHECK(p.side == B5_SIDE_NONE, "both blocked: stay stopped");
    CHECK(rl.result != B5_SWEEP_PASS && rl.result != B5_SWEEP_PASS_ASSUMED &&
          rr.result != B5_SWEEP_PASS && rr.result != B5_SWEEP_PASS_ASSUMED, "neither footprint passes");
}

static void
test_lateral_exceeds (void)
{
    b5_scan_t scan; b5_profile_t p; b5_sweep_report_t rl, rr, r2;
    b5_params_t par = params();
    printf("enlarged lateral move vs. available clearance\n");

    /* Left walled off. Right: a post 440 mm ahead at x ~425 mm limits the
     * gap to ~295 mm, which passes the gap test (>= car + 2 x margin =
     * 255 mm) but the 327 mm lateral floor + half car + margin puts the
     * car's grown side at ~454 mm: through the post. */
    world_reset();
    add_seg(-50, 400, 50, 400);
    add_seg(400, 440, 460, 440);                 /* post right */
    add_seg(-230, 100, -230, 1500);              /* wall left  */
    plan(&scan, &p, &rl, &rr, "left walled, right post ~425 mm");
    CHECK(p.clear_right_mm >= (unsigned)(B5_CAR_WIDTH_MM + 2 * B5_MARGIN_MM),
          "right gap alone passes the gap test (%u)", (unsigned)p.clear_right_mm);
    CHECK(rr.result == B5_SWEEP_FAIL && (rr.why == B5_SWEEP_WHY_RETURN || rr.why == B5_SWEEP_WHY_BEYOND),
          "right footprint fails on the post (why %u leg %u at %d,%d)", rr.why, rr.fail_leg, rr.fail_x_mm, rr.fail_y_mm);
    CHECK(p.side == B5_SIDE_NONE, "stays stopped (got %s)", side_name(p.side));
    CHECK((p.evidence_flags & B5_PROFILE_EVIDENCE_SWEEP_FAIL_R) != 0, "SWEEP_FAIL_R flagged");

    (void)b5_check_sweep(&scan, &par, &p, B5_SIDE_RIGHT, 600, (int32_t)p.forward_mm + 300, B5_SEARCH_WIDTH_MM, &r2);
    CHECK(r2.result == B5_SWEEP_FAIL, "600 mm lateral right fails too");

    /* Tie band only among passing sides: right gap ~310 mm (post), left
     * gap ~349 mm (far wall at 700 mm): within the 50 mm tie band, right
     * preferred, but right's footprint fails -> LEFT. */
    CHECK(B5_PREFER_RIGHT != 0, "test assumes right preferred");
    world_reset();
    add_seg(-50, 400, 50, 400);
    add_seg(420, 455, 480, 455);                 /* post right */
    add_seg(-700, 0, -700, 1500);                /* far wall left */
    plan(&scan, &p, &rl, &rr, "tie: right post, left far wall");
    CHECK(p.clear_left_mm >= (unsigned)(B5_CAR_WIDTH_MM + 2 * B5_MARGIN_MM) &&
          p.clear_right_mm >= (unsigned)(B5_CAR_WIDTH_MM + 2 * B5_MARGIN_MM), "both gaps pass the gap test");
    CHECK((int)p.clear_left_mm - (int)p.clear_right_mm <= B5_SIDE_TIE_MM &&
          (int)p.clear_right_mm - (int)p.clear_left_mm <= B5_SIDE_TIE_MM,
          "gaps within the tie band (%u/%u)", (unsigned)p.clear_left_mm, (unsigned)p.clear_right_mm);
    CHECK(rr.result == B5_SWEEP_FAIL && (rl.result == B5_SWEEP_PASS || rl.result == B5_SWEEP_PASS_ASSUMED),
          "right footprint fails, left passes");
    CHECK(p.side == B5_SIDE_LEFT, "takes the passing side, not the preference (got %s)", side_name(p.side));
}

static void
test_unscanned (void)
{
    b5_scan_t scan; b5_profile_t p; b5_sweep_report_t rl, rr;
    printf("unscanned footprint and stop distance\n");
    world_reset(); add_seg(-50, 400, 50, 400); g_far_as_echo = 1;
    plan(&scan, &p, &rl, &rr, "open, far readings measured");
    CHECK(p.side != B5_SIDE_NONE, "picks a side");
    CHECK((p.side == B5_SIDE_RIGHT ? rr.n_assumed : rl.n_assumed) > 0, "rear / outside-arc footprint is ASSUMED");
    CHECK((p.evidence_flags & B5_PROFILE_EVIDENCE_SWEEP_ASSUMED) != 0, "SWEEP_ASSUMED flagged");

    world_reset(); add_seg(-50, 130, 50, 130); g_far_as_echo = 1;
    plan(&scan, &p, &rl, &rr, "stopped at 130 mm");
    CHECK(p.side == B5_SIDE_NONE && rl.result == B5_SWEEP_FAIL && rr.result == B5_SWEEP_FAIL &&
          rl.why == B5_SWEEP_WHY_ENVELOPE && rl.fail_leg <= 2U,
          "130 mm: spin/diagonal footprint meets the envelope -> Blocked/reverse (leg %u)", rl.fail_leg);
}

/* 2026-11-01: asymmetric sweep (-55..+44) must not bias the side choice. */
static void
test_symmetric_compare (void)
{
    b5_scan_t scan; b5_profile_t p; b5_sweep_report_t rl, rr;
    printf("side comparison over the same angles (sweep %d..%d dd)\n",
           B5_SCAN_START_DD, B5_SCAN_END_DD);

    world_reset(); add_seg(-92, 400, 92, 400); g_far_as_echo = 1;
    plan(&scan, &p, &rl, &rr, "185 box centred, sides open");
    CHECK(((int)p.clear_left_mm - (int)p.clear_right_mm <= B5_SIDE_TIE_MM) &&
          ((int)p.clear_right_mm - (int)p.clear_left_mm <= B5_SIDE_TIE_MM),
          "centred box: clearances within the tie band despite the longer left sweep (%u/%u)",
          (unsigned)p.clear_left_mm, (unsigned)p.clear_right_mm);

    world_reset(); add_seg(-152, 400, 32, 400); g_far_as_echo = 1;
    plan(&scan, &p, &rl, &rr, "185 box shifted 60 mm left");
    CHECK(p.clear_right_mm > p.clear_left_mm, "left-shifted box: more room measured on the right (%u/%u)",
          (unsigned)p.clear_left_mm, (unsigned)p.clear_right_mm);
    CHECK(p.side != B5_SIDE_LEFT, "left-shifted box: never LEFT (got %s)", side_name(p.side));

    world_reset(); add_seg(-32, 400, 152, 400); g_far_as_echo = 1;
    plan(&scan, &p, &rl, &rr, "185 box shifted 60 mm right");
    CHECK(p.clear_left_mm > p.clear_right_mm, "right-shifted box: more room measured on the left (%u/%u)",
          (unsigned)p.clear_left_mm, (unsigned)p.clear_right_mm);
    CHECK(p.side != B5_SIDE_RIGHT, "right-shifted box: never RIGHT (got %s)", side_name(p.side));

#if (B5_SCAN_START_DD != -B5_SCAN_END_DD)
    /* An object in the left bypass path but seen only beyond -44 deg (outside
     * the comparison window) still blocks the left via the footprint check.
     * Only meaningful while the sweep is asymmetric. */
    world_reset(); add_seg(-50, 400, 50, 400); add_seg(-460, 380, -380, 380); g_far_as_echo = 1;
    plan(&scan, &p, &rl, &rr, "object in left path at about -49 deg");
    CHECK(rl.result == B5_SWEEP_FAIL, "left footprint fails on it (got %u)", rl.result);
    CHECK(p.side != B5_SIDE_LEFT, "never LEFT (got %s)", side_name(p.side));
#endif
}

int
main (void)
{
    printf("rule B host tests (pivot-to-front %d mm ASSUMED, margin %d mm, turn %d dd)\n",
           B5_PIVOT_TO_FRONT_MM, B5_MARGIN_MM, B5_SWEEP_TURN_DD);
    test_classifier();
    test_combiner();
    test_faults();
    test_mixed();
    test_both_blocked();
    test_lateral_exceeds();
    test_unscanned();
    test_symmetric_compare();
    printf("%d checks, %d failed\n", g_checks, g_fails);
    return g_fails != 0;
}
