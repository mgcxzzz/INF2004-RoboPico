/* Host tests for v2.3 + Buddy4 ramp-assist v0.1 (no SDK, no hardware).
 * Build from interface_wifi_v2_3/:  see run_ramp_assist_tests.sh
 *  A. boost 0 => continuous_drive_step bit-identical to the 8d7f3aa original
 *  B. decision logic: persistence, hysteresis, bump rejection, descent,
 *     level re-arm, invalid/stale, timeout latch, pause/stop/not-following
 *  C. controller + simple wheel plant: limits kept, no residual boost or
 *     integrator effect after release, help during a simulated climb load
 *  D. telemetry packing (decoded again by ramp_assist_decode.py in the runner)
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "continuous_drive.h"
#include "ramp_assist_tests/continuous_drive_ref_8d7f3aa.h"
#include "ramp_assist.h"

static int checks;
#define CHECK(c, msg) do { if (!(c)) { printf("FAIL %s (line %d)\n", msg, __LINE__); return 1; } checks++; } while (0)
static unsigned rng = 12345u;
static float frand(float lo, float hi) { rng = rng * 1103515245u + 12345u; return lo + (hi - lo) * (float)((rng >> 8) & 0xffff) / 65535.0f; }

/* ---------- A ---------- */
static int test_identity(void) {
    for (int trial = 0; trial < 200; ++trial) {
        ContinuousDrive a = {0}; SignedWheel pa = {0};
        RefContinuousDrive b = {0}; SignedWheel pb = {0};
        unsigned w = (unsigned)(trial & 1); int32_t out_a = 0, out_b = 0; uint64_t now = 0;
        for (int k = 0; k < 600; ++k) {
            int16_t wanted = (int16_t)((k % 97 < 5) ? 0 : (int)frand(-5, 140));
            float measured = frand(-10, 130), dt = frand(0.005f, 0.03f);
            if (k % 53 == 0) measured = NAN;
            now += (uint64_t)(dt * 1e6f);
            out_a = continuous_drive_step(&a, &pa, w, wanted, measured, dt, now, out_a);
            out_b = ref_continuous_drive_step(&b, &pb, w, wanted, measured, dt, now, out_b);
            CHECK(out_a == out_b, "boost 0 output identical to original");
            CHECK(a.integral == b.integral && a.reference == b.reference && a.duty == b.duty, "boost 0 state identical");
            CHECK(a.boost_pwm == 0.0f, "no added PWM without boost");
        }
    }
    return 0;
}

/* ---------- B ---------- */
static RampAssistInput base_in(uint64_t now) {
    RampAssistInput in = { .now_us = now, .dt = 0.01f, .run_active = true, .auto_mode = true, .following = true };
    return in;
}
/* Drive the logic at 10 ms with a Buddy4 sample every 20 ms from h_of(t). */
typedef int (*profile_fn)(double t, bool *valid);
static float run_profile(RampAssist *a, profile_fn f, double seconds, uint8_t *events, int *nev, float *alpha_max, uint64_t *t0) {
    float last = 0; *alpha_max = 0;
    uint32_t seq = 0; uint64_t tilt_us = 0; int16_t h = 0; bool valid = false;
    for (int k = 0; k < (int)(seconds * 100.0 + 0.5); ++k) {
        uint64_t now = *t0 + (uint64_t)k * 10000u;
        if (k % 2 == 0) { seq++; tilt_us = now; int v = f((double)k * 0.01, &valid); h = (int16_t)v; }
        RampAssistInput in = base_in(now);
        in.tilt_seq = seq; in.tilt_us = tilt_us; in.tilt_valid = valid; in.tilt_mm = valid ? h : 0;
        last = ramp_assist_step(a, &in);
        if (last > *alpha_max) *alpha_max = last;
        if (a->pending_event != 0xff) { if (*nev < 32) events[(*nev)++] = (uint8_t)(a->pending_event == RA_EVT_END ? 100 + a->last_end : a->pending_event); a->pending_event = 0xff; }
    }
    *t0 += (uint64_t)(seconds * 1e6);
    return last;
}
static int flat_noise(double t, bool *v) { *v = true; unsigned k = (unsigned)(t * 50.0 + 0.5); return (int)(k * 7919u % 5u) - 2; }        /* -2..2 */
static int single_spikes(double t, bool *v) { *v = true; unsigned k = (unsigned)(t * 50.0 + 0.5); return k % 25 == 3 ? 6 : 1; }      /* 1-sample 6 mm */
static int bump3(double t, bool *v) { *v = true; unsigned k = (unsigned)(t * 50.0 + 0.5); return (k >= 10 && k < 13) ? 7 : 0; }        /* 60 ms bump */
static int bump4(double t, bool *v) { *v = true; unsigned k = (unsigned)(t * 50.0 + 0.5); return (k >= 10 && k < 14) ? 7 : 0; }        /* 80 ms */
/* Right-wheel 10 mm ramp: flat 0.5 s, rise 0->10 mm over 1.5 s, descent 0.5 s, flat 1 s. */
static int ramp10(double t, bool *v) {
    *v = true;
    if (t < 0.5) return 0;
    if (t < 2.0) return (int)((t - 0.5) / 1.5 * 10.0 + 0.5);
    if (t < 2.5) return (int)(10.0 - (t - 2.0) / 0.5 * 10.0 + 0.5);
    return 0;
}
static int ramp_with_gap(double t, bool *v) { int h = ramp10(t, v); if (t > 1.2 && t < 1.30) *v = false; return h; } /* 100 ms invalid */
static int stuck_high(double t, bool *v) { *v = true; return t < 0.3 ? 0 : 8; }                                       /* stalled on top */

static int test_logic(void) {
    RampAssist a; uint8_t ev[32]; int n; float amax; uint64_t t0 = 1000000;
    ra_reset(&a); n = 0; run_profile(&a, flat_noise, 5.0, ev, &n, &amax, &t0);
    CHECK(n == 0 && amax == 0.0f, "flat noise +/-2 mm never activates");
    ra_reset(&a); n = 0; run_profile(&a, single_spikes, 5.0, ev, &n, &amax, &t0);
    CHECK(n == 0, "isolated one-sample 6 mm spikes never activate");
    ra_reset(&a); n = 0; run_profile(&a, bump3, 1.0, ev, &n, &amax, &t0);
    CHECK(n == 0, "60 ms 7 mm bump rejected by persistence");
    ra_reset(&a); n = 0; run_profile(&a, bump4, 1.0, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[0] == RA_EVT_ACTIVATE && ev[1] == 100 + RA_END_LEVEL, "80 ms bump activates then ends LEVEL (persistence boundary)");
    CHECK(amax < 120.0f, "short activation stays inside the slew ramp (well below full alpha)");

    ra_reset(&a); n = 0; run_profile(&a, ramp10, 3.5, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[0] == RA_EVT_ACTIVATE && ev[1] == 100 + RA_END_DESCENT, "10 mm ramp: one activation, ends at DESCENT");
    CHECK(fabsf(amax - RA_ALPHA_MILLI) < 0.01f, "full alpha reached during the climb, never exceeded");
    CHECK(a.alpha_milli == 0.0f && !a.assisting && !a.latched, "after level return alpha is 0 and episode cleared");
    CHECK(a.active_ms > 500 && a.active_ms < 1500, "active only during the rise");

    ra_reset(&a); n = 0; run_profile(&a, ramp_with_gap, 3.5, ev, &n, &amax, &t0);
    CHECK(n >= 3 && ev[1] == 100 + RA_END_INVALID && ev[2] == RA_EVT_ACTIVATE, "100 ms invalid gap ends INVALID, valid climb re-activates");

    ra_reset(&a); n = 0; run_profile(&a, stuck_high, 6.0, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[1] == 100 + RA_END_TIMEOUT, "held tilt ends at TIMEOUT");
    CHECK(a.latched && a.alpha_milli == 0.0f, "timeout latches: no re-activation while still tilted");
    CHECK(a.episode_us >= RA_TIMEOUT_US && a.episode_us < RA_TIMEOUT_US + 20000u, "cumulative timeout bound");

    /* Eligibility terminations are immediate (alpha forced to 0). */
    const char *names[3] = { "pause", "not following", "stop" };
    for (int which = 0; which < 3; ++which) {
        ra_reset(&a); n = 0; run_profile(&a, stuck_high, 1.0, ev, &n, &amax, &t0);
        CHECK(a.assisting && a.alpha_milli > 0.0f, "assisting before eligibility change");
        RampAssistInput in = base_in(t0); in.tilt_seq = a.last_seq; in.tilt_us = t0 - 5000; in.tilt_valid = true; in.tilt_mm = 8;
        if (which == 0) in.auto_mode = false; else if (which == 1) in.following = false; else in.run_active = false;
        float al = ramp_assist_step(&a, &in);
        CHECK(al == 0.0f && !a.assisting, names[which]);
        CHECK(a.last_end == (which == 0 ? RA_END_PAUSED : which == 1 ? RA_END_NOT_FOLLOWING : RA_END_STOPPED), "termination reason recorded");
        CHECK(a.pending_event == RA_EVT_END, "end event pending for telemetry");
    }
    /* Stale: Buddy4 stops producing samples. */
    ra_reset(&a); n = 0; run_profile(&a, stuck_high, 1.0, ev, &n, &amax, &t0);
    { uint32_t seq = a.last_seq; uint64_t last = a.last_sample_us; float al = 1;
      for (int k = 1; k <= 10; ++k) { RampAssistInput in = base_in(last + (uint64_t)k * 10000u); in.tilt_seq = seq; in.tilt_us = last; in.tilt_valid = true; in.tilt_mm = 8; al = ramp_assist_step(&a, &in); }
      CHECK(!a.assisting && a.last_end == RA_END_STALE, "no Buddy4 sample for >60 ms ends STALE");
      CHECK(al < RA_ALPHA_MILLI, "alpha released after stale"); }
    /* Never activates when not following / paused even with a clear ramp. */
    ra_reset(&a);
    for (int k = 0; k < 300; ++k) { RampAssistInput in = base_in(t0 + (uint64_t)k * 10000u); in.following = false;
        in.tilt_seq = (uint32_t)(k / 2 + 1); in.tilt_us = in.now_us; in.tilt_valid = true; in.tilt_mm = 9; CHECK(ramp_assist_step(&a, &in) == 0.0f, "no assist outside ordinary following"); }
    /* Slew: alpha never moves faster than RA_SLEW per second. */
    ra_reset(&a); { float prev = 0; uint32_t seq = 0;
      for (int k = 0; k < 200; ++k) { RampAssistInput in = base_in(t0 + (uint64_t)k * 10000u); if (k % 2 == 0) seq++; in.tilt_seq = seq; in.tilt_us = in.now_us; in.tilt_valid = true; in.tilt_mm = k < 100 ? 9 : 0;
        float al = ramp_assist_step(&a, &in); CHECK(fabsf(al - prev) <= RA_SLEW_MILLI_PER_S * 0.01f + 1e-3f, "alpha slew bound"); prev = al; } }
    return 0;
}

/* ---------- C ---------- */
typedef struct { float out[2], speed[2], integral[2], boost[2]; } Trace;
/* Wheel plant: first order (tau 80 ms) towards (pwm - load)/slope, where
 * slope is the controller's own feedforward slope (so ff alone holds speed
 * on the flat). The climb load is a PWM-equivalent drag on BOTH wheels, as
 * seen in the lower-bracket capture (both wheels slowed). */
static void simulate(bool assist, float load_pwm, Trace *tr, int steps, int climb_from, int climb_to, int *max_slew_violation, float *max_out, float *max_boost, float *max_abs_i) {
    ContinuousDrive cd[2]; SignedWheel p[2]; memset(cd, 0, sizeof cd); memset(p, 0, sizeof p); int32_t out[2] = {0}; float v[2] = {0};
    RampAssist a; ra_reset(&a); uint32_t seq = 0;
    *max_slew_violation = 0; *max_out = 0; *max_boost = 0; *max_abs_i = 0;
    for (int k = 0; k < steps; ++k) {
        uint64_t now = 1000000u + (uint64_t)k * 10000u; float dt = 0.01f;
        bool climbing = k >= climb_from && k < climb_to;
        int16_t h = climbing ? (int16_t)(1 + (k - climb_from) * 9 / (climb_to - climb_from)) : 0;
        if (k % 2 == 0) seq++;
        RampAssistInput in = base_in(now); in.tilt_seq = seq; in.tilt_us = now - (k % 2) * 10000u; in.tilt_valid = true; in.tilt_mm = h;
        float alpha = ramp_assist_step(&a, &in);
        for (unsigned w = 0; w < 2; ++w) {
            cd[w].boost_cap = RA_PWM_CAP;
            cd[w].boost_frac = assist && cd[w].active ? alpha / 1000.0f : 0.0f;
            int32_t prev = out[w];
            out[w] = continuous_drive_step(&cd[w], &p[w], w, 45, v[w], dt, now, out[w]);
            if (k > 30 && abs(out[w] - prev) > (int)(1200.0f * dt + 1.0f)) (*max_slew_violation)++;
            if (out[w] > *max_out) *max_out = (float)out[w];
            if (cd[w].boost_pwm > *max_boost) *max_boost = cd[w].boost_pwm;
            if (fabsf(cd[w].integral) > *max_abs_i) *max_abs_i = fabsf(cd[w].integral);
            float slope = w == 0 ? 150.0f / 45.0f : 180.0f / 45.0f;
            float drive = ((float)out[w] - (climbing ? load_pwm : 0.0f)) / slope;
            if (drive < 0) drive = 0;
            v[w] += (drive - v[w]) * dt / 0.08f;
            tr[k].out[w] = (float)out[w]; tr[k].speed[w] = v[w]; tr[k].integral[w] = cd[w].integral; tr[k].boost[w] = cd[w].boost_pwm;
        }
    }
}
static Trace tr_off[700], tr_on[700];
static int test_controller(void) {
    int sv0, sv1; float mo0, mo1, mb0, mb1, mi0, mi1;
    const int from = 150, to = 300, steps = 700;          /* 1.5 s climb at 45 mm/s target */
    simulate(false, 60.0f, tr_off, steps, from, to, &sv0, &mo0, &mb0, &mi0);
    simulate(true, 60.0f, tr_on, steps, from, to, &sv1, &mo1, &mb1, &mi1);
    CHECK(sv0 == 0 && sv1 == 0, "1200/s output slew never exceeded (with and without assist)");
    CHECK(mo1 <= CONTINUOUS_PWM_MAX && mo0 <= CONTINUOUS_PWM_MAX, "PWM limit 420 kept");
    CHECK(mb0 == 0.0f && mb1 > 30.0f && mb1 <= RA_PWM_CAP, "added PWM bounded by cap (35/36 at 45 mm/s right)");
    CHECK(mi1 <= CONTINUOUS_I_MAX + 1e-3f, "integral limit kept");
    /* help during the climb: mean speed deficit smaller */
    double def0 = 0, def1 = 0, i0 = 0, i1 = 0;
    for (int k = from + 20; k < to; ++k) for (int w = 0; w < 2; ++w) { def0 += 45 - tr_off[k].speed[w]; def1 += 45 - tr_on[k].speed[w]; if (fabsf(tr_off[k].integral[w]) > i0) i0 = fabsf(tr_off[k].integral[w]); if (fabsf(tr_on[k].integral[w]) > i1) i1 = fabsf(tr_on[k].integral[w]); }
    printf("climb: mean speed deficit off %.2f on %.2f mm/s; peak |I| off %.1f on %.1f\n", def0 / ((to - from - 20) * 2), def1 / ((to - from - 20) * 2), i0, i1);
    CHECK(def1 < def0, "assist reduces the speed deficit during the climb");
    CHECK(i1 <= i0 + 1e-3, "assist never increases integral build-up");
    /* after release: no residual boost, outputs return to the no-assist trajectory */
    int release = -1; for (int k = to; k < steps; ++k) if (tr_on[k].boost[0] == 0.0f && tr_on[k].boost[1] == 0.0f) { release = k; break; }
    CHECK(release > 0 && release - to <= 30, "added PWM gone within 300 ms of level return (200 ms ramp + 40 ms level)");
    double max_out_diff = 0, max_i_diff = 0, overshoot_on = 0, overshoot_off = 0, max_excess = -1e9, end_diff = 0;
    for (int k = release; k < steps; ++k) for (int w = 0; w < 2; ++w) {
        double d = tr_on[k].out[w] - tr_off[k].out[w];
        if (k >= release + 50) { if (fabs(d) > max_out_diff) max_out_diff = fabs(d); if (d > max_excess) max_excess = d; }
        if (k >= steps - 10 && fabs(d) > end_diff) end_diff = fabs(d);
        double di = tr_on[k].integral[w] - tr_off[k].integral[w]; if (di > max_i_diff) max_i_diff = di; }
    for (int k = to; k < steps; ++k) for (int w = 0; w < 2; ++w) {
        if (tr_on[k].speed[w] - 45 > overshoot_on) overshoot_on = tr_on[k].speed[w] - 45;
        if (tr_off[k].speed[w] - 45 > overshoot_off) overshoot_off = tr_off[k].speed[w] - 45; }
    printf("after release: integral excess vs no-assist %.3f; from +500 ms output on-off max %.2f (|max| %.2f), at end %.2f PWM; overshoot on %.2f off %.2f mm/s\n", max_i_diff, max_excess, max_out_diff, end_diff, overshoot_on, overshoot_off);
    CHECK(max_i_diff <= 0.5, "no integrator excess left by assistance (integral never above the no-assist run)");
    CHECK(max_excess <= 1.0, "no residual boost: from 500 ms after release the output is never above the no-assist output");
    CHECK(end_diff <= 3.0, "outputs converge to the no-assist trajectory");
    CHECK(overshoot_on <= overshoot_off + 1.0, "post-ramp overshoot within 1 mm/s of no-assist (abrupt load removal, worst case)");
    /* flat run: assist logic never touches the output */
    for (int k = 0; k < from; ++k) for (int w = 0; w < 2; ++w) CHECK(tr_on[k].out[w] == tr_off[k].out[w], "flat section identical with assist build");
    /* continuous_leave seeding: subtracting boost_pwm gives the no-boost seed */
    { ContinuousDrive s = {0}; SignedWheel pw = {0}; int32_t o = 0; float v = 30;
      for (int k = 0; k < 50; ++k) o = continuous_drive_step(&s, &pw, 1, 45, v, 0.01f, 1000000u + (uint64_t)k * 10000u, o);
      s.boost_cap = RA_PWM_CAP; s.boost_frac = 0.2f;
      for (int k = 50; k < 80; ++k) o = continuous_drive_step(&s, &pw, 1, 45, v, 0.01f, 1000000u + (uint64_t)k * 10000u, o);
      int32_t seed_out = o - (int32_t)(s.boost_pwm + 0.5f);
      CHECK(s.boost_pwm > 0 && seed_out < o && seed_out >= 0, "unboosted seed removes this step's added PWM"); }
    return 0;
}

/* ---------- D ---------- */
static int test_tlm(void) {
    RampAssistTlm t = { .uptime_ms = 0x01020304u, .event = RA_EVT_END, .flags = 0x0b, .last_end = RA_END_DESCENT, .activations = 3,
        .alpha_milli = 200, .pwm_l = 30, .pwm_r = 36, .height_mm = -5, .peak_mm = 9, .int_max = 9, .int_min = -2,
        .valid = 24, .invalid = 1, .longest = 12, .max_i_l = 100, .max_i_r = 55, .sat_ticks = 7, .active_ms = 1234, .peak_pwm = 36 };
    uint8_t f[RA_TLM_FRAME_BYTES]; ra_tlm_pack(&t, 0x5a, f);
    CHECK(sizeof f == 31 && f[0] == 0xf2 && f[1] == 28 && f[2] == 0x5a, "source 15 header, 28-byte payload, 31-byte frame (<= 64)");
    CHECK(f[3] == 4 && f[6] == 1 && f[7] == RA_EVT_END && f[9] == RA_END_DESCENT && (int8_t)f[17] == -5 && f[24] == 100 && f[27] == 0xd2 && f[28] == 0x04, "field offsets");
    for (unsigned i = 0; i < sizeof f; ++i) printf("%02x", f[i]);
    printf("\n");
    return 0;
}

int main(void) {
    if (test_identity() || test_logic() || test_controller() || test_tlm()) return 1;
    printf("PASS ramp-assist v0.1 host tests: %d checks (boost-0 identity vs 8d7f3aa, logic, controller limits/residual, telemetry)\n", checks);
    return 0;
}
