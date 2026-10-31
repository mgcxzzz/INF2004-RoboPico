/* Host tests for v2.3 + Buddy4 ramp-assist v0.2 (no SDK, no hardware).
 * v0.2 adds E: replay of the 2026-10-30 lower-bracket shadow climb (v0.1
 * descent cutoff reproduced; v0.2 holds assistance to the timeout) and
 * noisy-climb / genuine-descent / dip cases.
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
#include "src/motion/continuous_drive.h"
#include "tests/ramp/continuous_drive_ref_8d7f3aa.h"
#include "src/motion/ramp_assist.h"
#include "tests/ramp/ramp_assist_v0_1_ref.h"
#include "tests/ramp/ramp_assist_v0_2_ref.h"
#include "tests/ramp/ramp_assist_v0_3_ref.h"
#include "src/imu/imu_proc.h" /* B4_Q_* quality reason bits (v0.3 skip rule) */

static int checks;
#define CHECK(c, msg)                                                                              \
    do {                                                                                           \
        if (!(c)) {                                                                                \
            printf("FAIL %s (line %d)\n", msg, __LINE__);                                          \
            return 1;                                                                              \
        }                                                                                          \
        checks++;                                                                                  \
    } while (0)
static unsigned rng = 12345u;
static float frand(float lo, float hi) {
    rng = rng * 1103515245u + 12345u;
    return lo + (hi - lo) * (float)((rng >> 8) & 0xffff) / 65535.0f;
}

/* ---------- A ---------- */
static int test_identity(void) {
    for (int trial = 0; trial < 200; ++trial) {
        ContinuousDrive a = {0};
        SignedWheel pa = {0};
        RefContinuousDrive b = {0};
        SignedWheel pb = {0};
        unsigned w = (unsigned)(trial & 1);
        int32_t out_a = 0, out_b = 0;
        uint64_t now = 0;
        for (int k = 0; k < 600; ++k) {
            int16_t wanted = (int16_t)((k % 97 < 5) ? 0 : (int)frand(-5, 140));
            float measured = frand(-10, 130), dt = frand(0.005f, 0.03f);
            if (k % 53 == 0)
                measured = NAN;
            now += (uint64_t)(dt * 1e6f);
            out_a = continuous_drive_step(&a, &pa, w, wanted, measured, dt, now, out_a);
            out_b = ref_continuous_drive_step(&b, &pb, w, wanted, measured, dt, now, out_b);
            CHECK(out_a == out_b, "boost 0 output identical to original");
            CHECK(a.integral == b.integral && a.reference == b.reference && a.duty == b.duty,
                  "boost 0 state identical");
            CHECK(a.boost_pwm == 0.0f, "no added PWM without boost");
        }
    }
    return 0;
}

/* ---------- B ---------- */
static RampAssistInput base_in(uint64_t now) {
    RampAssistInput in = {.now_us = now,
                          .dt = 0.01f,
                          .run_active = true,
                          .auto_mode = true,
                          .following = true,
                          .speed_mm_s = {40.0f, 40.0f}}; /* moving unless a test stalls it (v0.4) */
    return in;
}
/* Drive the logic at 10 ms with a Buddy4 sample every 20 ms from h_of(t). */
typedef int (*profile_fn)(double t, bool *valid);
static float run_profile(RampAssist *a, profile_fn f, double seconds, uint8_t *events, int *nev,
                         float *alpha_max, uint64_t *t0) {
    float last = 0;
    *alpha_max = 0;
    uint32_t seq = 0;
    uint64_t tilt_us = 0;
    int16_t h = 0;
    bool valid = false;
    for (int k = 0; k < (int)(seconds * 100.0 + 0.5); ++k) {
        uint64_t now = *t0 + (uint64_t)k * 10000u;
        if (k % 2 == 0) {
            seq++;
            tilt_us = now;
            int v = f((double)k * 0.01, &valid);
            h = (int16_t)v;
        }
        RampAssistInput in = base_in(now);
        in.tilt_seq = seq;
        in.tilt_us = tilt_us;
        in.tilt_valid = valid;
        in.tilt_mm = valid ? h : 0;
        last = ramp_assist_step(a, &in);
        if (last > *alpha_max)
            *alpha_max = last;
        if (a->pending_event == RA_EVT_HOLD_END)
            a->pending_event = 0xff; /* v0.6 hold events are tested in H */
        if (a->pending_event != 0xff) {
            if (*nev < 32)
                events[(*nev)++] = (uint8_t)(a->pending_event == RA_EVT_END ? 100 + a->last_end
                                                                            : a->pending_event);
            a->pending_event = 0xff;
        }
    }
    *t0 += (uint64_t)(seconds * 1e6);
    return last;
}
static int flat_noise(double t, bool *v) {
    *v = true;
    unsigned k = (unsigned)(t * 50.0 + 0.5);
    return (int)(k * 7919u % 5u) - 2;
} /* -2..2 */
static int single_spikes(double t, bool *v) {
    *v = true;
    unsigned k = (unsigned)(t * 50.0 + 0.5);
    return k % 25 == 3 ? 6 : 1;
} /* 1-sample 6 mm */
static int bump3(double t, bool *v) {
    *v = true;
    unsigned k = (unsigned)(t * 50.0 + 0.5);
    return (k >= 10 && k < 13) ? 7 : 0;
} /* 60 ms bump */
static int bump4(double t, bool *v) {
    *v = true;
    unsigned k = (unsigned)(t * 50.0 + 0.5);
    return (k >= 10 && k < 14) ? 7 : 0;
} /* 80 ms */
/* Right-wheel 10 mm ramp: flat 0.5 s, rise 0->10 mm over 1.5 s, descent 0.5 s, flat 1 s. */
static int ramp10(double t, bool *v) {
    *v = true;
    if (t < 0.5)
        return 0;
    if (t < 2.0)
        return (int)((t - 0.5) / 1.5 * 10.0 + 0.5);
    if (t < 2.5)
        return (int)(10.0 - (t - 2.0) / 0.5 * 10.0 + 0.5);
    return 0;
}
static int ramp_with_gap(double t, bool *v) {
    int h = ramp10(t, v);
    if (t > 1.2 && t < 1.30)
        *v = false;
    return h;
} /* 100 ms invalid */
static int stuck_high(double t, bool *v) {
    *v = true;
    return t < 0.3 ? 0 : 8;
} /* stalled on top */

static int test_logic(void) {
    RampAssist a;
    uint8_t ev[32];
    int n;
    float amax;
    uint64_t t0 = 1000000;
    ra_reset(&a);
    n = 0;
    run_profile(&a, flat_noise, 5.0, ev, &n, &amax, &t0);
    CHECK(n == 0 && amax == 0.0f, "flat noise +/-2 mm never activates");
    ra_reset(&a);
    n = 0;
    run_profile(&a, single_spikes, 5.0, ev, &n, &amax, &t0);
    CHECK(n == 0, "isolated one-sample 6 mm spikes never activate");
    ra_reset(&a);
    n = 0;
    run_profile(&a, bump3, 1.0, ev, &n, &amax, &t0);
    CHECK(n == 0, "60 ms 7 mm bump rejected by persistence");
    ra_reset(&a);
    n = 0;
    run_profile(&a, bump4, 1.0, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[0] == RA_EVT_ACTIVATE && ev[1] == 100 + RA_END_LEVEL,
          "80 ms bump activates then ends LEVEL (persistence boundary)");
    CHECK(amax <= 200.0f, "short activation never exceeds full alpha");

    ra_reset(&a);
    n = 0;
    run_profile(&a, ramp10, 3.5, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[0] == RA_EVT_ACTIVATE && ev[1] == 100 + RA_END_DESCENT,
          "10 mm ramp: one activation, ends at DESCENT");
    CHECK(fabsf(amax - RA_ALPHA_MILLI) < 0.01f,
          "full alpha reached during the climb, never exceeded");
    CHECK(a.alpha_milli == 0.0f && !a.assisting && !a.latched,
          "after level return alpha is 0 and episode cleared");
    CHECK(a.active_ms > 500 && a.active_ms < 1500, "active only during the rise");

    ra_reset(&a);
    n = 0;
    run_profile(&a, ramp_with_gap, 3.5, ev, &n, &amax, &t0);
    CHECK(n >= 3 && ev[1] == 100 + RA_END_INVALID && ev[2] == RA_EVT_ACTIVATE,
          "100 ms invalid gap ends INVALID, valid climb re-activates");

    ra_reset(&a);
    n = 0;
    run_profile(&a, stuck_high, 6.0, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[1] == 100 + RA_END_TIMEOUT, "held tilt ends at TIMEOUT");
    CHECK(a.latched && a.alpha_milli == 0.0f,
          "timeout latches: no re-activation while still tilted");
    CHECK(a.episode_us >= RA_TIMEOUT_US && a.episode_us < RA_TIMEOUT_US + 20000u,
          "cumulative timeout bound");

    /* Eligibility terminations are immediate (alpha forced to 0). */
    const char *names[3] = {"pause", "not following", "stop"};
    for (int which = 0; which < 3; ++which) {
        ra_reset(&a);
        n = 0;
        run_profile(&a, stuck_high, 1.0, ev, &n, &amax, &t0);
        CHECK(a.assisting && a.alpha_milli > 0.0f, "assisting before eligibility change");
        RampAssistInput in = base_in(t0);
        in.tilt_seq = a.last_seq;
        in.tilt_us = t0 - 5000;
        in.tilt_valid = true;
        in.tilt_mm = 8;
        if (which == 0)
            in.auto_mode = false;
        else if (which == 1)
            in.following = false;
        else
            in.run_active = false;
        float al = ramp_assist_step(&a, &in);
        CHECK(al == 0.0f && !a.assisting, names[which]);
        CHECK(a.last_end == (which == 0   ? RA_END_PAUSED
                             : which == 1 ? RA_END_NOT_FOLLOWING
                                          : RA_END_STOPPED),
              "termination reason recorded");
        CHECK(a.pending_event == RA_EVT_END, "end event pending for telemetry");
    }
    /* Stale: Buddy4 stops producing samples. */
    ra_reset(&a);
    n = 0;
    run_profile(&a, stuck_high, 1.0, ev, &n, &amax, &t0);
    {
        uint32_t seq = a.last_seq;
        uint64_t last = a.last_sample_us;
        float al = 1;
        for (int k = 1; k <= 10; ++k) {
            RampAssistInput in = base_in(last + (uint64_t)k * 10000u);
            in.tilt_seq = seq;
            in.tilt_us = last;
            in.tilt_valid = true;
            in.tilt_mm = 8;
            al = ramp_assist_step(&a, &in);
        }
        CHECK(!a.assisting && a.last_end == RA_END_STALE, "no Buddy4 sample for >60 ms ends STALE");
        CHECK(al < RA_ALPHA_MILLI, "alpha released after stale");
    }
    /* Never activates when not following / paused even with a clear ramp. */
    ra_reset(&a);
    for (int k = 0; k < 300; ++k) {
        RampAssistInput in = base_in(t0 + (uint64_t)k * 10000u);
        in.following = false;
        in.tilt_seq = (uint32_t)(k / 2 + 1);
        in.tilt_us = in.now_us;
        in.tilt_valid = true;
        in.tilt_mm = 9;
        CHECK(ramp_assist_step(&a, &in) == 0.0f, "no assist outside ordinary following");
    }
    /* Slew: alpha never moves faster than RA_SLEW per second. */
    ra_reset(&a);
    {
        float prev = 0;
        uint32_t seq = 0;
        for (int k = 0; k < 200; ++k) {
            RampAssistInput in = base_in(t0 + (uint64_t)k * 10000u);
            if (k % 2 == 0)
                seq++;
            in.tilt_seq = seq;
            in.tilt_us = in.now_us;
            in.tilt_valid = true;
            in.tilt_mm = k < 100 ? 9 : 0;
            float al = ramp_assist_step(&a, &in);
            CHECK(fabsf(al - prev) <= RA_SLEW_MILLI_PER_S * 0.01f + 1e-3f, "alpha slew bound");
            prev = al;
        }
    }
    return 0;
}

/* ---------- C ---------- */
typedef struct {
    float out[2], speed[2], integral[2], boost[2];
} Trace;
/* Wheel plant: first order (tau 80 ms) towards (pwm - load)/slope, where
 * slope is the controller's own feedforward slope (so ff alone holds speed
 * on the flat). The climb load is a PWM-equivalent drag on BOTH wheels, as
 * seen in the lower-bracket capture (both wheels slowed). */
static void simulate(bool assist, float load_pwm, Trace *tr, int steps, int climb_from,
                     int climb_to, int *max_slew_violation, float *max_out, float *max_boost,
                     float *max_abs_i) {
    ContinuousDrive cd[2];
    SignedWheel p[2];
    memset(cd, 0, sizeof cd);
    memset(p, 0, sizeof p);
    int32_t out[2] = {0};
    float v[2] = {0};
    RampAssist a;
    ra_reset(&a);
    uint32_t seq = 0;
    *max_slew_violation = 0;
    *max_out = 0;
    *max_boost = 0;
    *max_abs_i = 0;
    for (int k = 0; k < steps; ++k) {
        uint64_t now = 1000000u + (uint64_t)k * 10000u;
        float dt = 0.01f;
        bool climbing = k >= climb_from && k < climb_to;
        int16_t h = climbing ? (int16_t)(1 + (k - climb_from) * 9 / (climb_to - climb_from)) : 0;
        if (k % 2 == 0)
            seq++;
        RampAssistInput in = base_in(now);
        in.tilt_seq = seq;
        in.tilt_us = now - (k % 2) * 10000u;
        in.tilt_valid = true;
        in.tilt_mm = h;
        float alpha = ramp_assist_step(&a, &in);
        for (unsigned w = 0; w < 2; ++w) {
            cd[w].boost_cap = RA_PWM_CAP;
            cd[w].boost_frac = assist && cd[w].active ? alpha / 1000.0f : 0.0f;
            int32_t prev = out[w];
            out[w] = continuous_drive_step(&cd[w], &p[w], w, 45, v[w], dt, now, out[w]);
            if (k > 30 && abs(out[w] - prev) > (int)(1200.0f * dt + 1.0f))
                (*max_slew_violation)++;
            if (out[w] > *max_out)
                *max_out = (float)out[w];
            if (cd[w].boost_pwm > *max_boost)
                *max_boost = cd[w].boost_pwm;
            if (fabsf(cd[w].integral) > *max_abs_i)
                *max_abs_i = fabsf(cd[w].integral);
            float slope = w == 0 ? 150.0f / 45.0f : 180.0f / 45.0f;
            float drive = ((float)out[w] - (climbing ? load_pwm : 0.0f)) / slope;
            if (drive < 0)
                drive = 0;
            v[w] += (drive - v[w]) * dt / 0.08f;
            tr[k].out[w] = (float)out[w];
            tr[k].speed[w] = v[w];
            tr[k].integral[w] = cd[w].integral;
            tr[k].boost[w] = cd[w].boost_pwm;
        }
    }
}
static Trace tr_off[700], tr_on[700];
static int test_controller(void) {
    int sv0, sv1;
    float mo0, mo1, mb0, mb1, mi0, mi1;
    const int from = 150, to = 300, steps = 700; /* 1.5 s climb at 45 mm/s target */
    simulate(false, 60.0f, tr_off, steps, from, to, &sv0, &mo0, &mb0, &mi0);
    simulate(true, 60.0f, tr_on, steps, from, to, &sv1, &mo1, &mb1, &mi1);
    CHECK(sv0 == 0 && sv1 == 0, "1200/s output slew never exceeded (with and without assist)");
    CHECK(mo1 <= CONTINUOUS_PWM_MAX && mo0 <= CONTINUOUS_PWM_MAX, "PWM limit 420 kept");
    CHECK(mb0 == 0.0f && mb1 > 30.0f && mb1 <= RA_PWM_CAP,
          "added PWM bounded by cap (35/36 at 45 mm/s right)");
    CHECK(mi1 <= CONTINUOUS_I_MAX + 1e-3f, "integral limit kept");
    /* help during the climb: mean speed deficit smaller */
    double def0 = 0, def1 = 0, i0 = 0, i1 = 0;
    for (int k = from + 20; k < to; ++k)
        for (int w = 0; w < 2; ++w) {
            def0 += 45 - tr_off[k].speed[w];
            def1 += 45 - tr_on[k].speed[w];
            if (fabsf(tr_off[k].integral[w]) > i0)
                i0 = fabsf(tr_off[k].integral[w]);
            if (fabsf(tr_on[k].integral[w]) > i1)
                i1 = fabsf(tr_on[k].integral[w]);
        }
    printf("climb: mean speed deficit off %.2f on %.2f mm/s; peak |I| off %.1f on %.1f\n",
           def0 / ((to - from - 20) * 2), def1 / ((to - from - 20) * 2), i0, i1);
    CHECK(def1 < def0, "assist reduces the speed deficit during the climb");
    CHECK(i1 <= i0 + 1e-3, "assist never increases integral build-up");
    /* after release: no residual boost, outputs return to the no-assist trajectory */
    int release = -1;
    for (int k = to; k < steps; ++k)
        if (tr_on[k].boost[0] == 0.0f && tr_on[k].boost[1] == 0.0f) {
            release = k;
            break;
        }
    CHECK(release > 0 && release - to <= 30,
          "added PWM gone within 300 ms of level return (200 ms ramp + 40 ms level)");
    double max_out_diff = 0, max_i_diff = 0, overshoot_on = 0, overshoot_off = 0, max_excess = -1e9,
           end_diff = 0;
    for (int k = release; k < steps; ++k)
        for (int w = 0; w < 2; ++w) {
            double d = tr_on[k].out[w] - tr_off[k].out[w];
            if (k >= release + 50) {
                if (fabs(d) > max_out_diff)
                    max_out_diff = fabs(d);
                if (d > max_excess)
                    max_excess = d;
            }
            if (k >= steps - 10 && fabs(d) > end_diff)
                end_diff = fabs(d);
            double di = tr_on[k].integral[w] - tr_off[k].integral[w];
            if (di > max_i_diff)
                max_i_diff = di;
        }
    for (int k = to; k < steps; ++k)
        for (int w = 0; w < 2; ++w) {
            if (tr_on[k].speed[w] - 45 > overshoot_on)
                overshoot_on = tr_on[k].speed[w] - 45;
            if (tr_off[k].speed[w] - 45 > overshoot_off)
                overshoot_off = tr_off[k].speed[w] - 45;
        }
    printf(
        "after release: integral excess vs no-assist %.3f; from +500 ms output on-off max %.2f (|max| %.2f), at end %.2f PWM; overshoot on %.2f off %.2f mm/s\n",
        max_i_diff, max_excess, max_out_diff, end_diff, overshoot_on, overshoot_off);
    CHECK(max_i_diff <= 0.5,
          "no integrator excess left by assistance (integral never above the no-assist run)");
    CHECK(
        max_excess <= 1.0,
        "no residual boost: from 500 ms after release the output is never above the no-assist output");
    CHECK(end_diff <= 3.0, "outputs converge to the no-assist trajectory");
    CHECK(
        overshoot_on <= overshoot_off + 3.0,
        "post-ramp overshoot within 3 mm/s of no-assist (v0.2 100 ms filtered level + 200 ms release; abrupt load removal = worst case)");
    /* flat run: assist logic never touches the output */
    for (int k = 0; k < from; ++k)
        for (int w = 0; w < 2; ++w)
            CHECK(tr_on[k].out[w] == tr_off[k].out[w], "flat section identical with assist build");
    /* continuous_leave seeding: subtracting boost_pwm gives the no-boost seed */
    {
        ContinuousDrive s = {0};
        SignedWheel pw = {0};
        int32_t o = 0;
        float v = 30;
        for (int k = 0; k < 50; ++k)
            o = continuous_drive_step(&s, &pw, 1, 45, v, 0.01f, 1000000u + (uint64_t)k * 10000u, o);
        s.boost_cap = RA_PWM_CAP;
        s.boost_frac = 0.2f;
        for (int k = 50; k < 80; ++k)
            o = continuous_drive_step(&s, &pw, 1, 45, v, 0.01f, 1000000u + (uint64_t)k * 10000u, o);
        int32_t seed_out = o - (int32_t)(s.boost_pwm + 0.5f);
        CHECK(s.boost_pwm > 0 && seed_out < o && seed_out >= 0,
              "unboosted seed removes this step's added PWM");
    }
    return 0;
}

/* ---------- E: replay ---------- */
/* Sequence reconstructed (20 ms samples) from the 2026-10-30 02:27 shadow run
 * (capture 2026-10-29_buddy4_drive_v1_1_codex1_capture.txt lines 593618..594558):
 * source-15 interval stats (max/min/valid/longest>=4) and the activation/end
 * frames (activate at h=4; v0.1 END=DESCENT after 241 ms, peak 7, current 4),
 * source-2 snapshots (7,5,7,5,8 rocking), then 8..9 mm while stalled at the
 * crest under command for ~7 s with both integrals at the limit. */
static const int8_t replay_seq[] = {
    /* interval to 254669: max 6 min 0, longest run>=4 = 2 */
    0, 1, 0, 1, 2, 1, 2, 1, 2, 3, 2, 3, 2, 3, 2, 3, 1, 2, 3, 2, 3, 4, 6, 2, 3,
    /* to activation (254922): 4 consecutive >= 4, current 4 */
    3, 2, 3, 2, 3, 3, 2, 3, 4, 4, 5, 4,
    /* 241 ms after activation: rocking up to 7, two samples at 4 (v0.1 cut) */
    5, 6, 7, 6, 5, 7, 6, 5, 6, 4, 4, 4,
    /* interval max 8 min 4 */
    5, 6, 7, 8, 6, 5, 4, 6, 7, 8, 6, 5, 7, 6, 4, 5, 7, 8, 6, 5, 7, 6, 5, 6, 7,
    /* interval max 10 min 6 */
    7, 8, 9, 8, 7, 6, 8, 9, 10, 9, 8, 7, 8, 9, 10, 9, 8, 9, 8, 7, 8, 9, 10, 9, 8,
    /* interval max 11 min 5 */
    9, 10, 11, 9, 8, 5, 9, 10, 9, 8, 9, 10, 9, 8, 9, 9, 10, 9, 8, 9, 10, 9, 8, 9, 9};
static int replay_run(int which, uint8_t *end_reason, uint32_t *active_ms, uint64_t *end_at_ms,
                      float *alpha_at_2s) {
    RampAssist a;
    RampAssist01 b;
    ra_reset(&a);
    ra01_reset(&b);
    const int n_seq = (int)sizeof replay_seq;
    const int samples = n_seq + 7 * 50; /* + ~7 s stalled at 8/9 mm */
    uint64_t t0 = 254163000ull, act_at = 0;
    *end_reason = 0;
    *active_ms = 0;
    *end_at_ms = 0;
    *alpha_at_2s = -1;
    for (int k = 0; k < samples * 2; ++k) { /* 10 ms control steps */
        uint64_t now = t0 + (uint64_t)k * 10000u;
        int i = k / 2;
        int16_t h = (int16_t)(i < n_seq ? replay_seq[i] : (i % 2 ? 9 : 8));
        RampAssistInput in = base_in(now);
        in.tilt_seq = (uint32_t)i + 1;
        in.tilt_us = t0 + (uint64_t)i * 20000u;
        in.tilt_valid = true;
        in.tilt_mm = h;
        float al;
        if (which == 1) {
            RampAssist01Input in1 = {in.now_us,   in.dt,      true, true, true,
                                     in.tilt_seq, in.tilt_us, true, h};
            al = ramp_assist01_step(&b, &in1);
            if (b.pending_event == RA01_EVT_ACTIVATE)
                act_at = now;
            if (b.pending_event == RA01_EVT_END && !*end_reason) {
                *end_reason = b.last_end;
                *active_ms = b.active_ms;
                *end_at_ms = (now - act_at) / 1000;
            }
            b.pending_event = 0xff;
        } else {
            al = ramp_assist_step(&a, &in);
            if (a.pending_event == RA_EVT_ACTIVATE)
                act_at = now;
            if (a.pending_event == RA_EVT_END && !*end_reason) {
                *end_reason = a.last_end;
                *active_ms = a.active_ms;
                *end_at_ms = (now - act_at) / 1000;
            }
            a.pending_event = 0xff;
        }
        if (act_at && now - act_at == 2000000u)
            *alpha_at_2s = al;
    }
    return 0;
}
static int
noisy_climb(double t, bool *v) { /* 10 mm ramp with +/-3 mm rocking, crest held 1 s, real descent */
    *v = true;
    unsigned k = (unsigned)(t * 50.0 + 0.5);
    int noise = (int)(k * 2654435761u >> 29) - 3; /* -3..+4 */
    if (noise > 3)
        noise = 3;
    double base = t < 0.5   ? 0
                  : t < 2.0 ? (t - 0.5) / 1.5 * 10.0
                  : t < 3.0 ? 10.0
                  : t < 3.5 ? 10.0 - (t - 3.0) / 0.5 * 10.0
                            : 0.0;
    int h = (int)(base + 0.5) + (base > 1.0 ? noise : 0);
    return h < 0 ? 0 : h;
}
static int dip_profile(double t, bool *v) { /* climb to 9, single 60 ms dip to 3, keep climbing */
    *v = true;
    unsigned k = (unsigned)(t * 50.0 + 0.5);
    if (k < 10)
        return 0;
    if (k < 40)
        return 4 + (int)(k - 10) / 6;
    if (k < 43)
        return 3;
    if (k < 120)
        return 9;
    return 0;
}
static int test_replay(void) {
    uint8_t r1, r2;
    uint32_t ms1, ms2;
    uint64_t at1, at2;
    float al1, al2;
    replay_run(1, &r1, &ms1, &at1, &al1);
    replay_run(2, &r2, &ms2, &at2, &al2);
    printf(
        "replay v0.1: end %u after %u ms; v0.2: end %u after %u ms, alpha 2 s into assist %.0f\n",
        r1, ms1, r2, ms2, al2);
    CHECK(r1 == RA01_END_DESCENT && ms1 >= 200 && ms1 <= 260,
          "v0.1 reference reproduces the captured DESCENT cutoff (241 ms, peak 7, current 4)");
    CHECK(
        r2 == RA_END_TIMEOUT,
        "v0.2 keeps assisting through the rocking climb and crest stall until the unchanged 2.5 s timeout");
    CHECK(ms2 >= 2480 && ms2 <= 2520, "timeout still bounds total assistance at 2.5 s");
    CHECK(fabsf(al2 - RA_ALPHA_MILLI) < 0.01f, "full (unchanged) alpha held during the stall");
    RampAssist a;
    uint8_t ev[32];
    int n;
    float amax;
    uint64_t t0 = 50000000;
    ra_reset(&a);
    n = 0;
    run_profile(&a, noisy_climb, 4.5, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[0] == RA_EVT_ACTIVATE &&
              (ev[1] == 100 + RA_END_DESCENT || ev[1] == 100 + RA_END_LEVEL),
          "noisy 10 mm climb: one activation, ends only on the real descent/level");
    CHECK(a.active_ms >= 2100 && a.active_ms <= 2500,
          "noisy climb: assistance covers rise + 1 s crest, released during descent");
    ra_reset(&a);
    n = 0;
    run_profile(&a, dip_profile, 3.0, ev, &n, &amax, &t0);
    CHECK(n == 2 && ev[1] == 100 + RA_END_LEVEL,
          "60 ms dip to 3 mm mid-climb does not end assistance; ends at level");
    CHECK(a.active_ms > 1500, "dip case active through the climb");
    return 0;
}

/* ---------- D ---------- */
static int test_tlm(void) {
    RampAssistTlm t = {.uptime_ms = 0x01020304u,
                       .event = RA_EVT_END,
                       .flags = 0x2b,
                       .last_end = RA_END_DESCENT,
                       .activations = 3,
                       .alpha_milli = 200,
                       .pwm_l = 30,
                       .pwm_r = 36,
                       .height_mm = -5,
                       .peak_mm = 9,
                       .int_max = 9,
                       .int_min = -2,
                       .valid = 24,
                       .invalid = 1,
                       .longest = 12,
                       .max_i_l = 100,
                       .max_i_r = 55,
                       .sat_ticks = 7,
                       .active_ms = 1234,
                       .peak_pwm = 36};
    uint8_t f[RA_TLM_FRAME_BYTES];
    ra_tlm_pack(&t, 0x5a, f);
    CHECK(sizeof f == 31 && f[0] == 0xf2 && f[1] == 28 && f[2] == 0x5a,
          "source 15 header, 28-byte payload, 31-byte frame (<= 64)");
    CHECK(f[3] == 4 && f[6] == 1 && f[7] == RA_EVT_END && f[8] == 0x2b && f[9] == RA_END_DESCENT &&
              (int8_t)f[17] == -5 && f[24] == 100 && f[27] == 0xd2 && f[28] == 0x04,
          "field offsets");
    for (unsigned i = 0; i < sizeof f; ++i)
        printf("%02x", f[i]);
    printf("\n");
    return 0;
}

/* ---------- F: v0.3 hold through Buddy4 ACCEL/SPREAD exclusions ---------- */
/* Sample kinds at 20 ms: V = valid height, S = invalid ONLY because of ACCEL/SPREAD (skip),
 * I = invalid for another reason (turn/unavailable/...). */
typedef struct {
    char kind;
    int16_t mm;
} HSample;
typedef struct {
    int activations, invalid_ends, end_reason;
    float min_alpha_assisting, max_alpha;
    uint32_t last_active_ms;
} HRes;
static int hold_cap; /* number of samples in the profile */
static HSample hprof[400];
static void hpush(char kind, int16_t mm, int n) {
    for (int i = 0; i < n && hold_cap < 400; ++i)
        hprof[hold_cap++] = (HSample){kind, mm};
}
/* run20-like: level, climb 4..10, crest with four 60 ms ACCEL/SPREAD exclusions, then descent and level */
static void build_run20(void) {
    hold_cap = 0;
    hpush('V', 0, 10);
    for (int i = 0; i < 30; ++i)
        hpush('V', (int16_t)(4 + i / 5), 1);
    for (int c = 0; c < 4; ++c) {
        hpush('V', (int16_t)(10 + c % 3), 6);
        hpush('S', 0, 3);
    }
    hpush('V', 10, 5);
    for (int i = 10; i >= 0; --i)
        hpush('V', (int16_t)i, 1);
    hpush('V', 0, 40);
}
typedef struct {
    bool stop_at, pause_at;
    int at;
    bool stale_from;
    int stale_at;
} HOpts;
static HRes hrun(int which, HOpts o) {
    HRes r = {0, 0, 0, 1e9f, 0, 0};
    RampAssist a;
    RampAssist02 b;
    ra_reset(&a);
    ra02_reset(&b);
    uint64_t t0 = 70000000;
    for (int k = 0; k < hold_cap * 2; ++k) {
        uint64_t now = t0 + (uint64_t)k * 10000u;
        int i = k / 2;
        if (o.stale_from && i >= o.stale_at)
            i = o.stale_at; /* Buddy4 stops producing samples */
        HSample h = hprof[i];
        bool run = !(o.stop_at && i >= o.at), autom = !(o.pause_at && i >= o.at);
        float al;
        uint8_t ev, end;
        bool assisting;
        if (which == 3) {
            RampAssistInput in = base_in(now);
            in.run_active = run;
            in.auto_mode = autom;
            in.tilt_seq = (uint32_t)i + 1;
            in.tilt_us = t0 + (uint64_t)i * 20000u;
            in.tilt_valid = h.kind == 'V';
            in.tilt_mm = h.kind == 'V' ? h.mm : 0;
            in.tilt_skip = ra_tilt_skip(true, h.kind == 'V',
                                        h.kind == 'S'   ? (uint8_t)(B4_Q_ACCEL | B4_Q_SPREAD)
                                        : h.kind == 'I' ? (uint8_t)B4_Q_TURN
                                                        : 0);
            al = ramp_assist_step(&a, &in);
            ev = a.pending_event;
            end = a.last_end;
            assisting = a.assisting;
            a.pending_event = 0xff;
            if (!run) {
                r.end_reason = end;
            }
            r.last_active_ms = a.active_ms;
        } else {
            RampAssist02Input in = {now,
                                    0.01f,
                                    run,
                                    autom,
                                    true,
                                    (uint32_t)i + 1,
                                    t0 + (uint64_t)i * 20000u,
                                    h.kind == 'V',
                                    h.kind == 'V' ? h.mm : 0};
            al = ramp_assist02_step(&b, &in);
            ev = b.pending_event;
            end = b.last_end;
            assisting = b.assisting;
            b.pending_event = 0xff;
        }
        if (ev == RA_EVT_ACTIVATE)
            ++r.activations;
        if (ev == RA_EVT_END) {
            r.end_reason = end;
            if (end == RA_END_INVALID)
                ++r.invalid_ends;
        }
        if (assisting && r.activations && k > 0) { /* after ramp-in */
        }
        if (al > r.max_alpha)
            r.max_alpha = al;
        if (assisting && al >= RA_ALPHA_MILLI - 0.5f)
            r.min_alpha_assisting = 0; /* reached full */
        if (r.min_alpha_assisting == 0 && assisting && al < r.max_alpha - 0.5f &&
            !(o.stop_at || o.pause_at))
            r.min_alpha_assisting = -1; /* dipped while assisting */
    }
    return r;
}
static int test_hold(void) {
    CHECK(RA_Q_SPREAD == B4_Q_SPREAD && RA_Q_ACCEL == B4_Q_ACCEL,
          "skip rule bits match Buddy4 B4_Q_SPREAD/B4_Q_ACCEL");
    CHECK(ra_tilt_skip(true, false, B4_Q_ACCEL) && ra_tilt_skip(true, false, B4_Q_SPREAD) &&
              ra_tilt_skip(true, false, B4_Q_ACCEL | B4_Q_SPREAD),
          "ACCEL / SPREAD / both -> skip");
    CHECK(!ra_tilt_skip(true, false, B4_Q_ACCEL | B4_Q_TURN) &&
              !ra_tilt_skip(true, false, B4_Q_REVERSE) && !ra_tilt_skip(true, false, B4_Q_IMPACT) &&
              !ra_tilt_skip(true, false, B4_Q_GRAVITY) && !ra_tilt_skip(true, false, B4_Q_TIMING) &&
              !ra_tilt_skip(true, false, B4_Q_UNAVAILABLE),
          "turn/reverse/impact/gravity/timing/unavailable (alone or mixed) never skip");
    CHECK(!ra_tilt_skip(false, false, B4_Q_ACCEL) && !ra_tilt_skip(true, true, B4_Q_ACCEL) &&
              !ra_tilt_skip(true, false, 0),
          "not tracking / valid / no reason -> not skip");
    HOpts none = {0};
    build_run20();
    HRes v2 = hrun(2, none), v3 = hrun(3, none);
    printf(
        "run20-like crest: v0.2 %d activations, %d INVALID ends; v0.3 %d activation(s), %d INVALID ends, end reason %d\n",
        v2.activations, v2.invalid_ends, v3.activations, v3.invalid_ends, v3.end_reason);
    CHECK(v2.activations >= 3 && v2.invalid_ends >= 3,
          "v0.2 reference reproduces the run-20 chatter (INVALID on each 60 ms ACCEL/SPREAD gap)");
    CHECK(v3.activations == 1 && v3.invalid_ends == 0 &&
              (v3.end_reason == RA_END_DESCENT || v3.end_reason == RA_END_LEVEL),
          "v0.3: one activation through the crest, ends on the real descent/level");
    CHECK(v3.min_alpha_assisting == 0,
          "v0.3: full alpha held through the 60 ms exclusions (no dip)");
    CHECK(v3.max_alpha <= RA_ALPHA_MILLI + 0.01f, "boost magnitude unchanged (alpha <= 200)");
    /* the same gaps but for another reason (turn): unchanged 40 ms rule ends it */
    for (int i = 0; i < hold_cap; ++i)
        if (hprof[i].kind == 'S')
            hprof[i].kind = 'I';
    HRes t = hrun(3, none);
    CHECK(t.invalid_ends >= 1 && t.activations >= 2,
          "non-ACCEL/SPREAD invalid (turn): v0.3 still ends after 40 ms like v0.2");
    /* long exclusion: 9 samples = 180 ms > 150 ms */
    hold_cap = 0;
    hpush('V', 0, 10);
    for (int i = 0; i < 25; ++i)
        hpush('V', (int16_t)(4 + i / 5), 1);
    hpush('S', 0, 9);
    hpush('V', 0, 40);
    t = hrun(3, none);
    CHECK(t.invalid_ends == 1 && t.end_reason == RA_END_INVALID,
          "exclusion longer than 150 ms ends the assist (INVALID)");
    /* skip then other invalid: the 40 ms rule takes over */
    hold_cap = 0;
    hpush('V', 0, 10);
    for (int i = 0; i < 25; ++i)
        hpush('V', (int16_t)(4 + i / 5), 1);
    hpush('S', 0, 2);
    hpush('I', 0, 3);
    hpush('V', 8, 10);
    hpush('V', 0, 40);
    t = hrun(3, none);
    CHECK(t.invalid_ends == 1, "skip followed by a turn-invalid gap: ends on the 40 ms rule");
    /* skips before activation never activate */
    hold_cap = 0;
    hpush('V', 0, 10);
    for (int c = 0; c < 20; ++c) {
        hpush('V', 6, 3);
        hpush('S', 0, 1);
    }
    hpush('V', 0, 40);
    t = hrun(3, none);
    CHECK(t.activations == 0,
          "ACCEL/SPREAD samples still block activation (need 4 consecutive valid >= 4 mm)");
    /* Stop / pause / stale during a hold */
    build_run20();
    int crest = 0;
    for (int i = 0; i < hold_cap; ++i)
        if (hprof[i].kind == 'S') {
            crest = i + 1;
            break;
        }
    HOpts st = {.stop_at = true, .at = crest};
    t = hrun(3, st);
    CHECK(t.end_reason == RA_END_STOPPED, "Stop during a hold ends immediately (STOPPED)");
    HOpts pa = {.pause_at = true, .at = crest};
    t = hrun(3, pa);
    CHECK(t.end_reason == RA_END_PAUSED, "Pause during a hold ends immediately (PAUSED)");
    HOpts sl = {.stale_from = true, .stale_at = crest};
    t = hrun(3, sl);
    CHECK(t.end_reason == RA_END_STALE,
          "Buddy4 stops producing samples during a hold: STALE after 60 ms");
    /* timeout still bounds: long elevated climb with frequent short exclusions */
    hold_cap = 0;
    hpush('V', 0, 10);
    for (int c = 0; c < 40; ++c) {
        hpush('V', 9, 4);
        hpush('S', 0, 3);
    }
    hpush('V', 0, 40);
    t = hrun(3, none);
    CHECK(t.end_reason == RA_END_TIMEOUT && t.last_active_ms >= 2480 && t.last_active_ms <= 2520,
          "hold time counts toward the unchanged 2.5 s timeout");
    return 0;
}

/* ---------- G: v0.4 stall push ---------- */
typedef struct {
    int pushes, ends, end_reason, bad_range, bad_slew;
    float push_max, alpha_max;
    uint32_t push_ms, assist_ms;
    bool push_seen_after_stop;
    float push_at_end_plus_300ms;
} PRes;
/* climb to 9 mm and stay elevated; speeds: moving 40 mm/s until stall_from_s, then 0 until move_from_s, then 40 */
static PRes prun(double stall_from_s, double move_from_s, double level_at_s, double stop_at_s,
                 double pause_at_s, double total_s) {
    PRes r = {0};
    RampAssist a;
    ra_reset(&a);
    uint64_t t0 = 90000000;
    double end_t = -1;
    float push_prev = 0;
    for (int k = 0; k < (int)(total_s * 100); ++k) {
        double t = k * 0.01;
        uint64_t now = t0 + (uint64_t)k * 10000u;
        int i = k / 2;
        RampAssistInput in = base_in(now);
        in.run_active = !(stop_at_s > 0 && t >= stop_at_s);
        in.auto_mode = !(pause_at_s > 0 && t >= pause_at_s);
        double ts = i * 0.02;
        int16_t h = ts < 0.2                               ? 0
                    : ts < 0.5                             ? (int16_t)(4 + (ts - 0.2) * 16)
                    : (level_at_s > 0 && ts >= level_at_s) ? 0
                                                           : 9;
        in.tilt_seq = (uint32_t)i + 1;
        in.tilt_us = t0 + (uint64_t)i * 20000u;
        in.tilt_valid = true;
        in.tilt_mm = h;
        float v = (t >= stall_from_s && t < move_from_s) ? 0.0f : 40.0f;
        in.speed_mm_s[0] = v;
        in.speed_mm_s[1] = v * 0.3f;
        float al = ramp_assist_step(&a, &in);
        if (a.pending_event == RA_EVT_PUSH)
            ++r.pushes;
        if (a.pending_event == RA_EVT_END) {
            ++r.ends;
            r.end_reason = a.last_end;
            if (end_t < 0)
                end_t = t;
        }
        a.pending_event = 0xff;
        if (a.push_pwm > r.push_max)
            r.push_max = a.push_pwm;
        if (al > r.alpha_max)
            r.alpha_max = al;
        if (a.pushing)
            r.push_ms += 10;
        if (a.assisting)
            r.assist_ms += 10;
        if (stop_at_s > 0 && t >= stop_at_s && a.push_pwm > 0)
            r.push_seen_after_stop = true;
        if (pause_at_s > 0 && t >= pause_at_s && a.push_pwm > 0)
            r.push_seen_after_stop = true;
        if (end_t >= 0 && t >= end_t + 0.3 && t < end_t + 0.31)
            r.push_at_end_plus_300ms = a.push_pwm;
        if (!(a.push_pwm <= RA_PUSH_PWM + 0.01f && a.push_pwm >= 0.0f))
            ++r.bad_range;
        if (!(fabsf(a.push_pwm - push_prev) <= RA_PUSH_SLEW * 0.01f + 0.01f || !in.run_active ||
              !in.auto_mode))
            ++r.bad_slew;
        push_prev = a.push_pwm;
    }
    return r;
}
static int test_push(void) {
    PRes r = prun(1.0, 9, 0, 0, 0, 2.0); /* stall at 1.0 s, never moves */
    printf("stall push: pushes %d, push max %.1f PWM, push %u ms, assist %u ms, end reason %d\n",
           r.pushes, r.push_max, r.push_ms, r.assist_ms, r.end_reason);
    CHECK(r.pushes == 1 && fabsf(r.push_max - RA_PUSH_PWM) < 0.01f,
          "stalled while assisting: one push, ramps to +60 PWM");
    CHECK(r.bad_range == 0 && r.bad_slew == 0, "push within 0..60 PWM and ramped at <= 200 PWM/s");
    r = prun(1.0, 9, 0, 0, 0, 6.0);
    CHECK(r.push_ms >= 1490 && r.push_ms <= 1510, "push budget 1.5 s per ramp episode");
    CHECK(r.end_reason == RA_END_TIMEOUT && r.assist_ms >= 3980 && r.assist_ms <= 4020,
          "push time does not count toward the 2.5 s timeout: assist ends at 2.5 s + 1.5 s");
    CHECK(r.push_at_end_plus_300ms < 0.01f,
          "push fully removed within 0.3 s after the assist ends");
    r = prun(1.0, 1.6, 0, 0, 0, 3.0); /* stall 0.6 s then moves */
    CHECK(r.pushes == 1 && r.push_ms <= 410 && r.push_ms >= 380,
          "moving again (> 15 mm/s) releases the push (0.2 s detect + 0.4 s push)");
    r = prun(1.0, 1.15, 0, 0, 0, 3.0); /* 150 ms dip: not a stall */
    CHECK(r.pushes == 0, "a stop shorter than 200 ms is not a stall: no push");
    r = prun(1.0, 9, 1.6, 0, 0, 3.0); /* stalled, then the tilt returns to level */
    CHECK(r.end_reason == RA_END_LEVEL && r.push_at_end_plus_300ms < 0.01f,
          "level return ends assist; push ramps out");
    r = prun(1.0, 9, 0, 1.5, 0, 2.0);
    CHECK(!r.push_seen_after_stop && r.end_reason == RA_END_STOPPED && r.bad_range == 0,
          "Stop during a push removes it at once (STOPPED)");
    r = prun(1.0, 9, 0, 0, 1.5, 2.0);
    CHECK(!r.push_seen_after_stop && r.end_reason == RA_END_PAUSED,
          "Pause during a push removes it at once (PAUSED)");
    r = prun(0.05, 9, 0, 0, 0, 0.4); /* stopped car before any activation */
    CHECK(r.pushes == 0 && r.push_max == 0.0f,
          "never pushes unless assisting (stopped on the flat = no push)");
    CHECK(r.alpha_max <= RA_ALPHA_MILLI + 0.01f, "feedforward boost unchanged (alpha <= 200)");
    /* controller: the push adds exactly its value below the 420 clamp, respects the clamp, no residual */
    ContinuousDrive c1 = {0}, c2 = {0};
    SignedWheel p1 = {0}, p2 = {0};
    int32_t o1 = 0, o2 = 0;
    float max_o = 0, d_mid = 0;
    for (int k = 0; k < 600; ++k) {
        uint64_t now = 1000000 + (uint64_t)k * 10000u;
        float meas = k < 50 ? 30.0f : 0.0f; /* stalled after 0.5 s */
        c2.boost_add = (k >= 100 && k < 300) ? 60.0f : 0.0f;
        o1 = continuous_drive_step(&c1, &p1, 0, 45, meas, 0.01f, now, o1);
        o2 = continuous_drive_step(&c2, &p2, 0, 45, meas, 0.01f, now, o2);
        if (o2 > max_o)
            max_o = (float)o2;
        if (k == 280)
            d_mid = (float)(o2 - o1);
    }
    CHECK(max_o <= CONTINUOUS_PWM_MAX, "total output never exceeds the 420 PWM clamp");
    CHECK(fabsf(d_mid - 60.0f) <= 1.0f, "below the clamp the push adds exactly +60 PWM");
    CHECK(abs(o2 - o1) <= 1 && fabsf(c1.integral - c2.integral) < 0.5f,
          "after release: same output and integral as without the push (no residual)");
    /* the 01:21 stall: targets 45 (left) / 13 (right, steering), both wheels at 0 mm/s */
    {
        ContinuousDrive w[2][2];
        SignedWheel sp[2][2];
        int32_t o[2][2];
        memset(w, 0, sizeof w);
        memset(sp, 0, sizeof sp);
        memset(o, 0, sizeof o);
        const int16_t tgt[2] = {45, 13};
        for (int k = 0; k < 400; ++k) {
            uint64_t now = 2000000 + (uint64_t)k * 10000u;
            for (int v = 0; v < 2; ++v)
                for (int wh = 0; wh < 2; ++wh) {
                    w[v][wh].boost_cap = RA_PWM_CAP;
                    w[v][wh].boost_frac = v ? RA_ALPHA_MILLI / 1000.0f : 0.0f;
                    w[v][wh].boost_add = v ? RA_PUSH_PWM : 0.0f;
                    o[v][wh] = continuous_drive_step(&w[v][wh], &sp[v][wh], (unsigned)wh, tgt[wh],
                                                     0.0f, 0.01f, now, o[v][wh]);
                }
        }
        printf(
            "01:21 stall reproduced: without assist %d/%d PWM (car: 295/165); with +20%% assist + push %d/%d PWM\n",
            o[0][0], o[0][1], o[1][0], o[1][1]);
        CHECK(abs(o[0][0] - 295) <= 2 && abs(o[0][1] - 165) <= 2,
              "controller model reproduces the logged stall output 295/165");
        CHECK(o[1][0] - o[0][0] >= 89 && o[1][0] - o[0][0] <= 91 && o[1][1] - o[0][1] >= 69 &&
                  o[1][1] - o[0][1] <= 71,
              "assist + push add +90 / +70 PWM at that stall (30+60 / 10+60)");
    }
    ContinuousDrive c3 = {0};
    continuous_drive_reset(&c3, &p1);
    CHECK(c3.boost_add == 0.0f && c3.boost_frac == 0.0f, "controller reset clears the push");
    return 0;
}

/* ---------- H: v0.6 speed hold (replaces the v0.5 tilt floor) ---------- */
typedef struct {
    int hold_end_events, activations;
    float level_max, level_at[100];
    bool level_after_stop, bad_slew;
    double off_t, on_again_t;
} SRes;
typedef int16_t (*tilt_fn)(double ts);
static int16_t tf_ramp_flat(double ts) {
    return ts < 0.2   ? 0
           : ts < 0.5 ? (int16_t)(4 + (ts - 0.2) * 16)
           : ts < 2.0 ? 9
           : ts < 2.3 ? (int16_t)(9 - (ts - 2.0) * 30)
                      : 0;
}
static int16_t tf_stuck(double ts) {
    return ts < 0.2 ? 0 : ts < 0.5 ? (int16_t)(4 + (ts - 0.2) * 16) : 9;
}
static SRes srun(tilt_fn f, double total_s, double stop_at, double pause_at, double invalid_from,
                 double invalid_to, bool skip) {
    SRes r = {0};
    RampAssist a;
    ra_reset(&a);
    uint64_t t0 = 50000000;
    r.off_t = r.on_again_t = -1;
    bool was_on = false;
    float prev = 0;
    for (int k = 0; k < (int)(total_s * 100); ++k) {
        double t = k * 0.01;
        int i = k / 2;
        double ts = i * 0.02;
        uint64_t now = t0 + (uint64_t)k * 10000u;
        RampAssistInput in = base_in(now);
        in.run_active = !(stop_at > 0 && t >= stop_at);
        in.auto_mode = !(pause_at > 0 && t >= pause_at);
        bool inv = invalid_to > 0 && ts >= invalid_from && ts < invalid_to;
        in.tilt_seq = (uint32_t)i + 1;
        in.tilt_us = t0 + (uint64_t)i * 20000u;
        in.tilt_valid = !inv;
        in.tilt_mm = inv ? 0 : f(ts);
        in.tilt_skip = inv && skip;
        (void)ramp_assist_step(&a, &in);
        if (a.pending_event == RA_EVT_HOLD_END)
            ++r.hold_end_events;
        if (a.pending_event == RA_EVT_ACTIVATE)
            ++r.activations;
        a.pending_event = 0xff;
        if (a.hold_level > r.level_max)
            r.level_max = a.hold_level;
        if (k % 10 == 0 && k / 10 < 100)
            r.level_at[k / 10] = a.hold_level;
        if (((stop_at > 0 && t >= stop_at) || (pause_at > 0 && t >= pause_at)) && a.hold_level > 0)
            r.level_after_stop = true;
        if (in.run_active && in.auto_mode &&
            fabsf(a.hold_level - prev) > RA_HOLD_SLEW * 0.01f + 1e-4f)
            r.bad_slew = true;
        prev = a.hold_level;
        if (was_on && !a.hold_on && r.off_t < 0)
            r.off_t = t;
        if (!was_on && a.hold_on && r.off_t >= 0 && r.on_again_t < 0)
            r.on_again_t = t;
        was_on = a.hold_on;
    }
    return r;
}
/* harness mapping (ramp_assist_harness.h) */
static void hold_apply(ContinuousDrive *s, float hl) {
    s->hold_min_speed = hl > 0.0f ? RA_HOLD_MIN_MM_S * hl : 0.0f;
    s->hold_i_max =
        hl > 0.0f ? (float)CONTINUOUS_I_MAX + (RA_HOLD_I_MAX - (float)CONTINUOUS_I_MAX) * hl : 0.0f;
    s->hold_ki = hl > 0.0f ? 1.0f + (RA_HOLD_KI - 1.0f) * hl : 0.0f;
}
/* One wheel, plant as in C (tau 80 ms, PWM-equivalent load). Hold level ramps
 * in at hold_from and out at hold_to (5/s). load(k) in PWM; target fixed. */
typedef struct {
    float v_max, vn_max, v_max_held, v_min_held, v_at[8], out_max, i_max_seen, i_after,
        out_end_diff;
    int bad_slew;
} CRes;
static CRes crun(unsigned w, int16_t target, float (*load)(int k), bool hold, int hold_from,
                 int hold_to, int steps) {
    CRes r = {0};
    r.v_min_held = 1e9f;
    ContinuousDrive c, n;
    SignedWheel p, q;
    memset(&c, 0, sizeof c);
    memset(&n, 0, sizeof n);
    memset(&p, 0, sizeof p);
    memset(&q, 0, sizeof q);
    int32_t o = 0, on = 0;
    float v = 0, vn = 0, hl = 0;
    const float slope = w == 0 ? 150.0f / 45.0f : 180.0f / 45.0f;
    for (int k = 0; k < steps; ++k) {
        uint64_t now = 7000000 + (uint64_t)k * 10000u;
        float goal = hold && k >= hold_from && k < hold_to ? 1.0f : 0.0f, st = RA_HOLD_SLEW * 0.01f,
              d = goal - hl;
        hl += d > st ? st : (d < -st ? -st : d);
        hold_apply(&c, hl);
        int32_t prev = o;
        o = continuous_drive_step(&c, &p, w, target, v, 0.01f, now, o);
        on = continuous_drive_step(&n, &q, w, target, vn, 0.01f, now, on);
        if (k > 0 && abs(o - prev) > 13)
            ++r.bad_slew;
        float drv = ((float)o - load(k)) / slope;
        if (drv < 0)
            drv = 0;
        v += (drv - v) * 0.01f / 0.08f;
        float drn = ((float)on - load(k)) / slope;
        if (drn < 0)
            drn = 0;
        vn += (drn - vn) * 0.01f / 0.08f;
        if (v > r.v_max)
            r.v_max = v;
        if (vn > r.vn_max)
            r.vn_max = vn;
        if (k >= hold_from && k < hold_to && load(k) == load(hold_from) && v > r.v_max_held)
            r.v_max_held = v; /* while the load is unchanged */
        if (k >= hold_from + 150 && k < hold_to && v < r.v_min_held)
            r.v_min_held = v;
        if (k % 100 == 0 && k / 100 < 8)
            r.v_at[k / 100] = v;
        if (o > r.out_max)
            r.out_max = (float)o;
        if (fabsf(c.integral) > r.i_max_seen)
            r.i_max_seen = fabsf(c.integral);
        if (k == hold_to + 30)
            r.i_after = fabsf(c.integral);
        if (k == steps - 1)
            r.out_end_diff = (float)abs(o - on);
    }
    return r;
}
static float ld_crest(int k) {
    return k >= 50 && k < 420 ? 250.0f : 0.0f;
} /* nose on the mat (needs > 250 PWM to move), then flat */
static float ld_climb(int k) {
    return k >= 50 && k < 250 ? 60.0f : 0.0f;
}
static float ld_none(int k) {
    (void)k;
    return 0.0f;
}
static int test_speed_hold(void) {
    SRes r = srun(tf_ramp_flat, 4.0, 0, 0, 0, 0, false);
    printf(
        "speed hold: level max %.2f, on until %.2f s (|h| <= 2 mm from 2.22 s), hold-end events %d\n",
        r.level_max, r.off_t, r.hold_end_events);
    CHECK(r.activations == 1 && fabsf(r.level_max - 1.0f) < 1e-4f,
          "hold ramps to full from activation");
    CHECK(
        r.off_t >= 2.4 && r.off_t <= 2.5 && r.hold_end_events == 1,
        "hold kept through crest/descent, ends 200 ms after the tilt is within 2 mm (|h| <= 2 from 2.22 s)");
    CHECK(r.level_at[30] < 1e-4f && !r.bad_slew, "hold ramped out after flat; level slew <= 5/s");
    r = srun(tf_stuck, 8.0, 0, 0, 0, 0, false);
    CHECK(r.off_t >= 5.2 && r.off_t <= 5.4, "stuck tilted: hold ends at the 5 s budget");
    r = srun(tf_stuck, 3.0, 1.5, 0, 0, 0, false);
    CHECK(!r.level_after_stop, "Stop removes the hold at once");
    r = srun(tf_stuck, 3.0, 0, 1.5, 0, 0, false);
    CHECK(!r.level_after_stop, "Pause removes the hold at once");
    r = srun(tf_stuck, 3.0, 0, 0, 1.0, 1.3, false); /* 300 ms non-ACCEL invalid (the 02:07 case) */
    CHECK(r.off_t < 0 && r.level_at[12] > 0.999f,
          "300 ms invalid tilt of any reason: hold kept (v0.5 floor ended after 40 ms)");
    r = srun(tf_stuck, 3.0, 0, 0, 1.0, 1.3, true);
    CHECK(r.off_t < 0, "300 ms ACCEL/SPREAD exclusion: hold kept");
    r = srun(tf_stuck, 4.0, 0, 0, 1.0, 1.7, false); /* 700 ms invalid */
    CHECK(r.off_t >= 1.49 && r.off_t <= 1.53, "invalid longer than 500 ms ends the hold");
    CHECK(r.on_again_t > 1.7 && r.on_again_t < 1.9 && r.activations == 2,
          "valid tilt re-activates and restarts the hold (budget left)");
    r = srun(tf_ramp_flat, 0.15, 0, 0, 0, 0, false);
    CHECK(r.level_max == 0.0f, "no hold on the flat before a ramp is detected");
    {
        RampAssist a;
        ra_reset(&a);
        uint64_t t0 = 60000000;
        uint32_t seq = 0;
        float lvl = 1; /* stale Buddy4 */
        for (int k = 0; k < 200; ++k) {
            RampAssistInput in = base_in(t0 + (uint64_t)k * 10000u);
            if (k < 100 && k % 2 == 0)
                seq++;
            in.tilt_seq = seq;
            in.tilt_us = t0 + (uint64_t)((k < 100 ? k : 99) / 2) * 20000u;
            in.tilt_valid = true;
            in.tilt_mm = k < 20 ? 0 : 9;
            (void)ramp_assist_step(&a, &in);
            if (k == 99)
                CHECK(a.hold_on, "held before Buddy4 goes silent");
            lvl = a.hold_level;
        }
        CHECK(!a.hold_on && lvl == 0.0f, "no Buddy4 sample for 200 ms ends the hold");
    }

    /* controller: 02:07 descent stall - ramp wheel asked for 2 mm/s, nose load 250 PWM */
    CRes off = crun(1, 2, ld_crest, false, 0, 0, 1200),
         on = crun(1, 2, ld_crest, true, 60, 460, 1200);
    printf(
        "descent stall (steer target 2 mm/s, 250 PWM load): without hold v %.1f mm/s, out max %.0f; with hold v(held, min) %.1f mm/s, out max %.0f, |I| max %.0f\n",
        off.v_at[3], off.out_max, on.v_min_held, on.out_max, on.i_max_seen);
    CHECK(off.v_at[3] < 1.0f,
          "without hold the steered ramp wheel stays stalled (reproduces the log)");
    CHECK(
        on.v_min_held >= 28.0f && on.v_min_held <= 40.0f,
        "with hold the wheel is driven to ~35 mm/s through the 250 PWM load (from 1.5 s after the hold starts)");
    CHECK(on.out_max <= CONTINUOUS_PWM_MAX && on.i_max_seen <= RA_HOLD_I_MAX + 1e-3f &&
              on.bad_slew == 0,
          "420 clamp, held I limit 300 and 1200/s slew kept");
    CHECK(
        on.i_after <= CONTINUOUS_I_MAX + 1e-3f && on.out_end_diff <= 1.0f,
        "after the hold (flat again) the integral is back within 100 and the output converges to the no-hold controller");
    /* climb at 45: no 130 mm/s surge (v0.5) - speed controlled */
    CRes cl = crun(0, 45, ld_climb, true, 60, 260, 500),
         cl0 = crun(0, 45, ld_none, true, 60, 260, 500);
    printf(
        "climb with hold: v max while loaded %.1f mm/s (60 PWM load), %.1f (no load); overall peak %.1f vs no-hold %.1f (abrupt load removal); v0.5 floor 300 = %.0f mm/s unloaded left wheel\n",
        cl.v_max_held, cl0.v_max_held, cl.v_max, cl.vn_max, 300.0f / (150.0f / 45.0f));
    CHECK(cl.v_max_held <= 50.0f && cl0.v_max_held <= 50.0f,
          "hold keeps the climb near 45 mm/s (no fixed-PWM surge)");
    CHECK(cl.v_max <= cl.vn_max + 2.0f,
          "overshoot when the load suddenly disappears within 2 mm/s of no hold");
    CHECK(cl.v_min_held >= 38.0f, "hold keeps speed through the climb load");
    /* minimum target applies to a steering target of 0, never to a negative target */
    {
        ContinuousDrive s;
        SignedWheel pw;
        memset(&s, 0, sizeof s);
        memset(&pw, 0, sizeof pw);
        int32_t o = 0;
        hold_apply(&s, 1.0f);
        for (int k = 0; k < 100; ++k)
            o = continuous_drive_step(&s, &pw, 1, 0, 0.0f, 0.01f, 8000000u + (uint64_t)k * 10000u,
                                      o);
        CHECK(o > 0 && s.active && fabsf(s.reference - RA_HOLD_MIN_MM_S) < 0.5f,
              "steering 0 with hold: wheel driven toward 35 mm/s");
        hold_apply(&s, 1.0f);
        o = continuous_drive_step(&s, &pw, 1, -5, 0.0f, 0.01f, 9000000u, o);
        CHECK(o == 0 && !s.active, "negative target with hold: reset to 0 (unchanged)");
        s.hold_min_speed = 35;
        s.hold_i_max = 300;
        s.hold_ki = 3;
        continuous_drive_reset(&s, &pw);
        CHECK(s.hold_min_speed == 0.0f && s.hold_i_max == 0.0f && s.hold_ki == 0.0f,
              "controller reset clears the hold");
    }
    return 0;
}

int main(void) {
    if (test_identity() || test_logic() || test_replay() || test_hold() || test_push() ||
        test_speed_hold() || test_controller() || test_tlm())
        return 1;
    printf(
        "PASS ramp-assist v0.6 host tests: %d checks (boost-0 identity vs 8d7f3aa, logic, captured-climb replay, v0.3 ACCEL/SPREAD hold, v0.4 stall push, v0.6 speed hold, controller limits/residual, telemetry)\n",
        checks);
    return 0;
}
