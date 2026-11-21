/* Host test: Buddy5 straight-only takeover gate (b5_straight.c).
 * The bend flags come from the REAL Buddy3 bend monitor
 * (continuous_steering_update_bend, CONTINUOUS_BEND_ASSIST=1 as in the
 * release profile), driven by a kinematic follower on straights, tight and
 * gentle curves (left and right). The sample mapping below is the one in
 * rtos_line_straight_sample() (line_service.c).
 * LOGIC EVIDENCE ONLY: no hardware, no real line sensor. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "b5_straight.h"
#include "src/line_following/continuous_steering.h"
#include "src/line_following/circuit_logic.h"
#include "src/line_following/symmetric_logic.h"
#include "src/line_following/line_search.h"
#include "src/motion/dual_logic.h"
#define APP_RTOS_BUDDY5 1
#include "app_program/line_navigation/line_model.h"

#define PI_F 3.14159265358979323846f
#define DT_US 10000u
#define SPEED 45.0f /* mm/s, baseline cruise */

static unsigned g_failures, g_checks;
#define CHECK(c) do { ++g_checks; if (!(c)) { printf("  FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++g_failures; } } while (0)

static const char *const NAMES[B5S_REASON_COUNT] = {"UNKNOWN", "STRAIGHT", "SETTLING", "STALE", "UNAVAILABLE",
    "BEND_ENTRY", "BEND", "BEND_EXIT", "CURVE", "JUNCTION", "TURN", "LINE_LOST", "OTHER"};

typedef struct {
    uint64_t now;
    float wheel[2], path, heading; /* heading rad, +LEFT */
    continuous_steering_t steer;
    b5_straight_t gate;
    unsigned seen;   /* bit per reason observed */
} sim_t;

static void sim_init(sim_t *s) {
    memset(s, 0, sizeof *s);
    s->now = 1000000;
    s->steer.active = 1;
    b5_straight_reset(&s->gate, B5S_UNKNOWN);
}

/* Same mapping as rtos_line_straight_sample(). */
static b5_line_sample_t sample_of(const sim_t *s, uint8_t decision) {
    const continuous_steering_t *p = &s->steer;
    return (b5_line_sample_t){.now_us = s->now, .wheel_mm = {s->wheel[0], s->wheel[1]}, .phase = C_FOLLOW,
        .decision = decision, .valid = true, .steering = p->active};
}

/* One 10 ms frame. curvature_signed = 1/R (+LEFT), 0 on a straight; error and
 * the requested half differential follow the line (weave on straights). */
static uint8_t step(sim_t *s, float curvature_signed, float weave_half, float curve_error) {
    float ds = SPEED * DT_US / 1e6f;
    float weave = sinf(2 * PI_F * s->path / 250.0f);
    float half = SPEED * curvature_signed * TRACK_MM / 2 + weave_half * weave; /* measured = requested (ideal plant) */
    float error = curvature_signed != 0 ? (curvature_signed > 0 ? curve_error : -curve_error) : 0.10f * weave;
    float dtheta = 2 * half / TRACK_MM * (DT_US / 1e6f);
    s->heading += dtheta;
    s->wheel[0] += ds - TRACK_MM / 2 * dtheta;
    s->wheel[1] += ds + TRACK_MM / 2 * dtheta;
    s->path += ds;
    s->now += DT_US;
    continuous_steering_update_bend(&s->steer, error, half, SPEED, half, SPEED, DT_US / 1e6f);
    /* the RTOS demand after the continuous steering step carries 25 (24 while
     * scanning for barcodes); alternate the symmetric codes too */
    uint8_t decision = (s->now / DT_US) % 3 == 0 ? 25 : (s->now / DT_US) % 3 == 1 ? 24
                     : half > 0.5f ? SYM_LEFT : half < -0.5f ? SYM_RIGHT : SYM_STRAIGHT;
    b5_line_sample_t ls = sample_of(s, decision);
    b5_straight_step(&s->gate, &ls);
    uint8_t r = b5_straight_check(&s->gate, s->now);
    s->seen |= 1u << r;
    return r;
}

/* Run a straight until the gate opens; returns the path at opening. */
static float open_on_straight(sim_t *s, float weave_half, float limit_mm) {
    float start = s->path;
    while (s->path - start < limit_mm) {
        if (step(s, 0, weave_half, 0) == B5S_STRAIGHT) return s->path - start;
    }
    return -1;
}

static void test_straight_with_corrections(void) {
    sim_t s; sim_init(&s);
    float at = open_on_straight(&s, 2.0f, 1000);
    CHECK(at >= B5S_WINDOW_MM && at < B5S_WINDOW_MM + 15);
    float worst = 0; unsigned closed = 0, lefts = 0;
    for (int k = 0; k < 6000; ++k) { /* 2.7 m of weaving straight */
        uint8_t r = step(&s, 0, 2.0f, 0);
        closed += r != B5S_STRAIGHT;
        worst = fmaxf(worst, s.gate.heading_cdeg / 100);
        lefts += s.steer.bend_candidate || s.steer.bend_demand_duration > 0;
    }
    CHECK(closed == 0 && lefts == 0);
    printf("  straight, small corrections (+/-2 mm/s half differential, +/-0.10 error, decisions 25/24/SYM_LEFT/RIGHT): opens after "
           "%.0f mm, stays open over 2.7 m, worst window heading %.1f deg (limit %.1f)\n",
           at, worst, B5S_HEADING_MAX_CDEG / 100.0);
}

static void test_brief_bend_signals_on_a_straight(void) {
    sim_t s; sim_init(&s);
    unsigned pending_frames = 0;
    const float dt = DT_US / 1e6f;
    for (unsigned frame = 0; frame < 900; ++frame) {
        int direction = (frame / 20) % 2 ? 1 : -1;
        bool correcting = frame % 20 < 3;
        float half = correcting ? direction * 10.0f : 0.0f;
        float error = correcting ? direction * 0.30f : 0.0f;
        s.now += DT_US;
        s.path += SPEED * dt;
        s.wheel[0] += (SPEED - half) * dt;
        s.wheel[1] += (SPEED + half) * dt;
        continuous_steering_update_bend(&s.steer, error, half, SPEED, half, SPEED, dt);
        pending_frames += s.steer.bend_demand_duration > 0.0f;
        CHECK(!s.steer.bend_candidate);
        b5_line_sample_t sample = sample_of(&s, 25);
        b5_straight_step(&s.gate, &sample);
        if (s.path >= B5S_WINDOW_MM + 15.0f) {
            CHECK(b5_straight_check(&s.gate, s.now) == B5S_STRAIGHT);
        }
    }
    CHECK(pending_frames > 0);
}

static void test_confirmed_bend_label_on_straight(void) {
    sim_t s; sim_init(&s);
    const float dt = DT_US / 1e6f;
    CHECK(open_on_straight(&s, 0, 1000) > 0);
    for (unsigned frame = 0; frame < 22; ++frame) {
        s.now += DT_US;
        s.wheel[0] += (SPEED - 10.0f) * dt;
        s.wheel[1] += (SPEED + 10.0f) * dt;
        s.path += SPEED * dt;
        continuous_steering_update_bend(&s.steer, 0.30f, 10.0f, SPEED, 10.0f, SPEED, dt);
        b5_line_sample_t sample = sample_of(&s, 25);
        b5_straight_step(&s.gate, &sample);
    }
    CHECK(s.steer.bend_candidate && s.steer.bend_confirmed);
    CHECK(s.gate.reason == B5S_STRAIGHT && s.gate.open);
    CHECK(s.gate.travel_mm >= B5S_WINDOW_MM);
    CHECK(step(&s, 0, 0, 0) == B5S_STRAIGHT);
    CHECK(s.steer.bend_exiting && s.steer.bend_confirmed);
    CHECK(s.gate.open);

    /* The same exit flag with encoder evidence of a curve blocks takeover. */
    b5_line_sample_t corner = sample_of(&s, 25);
    corner.now_us += DT_US;
    corner.wheel_mm[0] -= 1.0f;
    corner.wheel_mm[1] += 1.0f;
    b5_straight_step(&s.gate, &corner);
    CHECK(s.gate.reason == B5S_CURVE && !s.gate.open);
    CHECK(s.gate.heading_cdeg < B5S_HEADING_MAX_CDEG);
    puts("  bend exit on a straight qualifies; recent encoder curve blocks before the long-window limit");
}

/* Tight curve: the real bend monitor engages (entry, confirmed, exit). */
static void test_tight_curve(int dir) {
    sim_t s; sim_init(&s);
    CHECK(open_on_straight(&s, 2.0f, 1000) > 0);
    float r_mm = 250, c = dir / r_mm, curve_len = r_mm * PI_F / 2; /* 90 deg */
    bool open_in_curve = false; float curve_end = 0;
    unsigned first_bad = 0;
    for (float p0 = s.path; s.path - p0 < curve_len;) {
        uint8_t r = step(&s, c, 0, 0.30f);
        if (!first_bad && r != B5S_STRAIGHT) first_bad = r;
        open_in_curve |= r == B5S_STRAIGHT && s.path - p0 > 20; /* a few frames before the first dwell */
    }
    curve_end = s.path;
    float reopened = -1;
    for (int k = 0; k < 2000 && reopened < 0; ++k) {
        if (step(&s, 0, 2.0f, 0) == B5S_STRAIGHT) reopened = s.path - curve_end;
    }
    CHECK(!open_in_curve);
    CHECK(first_bad == B5S_CURVE);
    CHECK(reopened >= B5S_WINDOW_MM - 30);
    CHECK(reopened <= B5S_WINDOW_MM + 20);
    printf("  %s curve R=%.0f mm (bend assist engages): blocked from %s; reasons seen:", dir > 0 ? "left " : "right",
           r_mm, NAMES[first_bad]);
    for (unsigned i = 0; i < B5S_REASON_COUNT; ++i) if (s.seen & (1u << i)) printf(" %s", NAMES[i]);
    printf("; re-opens %.0f mm after the curve\n", reopened);
}

/* Gentle curve: below the bend-assist thresholds, so only the encoder
 * heading window can block it. (a) Reached from a closed gate (after a
 * junction, a stop or the start) it never opens; (b) entered from an open
 * straight it closes within the window's heading budget and
 * stays closed for the rest of the curve. */
static void test_gentle_curve(int dir, float r_mm, bool expect_block) {
    sim_t s; sim_init(&s);
    unsigned open_frames = 0, bend = 0;
    for (float start = s.path; s.path - start < 1500;) { /* (a) closed start, on the curve */
        uint8_t r = step(&s, dir / r_mm, 0, 0.08f);
        bend |= s.steer.bend_candidate;
        open_frames += r == B5S_STRAIGHT;
    }
    sim_t e; sim_init(&e);
    CHECK(open_on_straight(&e, 2.0f, 1000) > 0);
    float closed_at = -1; unsigned reopen = 0;
    for (float start = e.path; e.path - start < 1500;) { /* (b) open straight, then the curve */
        uint8_t r = step(&e, dir / r_mm, 0, 0.08f);
        bend |= e.steer.bend_candidate;
        if (closed_at < 0 && r != B5S_STRAIGHT) closed_at = e.path - start;
        else if (closed_at >= 0 && r == B5S_STRAIGHT) ++reopen;
    }
    /* entry latency: the window's heading budget plus the straight weave
     * already in the window (worst +/-1.9 deg above, 2.5 deg allowed) */
    float budget = (B5S_HEADING_MAX_CDEG / 100.0f + 2.5f) * PI_F / 180.0f * r_mm;
    CHECK(bend == 0); /* bend assist never engaged */
    if (expect_block) {
        CHECK(open_frames == 0 && (s.seen & (1u << B5S_CURVE)));
        CHECK(closed_at > 0 && closed_at <= budget + B5S_STEP_MM && reopen == 0);
    } else {
        CHECK(open_frames > 0);
    }
    printf("  gentle %s curve R=%.0f mm, no bend flags: window heading %.1f deg; from closed: %s; from an open "
           "straight: closes after %.0f mm of curve (bound %.0f)%s\n", dir > 0 ? "left " : "right", r_mm,
           s.gate.heading_cdeg / 100, open_frames ? "OPENS (treated as straight)" : "never opens",
           closed_at, budget, reopen ? ", re-opens" : ", stays closed");
}

/* Every Buddy3 state that must block; each also restarts the evidence. */
static void test_states(void) {
    struct { const char *name; b5_line_sample_t mod; uint8_t want; bool steer_off, invalid; } cases[] = {
        {"junction centre phase", {.phase = J_CENTRE}, B5S_JUNCTION, false, false},
        {"junction approach phase", {.phase = J_APPROACH}, B5S_JUNCTION, false, false},
        {"junction exit phase", {.phase = J_EXIT}, B5S_JUNCTION, false, false},
        {"junction turn phase", {.phase = J_TURN}, B5S_TURN, false, false},
        {"turn settle phase", {.phase = J_TURN_SETTLE}, B5S_TURN, false, false},
        {"line acquisition after a turn", {.phase = J_ACQUIRE}, B5S_TURN, false, false},
        {"barcode route turn / U-turn", {.route_turn = true}, B5S_TURN, false, false},
        {"circuit busy (junction manoeuvre)", {.busy = true}, B5S_JUNCTION, false, false},
        {"qualified junction edge / cross mark", {.junction_mark = true}, B5S_JUNCTION, false, false},
        {"line lost (white)", {.line_lost = true}, B5S_LINE_LOST, false, false},
        {"recovery search", {.searching = true}, B5S_LINE_LOST, false, false},
        {"recovery decision", {.decision = SYM_RECOVER_RIGHT}, B5S_LINE_LOST, false, false},
        {"edge probe decision", {.decision = C_PROBE_LEFT}, B5S_LINE_LOST, false, false},
        {"line search stage decision", {.decision = LS_DECISION_FIRST + 2}, B5S_LINE_LOST, false, false},
        {"line lost decision", {.decision = AL_LOST}, B5S_LINE_LOST, false, false},
        {"unexpected decision code", {.decision = 99}, B5S_OTHER, false, false},
        {"follower stop", {.stop = true}, B5S_OTHER, false, false},
        {"circuit fault", {.fault = true}, B5S_OTHER, false, false},
        {"bend monitor not running", {0}, B5S_UNAVAILABLE, true, false},
        {"provider did not run", {0}, B5S_UNAVAILABLE, false, true},
    };
    for (unsigned i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
        sim_t s; sim_init(&s);
        CHECK(open_on_straight(&s, 2.0f, 1000) > 0);
        for (int k = 0; k < 20; ++k) { /* 9 mm in that state */
            s.now += DT_US; s.wheel[0] += .45f; s.wheel[1] += .45f; s.path += .45f;
            b5_line_sample_t ls = sample_of(&s, SYM_STRAIGHT);
            const b5_line_sample_t *m = &cases[i].mod;
            if (m->phase) ls.phase = m->phase;
            if (m->decision) ls.decision = m->decision;
            ls.route_turn |= m->route_turn; ls.busy |= m->busy; ls.junction_mark |= m->junction_mark;
            ls.line_lost |= m->line_lost; ls.searching |= m->searching; ls.stop |= m->stop; ls.fault |= m->fault;
            if (cases[i].steer_off) ls.steering = false;
            if (cases[i].invalid) ls.valid = false;
            b5_straight_step(&s.gate, &ls);
            CHECK(b5_straight_check(&s.gate, s.now) == cases[i].want);
            CHECK(!s.gate.open);
        }
        float back = open_on_straight(&s, 2.0f, 1000); /* evidence restarted */
        CHECK(back >= B5S_WINDOW_MM);
        printf("  %-38s -> %-11s, re-opens after %.0f mm of straight\n", cases[i].name, NAMES[cases[i].want], back);
    }
}

static void test_stale_and_misc(void) {
    sim_t s; sim_init(&s);
    CHECK(b5_straight_check(&s.gate, s.now) == B5S_UNKNOWN); /* never evaluated: closed */
    CHECK(open_on_straight(&s, 0, 1000) > 0);
    uint64_t last = s.now;
    CHECK(b5_straight_check(&s.gate, last + B5S_SAMPLE_MAX_US) == B5S_STRAIGHT);
    CHECK(b5_straight_check(&s.gate, last + B5S_SAMPLE_MAX_US + 1) == B5S_STALE && !s.gate.open);
    CHECK(open_on_straight(&s, 0, 1000) >= B5S_WINDOW_MM); /* stale restarts the evidence */
    /* The production loop receives the new frame before checking the gate. */
    b5_line_sample_t next = sample_of(&s, SYM_STRAIGHT);
    s.now += B5S_SAMPLE_MAX_US;
    next.now_us = s.now;
    b5_straight_step(&s.gate, &next);
    CHECK(b5_straight_check(&s.gate, s.now) == B5S_STRAIGHT && s.gate.open);
    s.now += B5S_SAMPLE_MAX_US + 1U;
    next.now_us = s.now;
    b5_straight_step(&s.gate, &next);
    CHECK(b5_straight_check(&s.gate, s.now) == B5S_STALE && !s.gate.open);
    CHECK(s.gate.count == 0 && s.gate.open_us == 0);
    CHECK(open_on_straight(&s, 0, 1000) >= B5S_WINDOW_MM);
    printf("  a 50001 us gap discards history when the next frame is supplied first\n");
    /* dwell: a fast robot still needs B5S_DWELL_US */
    sim_t f; sim_init(&f);
    unsigned frames = 0; uint8_t r = 0;
    for (; frames < 200 && r != B5S_STRAIGHT; ++frames) {
        f.now += DT_US; f.wheel[0] += 30; f.wheel[1] += 30; /* 3 m/s */
        b5_line_sample_t ls = sample_of(&f, SYM_STRAIGHT);
        b5_straight_step(&f.gate, &ls); r = b5_straight_check(&f.gate, f.now);
    }
    CHECK(frames * DT_US >= B5S_DWELL_US);
    /* backwards travel is not following */
    b5_line_sample_t back = sample_of(&f, SYM_STRAIGHT);
    back.now_us = f.now + DT_US; back.wheel_mm[0] -= 5; back.wheel_mm[1] -= 5;
    b5_straight_step(&f.gate, &back);
    CHECK(f.gate.reason == B5S_OTHER && !f.gate.open);
    CHECK(fabsf(B5S_TRACK_MM - TRACK_MM) < 1e-6f);
    printf("  stale after %u ms without a Buddy3 frame closes the gate; dwell %u ms enforced; reverse travel blocks\n",
           (unsigned)(B5S_SAMPLE_MAX_US / 1000), (unsigned)(B5S_DWELL_US / 1000));
}

static void test_completed_barcode_turn_reopens_gate(void) {
    const char routes[] = {'A', 'F', 'Z'};
    const char actions[] = {'L', 'R', 'U'};
    const line_calibration_t cal = successful_calibration();
    for (unsigned route = 0; route < sizeof routes; ++route) {
        line_planner_t planner = {0};
        circuit_input_t input = {
            .now = 1000000U, .adc = {3400, 3600}, .speed = {45, 45}};
        b5_straight_t gate = {0};
        rl_begin(&planner, &input, 180);
        planner.circuit.phase = J_ACQUIRE;
        planner.circuit.turn.phase = J_ACQUIRE;
        planner.circuit.turn.phase_us = input.now;
        planner.circuit.busy = planner.nav.busy = true;
        planner.nav.action = routes[route];
        planner.nav.executing_action = actions[route];
        planner.nav.uturn.stage = route == 2U ? 4U : 0U;
        b5_line_sample_t sample;
        uint8_t reason = B5S_UNKNOWN;
        double completion_path = 0;
        bool completed = false;
        for (unsigned tick = 0; tick < 400U && reason != B5S_STRAIGHT; ++tick) {
            input.now += DT_US;
            input.path += .45;
            input.wheel[0] += .45;
            input.wheel[1] += .45;
            analog_demand_t demand = rl_follow(&planner, &cal, &input, .01f);
            rl_buddy5_straight_sample(&planner, &input, &demand, &sample);
            b5_straight_step(&gate, &sample);
            reason = b5_straight_check(&gate, input.now);
            if (planner.nav.busy) {
                CHECK(sample.route_turn && reason == B5S_TURN);
            } else if (!completed) {
                completed = true;
                completion_path = input.path;
                CHECK(planner.circuit.completed && !sample.route_turn);
                CHECK(planner.nav.action == routes[route]);
                CHECK(planner.nav.executing_action == actions[route]);
                CHECK(planner.nav.uturn.stage == (route == 2U ? 4U : 0U));
            }
        }
        CHECK(completed && reason == B5S_STRAIGHT && gate.open);
        CHECK(input.path - completion_path >= B5S_WINDOW_MM);
    }
    printf("  completed barcode turns reopen the gate after fresh straight travel\n");
}

int main(void) {
    test_straight_with_corrections();
    test_brief_bend_signals_on_a_straight();
    test_confirmed_bend_label_on_straight();
    test_tight_curve(+1);
    test_tight_curve(-1);
    test_gentle_curve(+1, 500, true);
    test_gentle_curve(-1, 500, true);
    test_gentle_curve(+1, 1200, false);
    test_gentle_curve(-1, 1200, false);
    test_gentle_curve(+1, 3000, false);
    test_states();
    test_stale_and_misc();
    test_completed_barcode_turn_reopens_gate();
    printf("%s: straight gate, %u checks, %u failure(s)\n", g_failures ? "FAIL" : "PASS", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
