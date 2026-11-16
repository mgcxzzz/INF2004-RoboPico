#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "src/line_following/continuous_steering.h"

static circuit_input_t input(float left, float right) {
    circuit_input_t i = {0};
    i.speed[0] = left;
    i.speed[1] = right;
    return i;
}
static analog_demand_t demand(int left, int right, unsigned l, unsigned r, unsigned decision) {
    analog_demand_t d = {0};
    d.target[0] = (int16_t)left;
    d.target[1] = (int16_t)right;
    d.level[0] = (uint16_t)l;
    d.level[1] = (uint16_t)r;
    d.decision = (uint8_t)decision;
    return d;
}
static void circuit(circuit_t *c) {
    *c = (circuit_t){0};
    c->phase = C_FOLLOW;
}
static analog_demand_t step(continuous_steering_t *s, analog_demand_t d, circuit_t *c, circuit_input_t *i,
                         float dt) {
    return continuous_steering_step(s, d, c, i, false, 1, dt);
}

static void initial_gain_and_full_authority(void) {
    circuit_t c;
    circuit(&c);
    continuous_steering_t s = {0};
    circuit_input_t i = input(45, 45);
    analog_demand_t d = demand(0, 45, 900, 100, SYM_LEFT), out;
    out = step(&s, d, &c, &i, .01f);
    assert(fabsf(s.gain - .70f) < .0001f);
    assert(out.target[0] >= 0 && out.target[1] <= 47);
    assert(abs(out.target[0] - 45) <= 5 && abs(out.target[1] - 45) <= 5);
    for (unsigned k = 0; k < 80; ++k)
        out = step(&s, d, &c, &i, .01f);
    assert(s.same_side_duration >= .35f && fabsf(s.gain - 1.0f) < .0001f);
    assert(out.target[0] == 0 && out.target[1] == 45);
    assert(s.damp_weight < .1f); /* constant curve is not opposed by encoder yaw */
}

static void mirrored_and_side_dwell(void) {
    circuit_t a, b;
    circuit(&a);
    circuit(&b);
    continuous_steering_t sa = {0}, sb = {0};
    circuit_input_t ia = input(45, 45), ib = input(45, 45);
    analog_demand_t da = demand(0, 45, 900, 100, SYM_LEFT);
    analog_demand_t db = demand(45, 0, 100, 900, SYM_RIGHT);
    analog_demand_t oa = {0}, ob = {0};
    const unsigned noisy_left[] = {800, 980, 650, 900, 720, 990};
    const unsigned noisy_right[] = {350, 150, 420, 300, 500, 200};
    for (unsigned k = 0; k < 30; ++k) {
        unsigned q = k % 6;
        da.level[0] = (uint16_t)noisy_left[q];
        da.level[1] = (uint16_t)noisy_right[q];
        db.level[0] = da.level[1];
        db.level[1] = da.level[0];
        oa = step(&sa, da, &a, &ia, .01f);
        ob = step(&sb, db, &b, &ib, .01f);
        assert(abs(oa.target[0] - ob.target[1]) <= 1);
        assert(abs(oa.target[1] - ob.target[0]) <= 1);
        assert(fabsf(sa.gain - sb.gain) < .0001f);
    }
    assert(sa.gain < 1 && fabsf(sa.gain - (.70f + .30f * (.29f / .35f))) < .02f);
    da = demand(45, 0, 100, 900, SYM_RIGHT);
    oa = step(&sa, da, &a, &ia, .01f);
    assert(sa.same_side_duration == 0 && fabsf(sa.gain - .70f) < .0001f);
    da = demand(45, 45, 500, 500, SYM_STRAIGHT);
    oa = step(&sa, da, &a, &ia, .01f);
    assert(sa.same_side_duration == 0 && fabsf(sa.gain - .70f) < .0001f);
}

static void brightness_independent_gain(void) {
    circuit_t a, b;
    circuit(&a);
    circuit(&b);
    continuous_steering_t sa = {0}, sb = {0};
    circuit_input_t ia = input(45, 45), ib = input(45, 45);
    analog_demand_t da = demand(24, 42, 700, 300, SYM_LEFT);
    analog_demand_t db = demand(24, 42, 950, 550, SYM_LEFT);
    analog_demand_t oa = {0}, ob = {0};
    for (unsigned k = 0; k < 20; ++k) {
        oa = step(&sa, da, &a, &ia, .01f);
        ob = step(&sb, db, &b, &ib, .01f);
        assert(oa.target[0] == ob.target[0] && oa.target[1] == ob.target[1]);
        assert(fabsf(sa.gain - sb.gain) < .0001f);
        assert(fabsf(sa.damp_weight - sb.damp_weight) < .0001f);
    }
}

static void dwell_uses_core_half_and_return_scales_with_fall_rate(void) {
    circuit_t c;
    circuit(&c);
    continuous_steering_t s = {0};
    circuit_input_t i = input(45, 45);
    analog_demand_t straight = demand(45, 45, 600, 590, SYM_STRAIGHT);
    for (unsigned k = 0; k < 500; ++k)
        (void)step(&s, straight, &c, &i, .01f);
    assert(s.same_side_duration == 0 && fabsf(s.gain - .70f) < .0001f);
    analog_demand_t brief_turn = demand(40, 50, 700, 300, SYM_LEFT);
    (void)step(&s, brief_turn, &c, &i, .01f);
    assert(s.same_side_duration == 0 && fabsf(s.gain - .70f) < .0001f);

    circuit_t tiny_c, large_c;
    circuit(&tiny_c);
    circuit(&large_c);
    continuous_steering_t tiny = {.mean = 45,
                               .emitted = {45, 45},
                               .previous_error = .601f,
                               .gain = .7f,
                               .active = 1,
                               .side = 1};
    continuous_steering_t large = {.mean = 45,
                                .emitted = {45, 45},
                                .previous_error = .8f,
                                .gain = .7f,
                                .active = 1,
                                .side = 1};
    analog_demand_t returning = demand(40, 50, 800, 200, SYM_LEFT);
    (void)continuous_steering_step(&tiny, returning, &tiny_c, &i, false, 1, .01f);
    (void)continuous_steering_step(&large, returning, &large_c, &i, false, 1, .01f);
    assert(tiny.damp_weight > 0 && tiny.damp_weight < .01f);
    assert(large.damp_weight > tiny.damp_weight * 20);
}

static void encoder_damping_only_on_return(void) {
    circuit_t yaw, still;
    circuit(&yaw);
    circuit(&still);
    continuous_steering_t a = {.mean = 45,
                            .half = 0,
                            .emitted = {45, 45},
                            .previous_error = .8f,
                            .same_side_duration = .35f,
                            .gain = 1,
                            .active = 1,
                            .side = 1};
    continuous_steering_t b = a;
    circuit_input_t moving = input(15, 45), stopped = input(30, 30);
    analog_demand_t d = demand(45, 45, 600, 200, SYM_STRAIGHT);
    analog_demand_t out_yaw = continuous_steering_step(&a, d, &yaw, &moving, false, 1, .05f);
    analog_demand_t out_still = continuous_steering_step(&b, d, &still, &stopped, false, 1, .05f);
    assert(a.damp_weight > 0 && b.damp_weight > 0);
    assert(fabsf(a.damp_weight - b.damp_weight) < .0001f);
    assert(out_yaw.target[0] > out_yaw.target[1]);
    assert(out_still.target[0] == out_still.target[1]);

    /* On a persistent error the return confidence decays, restoring the full
     * optical bend instead of continuously fighting measured curve speed. */
    a = (continuous_steering_t){.mean = 22.5f,
                             .half = 20,
                             .emitted = {3, 43},
                             .previous_error = .8f,
                             .same_side_duration = .35f,
                             .gain = 1,
                             .active = 1,
                             .side = 1};
    circuit_input_t curve = input(25, 65);
    analog_demand_t bend = demand(0, 45, 900, 100, SYM_LEFT);
    for (unsigned k = 0; k < 40; ++k)
        out_yaw = continuous_steering_step(&a, bend, &yaw, &curve, false, 1, .01f);
    assert(fabsf(a.gain - 1) < .0001f && a.damp_weight < .1f);
    assert(out_yaw.target[0] == 0 && out_yaw.target[1] == 45);
}

static void bounded_changes_and_profile_limits(void) {
    circuit_t c;
    circuit(&c);
    continuous_steering_t s = {0};
    circuit_input_t i = input(45, 45);
    analog_demand_t left = demand(0, 45, 900, 100, SYM_LEFT);
    analog_demand_t right = demand(45, 0, 100, 900, SYM_RIGHT);
    analog_demand_t out = step(&s, left, &c, &i, .01f);
    for (unsigned k = 0; k < 80; ++k)
        out = step(&s, left, &c, &i, .01f);
    int p0 = out.target[0], p1 = out.target[1];
    out = step(&s, right, &c, &i, .01f);
    assert(abs(out.target[0] - p0) <= 5 && abs(out.target[1] - p1) <= 5);
    assert(out.target[0] >= 0 && out.target[1] >= 0);

    continuous_steering_reset(&s);
    circuit(&c);
    i = input(45, 45);
    analog_demand_t fast = demand(135, 135, 800, 800, C_FAST);
    for (unsigned k = 0; k < 120; ++k)
        out = continuous_steering_step(&s, fast, &c, &i, false, 3, .01f);
    assert(out.target[0] == 135 && out.target[1] == 135 && c.fast);
    for (unsigned k = 0; k < 120; ++k)
        out = continuous_steering_step(&s, fast, &c, &i, true, 3, .01f);
    assert(out.target[0] <= 45 && out.target[1] <= 45 && !c.fast && out.decision == 24);

    continuous_steering_reset(&s);
    circuit(&c);
    i = input(NAN, NAN);
    out =
        continuous_steering_step(&s, demand(45, 45, 500, 500, SYM_STRAIGHT), &c, &i, false, 1, NAN);
    assert(out.target[0] >= 0 && out.target[1] >= 0 && isfinite(s.mean));
}

static void immediate_bypass_and_reset(void) {
    circuit_t c;
    circuit(&c);
    continuous_steering_t s = {
        .mean = 45, .half = 8, .emitted = {37, 53}, .active = 1, .gain = .9f, .damp_weight = .8f};
    circuit_input_t i = input(45, 45);
    analog_demand_t d = demand(0, 45, 0, 800, SYM_LEFT);
    c.follower.core.white = true;
    analog_demand_t out = step(&s, d, &c, &i, .01f);
    assert(out.target[0] == 0 && out.target[1] == 45 && !s.active && s.gain == 0);
    s.active = 1;
    c.follower.core.white = false;
    d.stop = true;
    out = step(&s, d, &c, &i, .01f);
    assert(out.stop && out.target[0] == 0 && out.target[1] == 45 && !s.active);
    s.active = 1;
    d.stop = false;
    d.target[0] = 31;
    d.target[1] = 22;
    d.decision = SYM_RIGHT;
    c.phase = C_APPROACH_FAST;
    out = step(&s, d, &c, &i, .01f);
    assert(out.target[0] == 31 && out.target[1] == 22 && out.decision == SYM_RIGHT);
    assert(!s.active && s.gain == 0 && s.damp_weight == 0);
}

static void runtime_cruise_modes_are_opt_in_and_bounded(void) {
    circuit_t baseline_c, normal_c, trial_c;
    circuit(&baseline_c);
    circuit(&normal_c);
    circuit(&trial_c);
    continuous_steering_t baseline = {0}, normal = {0}, trial = {0};
    circuit_input_t i = input(45, 45);
    analog_demand_t straight = demand(45, 45, 500, 500, SYM_STRAIGHT);
    analog_demand_t a = {0}, b = {0}, c = {0};
    for (unsigned k = 0; k < 65; ++k) {
        a = continuous_steering_step(&baseline, straight, &baseline_c, &i, false, 1, .01f);
        b = continuous_steering_step_with_cruise(&normal, straight, &normal_c, &i, false, 1,
                                                 .01f, DRIVE_MODE_NORMAL_SPEED);
        c = continuous_steering_step_with_cruise(&trial, straight, &trial_c, &i, false, 1,
                                                 .01f, DRIVE_MODE_TRIAL_SPEED);
    }
    assert(a.target[0] == 45 && a.target[1] == 45);
    assert(b.target[0] == 60 && b.target[1] == 60);
    assert(c.target[0] == 75 && c.target[1] == 75);

    circuit_t scan_c, turn_c, high_c;
    circuit(&scan_c);
    turn_c = (circuit_t){.phase = J_TURN};
    circuit(&high_c);
    continuous_steering_t scan = {0}, turn = {0}, high = {0};
    analog_demand_t scanned = {0}, turning = {0};
    analog_demand_t high_out = {0};
    for (unsigned k = 0; k < 65; ++k) {
        scanned = continuous_steering_step_with_cruise(&scan, straight, &scan_c, &i, true, 1,
                                                       .01f, DRIVE_MODE_TRIAL_SPEED);
        turning = continuous_steering_step_with_cruise(
            &turn, demand(30, 45, 700, 300, SYM_LEFT), &turn_c, &i, false, 1, .01f,
            DRIVE_MODE_TRIAL_SPEED);
        high_out = continuous_steering_step_with_cruise(
            &high, demand(110, 110, 500, 500, SYM_STRAIGHT), &high_c, &i, false, 1, .01f,
            DRIVE_MODE_TRIAL_SPEED);
    }
    assert(scanned.target[0] == 45 && scanned.target[1] == 45 && scanned.decision == 24);
    assert(turning.target[0] == 30 && turning.target[1] == 45);
    assert(high_out.target[0] <= DRIVE_MODE_TRIAL_SPEED &&
           high_out.target[1] <= DRIVE_MODE_TRIAL_SPEED);

    circuit_t zero_a_c, zero_b_c;
    circuit(&zero_a_c);
    circuit(&zero_b_c);
    continuous_steering_t zero_a = {0}, zero_b = {0};
    analog_demand_t pivot = demand(0, 45, 900, 100, SYM_LEFT), pivot_a = {0}, pivot_b = {0};
    for (unsigned k = 0; k < 65; ++k) {
        pivot_a = continuous_steering_step(&zero_a, pivot, &zero_a_c, &i, false, 1, .01f);
        pivot_b = continuous_steering_step_with_cruise(&zero_b, pivot, &zero_b_c, &i, false, 1,
                                                       .01f, DRIVE_MODE_TRIAL_SPEED);
        assert(pivot_a.target[0] == pivot_b.target[0] &&
               pivot_a.target[1] == pivot_b.target[1]);
    }
}

int main(void) {
    initial_gain_and_full_authority();
    mirrored_and_side_dwell();
    brightness_independent_gain();
    dwell_uses_core_half_and_return_scales_with_fall_rate();
    encoder_damping_only_on_return();
    bounded_changes_and_profile_limits();
    immediate_bypass_and_reset();
    runtime_cruise_modes_are_opt_in_and_bounded();
    puts("continuous steering v2 tests passed");
    return 0;
}
