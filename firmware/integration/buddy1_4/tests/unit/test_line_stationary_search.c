#ifndef CIRCUIT_STATIONARY_SEARCH
#define CIRCUIT_STATIONARY_SEARCH 1
#endif
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "app_program/line_navigation/line_model.h"
#include "src/navigation/nav_coordinator.h"

static const LineCalibration cal = {{184, 184}, {3464, 3692}, 7};

static LineSearchDemand step(LineSearch *s, uint64_t now, uint32_t path, const double wheel[2],
                             const float speed[2], bool candidate, bool confirmed) {
    return line_search_stationary_step(s, now, path, wheel, speed, candidate, confirmed);
}

static LineSearchDemand settle(LineSearch *s, uint64_t *now, uint32_t path, const double wheel[2],
                               const float stopped[2]) {
    LineSearchDemand d = {0};
    for (unsigned n = 0; n <= LS_STILL_US / 10000 + 1; ++n) {
        *now += 10000;
        d = step(s, *now, path, wheel, stopped, false, false);
        if (d.reason)
            return d;
    }
    return d;
}

static LineSearchDemand move_to_endpoint(LineSearch *s, uint64_t *now, double wheel[2],
                                         double *path, float speed[2]) {
    LineSearchDemand d = {0};
    for (unsigned n = 0; n < 300; ++n) {
        d = step(s, *now, (uint32_t)*path, wheel, speed, false, false);
        if (d.phase == LS_PHASE_SETTLE_REVERSAL)
            return d;
        assert(!d.reason && d.phase == LS_PHASE_SWEEP);
        double left = d.target[0] * .01, right = d.target[1] * .01;
        wheel[0] += left;
        wheel[1] += right;
        *path += (fabs(left) + fabs(right)) * .5;
        speed[0] = (float)d.target[0];
        speed[1] = (float)d.target[1];
        *now += 10000;
    }
    assert(!"target angle was not reached");
    return d;
}

static void test_sweeps(void) {
    const float stopped[2] = {0, 0};
    for (int first = -1; first <= 1; first += 2) {
        LineSearch s = {0};
        double wheel[2] = {0, 0}, path = 0;
        float speed[2] = {0, 0};
        uint64_t now = 1000;
        line_search_stationary_begin(&s, now, 0, first, wheel);
        LineSearchDemand d = step(&s, now, 0, wheel, speed, false, false);
        assert(d.phase == LS_PHASE_SETTLE_START && !d.target[0] && !d.target[1]);
        d = settle(&s, &now, 0, wheel, stopped);
        assert(d.phase == LS_PHASE_SWEEP && d.target[0] == -first * LS_STATIONARY_SPEED &&
               d.target[1] == first * LS_STATIONARY_SPEED && d.target[0] == -d.target[1]);

        for (unsigned stage = 0; stage < 4; ++stage) {
            d = move_to_endpoint(&s, &now, wheel, &path, speed);
            assert(d.target[0] == 0 && d.target[1] == 0);
            /* A zero target alone cannot authorize reversal while encoders or
               measured wheel speed still show motion. */
            now += 10000;
            d = step(&s, now, (uint32_t)path, wheel, speed, false, false);
            assert(d.phase == LS_PHASE_SETTLE_REVERSAL && !d.target[0] && !d.target[1]);
            speed[0] = speed[1] = 0;
            d = settle(&s, &now, (uint32_t)path, wheel, stopped);
            if (stage == 3) {
                assert(d.reason == LS_SWEEPS_EXHAUSTED && d.stage == 3);
                break;
            }
            int direction = ((stage + 1U) & 1U) ? -first : first;
            assert(d.phase == LS_PHASE_SWEEP && d.stage == stage + 1 &&
                   d.target[0] == -direction * LS_STATIONARY_SPEED &&
                   d.target[1] == direction * LS_STATIONARY_SPEED &&
                   d.target[0] == -d.target[1]);
        }
        assert(path < LS_MAX_PATH_MM);
    }
}

static void test_bounded_failure(void) {
    LineSearch s = {0};
    double wheel[2] = {0, 0};
    float stopped[2] = {0, 0};
    uint64_t now = 1000;
    line_search_stationary_begin(&s, now, 0, 1, wheel);
    LineSearchDemand d = step(&s, now, 0, wheel, stopped, false, false);
    assert(d.phase == LS_PHASE_SETTLE_START);
    float moving[2] = {10, 10};
    double path = 0;
    for (unsigned n = 1; n <= LS_SETTLE_LIMIT_US / 10000; ++n) {
        wheel[0] += .1;
        wheel[1] += .1;
        path += .1;
        d = step(&s, now + n * 10000, (uint32_t)path, wheel, moving, false, false);
    }
    assert(d.reason == LS_SETTLE_TIMEOUT);

    s = (LineSearch){0};
    now = 1000;
    line_search_stationary_begin(&s, now, 0, -1, wheel);
    for (unsigned n = 0; n <= LS_STILL_US / 10000 + 1; ++n) {
        now += 10000;
        d = step(&s, now, 0, wheel, stopped, false, false);
    }
    assert(d.phase == LS_PHASE_SWEEP);
    d = step(&s, now + LS_STALL_US, 0, wheel, stopped, false, false);
    assert(d.reason == LS_STALL);

    s = (LineSearch){0};
    line_search_stationary_begin(&s, 1000, 0, 1, wheel);
    wheel[0] = 25;
    wheel[1] = -25;
    d = step(&s, 2000, 0, wheel, stopped, false, false);
    assert(d.reason == LS_ANGLE);

    s = (LineSearch){0};
    line_search_stationary_begin(&s, 1000, 0, 1, wheel);
    d = step(&s, 1000 + LS_TOTAL_US, 0, wheel, stopped, false, false);
    assert(d.reason == LS_TIMEOUT);

    s = (LineSearch){0};
    line_search_stationary_begin(&s, 1000, 0, 1, wheel);
    float invalid_speed[2] = {NAN, 0};
    d = step(&s, 2000, 0, wheel, invalid_speed, false, false);
    assert(d.reason == LS_INPUT);
    line_search_stationary_begin(&s, 3000, 0, 1, NULL);
    d = step(&s, 4000, 0, wheel, stopped, false, false);
    assert(d.reason == LS_INPUT);
    double invalid_wheel[2] = {NAN, 0};
    line_search_stationary_begin(&s, 5000, 0, 1, invalid_wheel);
    d = step(&s, 6000, 0, wheel, stopped, false, false);
    assert(d.reason == LS_INPUT);
    CircuitTracker invalid_tracker = {0};
    AnalogDemand invalid = circuit_line_demand_motion(&invalid_tracker, &cal, 184, 184,
        7000, 0, NULL, stopped);
    assert(invalid.stop && invalid_tracker.loss_reason == LS_INPUT);
}

static void test_production_tracker_and_handoff(void) {
    CircuitTracker t = {0};
    const double wheel[2] = {0, 0};
    const float stopped[2] = {0, 0};
    uint64_t now = 1000;
    AnalogDemand d = circuit_line_demand_motion(&t, &cal, 3464, 184, now, 0, wheel, stopped);
    assert(t.core.direction == 1 && !d.stop);
    now += 10000;
    d = circuit_line_demand_motion(&t, &cal, 184, 184, now, 0, wheel, stopped);
    assert(t.core.white && t.search.waiting);
    now += LS_BRIDGE_US + 10000;
    d = circuit_line_demand_motion(&t, &cal, 184, 184, now, 0, wheel, stopped);
    assert(!d.stop && !d.target[0] && !d.target[1] && !t.search.active);
    now = t.core.white_since + LS_CONFIRM_US;
    d = circuit_line_demand_motion(&t, &cal, 184, 184, now, 0, wheel, stopped);
    assert(t.search.active && t.search.phase == LS_PHASE_SETTLE_START && !d.target[0] &&
           !d.target[1]);
    now += LS_STILL_US;
    d = circuit_line_demand_motion(&t, &cal, 184, 184, now, 0, wheel, stopped);
    assert(t.search.phase == LS_PHASE_SWEEP && d.target[0] == -LS_STATIONARY_SPEED &&
           d.target[1] == LS_STATIONARY_SPEED);

    /* A strong one-sided return must first meet the existing40ms confirmation,
       then keep zero targets through a separate encoder-still settle. */
    now += 10000;
    d = circuit_line_demand_motion(&t, &cal, 3464, 184, now, 0, wheel, stopped);
    assert(t.core.white && t.search.phase == LS_PHASE_CONFIRMING && !d.target[0] &&
           !d.target[1]);
    now += C_REACQUIRE_STRONG_US;
    d = circuit_line_demand_motion(&t, &cal, 3464, 184, now, 0, wheel, stopped);
    assert(t.core.white && t.search.phase == LS_PHASE_SETTLE_FOUND && !d.target[0] &&
           !d.target[1]);
    now += LS_STILL_US - 10000;
    d = circuit_line_demand_motion(&t, &cal, 3464, 184, now, 0, wheel, stopped);
    assert(t.core.white && !d.target[0] && !d.target[1]);
    now += 10000;
    d = circuit_line_demand_motion(&t, &cal, 3464, 184, now, 0, wheel, stopped);
    assert(!t.core.white && !t.search.active && !d.stop);

    LinePlanner planner = {0};
    planner.circuit.phase = C_FOLLOW;
    planner.circuit.follower.core.white = true;
    planner.circuit.follower.search.active = true;
    LinePower power = {0};
    CircuitInput input = {.now = now, .speed = {0, 0}};
    AnalogDemand search = {.target = {-LS_STATIONARY_SPEED, LS_STATIONARY_SPEED}};
    LineEnvelope env = {0};
    for (unsigned n = 0; n < 20; ++n) {
        env = rl_power(&power, &planner, &input, &search, C_FOLLOW, .01f);
        power.output[0] = env.output[0];
        power.output[1] = env.output[1];
        input.now += 10000;
    }
    now = input.now;
    int32_t left_pwm = rl_cap(&env, 0, now), right_pwm = rl_cap(&env, 1, now);
    assert(env.recovery && !env.maneuver && left_pwm < 0 && right_pwm > 0 &&
           abs(left_pwm) <= LINE_RECOVERY_PWM_LIMIT && right_pwm <= LINE_RECOVERY_PWM_LIMIT);
    DualLevels left = rl_levels(&env, 0, now), right = rl_levels(&env, 1, now);
    assert(left.a == 0 && left.b > 0 && right.a == 0 && right.b > 0);
    search.target[0] = search.target[1] = 0;
    env = rl_power(&power, &planner, &input, &search, C_FOLLOW, .01f);
    assert(env.recovery && !env.output[0] && !env.output[1]);
    for (unsigned w = 0; w < 2; ++w)
        for (int sign = -1; sign <= 1; sign += 2) {
            DualLevels v = line_recovery_levels(w, sign * LINE_RECOVERY_PWM_LIMIT);
            assert(v.a <= LINE_RECOVERY_PWM_LIMIT && v.b <= LINE_RECOVERY_PWM_LIMIT &&
                   ((v.a == LINE_RECOVERY_PWM_LIMIT) != (v.b == LINE_RECOVERY_PWM_LIMIT)));
        }
    assert(!line_recovery_levels(2, LINE_RECOVERY_PWM_LIMIT).a &&
           !line_recovery_levels(2, LINE_RECOVERY_PWM_LIMIT).b);
    LineRecoveryWheel invalid = {0};
    assert(!line_recovery_wheel_step(&invalid, 2, LS_STATIONARY_SPEED, 0, .01f, now, 0));
    assert(!line_recovery_wheel_step(&invalid, 0, LS_STATIONARY_SPEED, NAN, .01f, now, 0));
}

static void test_unknown_direction_is_symmetric(void) {
    CircuitTracker t = {0};
    const double wheel[2] = {0, 0};
    const float stopped[2] = {0, 0};
    uint64_t now = 1000;
    AnalogDemand d = circuit_line_demand_motion(&t, &cal, 184, 184, now, 0, wheel, stopped);
    assert(t.core.white && !t.search.active && !d.target[0] && !d.target[1]);
    now = t.core.white_since + LS_CONFIRM_US;
    d = circuit_line_demand_motion(&t, &cal, 184, 184, now, 0, wheel, stopped);
    assert(t.search.active && t.search.first == 1 && t.search.phase == LS_PHASE_SETTLE_START &&
           !d.target[0] && !d.target[1]);
    now += LS_STILL_US;
    d = circuit_line_demand_motion(&t, &cal, 184, 184, now, 0, wheel, stopped);
    assert(t.search.phase == LS_PHASE_SWEEP && d.target[0] == -LS_STATIONARY_SPEED &&
           d.target[1] == LS_STATIONARY_SPEED && d.target[0] == -d.target[1]);
}

static void test_signed_coordinator_targets(void) {
    nav_coordinator_init(0);
    nav_coordinator_begin(1000);
    assert(nav_coordinator_navigation(MOTION_BUDDY3, -24, 24) == MOTION_OK);
    MotionFeedback feedback = {.now_us = 1000};
    MotionOutput output = nav_coordinator_update(&feedback);
    assert(!output.external && output.target_mm_s[0] == -24 && output.target_mm_s[1] == 24);
    nav_coordinator_end(0);
}

int main(void) {
    test_sweeps();
    test_bounded_failure();
    test_production_tracker_and_handoff();
    test_unknown_direction_is_symmetric();
    test_signed_coordinator_targets();
    puts("PASS: bounded stationary sweep, still/reverse gates, credible reacquisition, recovery PWM envelope, and signed coordinator targets");
}
