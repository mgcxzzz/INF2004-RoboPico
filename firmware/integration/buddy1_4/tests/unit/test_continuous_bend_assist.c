#include <assert.h>
#include <math.h>
#include <stdio.h>

#define CONTINUOUS_BEND_ASSIST 1
#include "src/line_following/continuous_steering.h"

static void init_circuit(Circuit *c) {
    *c = (Circuit){0};
    c->phase = C_FOLLOW;
}

static CircuitInput input(float left, float right) {
    CircuitInput i = {0};
    i.speed[0] = left;
    i.speed[1] = right;
    return i;
}

static AnalogDemand demand(int direction) {
    AnalogDemand d = {0};
    if (direction > 0) {
        d.target[0] = 25;
        d.target[1] = 50;
        d.level[0] = 750;
        d.level[1] = 250;
        d.decision = SYM_LEFT;
    } else if (direction < 0) {
        d.target[0] = 50;
        d.target[1] = 25;
        d.level[0] = 250;
        d.level[1] = 750;
        d.decision = SYM_RIGHT;
    } else {
        d.target[0] = 45;
        d.target[1] = 45;
        d.level[0] = 500;
        d.level[1] = 500;
        d.decision = SYM_STRAIGHT;
    }
    return d;
}

static AnalogDemand step(ContinuousSteering *s, AnalogDemand d, Circuit *c, CircuitInput *i,
                         float dt) {
    return continuous_steering_step(s, d, c, i, false, 1, dt);
}

static void straight_output_stays_at_v2_baseline(void) {
    Circuit c;
    init_circuit(&c);
    ContinuousSteering s = {0};
    CircuitInput i = input(45, 45);
    AnalogDemand d = demand(0), out = {0};
    for (unsigned k = 0; k < 40; ++k) {
        out = step(&s, d, &c, &i, .01f);
        assert(out.target[0] == 45 && out.target[1] == 45);
        assert(s.bend_speed_scale == 1.0f && s.bend_authority_scale == 1.0f);
        assert(!s.bend_candidate && !s.bend_confirmed);
    }
}

static void spikes_and_alternating_requests_do_not_qualify(void) {
    Circuit c;
    init_circuit(&c);
    ContinuousSteering s = {0};
    CircuitInput i = input(45, 45);
    (void)step(&s, demand(1), &c, &i, .01f); /* one sensor/request spike */
    (void)step(&s, demand(0), &c, &i, .01f);
    assert(!s.bend_candidate && !s.bend_confirmed);

    for (unsigned k = 0; k < 20; ++k)
        (void)step(&s, demand(k & 1u ? -1 : 1), &c, &i, .01f);
    assert(!s.bend_candidate && !s.bend_confirmed);
    assert(s.bend_speed_scale == 1.0f && s.bend_authority_scale == 1.0f);
}

static void sustained_direction_confirms_only_with_matching_encoder_yaw(int direction) {
    Circuit c;
    init_circuit(&c);
    ContinuousSteering s = {0};
    CircuitInput i = direction > 0 ? input(40, 50) : input(50, 40);
    AnalogDemand d = demand(direction), out = {0};
    for (unsigned k = 0; k < 40; ++k)
        out = step(&s, d, &c, &i, .01f);
    assert(s.bend_candidate && s.bend_confirmed);
    assert(s.bend_direction == direction);
    assert(s.bend_speed_scale < 0.81f && s.bend_speed_scale >= .79f);
    assert(s.bend_authority_scale > 1.0f && s.bend_authority_scale <= 1.10f);
    assert(out.target[0] >= 0 && out.target[1] >= 0);
}

static void encoder_yaw_without_response_cannot_confirm(void) {
    Circuit c;
    init_circuit(&c);
    ContinuousSteering s = {0};
    CircuitInput i = input(45, 45);
    AnalogDemand d = demand(1);
    for (unsigned k = 0; k < 35; ++k)
        (void)step(&s, d, &c, &i, .01f);
    assert(s.bend_candidate && !s.bend_confirmed);
    assert(s.bend_speed_scale < 1.0f);
    assert(s.bend_authority_scale == 1.0f);
}

static void candidate_only_dropout_blends_back(void) {
    Circuit c;
    init_circuit(&c);
    ContinuousSteering s = {0};
    CircuitInput i = input(45, 45); /* optical candidate, no measured yaw */
    for (unsigned k = 0; k < 30; ++k)
        (void)step(&s, demand(1), &c, &i, .01f);
    assert(s.bend_candidate && !s.bend_confirmed);
    assert(s.bend_speed_scale >= .79f && s.bend_speed_scale < .81f);

    i = input(45, 45);
    (void)step(&s, demand(0), &c, &i, .05f);
    (void)step(&s, demand(0), &c, &i, .05f);
    assert(!s.bend_candidate && !s.bend_confirmed && s.bend_exiting);
    assert(s.bend_speed_scale > .80f && s.bend_speed_scale < 1.0f);
    assert(s.bend_authority_scale == 1.0f);
    for (unsigned k = 0; k < 5; ++k)
        (void)step(&s, demand(0), &c, &i, .05f);
    assert(!s.bend_exiting && s.bend_speed_scale == 1.0f);
}

static void exit_blends_then_busy_resets_state(void) {
    Circuit c;
    init_circuit(&c);
    ContinuousSteering s = {0};
    CircuitInput i = input(40, 50);
    for (unsigned k = 0; k < 40; ++k)
        (void)step(&s, demand(1), &c, &i, .01f);
    assert(s.bend_confirmed);

    i = input(45, 45);
    (void)step(&s, demand(0), &c, &i, .05f);
    (void)step(&s, demand(0), &c, &i, .05f);
    assert(!s.bend_candidate && s.bend_exiting);
    assert(s.bend_speed_scale > .80f && s.bend_speed_scale < 1.0f);
    assert(s.bend_authority_scale > 1.0f && s.bend_authority_scale < 1.10f);
    for (unsigned k = 0; k < 8; ++k)
        (void)step(&s, demand(0), &c, &i, .05f);
    assert(!s.bend_candidate && !s.bend_confirmed && !s.bend_exiting);
    assert(s.bend_speed_scale == 1.0f && s.bend_authority_scale == 1.0f);

    s.bend_candidate = 1;
    s.bend_confirmed = 1;
    s.bend_direction = 1;
    s.bend_speed_scale = .8f;
    s.bend_authority_scale = 1.1f;
    c.busy = true;
    (void)step(&s, demand(1), &c, &i, .01f);
    assert(!s.active && !s.bend_candidate && !s.bend_confirmed && !s.bend_exiting);
    assert(s.bend_speed_scale == 0.0f && s.bend_authority_scale == 0.0f);
}

int main(void) {
    straight_output_stays_at_v2_baseline();
    spikes_and_alternating_requests_do_not_qualify();
    sustained_direction_confirms_only_with_matching_encoder_yaw(1);
    sustained_direction_confirms_only_with_matching_encoder_yaw(-1);
    encoder_yaw_without_response_cannot_confirm();
    candidate_only_dropout_blends_back();
    exit_blends_then_busy_resets_state();
    puts("continuous bend assist tests passed");
    return 0;
}
