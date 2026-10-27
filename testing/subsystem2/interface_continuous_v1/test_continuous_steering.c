#include <assert.h>
#include <stdio.h>
#include "continuous_steering.h"

static CircuitInput input(float left_speed, float right_speed) {
    CircuitInput i = {0};
    i.speed[0] = left_speed;
    i.speed[1] = right_speed;
    return i;
}

static AnalogDemand demand(int left, int right, unsigned llevel, unsigned rlevel,
                           unsigned decision) {
    AnalogDemand d = {0};
    d.target[0] = (int16_t)left;
    d.target[1] = (int16_t)right;
    d.level[0] = (uint16_t)llevel;
    d.level[1] = (uint16_t)rlevel;
    d.decision = (uint8_t)decision;
    return d;
}

static void ordinary_setup(Circuit *c) {
    *c = (Circuit){0};
    c->phase = C_FOLLOW;
}

static void ramp_and_converge(void) {
    Circuit c;
    ContinuousSteering s = {0};
    ordinary_setup(&c);
    CircuitInput i = input(45, 45);
    AnalogDemand d = demand(45, 45, 700, 700, SYM_STRAIGHT);
    AnalogDemand out = continuous_steering_step(&s, d, &c, &i, false, 1, .01f);
    assert(out.target[0] == 45 && out.target[1] == 45);

    /* A noisy old-style correction changes from 45/38 to 47/13. Both wheel
     * targets move by a bounded amount, then converge to the exact request. */
    s = (ContinuousSteering){.mean=41.5f, .half=-3.5f,
        .emitted={45,38}, .active=true};
    d = demand(47, 13, 760, 300, SYM_RIGHT);
    out = continuous_steering_step(&s, d, &c, &i, false, 1, .01f);
    assert(abs(out.target[0] - 45) <= 4);
    assert(abs(out.target[1] - 38) <= 4);
    for (unsigned k = 0; k < 120; ++k)
        out = continuous_steering_step(&s, d, &c, &i, false, 1, .01f);
    assert(out.target[0] == 47 && out.target[1] == 13);
    assert(out.target[0] >= 0 && out.target[1] >= 0);
}

static void turn_symmetry_and_authority(void) {
    Circuit a, b;
    ContinuousSteering sa = {0}, sb = {0};
    ordinary_setup(&a);
    ordinary_setup(&b);
    CircuitInput ia = input(45, 45), ib = input(45, 45);
    AnalogDemand left = demand(0, 45, 1000, 100, SYM_LEFT);
    AnalogDemand right = demand(45, 0, 100, 1000, SYM_RIGHT);
    AnalogDemand oa = {0}, ob = {0};
    for (unsigned k = 0; k < 80; ++k) {
        oa = continuous_steering_step(&sa, left, &a, &ia, false, 1, .01f);
        ob = continuous_steering_step(&sb, right, &b, &ib, false, 1, .01f);
        assert(abs(oa.target[0] - ob.target[1]) <= 1);
        assert(abs(oa.target[1] - ob.target[0]) <= 1);
    }
    assert(oa.target[0] == 0 && oa.target[1] == 45);
    assert(ob.target[0] == 45 && ob.target[1] == 0);
}

static void profile_and_scan_caps(void) {
    Circuit c;
    ContinuousSteering s = {0};
    ordinary_setup(&c);
    CircuitInput i = input(45, 45);
    AnalogDemand d = demand(135, 135, 800, 800, C_FAST);
    AnalogDemand out = {0};
    for (unsigned k = 0; k < 120; ++k)
        out = continuous_steering_step(&s, d, &c, &i, false, 3, .01f);
    assert(out.target[0] == 135 && out.target[1] == 135 && c.fast);

    for (unsigned k = 0; k < 120; ++k)
        out = continuous_steering_step(&s, d, &c, &i, true, 3, .01f);
    assert(out.target[0] <= 45 && out.target[1] <= 45);
    assert(!c.fast && out.decision == 24);
}

static void exits_are_immediate_and_reset_state(void) {
    Circuit c;
    ContinuousSteering s = {.mean=45, .half=8, .emitted={37,53}, .active=true};
    ordinary_setup(&c);
    CircuitInput i = input(45, 45);
    AnalogDemand loss = demand(0, 45, 0, 800, SYM_LEFT);
    c.follower.core.white = true;
    AnalogDemand out = continuous_steering_step(&s, loss, &c, &i, false, 1, .01f);
    assert(out.target[0] == 0 && out.target[1] == 45);
    assert(!s.active && s.mean == 0 && s.half == 0);

    s = (ContinuousSteering){.mean=45, .half=8, .emitted={37,53}, .active=true};
    c.follower.core.white = false;
    loss.stop = true;
    loss.decision = SYM_STRAIGHT;
    out = continuous_steering_step(&s, loss, &c, &i, false, 1, .01f);
    assert(out.stop && out.target[0] == 0 && out.target[1] == 45);
    assert(!s.active);

    /* A non-follow phase is also passed through byte-for-byte in its target
     * and decision fields, and clears accumulated steering state. */
    s.active = true;
    c.phase = C_APPROACH_FAST;
    loss.stop = false;
    loss.target[0] = 31;
    loss.target[1] = 22;
    loss.decision = SYM_RIGHT;
    out = continuous_steering_step(&s, loss, &c, &i, false, 3, .01f);
    assert(out.target[0] == 31 && out.target[1] == 22);
    assert(out.decision == SYM_RIGHT && !s.active);
}

int main(void) {
    ramp_and_converge();
    turn_symmetry_and_authority();
    profile_and_scan_caps();
    exits_are_immediate_and_reset_state();
    puts("continuous steering tests passed");
    return 0;
}
