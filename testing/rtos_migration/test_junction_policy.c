#include <assert.h>
#include <stdio.h>
#ifndef JUNCTION_ANGLE_GATE_ONLY
#define JUNCTION_ANGLE_GATE_ONLY 1
#endif
#define BUDDY_WIFI_LOGGING 1
#define LINE_CAL_MIN_SPAN 50
#define CIRCUIT_SWEEP_SEARCH 1
#include "app_program/line_navigation/line_model.h"

static const LineCalibration cal = {{100, 100}, {1100, 1100}, 7};
static float distance(float angle) {
    return angle * TRACK_MM * (3.14159265358979323846f / 180.0f);
}
static Junction seed(void) {
    return (Junction){.phase = J_TURN, .phase_us = 100000, .rolling = true};
}
static JunctionInput sample(uint64_t now, float angle, bool white) {
    return (JunctionInput){
        .now = now, .wheel = {0, distance(angle)}, .adc = {white ? 100 : 1100, white ? 100 : 1100}};
}
static void test_departure_dwell(void) {
    Junction j = seed();
    JunctionInput i = sample(100000, 9, false);
    (void)junction_step(&j, &cal, &i);
    i.now += 40000;
    (void)junction_step(&j, &cal, &i);
    assert(!j.off_line);
    i = sample(150000, 11, false);
    (void)junction_step(&j, &cal, &i);
    i.now += 29000;
    (void)junction_step(&j, &cal, &i);
    assert(!j.off_line);
    i.now += 1000;
    (void)junction_step(&j, &cal, &i);
    assert(j.off_line == !!JUNCTION_ANGLE_GATE_ONLY);

    j = seed();
    i = sample(100000, 11, true);
    (void)junction_step(&j, &cal, &i);
    i.now += 20000;
    i.wheel[1] = distance(9);
    (void)junction_step(&j, &cal, &i);
    i.now += 10000;
    i.wheel[1] = distance(11);
    (void)junction_step(&j, &cal, &i);
    i.now += 29000;
    (void)junction_step(&j, &cal, &i);
    assert(!j.off_line);
    i.now += 1000;
    (void)junction_step(&j, &cal, &i);
    assert(j.off_line); /* The original optical path still qualifies normally. */
}
static void test_branch_window(void) {
    Junction j = seed();
    JunctionInput i = sample(100000, 11, false);
    (void)junction_step(&j, &cal, &i);
    i.now += 30000;
    (void)junction_step(&j, &cal, &i);
    i = sample(140000, 74, false);
    (void)junction_step(&j, &cal, &i);
    assert(j.phase == J_TURN && !j.line_timing);
    i = sample(150000, 76, false);
    (void)junction_step(&j, &cal, &i);
    i.now += 29000;
    (void)junction_step(&j, &cal, &i);
    assert(j.phase == J_TURN);
    i.now += 1000;
    AnalogDemand d = junction_step(&j, &cal, &i);
    assert(j.phase == (JUNCTION_ANGLE_GATE_ONLY ? J_ACQUIRE : J_TURN));
    assert(!d.stop);
    if (!JUNCTION_ANGLE_GATE_ONLY) {
        i.now += 10000;
        i.wheel[1] = distance(111);
        d = junction_step(&j, &cal, &i);
        assert(d.stop && j.reason == J_BRANCH_MISSING);
    }

    j = seed();
    j.off_line = true;
    i = sample(100000, 76, false);
    (void)junction_step(&j, &cal, &i);
    i.now += 20000;
    i.adc[0] = i.adc[1] = 100;
    (void)junction_step(&j, &cal, &i);
    i.now += 10000;
    i.adc[0] = i.adc[1] = 1100;
    (void)junction_step(&j, &cal, &i);
    i.now += 29000;
    (void)junction_step(&j, &cal, &i);
    assert(j.phase == J_TURN); /* Black must be continuous for its own30ms. */
    i.now += 1000;
    (void)junction_step(&j, &cal, &i);
    assert(j.phase == J_ACQUIRE);

    j = seed();
    j.off_line = true;
    i = sample(100000, 111, false);
    d = junction_step(&j, &cal, &i);
    assert(d.stop && j.reason == J_BRANCH_MISSING && !d.target[0] && !d.target[1]);
    j = seed();
    j.off_line = true;
    i = sample(9100000, 76, false);
    d = junction_step(&j, &cal, &i);
    assert(d.stop && j.reason == J_TURN_TIMEOUT);
}
static void test_uturn_window(void) {
    Junction j = seed();
    BarcodeUTurn u = {0};
    JunctionInput i = sample(100000, 11, false);
    (void)barcode_uturn_step(&u, &j, &cal, &i);
    i.now += 30000;
    (void)barcode_uturn_step(&u, &j, &cal, &i);
    assert(j.off_line == !!JUNCTION_ANGLE_GATE_ONLY);
    u.stage = 2;
    u.reverse_origin[1] = distance(90);
    i.wheel[1] = distance(90);
    i.wheel[0] = -distance(164 - 90);
    i.now += 10000;
    (void)barcode_uturn_step(&u, &j, &cal, &i);
    assert(u.stage == 2 && !j.line_timing);
    i.wheel[0] = -distance(166 - 90);
    i.now += 10000;
    (void)barcode_uturn_step(&u, &j, &cal, &i);
    i.now += 29000;
    (void)barcode_uturn_step(&u, &j, &cal, &i);
    assert(u.stage == 2);
    i.now += 1000;
    (void)barcode_uturn_step(&u, &j, &cal, &i);
    assert(u.stage == (JUNCTION_ANGLE_GATE_ONLY ? 3 : 2));
    j.phase = J_TURN;
    u.stage = 2;
    i.now += 10000;
    i.wheel[0] = -distance(196 - 90);
    AnalogDemand d = barcode_uturn_step(&u, &j, &cal, &i);
    assert(d.stop && j.reason == J_BRANCH_MISSING && !d.target[0] && !d.target[1]);
}
static void test_route_diagnostics(void) {
    const char routes[] = {'A', 'F', 'Z'};
    for (unsigned n = 0; n < 3; ++n) {
        LinePlanner p;
        CircuitInput i = {.now = 100000};
        rl_begin(&p, &i, 180);
        p.circuit.phase = J_TURN;
        p.circuit.turn = seed();
        p.circuit.busy = p.nav.busy = true;
        p.nav.action = routes[n];
        unsigned outer = n == 1 ? 0 : 1;
        i.adc[0] = i.adc[1] = 1100;
        i.wheel[outer] = distance(11);
        (void)rl_follow(&p, &cal, &i, .01f);
        i.now += 30000;
        (void)rl_follow(&p, &cal, &i, .01f);
        if (n == 2) {
            p.nav.uturn.stage = 2;
            p.nav.uturn.reverse_origin[1] = distance(90);
            i.wheel[1] = distance(90);
            i.wheel[0] = -distance(76);
        } else
            i.wheel[outer] = distance(76);
        i.now += 10000;
        (void)rl_follow(&p, &cal, &i, .01f);
        i.now += 30000;
        (void)rl_follow(&p, &cal, &i, .01f);
        assert(!!(p.junction.flags & RL_J_ACCEPT_EVENT) == !!JUNCTION_ANGLE_GATE_ONLY);
        assert(!!(p.junction.flags & RL_J_ACCEPTED_WITHOUT_WHITE) == !!JUNCTION_ANGLE_GATE_ONLY);
        assert(!(p.junction.flags & RL_J_WHITE_QUALIFIED));
        if (JUNCTION_ANGLE_GATE_ONLY)
            assert(n == 2 ? p.nav.uturn.stage == 3 : p.circuit.phase == J_ACQUIRE);
        LineLogRow r = {0};
        rl_pack_steering(&r, i.adc, 700, 500);
        rl_pack_junction(&r, &p);
        assert(((r.aux[0] >> 24) | ((r.aux[1] & 3U) << 8)) == 700);
        assert(((r.aux[1] >> 2) & 1023U) == 500);
        assert(rl_log_angle(&r) == (n == 2 ? 1660 : 760));
        i.now += 10000;
        (void)rl_follow(&p, &cal, &i, .01f);
        assert(!(p.junction.flags & RL_J_ACCEPT_EVENT));
        assert(!!(p.junction.flags & RL_J_BRANCH_ACCEPTED) == !!JUNCTION_ANGLE_GATE_ONLY);
    }
    LinePlanner p = {0};
    CircuitInput i = {.now = 100000, .adc = {100, 100}};
    p.circuit.turn.turn_degrees = 20;
    rl_junction_observe(&p, &cal, &i, J_TURN, 0);
    i.now += 30000;
    p.circuit.phase = J_ACQUIRE;
    rl_junction_observe(&p, &cal, &i, J_TURN, 0);
    assert(p.junction.flags & RL_J_WHITE_QUALIFIED);
    assert(p.junction.flags & RL_J_BRANCH_ACCEPTED);
    assert(!(p.junction.flags & RL_J_ACCEPTED_WITHOUT_WHITE));
    p.circuit.phase = J_TURN;
    rl_junction_observe(&p, &cal, &i, J_CENTRE, 0);
    assert(!(p.junction.flags & (RL_J_BRANCH_ACCEPTED | RL_J_WHITE_QUALIFIED)));
    LineLogRow r = {0};
    p.circuit.turn.turn_degrees = -5.2f;
    rl_pack_junction(&r, &p);
    assert(rl_log_angle(&r) == -52);
}
int main(void) {
    test_departure_dwell();
    test_branch_window();
    test_uturn_window();
    test_route_diagnostics();
    printf("PASS junction policy=%s: L/R/U windows, two dwells, faults and diagnostic packing.\n",
           JUNCTION_ANGLE_GATE_ONLY ? "angle_only" : "reference_white");
    return 0;
}
