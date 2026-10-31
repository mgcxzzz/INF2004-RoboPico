#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "src/barcode/barcode_nav.h"

static AnalogDemand observed_raw, observed_soft;
static unsigned raw_calls, soft_calls;
#define DRIVE_DIAG_RAW(d)                                                                          \
    do {                                                                                           \
        observed_raw = (d);                                                                        \
        ++raw_calls;                                                                               \
    } while (0)
#define DRIVE_DIAG_SOFT(d, kind, scan, active)                                                     \
    do {                                                                                           \
        observed_soft = (d);                                                                       \
        ++soft_calls;                                                                              \
        (void)(kind);                                                                              \
        (void)(scan);                                                                              \
        (void)(active);                                                                            \
    } while (0)
#include "src/line_following/continuous_follow.h"

static const LineCalibration cal = {{163, 160}, {3457, 3700}, 7};

typedef struct {
    Circuit c;
    BarcodeNav n;
    BarcodeAnalog a;
    SmoothFollow legacy;
    ScanSmooth scan;
    ContinuousSteering steering;
    CircuitInput i;
    AnalogDemand d;
} Rig;

static void init(Rig *r) {
    memset(r, 0, sizeof(*r));
    r->i = (CircuitInput){.now = 1000, .adc = {3457, 3700}, .speed = {45, 45}};
    circuit_begin(&r->c, &r->i);
    barcode_nav_begin(&r->n, &r->i);
    barcode_analog_begin(&r->a, &r->n, 180, r->i.now, r->i.wheel);
}

static void same_demand(AnalogDemand a, AnalogDemand b) {
    assert(a.target[0] == b.target[0] && a.target[1] == b.target[1]);
    assert(a.level[0] == b.level[0] && a.level[1] == b.level[1]);
    assert(a.decision == b.decision && a.stop == b.stop);
}

static void same_navigation(const Circuit *a, const Circuit *b, const BarcodeNav *na,
                            const BarcodeNav *nb) {
    assert(a->phase == b->phase && a->busy == b->busy && a->fault == b->fault);
    assert(a->turns == b->turns && a->ignored == b->ignored);
    assert(memcmp(&a->follower, &b->follower, sizeof(a->follower)) == 0);
    assert(na->decoded_a == nb->decoded_a && na->decoded_f == nb->decoded_f);
    assert(na->decoded_z == nb->decoded_z && na->pending == nb->pending);
    assert(na->action == nb->action && na->busy == nb->busy);
}

static void sample_bars(Rig *a, Rig *b, unsigned adc) {
    for (unsigned t = 0; t < 5; ++t) {
        uint64_t stamp = a->i.now - 8000 + 2000 * t;
        barcode_analog_sample(&a->a, &a->n, (uint16_t)adc, stamp, a->i.wheel);
        barcode_analog_sample(&b->a, &b->n, (uint16_t)adc, stamp, b->i.wheel);
    }
    a->i.side_black = a->a.black;
    b->i.side_black = b->a.black;
}

static void ordinary_and_optical_parity(unsigned profile) {
    raw_calls = soft_calls = 0;
    Rig follow, raw;
    init(&follow);
    init(&raw);
    unsigned ordinary = 0, changed = 0, bypass = 0, begin = 0, end = 0;
    for (unsigned k = 0; k < 4000; ++k) {
        follow.i.now += 10000;
        follow.i.wheel[0] += .44;
        follow.i.wheel[1] += .45;
        follow.i.path += .445;
        raw.i = follow.i;
        const unsigned l[] = {1000, 800, 500, 1000, 0, 0, 250, 700, 500, 900};
        const unsigned r[] = {950, 650, 300, 0, 1000, 0, 900, 700, 950, 250};
        unsigned part = (k / 31) % 10;
        follow.i.adc[0] = raw.i.adc[0] = (uint16_t)(163 + 3294 * l[part] / 1000);
        follow.i.adc[1] = raw.i.adc[1] = (uint16_t)(160 + 3540 * r[part] / 1000);
        unsigned bar_adc = k % 170 < 40 ? 220 : 180;
        sample_bars(&follow, &raw, bar_adc);
        /* Force short scanner activity windows independent of junction or
         * decoder outcomes, then leave enough quiet time for soft-end. */
        follow.a.adc = raw.a.adc = (k % 500 < 25) ? 220 : 180;

        bool was_scanning = follow.scan.scanning;
        follow.d =
            continuous_follow_step(&follow.c, &follow.n, &cal, &follow.i, profile, &follow.legacy,
                                   &follow.a, &follow.scan, &follow.steering, .01f);
        AnalogDemand expected = barcode_circuit_step(&raw.c, &raw.n, &cal, &raw.i, profile);
        same_demand(observed_raw, expected);
        same_navigation(&follow.c, &raw.c, &follow.n, &raw.n);
        bool eligible = follow.c.phase == C_FOLLOW && !follow.c.busy && !expected.stop &&
                        !follow.c.follower.core.white &&
                        (expected.decision == SYM_STRAIGHT || expected.decision == SYM_LEFT ||
                         expected.decision == SYM_RIGHT || expected.decision == C_FAST);
        if (eligible) {
            ++ordinary;
            assert(follow.d.decision == 24 || follow.d.decision == 25);
            if (follow.d.target[0] != expected.target[0] ||
                follow.d.target[1] != expected.target[1])
                ++changed;
        } else {
            ++bypass;
            same_demand(follow.d, expected);
        }
        assert(observed_soft.target[0] == follow.d.target[0] &&
               observed_soft.target[1] == follow.d.target[1]);
        if (!was_scanning && follow.scan.scanning)
            ++begin;
        if (was_scanning && !follow.scan.scanning)
            ++end;
    }
    assert(ordinary > 0 && changed > 0 && bypass > 0);
    assert(begin > 0 && end > 0);
    assert(raw_calls == 4000 && soft_calls == 4000);
}

static void profile2_is_exact_legacy(void) {
    raw_calls = soft_calls = 0;
    Rig modern, legacy;
    init(&modern);
    init(&legacy);
    for (unsigned k = 0; k < 1200; ++k) {
        modern.i.now += 10000;
        modern.i.wheel[0] += .44;
        modern.i.wheel[1] += .45;
        modern.i.path += .445;
        legacy.i = modern.i;
        unsigned q = (k / 23) % 8;
        const unsigned l[] = {1000, 850, 600, 300, 0, 0, 500, 1000};
        const unsigned r[] = {1000, 700, 250, 0, 1000, 0, 500, 850};
        modern.i.adc[0] = legacy.i.adc[0] = (uint16_t)(163 + 3294 * l[q] / 1000);
        modern.i.adc[1] = legacy.i.adc[1] = (uint16_t)(160 + 3540 * r[q] / 1000);
        unsigned bar_adc = k % 160 < 38 ? 220 : 180;
        sample_bars(&modern, &legacy, bar_adc);
        modern.d = continuous_follow_step(&modern.c, &modern.n, &cal, &modern.i, 2, &modern.legacy,
                                          &modern.a, &modern.scan, &modern.steering, .01f);
        AnalogDemand expected = scan_smooth_step(&legacy.c, &legacy.n, &cal, &legacy.i, 2,
                                                 &legacy.legacy, &legacy.a, &legacy.scan, .01f);
        same_demand(modern.d, expected);
        same_navigation(&modern.c, &legacy.c, &modern.n, &legacy.n);
        assert(modern.scan.mean == legacy.scan.mean && modern.scan.half == legacy.scan.half);
        assert(modern.scan.emitted[0] == legacy.scan.emitted[0] &&
               modern.scan.emitted[1] == legacy.scan.emitted[1]);
        assert(modern.scan.active == legacy.scan.active &&
               modern.scan.scanning == legacy.scan.scanning);
        assert(modern.scan.activity_us == legacy.scan.activity_us &&
               modern.scan.start_mm == legacy.scan.start_mm);
        assert(!modern.steering.active);
        if (modern.d.stop) {
            circuit_begin(&modern.c, &modern.i);
            circuit_begin(&legacy.c, &legacy.i);
            modern.scan = (ScanSmooth){0};
            legacy.scan = (ScanSmooth){0};
        }
    }
}

static void bypass_paths(void) {
    const unsigned phases[] = {J_CENTRE, J_TURN, J_ACQUIRE};
    for (unsigned ncase = 0; ncase < 5; ++ncase) {
        Rig modern, oracle;
        init(&modern);
        init(&oracle);
        modern.steering =
            (ContinuousSteering){.mean = 42, .half = 7, .emitted = {35, 49}, .active = true};
        oracle.i = modern.i;
        if (ncase == 0) {
            modern.c.follower.core.white = true;
            oracle.c.follower.core.white = true;
        } else if (ncase == 1) {
            modern.c.fault = oracle.c.fault = true;
            modern.c.phase = oracle.c.phase = J_FAULT;
        } else {
            unsigned phase = phases[ncase - 2];
            modern.c.busy = oracle.c.busy = true;
            modern.c.phase = oracle.c.phase = (uint8_t)phase;
            modern.c.turn.phase = oracle.c.turn.phase = (uint8_t)phase;
            modern.c.turn.phase_us = oracle.c.turn.phase_us = modern.i.now;
            modern.c.edge_path = oracle.c.edge_path = modern.i.path;
            modern.n.busy = oracle.n.busy = true;
            modern.n.action = oracle.n.action = 'A';
        }
        AnalogDemand out =
            continuous_follow_step(&modern.c, &modern.n, &cal, &modern.i, 1, &modern.legacy,
                                   &modern.a, &modern.scan, &modern.steering, .01f);
        AnalogDemand original = barcode_circuit_step(&oracle.c, &oracle.n, &cal, &oracle.i, 1);
        same_demand(observed_raw, original);
        same_demand(out, original);
        assert(!modern.steering.active);
    }
}

static unsigned barcode_pattern(char code) {
    for (unsigned p = 0; p < 512; ++p)
        if (barcode_character(p) == code)
            return p;
    assert(false);
    return 0;
}

static void route_parity(char code, bool reverse) {
    Rig modern, oracle;
    init(&modern);
    init(&oracle);
    unsigned p[3] = {0x094, barcode_pattern(code), 0x094};
    double widths[29];
    for (unsigned g = 0; g < 3; ++g)
        for (unsigned j = 0; j < 9; ++j)
            widths[g * 10 + j] = (p[g] & (1u << (8 - j))) ? 9 : 3;
    widths[9] = widths[19] = 3;
    if (reverse)
        for (unsigned j = 0; j < 14; ++j) {
            double temp = widths[j];
            widths[j] = widths[28 - j];
            widths[28 - j] = temp;
        }
    bool dispatched = false;
    for (unsigned k = 0; k < 1000; ++k) {
        modern.i.now += 10000;
        modern.i.wheel[0] += .45;
        modern.i.wheel[1] += .45;
        modern.i.path = (modern.i.wheel[0] + modern.i.wheel[1]) * .5;
        oracle.i = modern.i;
        double x = modern.i.path + .45 - 45;
        unsigned adc = 180;
        if (x >= 0)
            for (unsigned j = 0; j < 29; ++j) {
                if (x < widths[j]) {
                    adc = j % 2 ? 203 : widths[j] < 4 ? 220 : 3000;
                    break;
                }
                x -= widths[j];
            }
        if (modern.i.path + .45 >= 216 && modern.i.path + .45 < 234)
            adc = 3000;
        sample_bars(&modern, &oracle, adc);
        modern.d = continuous_follow_step(&modern.c, &modern.n, &cal, &modern.i, 1, &modern.legacy,
                                          &modern.a, &modern.scan, &modern.steering, .01f);
        AnalogDemand expected = barcode_circuit_step(&oracle.c, &oracle.n, &cal, &oracle.i, 1);
        same_demand(observed_raw, expected);
        same_navigation(&modern.c, &oracle.c, &modern.n, &oracle.n);
        if (modern.c.busy) {
            assert(modern.n.action == code && !modern.n.pending);
            same_demand(modern.d, expected);
            dispatched = true;
            break;
        }
    }
    assert(dispatched);
    assert(modern.n.decoded_a + modern.n.decoded_f + modern.n.decoded_z == 1);
}

int main(void) {
    ordinary_and_optical_parity(1);
    ordinary_and_optical_parity(3);
    profile2_is_exact_legacy();
    bypass_paths();
    for (unsigned reverse = 0; reverse < 2; ++reverse) {
        route_parity('A', reverse != 0);
        route_parity('F', reverse != 0);
        route_parity('Z', reverse != 0);
    }
    puts("continuous follow wrapper regression passed");
    return 0;
}
