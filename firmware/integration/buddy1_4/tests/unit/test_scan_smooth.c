#include <assert.h>
#include <stdio.h>
#include "src/line_following/scan_smooth.h"
static const LineCalibration cal = {{164, 168}, {3514, 3389}, 7};
typedef struct {
    Circuit c;
    BarcodeNav n;
    BarcodeAnalog a;
    SmoothFollow old;
    ScanSmooth s;
    CircuitInput i;
    AnalogDemand d;
} Rig;
static void init(Rig *r) {
    memset(r, 0, sizeof(*r));
    r->i = (CircuitInput){.now = 1000, .adc = {3514, 3389}, .speed = {45, 45}};
    circuit_begin(&r->c, &r->i);
    barcode_nav_begin(&r->n, &r->i);
    barcode_analog_begin(&r->a, &r->n, 180, r->i.now, r->i.wheel);
}
static void tick(Rig *r, unsigned adc) {
    r->i.now += 10000;
    for (unsigned w = 0; w < 2; ++w)
        r->i.wheel[w] += r->i.speed[w] * .01;
    r->i.path = (r->i.wheel[0] + r->i.wheel[1]) * .5;
    for (unsigned j = 0; j < 5; ++j)
        barcode_analog_sample(&r->a, &r->n, adc, r->i.now - 8000 + 2000 * j, r->i.wheel);
    r->i.side_black = r->a.black;
    r->d = scan_smooth_step(&r->c, &r->n, &cal, &r->i, 1, &r->old, &r->a, &r->s, .01f);
}
static void basics(void) {
    Rig r;
    init(&r);
    int previous = 45;
    for (unsigned k = 0; k < 100; ++k) {
        tick(&r, 180);
        assert(r.d.target[0] == r.d.target[1]);
        assert(r.d.target[0] <= 60 && r.d.target[0] <= previous + 1);
        previous = r.d.target[0];
    }
    assert(abs(r.d.target[0] - 60) <= 1 && !r.s.scanning);
    tick(&r, 220);
    assert(r.s.scanning && r.d.target[0] <= 50);
    for (unsigned k = 0; k < 60; ++k)
        tick(&r, 220);
    assert(abs(r.d.target[0] - 45) <= 1 && abs(r.d.target[1] - 45) <= 1);
    /* Neither a barcode nor a post-bend entry freezes the optical controller. */
    r.i.adc[0] = 3514;
    r.i.adc[1] = 168;
    tick(&r, 220);
    assert(r.d.target[0] == 0 && r.d.target[1] == 45 && r.c.follower.core.direction == 1);
    r.i.adc[0] = 164;
    r.i.adc[1] = 3389;
    tick(&r, 220);
    assert(r.d.target[0] == 45 && r.d.target[1] == 0);
    r.i.adc[0] = 164;
    r.i.adc[1] = 168;
    tick(&r, 220);
    assert(r.c.follower.core.white && r.d.decision != 24);
}
static void jitter(void) {
    Rig r;
    init(&r);
    Circuit c = r.c;
    BarcodeNav n = r.n;
    SmoothFollow old = {0};
    int prev[2] = {45, 45}, prev_old[2] = {45, 45};
    long variation = 0, old_variation = 0;
    for (unsigned k = 0; k < 400; ++k) {
        r.i.adc[0] = k % 2 ? 2400 : 3000;
        r.i.adc[1] = k % 2 ? 3000 : 2400;
        tick(&r, 220);
        AnalogDemand before = barcode_drive_step(&c, &n, &cal, &r.i, 1, &old, .01f);
        for (unsigned w = 0; w < 2; ++w) {
            variation += abs(r.d.target[w] - prev[w]);
            prev[w] = r.d.target[w];
            old_variation += abs(before.target[w] - prev_old[w]);
            prev_old[w] = before.target[w];
        }
        assert(!r.d.stop && r.d.target[0] >= 30 && r.d.target[1] >= 30);
    }
    printf("small-error target variation V12=%ld V12.2=%ld\n", old_variation, variation);
    assert(variation < old_variation);
    /* Persistent moderate correction converges; it is not mistaken for straight. */
    r.i.adc[0] = 3000;
    r.i.adc[1] = 1500;
    for (unsigned k = 0; k < 35; ++k)
        tick(&r, 220);
    assert(r.d.target[1] > r.d.target[0] + 10);
}
static void mirrored(void) {
    Rig a, b;
    init(&a);
    init(&b);
    for (unsigned k = 0; k < 100; ++k) {
        unsigned l = k % 20 < 10 ? 500 : 800, r = k % 20 < 10 ? 800 : 500;
        a.i.adc[0] = (uint16_t)(164 + 3350 * l / 1000);
        a.i.adc[1] = (uint16_t)(168 + 3221 * r / 1000);
        b.i.adc[0] = (uint16_t)(164 + 3350 * r / 1000);
        b.i.adc[1] = (uint16_t)(168 + 3221 * l / 1000);
        tick(&a, 220);
        tick(&b, 220);
        assert(abs(a.d.target[0] - b.d.target[1]) <= 1 && abs(a.d.target[1] - b.d.target[0]) <= 1);
    }
}
static unsigned pattern(char code) {
    for (unsigned p = 0; p < 512; ++p)
        if (barcode_character(p) == code)
            return p;
    assert(false);
    return 0;
}
static void route(char code, bool reverse) {
    Rig r;
    init(&r);
    unsigned p[3] = {0x094, pattern(code), 0x094};
    double widths[29];
    for (unsigned g = 0; g < 3; ++g)
        for (unsigned j = 0; j < 9; ++j)
            widths[g * 10 + j] = p[g] & (1u << (8 - j)) ? 9 : 3;
    widths[9] = widths[19] = 3;
    if (reverse)
        for (unsigned j = 0; j < 14; ++j) {
            double t = widths[j];
            widths[j] = widths[28 - j];
            widths[28 - j] = t;
        }
    bool soft = false, dispatch = false;
    for (unsigned k = 0; k < 1000; ++k) {
        double x = r.i.path + .45 - 45;
        unsigned adc = 180;
        if (x >= 0)
            for (unsigned j = 0; j < 29; ++j) {
                if (x < widths[j]) {
                    adc = j % 2 ? 203 : widths[j] < 4 ? 220 : 3000;
                    break;
                }
                x -= widths[j];
            }
        if (r.i.path + .45 >= 216 && r.i.path + .45 < 234)
            adc = 3000;
        tick(&r, adc);
        soft |= r.s.scanning;
        assert(!r.d.stop);
        if (r.c.busy) {
            assert(r.n.action == code && !r.n.pending && !r.s.scanning);
            dispatch = true;
            break;
        }
    }
    assert(soft && dispatch && r.n.decoded_a + r.n.decoded_f + r.n.decoded_z == 1);
}
int main(void) {
    basics();
    jitter();
    mirrored();
    for (unsigned r = 0; r < 2; ++r) {
        route('A', r);
        route('F', r);
        route('Z', r);
    }
    puts(
        "PASS V12.2:60mm/s gradual cruise;45mm/s scan;reduced small-error variation; persistent/mirrored steering; immediate strong departure after scan entry; real optical loss; generic A/F/Z both directions and cross dispatch priority.");
}
