#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "src/line_following/scan_smooth.h"
#include "src/motion/maneuver_power.h"
#include "include/interfaces/nav_api.h"
#include "src/motion/motion_port.h"
static uint64_t clock_us;
static uint64_t now(void *p) {
    (void)p;
    return clock_us;
}
static const LineCalibration cal = {{164, 168}, {3514, 3389}, 7};
typedef struct {
    Circuit c;
    BarcodeNav n;
    BarcodeAnalog a;
    SmoothFollow old;
    ScanSmooth s;
    SignedWheel wheel[2];
} Rig;
static void init(Rig *r, const CircuitInput *i) {
    memset(r, 0, sizeof(*r));
    circuit_begin(&r->c, i);
    barcode_nav_begin(&r->n, i);
    barcode_analog_begin(&r->a, &r->n, 180, i->now, i->wheel);
}
int main(void) {
    unsigned ticks = 0, restarts = 0;
    for (unsigned profile = 1; profile <= 3; ++profile) {
        Rig baseline, routed;
        CircuitInput i = {.now = 1000, .adc = {3514, 3389}, .speed = {45, 45}};
        init(&baseline, &i);
        init(&routed, &i);
        clock_us = i.now;
        MotionPort p = {.now_us = now};
        motion_init(&p);
        motion_session_begin(clock_us);
        for (unsigned k = 0; k < 3000; ++k) {
            clock_us = i.now += 10000;
            i.wheel[0] += .43;
            i.wheel[1] += .45;
            i.path += .44;
            unsigned stage = (k / 70) % 8;
            unsigned left[] = {1000, 700, 500, 1000, 100, 0, 1000, 700};
            unsigned right[] = {950, 800, 350, 0, 1000, 0, 1000, 700};
            i.adc[0] = (uint16_t)(164 + 3350 * left[stage] / 1000);
            i.adc[1] = (uint16_t)(168 + 3221 * right[stage] / 1000);
            unsigned adc = k % 190 < 15 ? 220 : 180;
            for (unsigned t = 0; t < 5; ++t) {
                barcode_analog_sample(&baseline.a, &baseline.n, adc, i.now - 8000 + 2000 * t,
                                      i.wheel);
                barcode_analog_sample(&routed.a, &routed.n, adc, i.now - 8000 + 2000 * t, i.wheel);
            }
            i.side_black = baseline.a.black;
            AnalogDemand a = scan_smooth_step(&baseline.c, &baseline.n, &cal, &i, profile,
                                              &baseline.old, &baseline.a, &baseline.s, .01f);
            AnalogDemand b = scan_smooth_step(&routed.c, &routed.n, &cal, &i, profile, &routed.old,
                                              &routed.a, &routed.s, .01f);
            assert(a.stop == b.stop && a.decision == b.decision &&
                   baseline.c.phase == routed.c.phase);
            assert(a.target[0] == b.target[0] && a.target[1] == b.target[1]);
            if (a.stop) {
                init(&baseline, &i);
                init(&routed, &i);
                motion_session_begin(clock_us);
                ++restarts;
                continue;
            }
            assert(nav_submit_navigation(MOTION_BUDDY3, b.target[0], b.target[1]) == MOTION_OK);
            MotionFeedback f = {.now_us = i.now,
                                .wheel_mm = {i.wheel[0], i.wheel[1]},
                                .speed_mm_s = {45, 45},
                                .line_level = {b.level[0], b.level[1]},
                                .navigation_busy = routed.c.busy,
                                .pending_route = routed.n.pending};
            MotionOutput o = motion_update(&f);
            assert(!o.external && !o.reset_controllers && !o.resume_navigation);
            for (unsigned w = 0; w < 2; ++w) {
                assert(a.target[w] == o.target_mm_s[w]);
                int pa = circuit_wheel_step(&baseline.wheel[w], w, a.target[w], k % 55, .01f, i.now,
                                            false, profile);
                int pb = circuit_wheel_step(&routed.wheel[w], w, o.target_mm_s[w], k % 55, .01f,
                                            i.now, false, profile);
                assert(pa == pb);
            }
            assert(baseline.n.decoded_a == routed.n.decoded_a &&
                   baseline.n.decoded_f == routed.n.decoded_f);
            ++ticks;
        }
    }
    printf(
        "PASS V12.2 routing parity: %u optical/PI ticks across all profiles, both departure directions, white loss/recovery; %u model resets. No external takeover changes normal targets/PWM.\n",
        ticks, restarts);
}
