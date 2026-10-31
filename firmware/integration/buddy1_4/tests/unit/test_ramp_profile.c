#include <assert.h>
#include <stdio.h>
#include "src/line_following/smooth_follow.h"
static void load_boost_and_deadline(void) {
    for (unsigned w = 0; w < 2; ++w) {
        RampWheel s = {0};
        SignedWheel p = {.sign = 1, .pi = {.ramp = 180}};
        int normal = ramp_wheel_step(&s, &p, w, 180, 180, .01f, 1000);
        assert(normal > 0 && normal < RAMP2_SUSTAIN && !s.boost_used);
        int peak = 0;
        uint64_t now = 11000;
        for (unsigned k = 0; k < 25; ++k, now += 10000) {
            int pwm = ramp_wheel_step(&s, &p, w, 180, 30, .01f, now);
            if (pwm > peak)
                peak = pwm;
            if (k < 4)
                assert(pwm <= RAMP2_SUSTAIN);
            assert(pwm <= RAMP2_BOOST);
        }
        assert(peak == 650 && s.boost_used && s.boost_until);
        uint64_t deadline = s.boost_until;
        assert(barcode_output_cap(650, true, deadline, deadline - 1) == 650);
        assert(barcode_output_cap(650, true, deadline, deadline) == 500);
        for (unsigned k = 0; k < 5; ++k) {
            now = deadline + k * 1000000;
            int pwm = ramp_wheel_step(&s, &p, w, 180, 0, .01f, now);
            assert(pwm <= 500 && s.boost_until == deadline);
        }
        /* Returning to commanded speed for200ms re-arms a later obstacle. */
        for (unsigned k = 0; k <= 21; ++k) {
            now += 10000;
            (void)ramp_wheel_step(&s, &p, w, 180, 180, .01f, now);
        }
        assert(!s.boost_used && !s.boost_until);
        for (unsigned k = 0; k < 6; ++k) {
            now += 10000;
            (void)ramp_wheel_step(&s, &p, w, 180, 0, .01f, now);
        }
        assert(s.boost_until > now);
        /* A reduced target ends extra effort; deliberate zero stays OFF. */
        assert(!ramp_wheel_step(&s, &p, w, 30, 180, .01f, now + 10000));
        assert(!s.extra && !p.pi.integral && p.pi.ramp == 30);
        assert(!ramp_wheel_step(&s, &p, w, 0, 0, .01f, now + 20000));
        assert(!s.active && !s.boost_until && !p.sign && !p.pi.integral);
    }
}
static void independent_targets_and_guards(void) {
    RampWheel left = {0}, right = {0};
    SignedWheel l = {.sign = 1, .pi = {.ramp = 180}}, r = l;
    int a = 0, b = 0;
    for (unsigned k = 0; k < 30; ++k) {
        a = ramp_wheel_step(&left, &l, 0, 180, 180, .01f, 1000 + k * 10000);
        b = ramp_wheel_step(&right, &r, 1, 180, 30, .01f, 1000 + k * 10000);
    }
    assert(b == 650 && a < 500 && right.boost_used && !left.boost_used);
    RampWheel slow = {0};
    SignedWheel p = {.sign = 1, .pi = {.ramp = 25}};
    for (unsigned k = 0; k < 40; ++k)
        (void)ramp_wheel_step(&slow, &p, 0, 25, 25, .01f, 1000 + k * 10000);
    assert(!slow.boost_used); /* slower than outer wheel is intentional */
    Circuit c = {.phase = C_FOLLOW};
    AnalogDemand d = {.level = {700, 700}, .target = {45, 45}};
    assert(ramp_profile_active(2, &c, &d));
    assert(!ramp_profile_active(1, &c, &d) && !ramp_profile_active(3, &c, &d));
    c.follower.core.white = true;
    assert(!ramp_profile_active(2, &c, &d));
    c.follower.core.white = false;
    d.level[0] = d.level[1] = 0;
    assert(!ramp_profile_active(2, &c, &d));
    d.level[0] = 700;
    d.stop = true;
    assert(!ramp_profile_active(2, &c, &d));
    d.stop = false;
    c.phase = J_TURN;
    c.busy = true;
    assert(!ramp_profile_active(2, &c, &d));
    ramp_wheel_leave(&right, &r, 30);
    assert(!right.active && !right.boost_until && !r.pi.integral && r.pi.ramp <= 30);
}
static void envelopes_and_baseline(void) {
    for (unsigned w = 0; w < 2; ++w)
        for (int value = -700; value <= 800; value += 25) {
            for (unsigned phase = 0; phase < 2; ++phase) {
                int cap = barcode_output_cap(value, false, phase ? 2000 : 0, 1000);
                assert(cap == boosted_junction_peak_cap(value, phase ? 2000 : 0, 1000));
                DualLevels a = barcode_output_levels(w, cap, false, true, 1),
                           b = circuit_levels(w, cap, true, 1);
                assert(a.a == b.a && a.b == b.b);
            }
            int cap = barcode_output_cap(value, true, 2000, 1000);
            DualLevels v = barcode_output_levels(w, cap, true, false, 2);
            assert(cap >= 0 && cap <= 650 && (w ? v.b : v.a) == cap && !(w ? v.a : v.b));
        }
    SignedWheel p = {0}, q = {0};
    RampWheel disabled = {0};
    for (unsigned k = 0; k < 300; ++k) {
        int wanted = k % 80 < 40 ? 45 : 25;
        float measured = k % 55;
        ramp_wheel_leave(&disabled, &p, wanted);
        int a = circuit_wheel_step(&p, 0, wanted, measured, .01f, 1000 + k * 10000, false, 1);
        int b = circuit_wheel_step(&q, 0, wanted, measured, .01f, 1000 + k * 10000, false, 1);
        assert(a == b && !memcmp(&p, &q, sizeof(p)));
    }
    assert(barcode_profile_cruise(1) == 45 && barcode_profile_cruise(2) == 180 &&
           barcode_profile_cruise(3) == 135);
}
int main(void) {
    load_boost_and_deadline();
    independent_targets_and_guards();
    envelopes_and_baseline();
    puts(
        "PASS: independent underspeed effort;650 boost after40ms;1500ms ISR expiry;stall cannot renew;recovery re-arm;intentional slow/off wheel;overspeed coast;loss/junction bypass;profile1 parity;physical PWM mapping.");
}
