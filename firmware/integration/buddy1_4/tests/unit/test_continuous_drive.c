#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "src/motion/continuous_drive.h"

static int step(ContinuousDrive *s, SignedWheel *p, int target, float speed, uint64_t now,
                int last) {
    return continuous_drive_step(s, p, 0, (int16_t)target, speed, 0.01f, now, last);
}

static void assert_slew(int before, int after) {
    assert(after >= 0 && after <= CONTINUOUS_PWM_MAX);
    assert(abs(after - before) <= 12);
}

static void test_feedforward_and_entry(void) {
    assert(continuous_feedforward(0, 0) == 0.0f);
    assert(fabsf(continuous_feedforward(0, 45) - 150) < 0.01f);
    assert(fabsf(continuous_feedforward(1, 45) - 180) < 0.01f);
    assert(continuous_feedforward(0, 15) < continuous_feedforward(0, 45));

    ContinuousDrive s = {0};
    SignedWheel p = {0};
    int out = continuous_drive_step(&s, &p, 0, 45, 18, 0.01f, 10000, 140);
    assert(out == 140);                      /* Entry blends from the actuator's actual output. */
    assert(fabsf(s.reference - 18) < 0.01f); /* No artificial reference jump. */
    assert(fabsf(p.pi.ramp - s.reference) < 0.01f);
    assert(fabsf(p.pi.integral - s.integral / 1.2f) < 0.01f);
}

static void test_observed_profile_sequences(void) {
    ContinuousDrive s = {0};
    SignedWheel p = {0};
    int previous = 0;
    uint64_t now = 1000000;
    /* Ordinary profile 1/3 cruise at 45 mm/s: controller output stays
     * continuous through measured-speed changes and remains under 42%. */
    for (int i = 0; i < 350; ++i) {
        float speed = i < 80 ? i * 0.55f : (i < 240 ? 44.0f : 62.0f);
        int out = step(&s, &p, 45, speed, now, previous);
        assert_slew(previous, out);
        assert(out <= 420);
        previous = out;
        now += 10000;
    }
    assert(previous < 220); /* High measured speed can unwind effort. */

    /* Reproduce the low correction segment, then restore cruise demand. */
    int before = previous;
    for (int i = 0; i < 120; ++i) {
        float speed = i < 60 ? 40.0f : 16.0f;
        int target = i < 60 ? 15 : 45;
        float old_reference = s.reference;
        int out = step(&s, &p, target, speed, now, previous);
        assert(fabsf(s.reference - old_reference) <= 2.51f);
        assert_slew(previous, out);
        previous = out;
        now += 10000;
    }
    assert(previous > 0 && previous <= 420);
    assert(fabsf(p.pi.ramp - s.reference) < 0.01f);
    assert(fabsf(p.pi.integral - s.integral / 1.2f) < 0.01f);
    assert(before <= 420);
}

static void test_start_assist_and_explicit_stop(void) {
    ContinuousDrive s = {0};
    SignedWheel p = {0};
    int previous = 0;
    uint64_t now = 5000000;
    bool saw_start = false;
    for (int i = 0; i < 100; ++i) {
        int out = step(&s, &p, 45, 0, now, previous);
        assert_slew(previous, out);
        if (s.start_active)
            saw_start = true;
        previous = out;
        now += 10000;
    }
    assert(saw_start && s.start_used);
    assert(!s.start_active); /* One 150 ms assistance window, then track. */
    float last_integral = s.integral;
    /* A continuing positive command and a stall never retrigger assistance. */
    for (int i = 0; i < 40; ++i) {
        int out = step(&s, &p, 45, 0, now, previous);
        assert_slew(previous, out);
        assert(!s.start_active && s.start_used);
        previous = out;
        now += 10000;
    }
    assert(s.integral >= -100 && s.integral <= 100);
    (void)last_integral;
    assert(continuous_drive_step(&s, &p, 0, 0, 0, 0.01f, now, previous) == 0);
    assert(!s.active && s.duty == 0 && s.integral == 0 && p.pi.ramp == 0);
    int restarted = continuous_drive_step(&s, &p, 0, 45, 0, 0.01f, now + 10000, 0);
    assert(restarted <= 12 && restarted >= 0); /* New positive segment. */
}

static void test_stiction_and_load_models(void) {
    /* Simple first-order plant: 45 mm/s needs about 150 permille on the left;
     * breakaway friction is 75 and the load changes the gain. This checks the
     * controller's shape only; it is not a claim about the physical robot. */
    ContinuousDrive s = {0};
    SignedWheel p = {0};
    float speed = 0.0f;
    int out = 0;
    uint64_t now = 9000000;
    for (int i = 0; i < 700; ++i) {
        out = step(&s, &p, 45, speed, now, out);
        float equilibrium = out <= 75 ? 0.0f : (out - 75.0f) / 1.67f;
        speed += (equilibrium - speed) * 0.08f;
        now += 10000;
    }
    assert(speed > 25 && speed < 60);
    assert(out <= 420);

    /* Apply a load at fixed target, then lighten it; PI should recover without
     * needing a profile/mode transition or a new startup pulse. */
    for (int i = 0; i < 300; ++i) {
        float equilibrium = out <= 110 ? 0.0f : (out - 110.0f) / 1.2f;
        speed += (equilibrium - speed) * 0.08f;
        out = step(&s, &p, 45, speed, now, out);
        now += 10000;
    }
    assert(s.integral >= -100 && s.integral <= 100);
}

static void test_invalid_input_fails_safe(void) {
    ContinuousDrive s = {.active = true, .duty = 200};
    SignedWheel p = {.sign = 1};
    assert(continuous_drive_step(&s, &p, 0, 45, NAN, 0.01f, 1, 200) == 0);
    assert(!s.active && p.sign == 0);
}

int main(void) {
    test_feedforward_and_entry();
    test_observed_profile_sequences();
    test_start_assist_and_explicit_stop();
    test_stiction_and_load_models();
    test_invalid_input_fails_safe();
    puts("continuous drive tests passed (software input/plant models only)");
    return 0;
}
