#define APP_RTOS_BARCODE 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#define RTOS_JUNCTION_MIN_MM 7
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "app_program/barcode_navigation/barcode_model.h"
#include "src/motion/motion_port.h"
static RbDecoder d;
static BarcodeNav n;
static uint64_t now;
static uint64_t clock_now(void *p) {
    (void)p;
    return now;
}
static void sample(unsigned ms, double mm, unsigned adc) {
    now = 1000000 + (uint64_t)ms * 1000;
    assert(motion_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
    MotionFeedback f = {
        .now_us = now, .wheel_mm = {mm, mm}, .speed_mm_s = {45, 45}, .line_level = {1000, 1000}};
    (void)motion_update(&f);
    double w[2] = {mm, mm};
    n.allow_cross = true; /* Isolate sensor qualification from optical straight gate. */
    rb_sample(&d, &n, (uint16_t)adc, now, w);
}
static unsigned count(unsigned type) {
    unsigned result = 0;
    for (unsigned k = 0; k < n.event_count; ++k)
        result += n.events[k].type == type;
    return result;
}
static void sensor(RbJunctionSensor *q, BarcodeNav *nav, double mm, unsigned adc) {
    double wheel[2] = {mm, mm};
    now += 2000;
    rb_junction_sample(q, nav, adc, now, wheel);
}
static void width_case(double width, unsigned result, bool candidate) {
    BarcodeNav nav = {.pending = 'Y', .pending_action = 'U', .allow_cross = true};
    RbJunctionSensor q = {0};
    rb_junction_learn(&q, &nav, 250, 2250, 250, now, 0);
    for (double mm = 0; mm < 8; mm += .1)
        sensor(&q, &nav, mm, 250);
    for (double mm = 8; mm < 8 + width; mm += .1)
        sensor(&q, &nav, mm, 2250);
    for (double mm = 8 + width; mm < 11 + width; mm += .1)
        sensor(&q, &nav, mm, 250);
    assert(q.last_result == result);
    assert(abs((int)q.last_width_centi_mm - (int)lround(width * 100)) <= 1);
    assert(q.last_candidate == candidate && q.decision_serial == 1);
    assert(nav.cross_candidate == candidate && !nav.cross_ready);
    if (candidate) {
        sensor(&q, &nav, 11.9 + width, 250); /* 3.9mm trailing white is short. */
        assert(nav.cross_candidate && !nav.cross_ready);
        sensor(&q, &nav, 12.1 + width, 250); /* 4mm white confirms. */
        assert(nav.cross_ready && q.last_result == RB_JUNCTION_RESULT_WHITE_CONFIRMED);
        assert(q.last_width_centi_mm == (int16_t)lround(width * 100));
        assert(q.decision_serial == 2);
    }
}
int main(void) {
    _Static_assert(sizeof(RbJunctionSensor) <= 64, "Bounded ADC-owner junction state");
    _Static_assert(B_SIDE_AHEAD_MM == 110, "Step4 measured forward offset");
    _Static_assert(B_CROSS_MIN_MM == 11, "SDK junction minimum remains unchanged");
    _Static_assert(RTOS_JUNCTION_MIN_MM == 7, "RTOS junction adapter uses its 7mm minimum");
    now = 1000000;
    MotionPort port = {.now_us = clock_now};
    motion_init(&port);
    motion_session_begin(now);
    CircuitInput input = {.now = now, .wheel = {1416.87, 1416.87}};
    barcode_nav_begin(&n, &input);
    n.mapping =
        wp_map_replace(0, ('N' - 'A') | ('Y' - 'A') << 5 | ('Q' - 'A') << 10 | ('Z' - 'A') << 15);
    rb_begin(&d, &n, 250, now, input.wheel);
    FILE *file = fopen("testing/rtos_migration/fixtures/step4_y_junction.csv", "r");
    assert(file);
    char row[160];
    unsigned ms, adc, rows = 0;
    int pos;
    bool learned = false, rearmed = false;
    while (fgets(row, sizeof(row), file)) {
        if (sscanf(row, "%u,%d,%u", &ms, &pos, &adc) != 3)
            continue;
        sample(ms, pos / 100.0, adc);
        ++rows;
        if (ms <= 5649)
            assert(!count(B_CROSS));
        learned |= d.junction.valid;
        rearmed |= d.junction.armed;
    }
    fclose(file);
    assert(rows > 1700 && learned && rearmed);
    assert(n.decoded_total == 1 && n.last_code == 'Y' && d.peak_applied == 1);
    assert(n.pending == 'Y' && n.pending_action == 'U' && n.pending_revision == 1);
    assert(d.analog.selected == AO_DEFAULT_BANK); /* Decoder212 remains unchanged. */
    assert(d.junction.threshold > 300 && d.junction.threshold < 2175);
    assert(count(B_CROSS_CANDIDATE) == 1 && count(B_CROSS) == 1 && n.cross_ready);
    assert(n.cross_width >= 11 && n.cross_width <= 30 && !d.analog.black);
    printf(
        "PASS captured Y: decoder=%u junction=%u hysteresis=%u width=%.2fmm confirmed=1 offset=%umm\n",
        barcode_analog_threshold(d.analog.selected), d.junction.threshold, d.junction.hysteresis,
        n.cross_width, B_SIDE_AHEAD_MM);

    /* Scale/offset-independent contrast, same physical width and white gate. */
    for (unsigned base = 100; base <= 900; base += 400) {
        BarcodeNav nav = {.pending = 'Y', .pending_action = 'U', .allow_cross = true};
        RbJunctionSensor q = {0};
        rb_junction_learn(&q, &nav, base, base + 1000, base + 1000, now, 0);
        /* Begin over black: no synthetic leading edge, even an 18mm remainder. */
        for (double mm = 0; mm < 18; mm += .1)
            sensor(&q, &nav, mm, base + 1000);
        for (double mm = 18; mm < 21; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(!q.armed && !nav.cross_ready && !nav.cross_candidate);
        for (double mm = 21; mm < 30; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(q.armed);
        /* A 6mm barcode-wide bar remains too narrow to be a junction. */
        for (double mm = 30; mm < 36; mm += .1)
            sensor(&q, &nav, mm, base + 1000);
        for (double mm = 36; mm < 46; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(!nav.cross_candidate && !nav.cross_ready);
        assert(q.last_result == RB_JUNCTION_RESULT_TOO_NARROW &&
               q.last_width_centi_mm == 600 && !q.last_candidate);
        for (double mm = 46; mm < 64; mm += .1)
            sensor(&q, &nav, mm, base + 1000);
        for (double mm = 64; mm < 67; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(nav.cross_candidate && !nav.cross_ready);
        assert(q.last_result == RB_JUNCTION_RESULT_CANDIDATE && q.last_candidate);
        for (double mm = 67; mm < 70; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(nav.cross_ready && nav.cross_gate);
        rb_junction_reset(&q, &nav, base + 1000, now, 70);
        assert(q.valid && !q.armed && !nav.cross_ready && !nav.cross_candidate);
    }
    /* Short white gap cancels a candidate; no bypass of the straight gate. */
    BarcodeNav nav = {.pending = 'Y', .allow_cross = false};
    RbJunctionSensor q = {0};
    rb_junction_learn(&q, &nav, 250, 2250, 250, now, 0);
    for (double mm = 0; mm < 10; mm += .1)
        sensor(&q, &nav, mm, 250);
    for (double mm = 10; mm < 28; mm += .1)
        sensor(&q, &nav, mm, 2250);
    for (double mm = 28; mm < 30; mm += .1)
        sensor(&q, &nav, mm, 250);
    assert(nav.cross_candidate && !nav.cross_gate);
    for (double mm = 30; mm < 33; mm += .1)
        sensor(&q, &nav, mm, 2250);
    assert(!nav.cross_candidate && !nav.cross_ready);
    assert(q.last_result == RB_JUNCTION_RESULT_WHITE_CANCELLED && q.last_candidate &&
           q.last_width_centi_mm == 1800);
    /* The 100Hz planner must not bypass the ADC owner's trailing-white
     * qualification while a new black edge is waiting for debounce. */
    nav.external_reader = nav.external_cross = true;
    nav.cross_candidate = true;
    nav.cross_trailing_mm = 28;
    Circuit circuit;
    CircuitInput in = {.now = now,
                       .path = 32.1,
                       .wheel = {32.1, 32.1},
                       .speed = {45, 45},
                       .adc = {1100, 1100},
                       .side_black = false};
    LineCalibration cal = {{100, 100}, {1100, 1100}, 7};
    circuit_begin(&circuit, &in);
    (void)barcode_circuit_step(&circuit, &nav, &cal, &in, 1);
    assert(nav.cross_candidate && !nav.cross_ready && !nav.busy);
    rb_junction_learn(&q, &nav, 250, 250, 250, now, 33);
    assert(!q.valid && !q.armed);

    /* Busy and absent-pending phases cannot accumulate the pre-cross white
     * gate; four fresh white millimetres are required after eligibility. */
    BarcodeNav waiting = {.pending = 'Y'};
    RbJunctionSensor waiting_q = {0};
    rb_junction_learn(&waiting_q, &waiting, 250, 2250, 250, now, 0);
    waiting.busy = true;
    for (double mm = 0; mm < 8; mm += .1)
        sensor(&waiting_q, &waiting, mm, 250);
    assert(!waiting_q.armed);
    waiting.busy = false;
    for (unsigned step = 0; step <= 41; ++step)
        sensor(&waiting_q, &waiting, 8 + step*.1, 250);
    assert(waiting_q.armed);
    waiting.pending = 0;
    sensor(&waiting_q, &waiting, 12.3, 250);
    assert(!waiting_q.armed && !waiting_q.white_tracking);

    /* 7mm is accepted, 6.9mm and widths above the shared 30mm maximum are
     * frozen as exact signed width decisions, and white confirmation retains
     * the candidate measurement. */
    width_case(6.9, RB_JUNCTION_RESULT_TOO_NARROW, false);
    width_case(7.0, RB_JUNCTION_RESULT_CANDIDATE, true);
    width_case(30.0, RB_JUNCTION_RESULT_CANDIDATE, true);
    width_case(30.1, RB_JUNCTION_RESULT_TOO_WIDE, false);

    BarcodeNav reverse_nav = {.pending = 'Y'};
    RbJunctionSensor reverse_q = {0};
    rb_junction_learn(&reverse_q, &reverse_nav, 250, 2250, 250, now, 0);
    for (double mm = 0; mm <= 15; mm += 1)
        sensor(&reverse_q, &reverse_nav, mm, 250);
    for (double mm = 14; mm >= 7; mm -= 1)
        sensor(&reverse_q, &reverse_nav, mm, 2250);
    for (double mm = 7; mm >= 5; mm -= 1)
        sensor(&reverse_q, &reverse_nav, mm, 250);
    assert(reverse_q.last_result == RB_JUNCTION_RESULT_TOO_NARROW);
    assert(reverse_q.last_width_centi_mm == -700);
    assert(!reverse_nav.cross_candidate && !reverse_q.last_candidate);

    bool saturated = false;
    assert(rb_junction_width_centi(-12.34, &saturated) == -1234 && !saturated);
    assert(rb_junction_width_centi(400.0, &saturated) == INT16_MAX && saturated);
    assert(rb_junction_width_centi(-400.0, &saturated) == -INT16_MAX && saturated);
    RbJunctionSensor saturated_q = {0};
    rb_junction_record_width(&saturated_q, 400.0, RB_JUNCTION_RESULT_TOO_WIDE, false);
    assert(saturated_q.last_width_centi_mm == INT16_MAX && saturated_q.last_width_saturated);

    RbJunctionSensor packed = {.threshold = 0xabc,
                               .hysteresis = 0x56,
                               .decision_serial = 0x78563412,
                               .decision_threshold = 0xabc,
                               .decision_hysteresis = 0x56,
                               .last_width_centi_mm = -INT16_MAX,
                               .valid = 1,
                               .armed = 1,
                               .last_result = RB_JUNCTION_RESULT_WHITE_CANCELLED,
                               .last_candidate = 1,
                               .last_width_saturated = 1,
                               .decision_valid = 1,
                               .decision_armed = 1};
    uint8_t frame[RB_JUNCTION_FRAME];
    rb_junction_pack(&packed, true, 0x11223344, 0x5566, 0x77, frame);
    assert(frame[0] == 0xf2 && frame[1] == 17 && frame[2] == 0x77 && frame[3] == 0xc7);
    assert(rb_get32(frame + 4) == 0x11223344 && rb_get32(frame + 8) == 0x78563412);
    assert(frame[12] == 0x66 && frame[13] == 0x55 && frame[14] == 0xbc);
    assert(frame[15] == 0x6a && frame[16] == 0x05 && (frame[16] & 0xf0U) == 0);
    assert((int16_t)((uint16_t)frame[17] | (uint16_t)frame[18] << 8) == -INT16_MAX);
    assert(frame[19] == (RB_JUNCTION_RESULT_WHITE_CANCELLED | (1U << 3) | (1U << 4) |
                          (1U << 5) | (1U << 6) | (1U << 7)));

    BarcodeNav frozen_nav = {.pending = 'Y'};
    RbJunctionSensor frozen = {0};
    rb_junction_learn(&frozen, &frozen_nav, 200, 2000, 200, now, 0);
    frozen.armed = true;
    rb_junction_record_width(&frozen, 10.4, RB_JUNCTION_RESULT_CANDIDATE, true);
    assert(frozen.decision_threshold == 1100 && frozen.decision_hysteresis == 90 &&
           frozen.decision_valid && frozen.decision_armed);
    rb_junction_learn(&frozen, &frozen_nav, 100, 2000, 100, now, 0);
    assert(frozen.threshold == 1050 && frozen.hysteresis == 95 && !frozen.armed);
    rb_junction_pack(&frozen, true, 0, 0, 0, frame);
    assert(frame[14] == (1100 & 0xff) && (frame[15] & 0x0fU) == (1100 >> 8));
    assert((frame[15] >> 4) == (90 & 0x0f) && frame[16] == (90 >> 4));
    assert((frame[19] & 0x18U) == 0x18U); /* Recorded valid + armed survive re-learn. */

    RbJunctionSensor fixture_q = {.threshold = 1100,
                                  .hysteresis = 90,
                                  .decision_serial = 7,
                                  .decision_threshold = 1100,
                                  .decision_hysteresis = 90,
                                  .last_width_centi_mm = 1040,
                                  .valid = 1,
                                  .armed = 1,
                                  .last_result = RB_JUNCTION_RESULT_WHITE_CONFIRMED,
                                  .last_candidate = 1,
                                  .decision_valid = 1,
                                  .decision_armed = 1};
    rb_junction_pack(&fixture_q, true, 0x11223344, 0x5566, 0x78, frame);
    assert(frame[2] == 0x78 && rb_get32(frame + 8) == 7 && frame[14] == (1100 & 0xff));
    assert((frame[15] & 0x0fU) == (1100 >> 8) && (frame[15] >> 4) == (90 & 0x0f));
    assert(frame[16] == 5 && (frame[16] & 0xf0U) == 0 && (frame[19] & 0x7fU) ==
                                    (RB_JUNCTION_RESULT_WHITE_CONFIRMED | (1U << 3) | (1U << 4) |
                                     (1U << 5) | (1U << 6)));
    assert((frame[19] & 0x80U) == 0);
    assert((int16_t)((uint16_t)frame[17] | (uint16_t)frame[18] << 8) == 1040);
    FILE *fixture = fopen("testing/rtos_migration/build-host/junction_decision.bin", "wb");
    assert(fixture && fwrite(frame, 1, sizeof(frame), fixture) == sizeof(frame));
    assert(fclose(fixture) == 0);
    puts(
        "PASS independent junction: 7..30mm boundaries, exact signed/saturated width diagnostics, frozen white outcomes, relative contrast, stop/narrow bar rejection, short-gap cancellation, reset, gate preservation, C7 LE codec");
    return 0;
}
