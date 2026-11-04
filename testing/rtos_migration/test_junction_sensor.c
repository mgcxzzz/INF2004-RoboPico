#define APP_RTOS_BARCODE 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#include <assert.h>
#include <stdio.h>
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
int main(void) {
    _Static_assert(sizeof(RbJunctionSensor) <= 64, "Bounded ADC-owner junction state");
    _Static_assert(B_SIDE_AHEAD_MM == 110, "Step4 measured forward offset");
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
        RbJunctionSensor q;
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
        /* A 9mm barcode-wide bar remains too narrow to be a junction. */
        for (double mm = 30; mm < 39; mm += .1)
            sensor(&q, &nav, mm, base + 1000);
        for (double mm = 39; mm < 46; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(!nav.cross_candidate && !nav.cross_ready);
        for (double mm = 46; mm < 64; mm += .1)
            sensor(&q, &nav, mm, base + 1000);
        for (double mm = 64; mm < 67; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(nav.cross_candidate && !nav.cross_ready);
        for (double mm = 67; mm < 70; mm += .1)
            sensor(&q, &nav, mm, base);
        assert(nav.cross_ready && nav.cross_gate);
        rb_junction_reset(&q, &nav, base + 1000, now, 70);
        assert(q.valid && !q.armed && !nav.cross_ready && !nav.cross_candidate);
    }
    /* Short white gap cancels a candidate; no bypass of the straight gate. */
    BarcodeNav nav = {.pending = 'Y', .allow_cross = false};
    RbJunctionSensor q;
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
    puts(
        "PASS independent junction: relative contrast, white re-arm, stop/narrow bar rejection, short-gap cancellation, reset, gate preservation");
    return 0;
}
