#define APP_RTOS_BARCODE 1
#define BUDDY_RUNTIME_MAPPING 1
#define BUDDY_COORDINATED_BARCODE 1
#define BUDDY_WIFI_LOGGING 1
#include <assert.h>
#include <stdio.h>
#include "app_program/barcode_navigation/barcode_model.h"
#include "src/motion/motion_port.h"
#include "tests/barcode/bp_captured_fixtures.h"
static RbDecoder d;
static LinePlanner p;
static uint64_t now = 1000000;
static unsigned steps;
static uint64_t clock_now(void *u) {
    (void)u;
    return now;
}
static uint32_t mapping(void) {
    return wp_map_replace(0,
                          ('Y' - 'A') | ('Z' - 'A') << 5 | ('A' - 'A') << 10 | ('F' - 'A') << 15);
}
static void begin(uint32_t map) {
    MotionPort port = {.now_us = clock_now};
    motion_init(&port);
    motion_session_begin(now);
    CircuitInput i = {.now = now, .adc = {240, 260}};
    rl_begin(&p, &i, 175);
    p.nav.mapping = map;
    rb_begin(&d, &p.nav, 175, now, i.wheel);
    steps = 0;
}
static void step(double mm, uint16_t adc) {
    now += 2000;
    double w[2] = {mm, mm};
    if (steps++ % 5 == 0) {
        assert(motion_submit_navigation(MOTION_BUDDY3, 45, 45) == MOTION_OK);
        MotionFeedback f = {
            .now_us = now, .wheel_mm = {mm, mm}, .speed_mm_s = {45, 45}, .line_level = {700, 700}};
        (void)motion_update(&f);
    }
    rb_sample(&d, &p.nav, adc, now, w);
}
static void capture(unsigned cap) {
    const BpCapture *c = &bp_captures[cap];
    for (unsigned n = 0; n < c->n; ++n)
        step(c->path100[n] / 100.0, c->adc[n]);
    double end = c->path100[c->n - 1] / 100.0;
    for (unsigned n = 1; n <= 300; ++n)
        step(end + n * .09, 175);
}
static uint16_t encoding(char code) {
    for (uint16_t bits = 0; bits < 512; ++bits)
        if (barcode_character(bits) == code)
            return bits;
    assert(0);
    return 0;
}
static void synthetic(char code, bool reverse) {
    uint16_t patterns[3] = {0x094, encoding(code), 0x094};
    unsigned widths[29], k = 0;
    for (unsigned c = 0; c < 3; ++c) {
        for (unsigned j = 0; j < 9; ++j)
            widths[k++] = (patterns[c] & (1 << (8 - j))) ? 9 : 3;
        if (c < 2)
            widths[k++] = 3;
    }
    unsigned element = 0;
    double start = 30, end = start + widths[reverse ? 28 : 0];
    for (double mm = 0; mm < 230; mm += .09) {
        while (element < 29 && mm >= end) {
            ++element;
            if (element < 29)
                end += widths[reverse ? 28 - element : element];
        }
        bool black = mm >= start && element < 29 && (element % 2 == 0);
        step(mm, black ? 3000 : 175);
    }
}
int main(void) {
    for (unsigned cap = 2; cap <= 3; ++cap) {
        begin(mapping());
        capture(cap);
        assert(p.nav.decoded_total == 1 && p.nav.last_code == bp_captures[cap].expect);
        assert(d.peak_applied == 1 && d.raw_frozen && p.nav.pending == p.nav.last_code);
        assert(barcode_nav_action(&p.nav, p.nav.pending) == (cap == 2 ? 'U' : 'S'));
        assert(motion_get_status().owner == MOTION_BUDDY3);
    }
    for (char letter = 'A'; letter <= 'Z'; ++letter)
        for (unsigned reverse = 0; reverse < 2; ++reverse) {
            begin(mapping());
            synthetic(letter, reverse);
            assert(p.nav.decoded_total == 1 && p.nav.last_code == letter);
            assert(d.raw_frozen);
        }
    begin(0);
    synthetic('A', false);
    assert(p.nav.last_code == 'A' && p.nav.pending == 0 && p.nav.unsupported == 1);
    /* Full migrated sample -> planner path: stop bar is not a junction;
     * a later separate cross dispatches the frozen mapped action. */
    const char letters[] = "YAFZ", actions[] = "SLRU";
    const LineCalibration cal = {{122, 138}, {246, 267}, 7};
    for (unsigned which = 0; which < 4; ++which) {
        begin(mapping());
        synthetic(letters[which], false);
        assert(!p.nav.busy && !p.nav.cross_ready && p.nav.pending == letters[which]);
        for (double mm = 230; mm < 310; mm += .09) {
            step(mm, mm >= 260 && mm < 278 ? 3000 : 175);
            if (steps % 5 == 0) {
                CircuitInput in = {.now = now,
                                   .path = mm,
                                   .wheel = {mm, mm},
                                   .speed = {45, 45},
                                   .adc = {246, 267},
                                   .side_black = d.analog.black};
                AnalogDemand output = rb_follow(&p, &d, &cal, &in, .01f);
                assert(!output.stop);
            }
            if (p.nav.busy)
                break;
        }
        if (which == 0)
            assert(!p.nav.busy && p.nav.pending == 0 && p.nav.straight_crosses == 1);
        else
            assert(p.nav.busy && p.nav.executing_action == actions[which]);
        if (which != 0)
            assert(fabs(p.circuit.turn.centre_mm - (p.nav.cross_width * .5 + 110 - 55)) < .01);
    }
    /* Runtime map revisions affect future labels, not a frozen pending turn. */
    begin(mapping());
    synthetic('A', false);
    assert(p.nav.pending_action == 'L' && p.nav.pending_revision == 1);
    uint32_t changed_mapping = wp_map_replace(mapping(), ('Y' - 'A') | ('Z' - 'A') << 5 |
                                                             ('F' - 'A') << 10 | ('A' - 'A') << 15);
    p.nav.mapping = changed_mapping;
    assert(barcode_nav_action(&p.nav, 'A') == 'L');
    uint32_t old_events = p.nav.event_sequence;
    CircuitInput i = {.now = now, .wheel = {230, 230}, .adc = {240, 260}};
    rb_resume(&p, &d, &i, 175);
    assert(p.nav.pending == 'A' && p.nav.pending_action == 'L' && p.nav.pending_revision == 1);
    assert(p.nav.event_sequence == old_events && p.nav.decoded_total == 1);
    assert(!d.peak.nbars && !p.nav.cross_candidate && !p.nav.cross_ready);
    assert(d.junction.valid && !d.junction.armed && !p.black);
    begin(changed_mapping);
    synthetic('A', false);
    assert(p.nav.pending_action == 'R' && p.nav.pending_revision == 2);
    begin(mapping());
    const BpCapture *c = &bp_captures[2];
    unsigned n = 0;
    for (; n < c->n && c->path100[n] < 10000; ++n)
        step(c->path100[n] / 100.0, c->adc[n]);
    assert(d.peak.nbars > 0);
    double wheel[2] = {100, 100};
    now += 30000000;
    rb_partial_reset(&d, &p.nav, 175, now, wheel);
    for (; n < c->n; ++n)
        step(c->path100[n] / 100.0, c->adc[n]);
    assert(p.nav.decoded_total == 0 && p.nav.pending == 0);
    /* Ring/freeze fields encode actual ADC/time/distance, never guessed edges. */
    begin(mapping());
    for (unsigned n = 0; n < RB_RAW_ROWS + 5; ++n)
        step(n * .09, 175);
    assert(d.raw_count == 0 && d.raw_overwritten == 0);
    assert(d.preroll_count == RB_RAW_PREROLL_ROWS && !d.raw_capture_active);
    const uint8_t *raw = d.preroll[d.preroll_head].bytes;
    assert(rb_get32(raw) > 0 && raw[8] == 175 && raw[9] == 0);
    puts(
        "PASS RTOS barcode: real failed Z/Y captures, synthetic A-Z both directions, coordinator approval, mapping freeze, unmapped letters, partial Resume and raw ring");
    return 0;
}
