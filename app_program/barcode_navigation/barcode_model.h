#ifndef RTOS_BARCODE_MODEL_H
#define RTOS_BARCODE_MODEL_H
/* Hardware-free migration adapter. Both Code39 decoders retain reference
 * thresholds; junction sensing uses independent verified-frame contrast.
 * Call on the ADC owner's 2ms GP27 samples, not a network task. */
#include "../line_navigation/line_model.h"
#include "src/barcode/barcode_peak.h"
#include "junction_sensor.h"

enum { RB_RAW_ROWS = 2048, RB_RAW_BYTES = 10, RB_PEAK_RUN_START = 10 };
typedef struct {
    uint8_t bytes[RB_RAW_BYTES];
} RbRaw;
_Static_assert(sizeof(RbRaw) == 10, "Raw GP27 record must stay explicitly packed");
typedef struct {
    BarcodeAnalog analog;
    BarcodePeak peak;
    BpResult last_peak;
    RbJunctionSensor junction;
    RbRaw raw[RB_RAW_ROWS];
    uint32_t event_seen, peak_serial, peak_decodes, peak_applied, peak_conflicts;
    uint32_t samples, raw_overwritten, max_sample_us;
    unsigned raw_count : 12, raw_head : 11;
    uint16_t groups;
    uint8_t peak_flags;
    bool raw_frozen;
} RbDecoder;
static inline char rb_log_route(const BarcodeNav *n, char code) {
    char a = barcode_nav_action(n, code);
    return a == 'L' ? 'A' : a == 'R' ? 'F' : a == 'U' ? 'Z' : 0;
}
static inline void rb_put32(uint8_t *p, uint32_t v) {
    for (unsigned n = 0; n < 4; ++n)
        p[n] = (uint8_t)(v >> (8 * n));
}
static inline uint32_t rb_get32(const uint8_t *p) {
    return p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}
static inline void rb_raw(RbDecoder *d, const BarcodeNav *n, uint16_t adc, uint64_t now,
                          double mm) {
    ++d->samples;
    if (d->raw_frozen)
        return;
    RbRaw *r = &d->raw[d->raw_head];
    rb_put32(r->bytes, (uint32_t)((now - n->start_us) / 1000));
    double v = mm * 100;
    int32_t pos = v > INT32_MAX ? INT32_MAX : v < INT32_MIN ? INT32_MIN : (int32_t)lround(v);
    rb_put32(r->bytes + 4, (uint32_t)pos);
    r->bytes[8] = (uint8_t)adc;
    r->bytes[9] = (uint8_t)((adc >> 8) & 15U);
    d->raw_head = (d->raw_head + 1) % RB_RAW_ROWS;
    if (d->raw_count < RB_RAW_ROWS)
        ++d->raw_count;
    else
        ++d->raw_overwritten;
}
static inline void rb_partial_reset(RbDecoder *d, BarcodeNav *n, uint16_t adc, uint64_t now,
                                    const double wheel[2]) {
    barcode_analog_begin(&d->analog, n, adc, now, wheel);
    n->external_cross = true;
    rb_junction_reset(&d->junction, n, adc, now, (wheel[0] + wheel[1]) * .5);
    if (d->junction.valid)
        d->analog.black = d->junction.black;
    bp_reset(&d->peak);
    n->cross_candidate = n->cross_ready = n->allow_cross = false;
    n->leading_gate = n->cross_gate = false;
    n->leading_us = 0;
    barcode_reader_reset(&n->reader, d->analog.black, now, (wheel[0] + wheel[1]) * .5);
}
static inline void rb_begin(RbDecoder *d, BarcodeNav *n, uint16_t adc, uint64_t now,
                            const double wheel[2]) {
    memset(d, 0, sizeof(*d));
    rb_partial_reset(d, n, adc, now, wheel);
    d->last_peak.code = '?';
    d->last_peak.reason = RB_PEAK_RUN_START;
    d->peak_flags = 1;
    ++d->peak_serial;
}
static inline void rb_sample(RbDecoder *d, BarcodeNav *n, uint16_t adc, uint64_t now,
                             const double wheel[2]) {
    double mm = (wheel[0] + wheel[1]) * .5;
    rb_raw(d, n, adc, now, mm);
    uint32_t before = n->decoded_total;
    barcode_analog_sample(&d->analog, n, adc, now, wheel);
    if (n->decoded_total != before) {
        d->raw_frozen = true;
        rb_junction_learn(&d->junction, n, d->analog.minimum, d->analog.maximum, adc, now, mm);
    }
    rb_junction_sample(&d->junction, n, adc, now, wheel);
    if (d->junction.valid)
        d->analog.black = d->junction.black;
    if (n->busy) {
        bp_reset(&d->peak);
        return;
    }
    BpResult r;
    if (!bp_sample(&d->peak, adc, mm, &r) || r.nbars < BP_REPORT_MIN_BARS)
        return;
    if (d->groups < 65535)
        ++d->groups;
    d->last_peak = r;
    ++d->peak_serial;
    d->peak_flags = 1u | (r.decoded ? 2u : 0u) | (r.reverse ? 16u : 0u) | (r.overflow ? 64u : 0u);
    if (!r.decoded)
        return;
    ++d->peak_decodes;
    bool overlap = d->analog.accepted && fabs(r.start_mm - d->analog.accepted_start) <= 12 &&
                   fabs(r.end_mm - d->analog.accepted_end) <= 12;
    if (overlap) {
        d->peak_flags |= 8u;
        if (n->last_code != r.code) {
            d->peak_flags |= 32u;
            ++d->peak_conflicts;
            if (n->pending == n->last_code)
                n->pending = 0;
            n->cross_candidate = n->cross_ready = false;
            d->analog.conflict = true;
            d->analog.conflict_end = mm;
            barcode_event(n, B_ANALOG_CONFLICT, now, mm, 0, r.code, 0, 0x40u | r.reverse,
                          r.pattern);
        }
    } else {
        uint32_t count = n->decoded_total;
        barcode_queue_code(n, r.code); /* Buddy3 -> nav_accept_barcode -> coordinator. */
        if (n->decoded_total == count)
            return; /* Inactive/takeover rejected by coordinator. */
        n->cross_candidate = n->cross_ready = false;
        d->analog.accepted = true;
        d->analog.accepted_start = r.start_mm;
        d->analog.accepted_end = r.end_mm;
        AnalogBank *v = &d->analog.bank[d->analog.selected];
        barcode_reader_reset(&v->reader, v->stable, now, mm);
        rb_junction_learn(&d->junction, n, r.base, r.max_wide_peak, adc, now, mm);
        if (d->junction.valid)
            d->analog.black = d->junction.black;
        barcode_event(n, B_DECODE, now, mm, r.end_mm - r.start_mm, r.code,
                      barcode_decode_action(n, r.code), 0x40u | r.reverse, r.pattern);
        ++d->peak_applied;
        d->peak_flags |= 4u;
    }
    d->raw_frozen = true;
}
static inline AnalogDemand rb_follow(LinePlanner *p, RbDecoder *b, const LineCalibration *cal,
                                     const CircuitInput *i, float dt) {
    unsigned before = p->circuit.phase, stage = p->nav.uturn.stage;
    SmoothFollow legacy = {0}; /* Profile1 never uses the legacy profile2 adapter. */
    AnalogDemand d = continuous_follow_step(&p->circuit, &p->nav, cal, i, RL_PROFILE, &legacy,
                                            &b->analog, &p->scan, &p->steering, dt);
    rl_junction_observe(p, cal, i, before, stage);
    p->optical = d;
    return d;
}
/* Preserve completed events/counts and the action/revision captured at decode;
 * discard partial frames, cross edges and interrupted tracking state. */
static inline void rb_resume(LinePlanner *p, RbDecoder *d, const CircuitInput *i, uint16_t adc) {
    circuit_begin(&p->circuit, i);
    p->steering = (ContinuousSteering){0};
    p->scan = (ScanSmooth){0};
    p->junction = (LineJunctionDiagnostic){0};
    p->nav.busy = false;
    p->nav.action = 0;
    p->nav.executing_action = 0;
    p->nav.executing_revision = 0;
    p->nav.uturn = (BarcodeUTurn){0};
    rb_partial_reset(d, &p->nav, adc, i->now, i->wheel);
    p->black = d->analog.black;
    p->side_adc = adc;
}
#endif
