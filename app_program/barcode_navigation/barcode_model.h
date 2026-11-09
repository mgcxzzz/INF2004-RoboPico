#ifndef RTOS_BARCODE_MODEL_H
#define RTOS_BARCODE_MODEL_H
/* Hardware-free migration adapter. Both Code39 decoders retain reference
 * thresholds; junction sensing uses independent verified-frame contrast.
 * Call on the ADC owner's 2ms GP27 samples, not a network task. */
#include "../line_navigation/line_model.h"
#include "src/barcode/barcode_peak.h"
#include "junction_sensor.h"

enum {
    RB_RAW_ROWS = 2048,
    RB_RAW_BYTES = 10,
    RB_RAW_PREROLL_ROWS = 512,
    RB_RAW_INTERVAL_US = 5000,
    RB_PEAK_RUN_START = 10,
    RB_ATTEMPT_PAYLOAD = 27,
    RB_ATTEMPT_FRAME = 30,
    RB_JUNCTION_PAYLOAD = 17,
    RB_JUNCTION_FRAME = 20,
    RB_ATTEMPT_IN_PROGRESS = 10
};
enum {
    RB_ATTEMPT_VALID = 1u << 0,
    RB_ATTEMPT_DECODED = 1u << 1,
    RB_ATTEMPT_FINALIZED = 1u << 2,
    RB_ATTEMPT_TRUNCATED = 1u << 3,
    RB_ATTEMPT_START_VALID = 1u << 4,
    RB_ATTEMPT_ORIGINAL_ACCEPTED = 1u << 5
};
typedef struct {
    uint8_t bytes[RB_RAW_BYTES];
} RbRaw;
_Static_assert(sizeof(RbRaw) == 10, "Raw GP27 record must stay explicitly packed");
typedef struct {
    uint32_t sequence, first_ms, last_ms, raw_overwritten;
    unsigned raw_count : 12, code : 7, reason : 4, bars : 5, flags : 6;
} RbAttemptInfo;
_Static_assert(RB_RAW_ROWS <= 2048 && RB_RAW_PREROLL_ROWS <= 512,
               "Raw ring sizes exceed their bounded counters");
typedef struct {
    BarcodeAnalog analog;
    BarcodePeak peak;
    BpResult last_peak;
    RbJunctionSensor junction;
    RbRaw raw[RB_RAW_ROWS];
    RbRaw preroll[RB_RAW_PREROLL_ROWS];
    RbAttemptInfo attempt;
    uint64_t raw_last_us;
    uint32_t event_seen, peak_serial, peak_decodes, peak_applied, peak_conflicts;
    uint32_t samples, raw_overwritten, max_sample_us;
    uint32_t attempt_sequence;
    unsigned raw_count : 12, raw_head : 11;
    unsigned preroll_count : 10, preroll_head : 9;
    uint16_t groups;
    uint8_t peak_flags;
    bool raw_frozen : 1, raw_have_time : 1, raw_capture_active : 1;
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
static inline void rb_raw_encode(RbRaw *r, const BarcodeNav *n, uint16_t adc, uint64_t now,
                                 double mm) {
    rb_put32(r->bytes, (uint32_t)((now - n->start_us) / 1000));
    double v = mm * 100;
    int32_t pos = v > INT32_MAX ? INT32_MAX : v < INT32_MIN ? INT32_MIN : (int32_t)lround(v);
    rb_put32(r->bytes + 4, (uint32_t)pos);
    r->bytes[8] = (uint8_t)adc;
    r->bytes[9] = (uint8_t)((adc >> 8) & 15U);
}
static inline void rb_raw_append_capture(RbDecoder *d, const RbRaw *r) {
    d->raw[d->raw_head] = *r;
    d->raw_head = (d->raw_head + 1) % RB_RAW_ROWS;
    if (d->raw_count < RB_RAW_ROWS)
        ++d->raw_count;
    else {
        ++d->raw_overwritten;
        ++d->attempt.raw_overwritten;
        d->attempt.flags |= RB_ATTEMPT_TRUNCATED;
        d->attempt.flags &= (uint8_t)~RB_ATTEMPT_START_VALID;
    }
    d->attempt.raw_count = (uint16_t)d->raw_count;
    if (d->raw_count) {
        unsigned oldest = (d->raw_head + RB_RAW_ROWS - d->raw_count) % RB_RAW_ROWS;
        d->attempt.first_ms = rb_get32(d->raw[oldest].bytes);
        unsigned newest = (d->raw_head + RB_RAW_ROWS - 1) % RB_RAW_ROWS;
        d->attempt.last_ms = rb_get32(d->raw[newest].bytes);
    }
}
static inline void rb_raw_store_preroll(RbDecoder *d, const RbRaw *r) {
    d->preroll[d->preroll_head] = *r;
    d->preroll_head = (d->preroll_head + 1) % RB_RAW_PREROLL_ROWS;
    if (d->preroll_count < RB_RAW_PREROLL_ROWS)
        ++d->preroll_count;
}
static inline double rb_raw_position(const RbRaw *r) {
    return (int32_t)rb_get32(r->bytes + 4) / 100.0;
}
static inline void rb_attempt_begin(RbDecoder *d, const BarcodeNav *n, double first_bar_mm,
                                    bool original_success) {
    if (d->raw_capture_active) {
        if (original_success) {
            d->attempt.flags |= RB_ATTEMPT_ORIGINAL_ACCEPTED | RB_ATTEMPT_DECODED;
            d->raw_frozen = false;
        }
        return;
    }
    ++d->attempt_sequence;
    if (!d->attempt_sequence)
        ++d->attempt_sequence;
    d->attempt = (RbAttemptInfo){.sequence = d->attempt_sequence,
                                 .code = 0,
                                 .reason = RB_ATTEMPT_IN_PROGRESS,
                                 .bars = d->peak.nbars,
                                 .flags = RB_ATTEMPT_VALID};
    d->raw_count = d->raw_head = 0;
    d->raw_overwritten = 0;
    d->raw_capture_active = true;
    d->raw_frozen = false;
    unsigned begin = (d->preroll_head + RB_RAW_PREROLL_ROWS - d->preroll_count) %
                     RB_RAW_PREROLL_ROWS;
    unsigned first = d->preroll_count;
    unsigned contiguous = RB_RAW_PREROLL_ROWS - begin;
    if (first > contiguous)
        first = contiguous;
    if (first)
        memcpy(d->raw, d->preroll + begin, first * sizeof(RbRaw));
    if (d->preroll_count > first)
        memcpy(d->raw + first, d->preroll, (d->preroll_count - first) * sizeof(RbRaw));
    d->raw_count = d->preroll_count;
    d->attempt.raw_count = d->preroll_count;
    d->raw_head = d->preroll_count % RB_RAW_ROWS;
    if (d->preroll_count) {
        d->attempt.first_ms = rb_get32(d->raw[0].bytes);
        d->attempt.last_ms = rb_get32(d->raw[d->preroll_count - 1].bytes);
    }
    if (d->preroll_count) {
        const RbRaw *oldest = &d->preroll[begin];
        if (isfinite(first_bar_mm) && rb_raw_position(oldest) <= first_bar_mm)
            d->attempt.flags |= RB_ATTEMPT_START_VALID;
        else
            d->attempt.flags |= RB_ATTEMPT_TRUNCATED;
    } else {
        d->attempt.flags |= RB_ATTEMPT_TRUNCATED;
    }
    if (original_success)
        d->attempt.flags |= RB_ATTEMPT_ORIGINAL_ACCEPTED | RB_ATTEMPT_DECODED;
    if (original_success && n->last_code >= 'A' && n->last_code <= 'Z')
        d->attempt.code = (uint8_t)n->last_code;
}
static inline void rb_attempt_finish(RbDecoder *d, const BpResult *r, bool finalized) {
    if (!d->raw_capture_active)
        return;
    d->attempt.reason = finalized ? r->reason : RB_ATTEMPT_IN_PROGRESS;
    d->attempt.bars = r->nbars;
    if (r->decoded) {
        d->attempt.code = (uint8_t)r->code;
        d->attempt.flags |= RB_ATTEMPT_DECODED;
    }
    if (finalized)
        d->attempt.flags |= RB_ATTEMPT_FINALIZED;
    else
        d->attempt.flags |= RB_ATTEMPT_TRUNCATED;
    d->attempt.raw_count = (uint16_t)d->raw_count;
    d->raw_capture_active = false;
    d->raw_frozen = true;
}
static inline void rb_attempt_original_success(RbDecoder *d, BarcodeNav *n) {
    double first_bar_mm = d->peak.nbars ? d->peak.bar[0].mm : NAN;
    rb_attempt_begin(d, n, first_bar_mm, true);
    d->attempt.flags |= RB_ATTEMPT_ORIGINAL_ACCEPTED | RB_ATTEMPT_DECODED;
    if (n->last_code >= 'A' && n->last_code <= 'Z')
        d->attempt.code = (uint8_t)n->last_code;
}
static inline void rb_attempt_interrupt(RbDecoder *d) {
    if (!d->raw_capture_active)
        return;
    BpResult partial = {.decoded = false, .code = '?', .reason = RB_ATTEMPT_IN_PROGRESS,
                        .nbars = d->peak.nbars};
    rb_attempt_finish(d, &partial, false);
}
static inline void rb_attempt_pack(const RbAttemptInfo *m, uint32_t uptime_ms, uint16_t run_id,
                                   uint8_t seq, uint8_t frame[RB_ATTEMPT_FRAME]) {
    memset(frame, 0, RB_ATTEMPT_FRAME);
    frame[0] = 0xF2;
    frame[1] = RB_ATTEMPT_PAYLOAD;
    frame[2] = seq;
    uint8_t *p = frame + 3;
    p[0] = 0xC5;
    rb_put32(p + 1, uptime_ms);
    rb_put32(p + 5, m->sequence);
    rb_put32(p + 9, m->first_ms);
    rb_put32(p + 13, m->last_ms);
    rb_put32(p + 17, m->raw_overwritten);
    p[21] = (uint8_t)run_id;
    p[22] = (uint8_t)(run_id >> 8);
    unsigned code = m->code >= 'A' && m->code <= 'Z' ? m->code - 'A' + 1U : 0U;
    unsigned reason = m->reason <= RB_ATTEMPT_IN_PROGRESS ? m->reason : RB_ATTEMPT_IN_PROGRESS;
    unsigned bars = m->bars > BP_MAX_BARS ? BP_MAX_BARS : m->bars;
    unsigned count = m->raw_count > RB_RAW_ROWS ? RB_RAW_ROWS : m->raw_count;
    uint32_t meta = (m->flags & 0x3fU) | (code << 6) | (reason << 11) | (bars << 15) |
                    (count << 20);
    rb_put32(p + 23, meta);
}
/* source15 C7 reports only on diagnostic changes/normal telemetry cadence.
 * Width is signed centi-mm, saturated symmetrically to +/-327.67mm. */
static inline void rb_junction_pack(const RbJunctionSensor *q, bool pending,
                                    uint32_t uptime_ms, uint16_t run_id, uint8_t seq,
                                    uint8_t frame[RB_JUNCTION_FRAME]) {
    memset(frame, 0, RB_JUNCTION_FRAME);
    frame[0] = 0xF2;
    frame[1] = RB_JUNCTION_PAYLOAD;
    frame[2] = seq;
    uint8_t *p = frame + 3;
    p[0] = 0xC7;
    rb_put32(p + 1, uptime_ms);
    rb_put32(p + 5, q->decision_serial);
    p[9] = (uint8_t)run_id;
    p[10] = (uint8_t)(run_id >> 8);
    unsigned threshold = q->decision_serial ? q->decision_threshold : q->threshold;
    unsigned hysteresis = q->decision_serial ? q->decision_hysteresis : q->hysteresis;
    bool valid = q->decision_serial ? q->decision_valid : q->valid;
    bool armed = q->decision_serial ? q->decision_armed : q->armed;
    p[11] = (uint8_t)threshold;
    p[12] = (uint8_t)(((threshold >> 8) & 0x0fU) | ((hysteresis & 0x0fU) << 4));
    p[13] = (uint8_t)((hysteresis >> 4) & 0x0fU);
    uint16_t width = (uint16_t)q->last_width_centi_mm;
    p[14] = (uint8_t)width;
    p[15] = (uint8_t)(width >> 8);
    unsigned result = q->last_result <= RB_JUNCTION_RESULT_WHITE_CANCELLED ? q->last_result : 0;
    p[16] = (uint8_t)(result | ((valid ? 1U : 0U) << 3) |
                      ((armed ? 1U : 0U) << 4) | ((pending ? 1U : 0U) << 5) |
                      ((q->last_candidate ? 1U : 0U) << 6) |
                      ((q->last_width_saturated ? 1U : 0U) << 7));
}
static inline void rb_raw(RbDecoder *d, const BarcodeNav *n, uint16_t adc, uint64_t now,
                          double mm) {
    ++d->samples;
    if (d->raw_have_time && now - d->raw_last_us < RB_RAW_INTERVAL_US)
        return;
    d->raw_have_time = true;
    d->raw_last_us = now;
    RbRaw sample;
    rb_raw_encode(&sample, n, adc, now, mm);
    rb_raw_store_preroll(d, &sample);
    if (d->raw_capture_active)
        rb_raw_append_capture(d, &sample);
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
    bool original_success = n->decoded_total != before;
    if (original_success) {
        rb_attempt_original_success(d, n);
        rb_junction_learn(&d->junction, n, d->analog.minimum, d->analog.maximum, adc, now, mm);
    }
    rb_junction_sample(&d->junction, n, adc, now, wheel);
    if (d->junction.valid)
        d->analog.black = d->junction.black;
    if (n->busy) {
        rb_attempt_interrupt(d);
        bp_reset(&d->peak);
        return;
    }
    BpResult r;
    bool group_ended = bp_sample(&d->peak, adc, mm, &r);
    if (!d->raw_capture_active && d->peak.nbars >= BP_REPORT_MIN_BARS)
        rb_attempt_begin(d, n, d->peak.bar[0].mm, original_success);
    if (group_ended && r.nbars >= BP_REPORT_MIN_BARS && !d->raw_capture_active)
        rb_attempt_begin(d, n, r.start_mm, original_success);
    if (!group_ended) {
        if (d->raw_capture_active)
            d->attempt.bars = d->peak.nbars;
        return;
    }
    if (r.nbars < BP_REPORT_MIN_BARS) {
        if (d->raw_capture_active)
            rb_attempt_finish(d, &r, true);
        return;
    }
    if (d->groups < 65535)
        ++d->groups;
    d->last_peak = r;
    ++d->peak_serial;
    d->peak_flags = 1u | (r.decoded ? 2u : 0u) | (r.reverse ? 16u : 0u) | (r.overflow ? 64u : 0u);
    rb_attempt_finish(d, &r, true);
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
    rb_attempt_interrupt(d);
    d->preroll_count = d->preroll_head = 0;
    d->raw_have_time = false;
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
