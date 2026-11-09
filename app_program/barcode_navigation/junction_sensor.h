#ifndef RTOS_BARCODE_JUNCTION_SENSOR_H
#define RTOS_BARCODE_JUNCTION_SENSOR_H
#ifndef RTOS_JUNCTION_MIN_MM
/* The production RTOS release overrides this adapter range to 7mm. */
#define RTOS_JUNCTION_MIN_MM B_CROSS_MIN_MM
#endif
enum {
    RB_JUNCTION_RESULT_NONE = 0,
    RB_JUNCTION_RESULT_CANDIDATE = 1,
    RB_JUNCTION_RESULT_TOO_NARROW = 2,
    RB_JUNCTION_RESULT_TOO_WIDE = 3,
    RB_JUNCTION_RESULT_WHITE_CONFIRMED = 4,
    RB_JUNCTION_RESULT_WHITE_CANCELLED = 5
};
/* ADC-owner-only state. A verified label supplies white/wide-bar contrast;
 * this classifier never participates in Code39 decoding or writes motors. */
typedef struct {
    double white_start_mm;
    uint64_t candidate_us;
    double candidate_wheel[2];
    uint32_t decision_serial;
    uint16_t decision_threshold;
    int16_t last_width_centi_mm;
    uint8_t decision_hysteresis;
    unsigned threshold : 12, hysteresis : 8;
    unsigned valid : 1, armed : 1, black : 1, candidate : 1, white_tracking : 1;
    unsigned have_leading : 1;
    unsigned last_result : 3, last_candidate : 1, last_width_saturated : 1;
    unsigned decision_valid : 1, decision_armed : 1;
} RbJunctionSensor;

static inline void rb_junction_decision_changed(RbJunctionSensor *q, unsigned result) {
    q->last_result = result <= RB_JUNCTION_RESULT_WHITE_CANCELLED
                         ? result
                         : RB_JUNCTION_RESULT_NONE;
    ++q->decision_serial;
    if (!q->decision_serial)
        ++q->decision_serial;
}

static inline int16_t rb_junction_width_centi(double mm, bool *saturated) {
    double v = mm * 100.0;
    *saturated = false;
    if (isnan(v)) {
        *saturated = true;
        return 0;
    }
    if (v > 32767.0) {
        *saturated = true;
        return INT16_MAX;
    }
    if (v < -32767.0) {
        *saturated = true;
        return -INT16_MAX;
    }
    return (int16_t)lround(v);
}

static inline void rb_junction_record_width(RbJunctionSensor *q, double width,
                                            unsigned result, bool was_candidate) {
    bool saturated = false;
    q->last_width_centi_mm = rb_junction_width_centi(width, &saturated);
    q->last_width_saturated = saturated;
    q->last_candidate = was_candidate;
    q->decision_threshold = (uint16_t)q->threshold;
    q->decision_hysteresis = (uint8_t)q->hysteresis;
    q->decision_valid = q->valid;
    q->decision_armed = q->armed;
    rb_junction_decision_changed(q, result);
}

static inline void rb_junction_record_white_outcome(RbJunctionSensor *q, unsigned result) {
    /* Keep the measured stripe and candidate bit frozen through white gating. */
    q->last_candidate = true;
    rb_junction_decision_changed(q, result);
}

static inline void rb_junction_cross_confirm(RbJunctionSensor *q, BarcodeNav *n, uint64_t now,
                                             double mm, bool white) {
    bool candidate = n->cross_candidate;
    barcode_cross_confirm(n, now, mm, white);
    if (candidate && !n->cross_candidate && n->cross_ready &&
        q->last_result == RB_JUNCTION_RESULT_CANDIDATE)
        rb_junction_record_white_outcome(q, RB_JUNCTION_RESULT_WHITE_CONFIRMED);
}

static inline void rb_junction_reset(RbJunctionSensor *q, BarcodeNav *n, uint16_t adc, uint64_t now,
                                     double mm) {
    if (n->cross_candidate && q->last_result == RB_JUNCTION_RESULT_CANDIDATE)
        rb_junction_record_white_outcome(q, RB_JUNCTION_RESULT_WHITE_CANCELLED);
    q->armed = q->have_leading = q->white_tracking = false;
    q->black = q->candidate = q->valid && adc >= q->threshold;
    q->candidate_us = now;
    q->candidate_wheel[0] = q->candidate_wheel[1] = mm;
    n->cross_candidate = n->cross_ready = false;
    n->leading_us = 0;
    n->leading_gate = n->cross_gate = false;
}

static inline void rb_junction_learn(RbJunctionSensor *q, BarcodeNav *n, double white, double wide,
                                     uint16_t adc, uint64_t now, double mm) {
    /* Four ADC codes are the minimum representable two-sided margin. No
     * fixed surface threshold: midpoint plus 5% contrast hysteresis. */
    uint32_t decision_serial = q->decision_serial;
    uint16_t decision_threshold = q->decision_threshold;
    uint8_t decision_hysteresis = q->decision_hysteresis;
    int32_t last_width_centi_mm = q->last_width_centi_mm;
    unsigned last_result = q->last_result;
    unsigned last_candidate = q->last_candidate;
    unsigned last_width_saturated = q->last_width_saturated;
    unsigned decision_valid = q->decision_valid;
    unsigned decision_armed = q->decision_armed;
    *q = (RbJunctionSensor){0};
    q->decision_serial = decision_serial;
    q->decision_threshold = decision_threshold;
    q->decision_hysteresis = decision_hysteresis;
    q->last_width_centi_mm = last_width_centi_mm;
    q->last_result = last_result;
    q->last_candidate = last_candidate;
    q->last_width_saturated = last_width_saturated;
    q->decision_valid = decision_valid;
    q->decision_armed = decision_armed;
    if (isfinite(white) && isfinite(wide) && white >= 0 && wide <= 4095 && wide - white >= 4) {
        q->threshold = (unsigned)lround((white + wide) * .5);
        unsigned h = (unsigned)lround((wide - white) * .05);
        q->hysteresis = h ? h : 1;
        q->valid = true;
    }
    rb_junction_reset(q, n, adc, now, mm);
}

static inline void rb_junction_sample(RbJunctionSensor *q, BarcodeNav *n, uint16_t adc,
                                      uint64_t now, const double wheel[2]) {
    if (!q->valid)
        return;
    double mm = (wheel[0] + wheel[1]) * .5;
    bool black =
        q->black ? adc > q->threshold - q->hysteresis : adc >= q->threshold + q->hysteresis;
    if (black != q->candidate) {
        q->candidate = black;
        q->candidate_us = now;
        memcpy(q->candidate_wheel, wheel, sizeof(q->candidate_wheel));
    }
    bool changed = q->candidate != q->black && now - q->candidate_us >= AO_DEBOUNCE_US;
    if (changed)
        q->black = q->candidate;
    if (n->busy || !n->pending) {
        q->armed = q->have_leading = q->white_tracking = false;
        return;
    }
    if (!q->armed) {
        /* Never reuse the label's stop bar, a partial bar at Resume, or a
         * classifier change as the leading edge of a junction. */
        if (q->black || q->candidate) {
            q->white_tracking = false;
        } else {
            if (!q->white_tracking) {
                q->white_start_mm = mm;
                q->white_tracking = true;
            }
            if (mm - q->white_start_mm >= B_CROSS_WHITE_MM)
                q->armed = true;
        }
        return;
    }
    if (changed) {
        double edge = (q->candidate_wheel[0] + q->candidate_wheel[1]) * .5;
        if (q->black) {
            /* Use the measured edge, not the later debounce position. */
            rb_junction_cross_confirm(q, n, q->candidate_us, edge, true);
            if (n->cross_candidate) {
                n->cross_candidate = false;
                barcode_event(n, B_CROSS_CANCEL, q->candidate_us, edge, edge - n->cross_trailing_mm,
                              n->pending, 0, 0, NULL);
                if (q->last_result == RB_JUNCTION_RESULT_CANDIDATE)
                    rb_junction_record_white_outcome(q, RB_JUNCTION_RESULT_WHITE_CANCELLED);
            }
            n->leading_us = q->candidate_us;
            memcpy(n->leading_wheel, q->candidate_wheel, sizeof(n->leading_wheel));
            n->leading_gate = n->allow_cross;
            q->have_leading = true;
        } else if (q->have_leading) {
            q->have_leading = false;
            double leading = (n->leading_wheel[0] + n->leading_wheel[1]) * .5;
            double width = edge - leading;
            if (isfinite(width) && width >= RTOS_JUNCTION_MIN_MM && width <= B_CROSS_MAX_MM) {
                n->cross_width = width;
                n->cross_us = n->leading_us;
                memcpy(n->cross_wheel, n->leading_wheel, sizeof(n->cross_wheel));
                n->cross_gate = n->leading_gate;
                n->cross_candidate = true;
                n->cross_trailing_mm = edge;
                rb_junction_record_width(q, width, RB_JUNCTION_RESULT_CANDIDATE, true);
                barcode_event(n, B_CROSS_CANDIDATE, q->candidate_us, edge, width, n->pending,
                              barcode_nav_action(n, n->pending), 0, NULL);
            } else {
                unsigned result = isnan(width) || width < RTOS_JUNCTION_MIN_MM
                                      ? RB_JUNCTION_RESULT_TOO_NARROW
                                      : RB_JUNCTION_RESULT_TOO_WIDE;
                rb_junction_record_width(q, width, result, false);
            }
        }
    }
    rb_junction_cross_confirm(q, n, now, mm, !q->black && !q->candidate);
}
#endif
