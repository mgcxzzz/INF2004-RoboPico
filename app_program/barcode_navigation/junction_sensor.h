#ifndef RTOS_BARCODE_JUNCTION_SENSOR_H
#define RTOS_BARCODE_JUNCTION_SENSOR_H
/* ADC-owner-only state. A verified label supplies white/wide-bar contrast;
 * this classifier never participates in Code39 decoding or writes motors. */
typedef struct {
    double white_start_mm;
    uint64_t candidate_us;
    double candidate_wheel[2];
    unsigned threshold : 12, hysteresis : 12;
    unsigned valid : 1, armed : 1, black : 1, candidate : 1, white_tracking : 1;
    unsigned have_leading : 1;
} RbJunctionSensor;

static inline void rb_junction_reset(RbJunctionSensor *q, BarcodeNav *n, uint16_t adc, uint64_t now,
                                     double mm) {
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
    *q = (RbJunctionSensor){0};
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
            barcode_cross_confirm(n, q->candidate_us, edge, true);
            if (n->cross_candidate) {
                n->cross_candidate = false;
                barcode_event(n, B_CROSS_CANCEL, q->candidate_us, edge, edge - n->cross_trailing_mm,
                              n->pending, 0, 0, NULL);
            }
            n->leading_us = q->candidate_us;
            memcpy(n->leading_wheel, q->candidate_wheel, sizeof(n->leading_wheel));
            n->leading_gate = n->allow_cross;
            q->have_leading = true;
        } else if (q->have_leading) {
            q->have_leading = false;
            double leading = (n->leading_wheel[0] + n->leading_wheel[1]) * .5;
            double width = edge - leading;
            if (width >= B_CROSS_MIN_MM && width <= B_CROSS_MAX_MM) {
                n->cross_width = width;
                n->cross_us = n->leading_us;
                memcpy(n->cross_wheel, n->leading_wheel, sizeof(n->cross_wheel));
                n->cross_gate = n->leading_gate;
                n->cross_candidate = true;
                n->cross_trailing_mm = edge;
                barcode_event(n, B_CROSS_CANDIDATE, q->candidate_us, edge, width, n->pending,
                              barcode_nav_action(n, n->pending), 0, NULL);
            }
        }
    }
    barcode_cross_confirm(n, now, mm, !q->black && !q->candidate);
}
#endif
