/**
 * @file junction_sensor.h
 * @brief Junction sensor.
 * @details The ADC owner feeds both optical decoders and junction qualification.
 * Accepted navigation crosses the coordinator; this module never writes PWM.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_BARCODE_JUNCTION_SENSOR_H
#define RTOS_BARCODE_JUNCTION_SENSOR_H
#ifndef RTOS_JUNCTION_MIN_MM
/* The production RTOS release overrides this adapter range to 7mm. */
#define RTOS_JUNCTION_MIN_MM B_CROSS_MIN_MM
#endif
enum
{
    RB_JUNCTION_RESULT_NONE = 0,
    RB_JUNCTION_RESULT_CANDIDATE = 1,
    RB_JUNCTION_RESULT_TOO_NARROW = 2,
    RB_JUNCTION_RESULT_TOO_WIDE = 3,
    RB_JUNCTION_RESULT_WHITE_CONFIRMED = 4,
    RB_JUNCTION_RESULT_WHITE_CANCELLED = 5
};
/* ADC-owner-only state. A verified label supplies white/wide-bar contrast;
 * this classifier never participates in Code39 decoding or writes motors. */
typedef struct
{
    double white_start_mm;
    uint64_t candidate_us;
    double candidate_wheel[2];
    uint32_t decision_serial;
    uint16_t decision_threshold;
    int16_t last_width_centi_mm;
    uint8_t decision_hysteresis;
    unsigned threshold : 12;
    unsigned hysteresis : 8;
    unsigned valid : 1;
    unsigned armed : 1;
    unsigned black : 1;
    unsigned candidate : 1;
    unsigned white_tracking : 1;
    unsigned have_leading : 1;
    unsigned last_result : 3;
    unsigned last_candidate : 1;
    unsigned last_width_saturated : 1;
    unsigned decision_valid : 1;
    unsigned decision_armed : 1;
} rb_junction_sensor_t;

static inline void
rb_junction_decision_changed (rb_junction_sensor_t * p_other_data,
                              unsigned result)
{
    p_other_data->last_result = result <= RB_JUNCTION_RESULT_WHITE_CANCELLED
                                    ? result
                                    : RB_JUNCTION_RESULT_NONE;
    ++p_other_data->decision_serial;
    if (!p_other_data->decision_serial)
    {
        ++p_other_data->decision_serial;
    }
}

static inline int16_t rb_junction_width_centi (double mm_value,
                                               bool * p_saturated)
{
    double value = mm_value * 100.0;
    *p_saturated = false;
    if (isnan(value))
    {
        *p_saturated = true;
        return 0;
    }
    if (value > 32767.0)
    {
        *p_saturated = true;
        return INT16_MAX;
    }
    if (value < -32767.0)
    {
        *p_saturated = true;
        return -INT16_MAX;
    }
    return (int16_t)lround(value);
}

static inline void
rb_junction_record_width (rb_junction_sensor_t * p_other_data, double width,
                          unsigned result, bool was_candidate)
{
    bool saturated = false;
    p_other_data->last_width_centi_mm =
        rb_junction_width_centi(width, &saturated);
    p_other_data->last_width_saturated = saturated;
    p_other_data->last_candidate = was_candidate;
    p_other_data->decision_threshold = (uint16_t)p_other_data->threshold;
    p_other_data->decision_hysteresis = (uint8_t)p_other_data->hysteresis;
    p_other_data->decision_valid = p_other_data->valid;
    p_other_data->decision_armed = p_other_data->armed;
    rb_junction_decision_changed(p_other_data, result);
}

static inline void
rb_junction_record_white_outcome (rb_junction_sensor_t * p_other_data,
                                  unsigned result)
{
    /* Keep the measured stripe and candidate bit frozen through white gating. */
    p_other_data->last_candidate = true;
    rb_junction_decision_changed(p_other_data, result);
}

static inline void
rb_junction_cross_confirm (rb_junction_sensor_t * p_other_data,
                           barcode_nav_t * p_count, uint64_t now,
                           double mm_value, bool white)
{
    bool candidate = p_count->cross_candidate;
    barcode_cross_confirm(p_count, now, mm_value, white);
    if (candidate && !p_count->cross_candidate && p_count->cross_ready &&
        p_other_data->last_result == RB_JUNCTION_RESULT_CANDIDATE)
    {
        rb_junction_record_white_outcome(p_other_data,
                                         RB_JUNCTION_RESULT_WHITE_CONFIRMED);
    }
}

static inline void rb_junction_reset (rb_junction_sensor_t * p_other_data,
                                      barcode_nav_t * p_count, uint16_t adc,
                                      uint64_t now, double mm_value)
{
    if (p_count->cross_candidate &&
        p_other_data->last_result == RB_JUNCTION_RESULT_CANDIDATE)
    {
        rb_junction_record_white_outcome(p_other_data,
                                         RB_JUNCTION_RESULT_WHITE_CANCELLED);
    }
    p_other_data->armed = p_other_data->have_leading =
        p_other_data->white_tracking = false;
    p_other_data->black = p_other_data->candidate =
        p_other_data->valid && adc >= p_other_data->threshold;
    p_other_data->candidate_us = now;
    p_other_data->candidate_wheel[0] = p_other_data->candidate_wheel[1] =
        mm_value;
    p_count->cross_candidate = p_count->cross_ready = false;
    p_count->leading_us = 0;
    p_count->leading_gate = p_count->cross_gate = false;
}

static inline void rb_junction_learn (rb_junction_sensor_t * p_other_data,
                                      barcode_nav_t * p_count, double white,
                                      double wide, uint16_t adc, uint64_t now,
                                      double mm_value)
{
    /* Four ADC codes are the minimum representable two-sided margin. No
     * fixed surface threshold: midpoint plus 5% contrast hysteresis. */
    uint32_t decision_serial = p_other_data->decision_serial;
    uint16_t decision_threshold = p_other_data->decision_threshold;
    uint8_t decision_hysteresis = p_other_data->decision_hysteresis;
    int32_t last_width_centi_mm = p_other_data->last_width_centi_mm;
    unsigned last_result = p_other_data->last_result;
    unsigned last_candidate = p_other_data->last_candidate;
    unsigned last_width_saturated = p_other_data->last_width_saturated;
    unsigned decision_valid = p_other_data->decision_valid;
    unsigned decision_armed = p_other_data->decision_armed;
    *p_other_data = (rb_junction_sensor_t){0};
    p_other_data->decision_serial = decision_serial;
    p_other_data->decision_threshold = decision_threshold;
    p_other_data->decision_hysteresis = decision_hysteresis;
    p_other_data->last_width_centi_mm = last_width_centi_mm;
    p_other_data->last_result = last_result;
    p_other_data->last_candidate = last_candidate;
    p_other_data->last_width_saturated = last_width_saturated;
    p_other_data->decision_valid = decision_valid;
    p_other_data->decision_armed = decision_armed;
    if (isfinite(white) && isfinite(wide) && white >= 0 && wide <= 4095 &&
        wide - white >= 4)
    {
        p_other_data->threshold = (unsigned)lround((white + wide) * .5);
        unsigned h_value = (unsigned)lround((wide - white) * .05);
        p_other_data->hysteresis = h_value ? h_value : 1;
        p_other_data->valid = true;
    }
    rb_junction_reset(p_other_data, p_count, adc, now, mm_value);
}

static inline void rb_junction_sample (rb_junction_sensor_t * p_other_data,
                                       barcode_nav_t * p_count, uint16_t adc,
                                       uint64_t now, const double wheel[2])
{
    if (!p_other_data->valid)
    {
        return;
    }
    double mm_value = (wheel[0] + wheel[1]) * .5;
    bool black =
        p_other_data->black
            ? adc > p_other_data->threshold - p_other_data->hysteresis
            : adc >= p_other_data->threshold + p_other_data->hysteresis;
    if (black != p_other_data->candidate)
    {
        p_other_data->candidate = black;
        p_other_data->candidate_us = now;
        memcpy(p_other_data->candidate_wheel, wheel,
               sizeof(p_other_data->candidate_wheel));
    }
    bool changed = p_other_data->candidate != p_other_data->black &&
                   now - p_other_data->candidate_us >= AO_DEBOUNCE_US;
    if (changed)
    {
        p_other_data->black = p_other_data->candidate;
    }
    if (p_count->busy || !p_count->pending)
    {
        p_other_data->armed = p_other_data->have_leading =
            p_other_data->white_tracking = false;
        return;
    }
    if (!p_other_data->armed)
    {
        /* Never reuse the label's stop bar, a partial bar at Resume, or a
         * classifier change as the leading edge of a junction. */
        if (p_other_data->black || p_other_data->candidate)
        {
            p_other_data->white_tracking = false;
        }
        else
        {
            if (!p_other_data->white_tracking)
            {
                p_other_data->white_start_mm = mm_value;
                p_other_data->white_tracking = true;
            }
            if (mm_value - p_other_data->white_start_mm >= B_CROSS_WHITE_MM)
            {
                p_other_data->armed = true;
            }
        }
        return;
    }
    if (changed)
    {
        double edge = (p_other_data->candidate_wheel[0] +
                       p_other_data->candidate_wheel[1]) *
                      .5;
        if (p_other_data->black)
        {
            /* Use the measured edge, not the later debounce position. */
            rb_junction_cross_confirm(p_other_data, p_count,
                                      p_other_data->candidate_us, edge, true);
            if (p_count->cross_candidate)
            {
                p_count->cross_candidate = false;
                barcode_event(p_count, B_CROSS_CANCEL,
                              p_other_data->candidate_us, edge,
                              edge - p_count->cross_trailing_mm,
                              p_count->pending, 0, 0, NULL);
                if (p_other_data->last_result == RB_JUNCTION_RESULT_CANDIDATE)
                {
                    rb_junction_record_white_outcome(
                        p_other_data, RB_JUNCTION_RESULT_WHITE_CANCELLED);
                }
            }
            p_count->leading_us = p_other_data->candidate_us;
            memcpy(p_count->leading_wheel, p_other_data->candidate_wheel,
                   sizeof(p_count->leading_wheel));
            p_count->leading_gate = p_count->allow_cross;
            p_other_data->have_leading = true;
        }
        else if (p_other_data->have_leading)
        {
            p_other_data->have_leading = false;
            double leading =
                (p_count->leading_wheel[0] + p_count->leading_wheel[1]) * .5;
            double width = edge - leading;
            if (isfinite(width) && width >= RTOS_JUNCTION_MIN_MM &&
                width <= B_CROSS_MAX_MM)
            {
                p_count->cross_width = width;
                p_count->cross_us = p_count->leading_us;
                memcpy(p_count->cross_wheel, p_count->leading_wheel,
                       sizeof(p_count->cross_wheel));
                p_count->cross_gate = p_count->leading_gate;
                p_count->cross_candidate = true;
                p_count->cross_trailing_mm = edge;
                rb_junction_record_width(p_other_data, width,
                                         RB_JUNCTION_RESULT_CANDIDATE, true);
                barcode_event(
                    p_count, B_CROSS_CANDIDATE, p_other_data->candidate_us,
                    edge, width, p_count->pending,
                    barcode_nav_action(p_count, p_count->pending), 0, NULL);
            }
            else
            {
                unsigned result = isnan(width) || width < RTOS_JUNCTION_MIN_MM
                                      ? RB_JUNCTION_RESULT_TOO_NARROW
                                      : RB_JUNCTION_RESULT_TOO_WIDE;
                rb_junction_record_width(p_other_data, width, result, false);
            }
        }
    }
    rb_junction_cross_confirm(p_other_data, p_count, now, mm_value,
                              !p_other_data->black && !p_other_data->candidate);
}
#endif

/* End of file. */
