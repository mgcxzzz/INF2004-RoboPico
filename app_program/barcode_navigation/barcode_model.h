/**
 * @file barcode_model.h
 * @brief Barcode model.
 * @details The ADC owner feeds both optical decoders and junction
 * qualification. Accepted navigation crosses the coordinator; this module never
 * writes PWM.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_BARCODE_MODEL_H
#define RTOS_BARCODE_MODEL_H
/* Hardware-free migration adapter. Both Code39 decoders retain reference
 * thresholds; junction sensing uses independent verified-frame contrast.
 * Call on the ADC owner's 2ms GP27 samples, not a network task. */
#include "../line_navigation/line_model.h"
#include "src/barcode/barcode_peak.h"
#include "junction_sensor.h"

enum
{
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
enum
{
    RB_ATTEMPT_VALID = 1u << 0,
    RB_ATTEMPT_DECODED = 1u << 1,
    RB_ATTEMPT_FINALIZED = 1u << 2,
    RB_ATTEMPT_TRUNCATED = 1u << 3,
    RB_ATTEMPT_START_VALID = 1u << 4,
    RB_ATTEMPT_ORIGINAL_ACCEPTED = 1u << 5
};
typedef struct
{
    uint8_t bytes[RB_RAW_BYTES];
} rb_raw_t;
_Static_assert(10 == sizeof(rb_raw_t),
               "Raw GP27 record must stay explicitly packed");
typedef struct
{
    uint32_t sequence;
    uint32_t first_ms;
    uint32_t last_ms;
    uint32_t raw_overwritten;
    unsigned raw_count : 12;
    unsigned code : 7;
    unsigned reason : 4;
    unsigned bars : 5;
    unsigned flags : 6;
} rb_attempt_info_t;
_Static_assert((RB_RAW_ROWS <= 2048) && (RB_RAW_PREROLL_ROWS <= 512),
               "Raw ring sizes exceed their bounded counters");
typedef struct
{
    barcode_analog_t analog;
    barcode_peak_t peak;
    bp_result_t last_peak;
    rb_junction_sensor_t junction;
    rb_raw_t raw[RB_RAW_ROWS];
    rb_raw_t preroll[RB_RAW_PREROLL_ROWS];
    rb_attempt_info_t attempt;
    uint64_t raw_last_us;
    uint32_t event_seen;
    uint32_t peak_serial;
    uint32_t peak_decodes;
    uint32_t peak_applied;
    uint32_t peak_conflicts;
    uint32_t samples;
    uint32_t raw_overwritten;
    uint32_t max_sample_us;
    uint32_t attempt_sequence;
    unsigned raw_count : 12;
    unsigned raw_head : 11;
    unsigned preroll_count : 10;
    unsigned preroll_head : 9;
    uint16_t groups;
    uint8_t peak_flags;
    bool raw_frozen : 1;
    bool raw_have_time : 1;
    bool raw_capture_active : 1;
} rb_decoder_t;
static inline char rb_log_route (const barcode_nav_t * p_count, char code)
{
    char first = barcode_nav_action(p_count, code);
    return 'L' == first ? 'A' : 'R' == first ? 'F' : 'U' == first ? 'Z' : 0;
}
static inline void rb_put32 (uint8_t * p_data, uint32_t value)
{
    for (unsigned count = 0; count < 4; ++count)
    {
        p_data[count] = (uint8_t)(value >> (8 * count));
    }
}
static inline uint32_t rb_get32 (const uint8_t * p_data)
{
    return p_data[0] | (uint32_t)p_data[1] << 8 | (uint32_t)p_data[2] << 16 |
           (uint32_t)p_data[3] << 24;
}
static inline void rb_raw_encode (rb_raw_t * p_result,
                                  const barcode_nav_t * p_count, uint16_t adc,
                                  uint64_t now, double mm_value)
{
    rb_put32(p_result->bytes, (uint32_t)((now - p_count->start_us) / 1000));
    double value = mm_value * 100;
    int32_t pos = value > INT32_MAX   ? INT32_MAX
                  : value < INT32_MIN ? INT32_MIN
                                      : (int32_t)lround(value);
    rb_put32(p_result->bytes + 4, (uint32_t)pos);
    p_result->bytes[8] = (uint8_t)adc;
    p_result->bytes[9] = (uint8_t)((adc >> 8) & 15U);
}
static inline void rb_raw_append_capture (rb_decoder_t * p_demand,
                                          const rb_raw_t * p_result)
{
    p_demand->raw[p_demand->raw_head] = *p_result;
    p_demand->raw_head = (p_demand->raw_head + 1) % RB_RAW_ROWS;
    if (p_demand->raw_count < RB_RAW_ROWS)
    {
        ++p_demand->raw_count;
    }
    else
    {
        ++p_demand->raw_overwritten;
        ++p_demand->attempt.raw_overwritten;
        p_demand->attempt.flags |= RB_ATTEMPT_TRUNCATED;
        p_demand->attempt.flags &= (uint8_t)~RB_ATTEMPT_START_VALID;
    }
    p_demand->attempt.raw_count = (uint16_t)p_demand->raw_count;
    if (p_demand->raw_count)
    {
        unsigned oldest =
            (p_demand->raw_head + RB_RAW_ROWS - p_demand->raw_count) %
            RB_RAW_ROWS;
        p_demand->attempt.first_ms = rb_get32(p_demand->raw[oldest].bytes);
        unsigned newest = (p_demand->raw_head + RB_RAW_ROWS - 1) % RB_RAW_ROWS;
        p_demand->attempt.last_ms = rb_get32(p_demand->raw[newest].bytes);
    }
}
static inline void rb_raw_store_preroll (rb_decoder_t * p_demand,
                                         const rb_raw_t * p_result)
{
    p_demand->preroll[p_demand->preroll_head] = *p_result;
    p_demand->preroll_head = (p_demand->preroll_head + 1) % RB_RAW_PREROLL_ROWS;
    if (p_demand->preroll_count < RB_RAW_PREROLL_ROWS)
    {
        ++p_demand->preroll_count;
    }
}
static inline double rb_raw_position (const rb_raw_t * p_result)
{
    return (int32_t)rb_get32(p_result->bytes + 4) / 100.0;
}
static inline void rb_attempt_begin (rb_decoder_t * p_demand,
                                     const barcode_nav_t * p_count,
                                     double first_bar_mm, bool original_success)
{
    if (p_demand->raw_capture_active)
    {
        if (original_success)
        {
            p_demand->attempt.flags |=
                RB_ATTEMPT_ORIGINAL_ACCEPTED | RB_ATTEMPT_DECODED;
            p_demand->raw_frozen = false;
        }
        return;
    }
    ++p_demand->attempt_sequence;
    if (!p_demand->attempt_sequence)
    {
        ++p_demand->attempt_sequence;
    }
    p_demand->attempt =
        (rb_attempt_info_t){.sequence = p_demand->attempt_sequence,
                            .code = 0,
                            .reason = RB_ATTEMPT_IN_PROGRESS,
                            .bars = p_demand->peak.nbars,
                            .flags = RB_ATTEMPT_VALID};
    p_demand->raw_count = p_demand->raw_head = 0;
    p_demand->raw_overwritten = 0;
    p_demand->raw_capture_active = true;
    p_demand->raw_frozen = false;
    unsigned begin = (p_demand->preroll_head + RB_RAW_PREROLL_ROWS -
                      p_demand->preroll_count) %
                     RB_RAW_PREROLL_ROWS;
    unsigned first = p_demand->preroll_count;
    unsigned contiguous = RB_RAW_PREROLL_ROWS - begin;
    if (first > contiguous)
    {
        first = contiguous;
    }
    if (first)
    {
        memcpy(p_demand->raw, p_demand->preroll + begin,
               first * sizeof(rb_raw_t));
    }
    if (p_demand->preroll_count > first)
    {
        memcpy(p_demand->raw + first, p_demand->preroll,
               (p_demand->preroll_count - first) * sizeof(rb_raw_t));
    }
    p_demand->raw_count = p_demand->preroll_count;
    p_demand->attempt.raw_count = p_demand->preroll_count;
    p_demand->raw_head = p_demand->preroll_count % RB_RAW_ROWS;
    if (p_demand->preroll_count)
    {
        p_demand->attempt.first_ms = rb_get32(p_demand->raw[0].bytes);
        p_demand->attempt.last_ms =
            rb_get32(p_demand->raw[p_demand->preroll_count - 1].bytes);
    }
    if (p_demand->preroll_count)
    {
        const rb_raw_t * p_oldest = &p_demand->preroll[begin];
        if ((isfinite(first_bar_mm)) &&
            (rb_raw_position(p_oldest) <= first_bar_mm))
        {
            p_demand->attempt.flags |= RB_ATTEMPT_START_VALID;
        }
        else
        {
            p_demand->attempt.flags |= RB_ATTEMPT_TRUNCATED;
        }
    }
    else
    {
        p_demand->attempt.flags |= RB_ATTEMPT_TRUNCATED;
    }
    if (original_success)
    {
        p_demand->attempt.flags |=
            RB_ATTEMPT_ORIGINAL_ACCEPTED | RB_ATTEMPT_DECODED;
    }
    if ((original_success && (p_count->last_code >= 'A')) &&
        (p_count->last_code <= 'Z'))
    {
        p_demand->attempt.code = (uint8_t)p_count->last_code;
    }
}
static inline void rb_attempt_finish (rb_decoder_t * p_demand,
                                      const bp_result_t * p_result,
                                      bool finalized)
{
    if (!p_demand->raw_capture_active)
    {
        return;
    }
    p_demand->attempt.reason =
        finalized ? p_result->reason : RB_ATTEMPT_IN_PROGRESS;
    p_demand->attempt.bars = p_result->nbars;
    if (p_result->decoded)
    {
        p_demand->attempt.code = (uint8_t)p_result->code;
        p_demand->attempt.flags |= RB_ATTEMPT_DECODED;
    }
    if (finalized)
    {
        p_demand->attempt.flags |= RB_ATTEMPT_FINALIZED;
    }
    else
    {
        p_demand->attempt.flags |= RB_ATTEMPT_TRUNCATED;
    }
    p_demand->attempt.raw_count = (uint16_t)p_demand->raw_count;
    p_demand->raw_capture_active = false;
    p_demand->raw_frozen = true;
}
static inline void rb_attempt_original_success (rb_decoder_t * p_demand,
                                                barcode_nav_t * p_count)
{
    double first_bar_mm = p_demand->peak.nbars ? p_demand->peak.bar[0].mm : NAN;
    rb_attempt_begin(p_demand, p_count, first_bar_mm, true);
    p_demand->attempt.flags |=
        RB_ATTEMPT_ORIGINAL_ACCEPTED | RB_ATTEMPT_DECODED;
    if ((p_count->last_code >= 'A') && (p_count->last_code <= 'Z'))
    {
        p_demand->attempt.code = (uint8_t)p_count->last_code;
    }
}
static inline void rb_attempt_interrupt (rb_decoder_t * p_demand)
{
    if (!p_demand->raw_capture_active)
    {
        return;
    }
    bp_result_t partial = {.decoded = false,
                           .code = '?',
                           .reason = RB_ATTEMPT_IN_PROGRESS,
                           .nbars = p_demand->peak.nbars};
    rb_attempt_finish(p_demand, &partial, false);
}
static inline void rb_attempt_pack (const rb_attempt_info_t * p_m_value,
                                    uint32_t uptime_ms, uint16_t run_id,
                                    uint8_t seq,
                                    uint8_t frame[RB_ATTEMPT_FRAME])
{
    memset(frame, 0, RB_ATTEMPT_FRAME);
    frame[0] = 0xF2;
    frame[1] = RB_ATTEMPT_PAYLOAD;
    frame[2] = seq;
    uint8_t * p_data = frame + 3;
    p_data[0] = 0xC5;
    rb_put32(p_data + 1, uptime_ms);
    rb_put32(p_data + 5, p_m_value->sequence);
    rb_put32(p_data + 9, p_m_value->first_ms);
    rb_put32(p_data + 13, p_m_value->last_ms);
    rb_put32(p_data + 17, p_m_value->raw_overwritten);
    p_data[21] = (uint8_t)run_id;
    p_data[22] = (uint8_t)(run_id >> 8);
    unsigned code = (p_m_value->code >= 'A') && (p_m_value->code <= 'Z')
                        ? p_m_value->code - 'A' + 1U
                        : 0U;
    unsigned reason = p_m_value->reason <= RB_ATTEMPT_IN_PROGRESS
                          ? p_m_value->reason
                          : RB_ATTEMPT_IN_PROGRESS;
    unsigned bars =
        p_m_value->bars > BP_MAX_BARS ? BP_MAX_BARS : p_m_value->bars;
    unsigned count =
        p_m_value->raw_count > RB_RAW_ROWS ? RB_RAW_ROWS : p_m_value->raw_count;
    uint32_t meta = (p_m_value->flags & 0x3fU) | (code << 6) | (reason << 11) |
                    (bars << 15) | (count << 20);
    rb_put32(p_data + 23, meta);
}
/* source15 C7 reports only on diagnostic changes/normal telemetry cadence.
 * Width is signed centi-mm, saturated symmetrically to +/-327.67mm. */
static inline void rb_junction_pack (const rb_junction_sensor_t * p_other_data,
                                     bool pending, uint32_t uptime_ms,
                                     uint16_t run_id, uint8_t seq,
                                     uint8_t frame[RB_JUNCTION_FRAME])
{
    memset(frame, 0, RB_JUNCTION_FRAME);
    frame[0] = 0xF2;
    frame[1] = RB_JUNCTION_PAYLOAD;
    frame[2] = seq;
    uint8_t * p_data = frame + 3;
    p_data[0] = 0xC7;
    rb_put32(p_data + 1, uptime_ms);
    rb_put32(p_data + 5, p_other_data->decision_serial);
    p_data[9] = (uint8_t)run_id;
    p_data[10] = (uint8_t)(run_id >> 8);
    unsigned threshold = p_other_data->decision_serial
                             ? p_other_data->decision_threshold
                             : p_other_data->threshold;
    unsigned hysteresis = p_other_data->decision_serial
                              ? p_other_data->decision_hysteresis
                              : p_other_data->hysteresis;
    bool valid = p_other_data->decision_serial ? p_other_data->decision_valid
                                               : p_other_data->valid;
    bool armed = p_other_data->decision_serial ? p_other_data->decision_armed
                                               : p_other_data->armed;
    p_data[11] = (uint8_t)threshold;
    p_data[12] =
        (uint8_t)(((threshold >> 8) & 0x0fU) | ((hysteresis & 0x0fU) << 4));
    p_data[13] = (uint8_t)((hysteresis >> 4) & 0x0fU);
    uint16_t width = (uint16_t)p_other_data->last_width_centi_mm;
    p_data[14] = (uint8_t)width;
    p_data[15] = (uint8_t)(width >> 8);
    unsigned result =
        p_other_data->last_result <= RB_JUNCTION_RESULT_WHITE_CANCELLED
            ? p_other_data->last_result
            : 0;
    p_data[16] =
        (uint8_t)(result | ((valid ? 1U : 0U) << 3) | ((armed ? 1U : 0U) << 4) |
                  ((pending ? 1U : 0U) << 5) |
                  ((p_other_data->last_candidate ? 1U : 0U) << 6) |
                  ((p_other_data->last_width_saturated ? 1U : 0U) << 7));
}
static inline void rb_raw (rb_decoder_t * p_demand,
                           const barcode_nav_t * p_count, uint16_t adc,
                           uint64_t now, double mm_value)
{
    ++p_demand->samples;
    if ((p_demand->raw_have_time) &&
        (now - p_demand->raw_last_us < RB_RAW_INTERVAL_US))
    {
        return;
    }
    p_demand->raw_have_time = true;
    p_demand->raw_last_us = now;
    rb_raw_t sample;
    rb_raw_encode(&sample, p_count, adc, now, mm_value);
    rb_raw_store_preroll(p_demand, &sample);
    if (p_demand->raw_capture_active)
    {
        rb_raw_append_capture(p_demand, &sample);
    }
}
static inline void rb_partial_reset (rb_decoder_t * p_demand,
                                     barcode_nav_t * p_count, uint16_t adc,
                                     uint64_t now, const double wheel[2])
{
    barcode_analog_begin(&p_demand->analog, p_count, adc, now, wheel);
    p_count->external_cross = true;
    rb_junction_reset(&p_demand->junction, p_count, adc, now,
                      (wheel[0] + wheel[1]) * .5);
    if (p_demand->junction.valid)
    {
        p_demand->analog.black = p_demand->junction.black;
    }
    bp_reset(&p_demand->peak);
    p_count->cross_candidate = p_count->cross_ready = p_count->allow_cross =
        false;
    p_count->leading_gate = p_count->cross_gate = false;
    p_count->leading_us = 0;
    barcode_reader_reset(&p_count->reader, p_demand->analog.black, now,
                         (wheel[0] + wheel[1]) * .5);
}
static inline void rb_begin (rb_decoder_t * p_demand, barcode_nav_t * p_count,
                             uint16_t adc, uint64_t now, const double wheel[2])
{
    memset(p_demand, 0, sizeof(*p_demand));
    rb_partial_reset(p_demand, p_count, adc, now, wheel);
    p_demand->last_peak.code = '?';
    p_demand->last_peak.reason = RB_PEAK_RUN_START;
    p_demand->peak_flags = 1;
    ++p_demand->peak_serial;
}
static inline void rb_sample (rb_decoder_t * p_demand, barcode_nav_t * p_count,
                              uint16_t adc, uint64_t now, const double wheel[2])
{
    double mm_value = (wheel[0] + wheel[1]) * .5;
    rb_raw(p_demand, p_count, adc, now, mm_value);
    uint32_t before = p_count->decoded_total;
    barcode_analog_sample(&p_demand->analog, p_count, adc, now, wheel);
    bool original_success = p_count->decoded_total != before;
    if (original_success)
    {
        rb_attempt_original_success(p_demand, p_count);
        rb_junction_learn(&p_demand->junction, p_count,
                          p_demand->analog.minimum, p_demand->analog.maximum,
                          adc, now, mm_value);
    }
    rb_junction_sample(&p_demand->junction, p_count, adc, now, wheel);
    if (p_demand->junction.valid)
    {
        p_demand->analog.black = p_demand->junction.black;
    }
    if (p_count->busy)
    {
        rb_attempt_interrupt(p_demand);
        bp_reset(&p_demand->peak);
        return;
    }
    bp_result_t result;
    bool group_ended = bp_sample(&p_demand->peak, adc, mm_value, &result);
    if ((!p_demand->raw_capture_active) &&
        (p_demand->peak.nbars >= BP_REPORT_MIN_BARS))
    {
        rb_attempt_begin(p_demand, p_count, p_demand->peak.bar[0].mm,
                         original_success);
    }
    if ((group_ended && (result.nbars >= BP_REPORT_MIN_BARS)) &&
        (!p_demand->raw_capture_active))
    {
        rb_attempt_begin(p_demand, p_count, result.start_mm, original_success);
    }
    if (!group_ended)
    {
        if (p_demand->raw_capture_active)
        {
            p_demand->attempt.bars = p_demand->peak.nbars;
        }
        return;
    }
    if (result.nbars < BP_REPORT_MIN_BARS)
    {
        if (p_demand->raw_capture_active)
        {
            rb_attempt_finish(p_demand, &result, true);
        }
        return;
    }
    if (p_demand->groups < 65535)
    {
        ++p_demand->groups;
    }
    p_demand->last_peak = result;
    ++p_demand->peak_serial;
    p_demand->peak_flags = 1u | (result.decoded ? 2u : 0u) |
                           (result.reverse ? 16u : 0u) |
                           (result.overflow ? 64u : 0u);
    rb_attempt_finish(p_demand, &result, true);
    if (!result.decoded)
    {
        return;
    }
    ++p_demand->peak_decodes;
    bool overlap =
        ((p_demand->analog.accepted) &&
         (fabs(result.start_mm - p_demand->analog.accepted_start) <= 12)) &&
        (fabs(result.end_mm - p_demand->analog.accepted_end) <= 12);
    if (overlap)
    {
        p_demand->peak_flags |= 8u;
        if (p_count->last_code != result.code)
        {
            p_demand->peak_flags |= 32u;
            ++p_demand->peak_conflicts;
            if (p_count->pending == p_count->last_code)
            {
                p_count->pending = 0;
            }
            p_count->cross_candidate = p_count->cross_ready = false;
            p_demand->analog.conflict = true;
            p_demand->analog.conflict_end = mm_value;
            barcode_event(p_count, B_ANALOG_CONFLICT, now, mm_value, 0,
                          result.code, 0, 0x40u | result.reverse,
                          result.pattern);
        }
    }
    else
    {
        uint32_t count = p_count->decoded_total;
        barcode_queue_code(
            p_count,
            result.code); /* Buddy3 -> nav_accept_barcode -> coordinator. */
        if (p_count->decoded_total == count)
        {
            return; /* Inactive/takeover rejected by coordinator. */
        }
        p_count->cross_candidate = p_count->cross_ready = false;
        p_demand->analog.accepted = true;
        p_demand->analog.accepted_start = result.start_mm;
        p_demand->analog.accepted_end = result.end_mm;
        analog_bank_t * p_value =
            &p_demand->analog.bank[p_demand->analog.selected];
        barcode_reader_reset(&p_value->reader, p_value->stable, now, mm_value);
        rb_junction_learn(&p_demand->junction, p_count, result.base,
                          result.max_wide_peak, adc, now, mm_value);
        if (p_demand->junction.valid)
        {
            p_demand->analog.black = p_demand->junction.black;
        }
        barcode_event(p_count, B_DECODE, now, mm_value,
                      result.end_mm - result.start_mm, result.code,
                      barcode_decode_action(p_count, result.code),
                      0x40u | result.reverse, result.pattern);
        ++p_demand->peak_applied;
        p_demand->peak_flags |= 4u;
    }
    p_demand->raw_frozen = true;
}
static inline analog_demand_t
rb_follow_with_cruise (line_planner_t * p_data, rb_decoder_t * p_second,
                       const line_calibration_t * p_cal,
                       const circuit_input_t * p_index, float delta_time,
                       unsigned cruise_speed_mm_s)
{
    unsigned before = p_data->circuit.phase, stage = p_data->nav.uturn.stage;
    smooth_follow_t legacy = {
        0}; /* Profile1 never uses the legacy profile2 adapter. */
    analog_demand_t demand = continuous_follow_step_with_cruise(
        &p_data->circuit, &p_data->nav, p_cal, p_index, RL_PROFILE, &legacy,
        &p_second->analog, &p_data->scan, &p_data->steering, delta_time,
        cruise_speed_mm_s);
    rl_junction_observe(p_data, p_cal, p_index, before, stage);
    p_data->optical = demand;
    return demand;
}
static inline analog_demand_t rb_follow (line_planner_t * p_data,
                                         rb_decoder_t * p_second,
                                         const line_calibration_t * p_cal,
                                         const circuit_input_t * p_index,
                                         float delta_time)
{
    return rb_follow_with_cruise(p_data, p_second, p_cal, p_index, delta_time,
                                 DRIVE_MODE_BASELINE_SPEED);
}
/* Preserve completed events/counts and the action/revision captured at decode;
 * discard partial frames, cross edges and interrupted tracking state. */
static inline void rb_resume (line_planner_t * p_data, rb_decoder_t * p_demand,
                              const circuit_input_t * p_index, uint16_t adc)
{
    rb_attempt_interrupt(p_demand);
    p_demand->preroll_count = p_demand->preroll_head = 0;
    p_demand->raw_have_time = false;
    circuit_begin(&p_data->circuit, p_index);
    p_data->steering = (continuous_steering_t){0};
    p_data->scan = (scan_smooth_t){0};
    p_data->junction = (line_junction_diagnostic_t){0};
    p_data->nav.busy = false;
    p_data->nav.action = 0;
    p_data->nav.executing_action = 0;
    p_data->nav.executing_revision = 0;
    p_data->nav.uturn = (barcode_u_turn_t){0};
    rb_partial_reset(p_demand, &p_data->nav, adc, p_index->now, p_index->wheel);
    p_data->black = p_demand->analog.black;
    p_data->side_adc = adc;
}
#endif

/* End of file. */
