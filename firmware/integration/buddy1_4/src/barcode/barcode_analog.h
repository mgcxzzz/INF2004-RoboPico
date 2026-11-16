/**
 * @file barcode_analog.h
 * @brief Barcode analog.
 * @details Pure barcode/route logic is called by the ADC owner and emits requests.
 * Native records are not wire packets; preserve both decode directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_ANALOG_H
#define BARCODE_ANALOG_H
#include "src/barcode/barcode_nav.h"

/* No operator white/black calibration: search raw ADC thresholds, and select
 * only after two neighbouring thresholds decode the same complete Code39
 * frame. Existing measured module polarity is BLACK_HIGH. No narrow bars
 * are removed or inferred. Original decoder validates every element. */
enum
{
    AO_BANKS = 63,
    AO_STEP = 128,
    AO_HYSTERESIS = 32,
    AO_FINE_HYSTERESIS = 3,
    AO_DEFAULT_BANK = 11,
    AO_DEBOUNCE_US = 2000
}; /*212 ADC diagnostic preview */
typedef struct
{
    barcode_reader_t reader;
    bool stable;
    bool candidate;
    bool changed;
    bool was_frame;
    uint64_t candidate_us;
    uint64_t vote_us;
    double candidate_wheel[2];
    double vote_start;
    double vote_end;
    char vote;
    bool reverse;
    uint16_t pattern[3];
} analog_bank_t;
typedef struct
{
    analog_bank_t bank[AO_BANKS];
    uint16_t adc;
    uint16_t baseline;
    uint16_t minimum;
    uint16_t maximum;
    uint8_t selected;
    bool black;
    bool busy;
    bool scanning;
    bool accepted;
    bool conflict;
    uint64_t last_us;
    uint32_t max_gap_us;
    uint32_t gap_count;
    uint32_t candidates;
    uint32_t selections;
    double accepted_start;
    double accepted_end;
    double conflict_end;
} barcode_analog_t;
/* Sorted union: original128,256,...3968 plus192,194,...254. Fine banks
 * target the user's measured narrow gap203..206 and narrow bar215..221.
 * These are sensitivity hypotheses, not a hardcoded barcode or calibration. */
static inline unsigned barcode_analog_threshold (unsigned bank)
{
    return bank == 0    ? 128
           : bank <= 32 ? 190 + 2 * bank
                        : (bank - 31) * AO_STEP;
}
static inline unsigned barcode_analog_hysteresis (unsigned bank)
{
    return bank >= 1 && bank <= 32 ? AO_FINE_HYSTERESIS : AO_HYSTERESIS;
}
static inline unsigned barcode_analog_log_threshold (unsigned version,
                                                     unsigned bank)
{
    return version < 8 ? (bank + 1) * AO_STEP : barcode_analog_threshold(bank);
}
static inline void barcode_analog_begin (barcode_analog_t * p_first,
                                         barcode_nav_t * p_count, uint16_t adc,
                                         uint64_t now, const double wheel[2])
{
    *p_first = (barcode_analog_t){.adc = adc,
                                  .baseline = adc,
                                  .minimum = adc,
                                  .maximum = adc,
                                  .selected = AO_DEFAULT_BANK,
                                  .last_us = now};
    for (unsigned second = 0; second < AO_BANKS; ++second)
    {
        analog_bank_t * p_value = &p_first->bank[second];
        p_value->stable = p_value->candidate =
            adc >= barcode_analog_threshold(second);
        p_value->candidate_us = now;
        memcpy(p_value->candidate_wheel, wheel,
               sizeof(p_value->candidate_wheel));
        barcode_reader_reset(&p_value->reader, p_value->stable, now,
                             (wheel[0] + wheel[1]) * .5);
    }
    p_first->black = p_first->bank[p_first->selected].stable;
    p_count->external_reader = true;
    p_count->external_scanning = false;
}
static inline bool barcode_analog_same_frame (const analog_bank_t * p_value,
                                              const analog_bank_t * p_wheel)
{
    return p_value->vote && p_value->vote == p_wheel->vote &&
           p_value->reverse == p_wheel->reverse &&
           fabs(p_value->vote_start - p_wheel->vote_start) <= 6 &&
           fabs(p_value->vote_end - p_wheel->vote_end) <= 6 &&
           (p_value->vote_us > p_wheel->vote_us
                ? p_value->vote_us - p_wheel->vote_us
                : p_wheel->vote_us - p_value->vote_us) <= 500000;
}
static inline bool barcode_analog_select (barcode_analog_t * p_first,
                                          barcode_nav_t * p_count, uint64_t now,
                                          double mm_value)
{
    int winner = -1;
    char code = 0;
    bool conflict = false;
    double best = 1e9, centre = (p_first->minimum + p_first->maximum) * .5;
    for (unsigned second = 0; second + 1 < AO_BANKS; ++second)
    {
        const analog_bank_t *p_value = &p_first->bank[second],
                            *p_wheel = &p_first->bank[second + 1];
        if (!barcode_analog_same_frame(p_value, p_wheel) ||
            now < p_value->vote_us || now - p_value->vote_us > 500000 ||
            mm_value - p_value->vote_end > 10 || mm_value < p_value->vote_end)
        {
            continue;
        }
        if (p_first->accepted &&
            fabs(p_value->vote_start - p_first->accepted_start) <= 12 &&
            fabs(p_value->vote_end - p_first->accepted_end) <= 12)
        {
            continue;
        }
        if (winner >= 0 && code != p_value->vote)
        {
            conflict = true;
        }
        else if (winner < 0 ||
                 fabs(barcode_analog_threshold(second) - centre) < best)
        {
            winner = (int)second;
            code = p_value->vote;
            best = fabs(barcode_analog_threshold(second) - centre);
        }
    }
    if (winner < 0)
    {
        return false;
    }
    if (conflict)
    {
        if (!p_first->conflict || mm_value - p_first->conflict_end > 12)
        {
            barcode_event(p_count, B_ANALOG_CONFLICT, now, mm_value, 0, '?', 0,
                          0, NULL);
            p_first->conflict = true;
            p_first->conflict_end = mm_value;
        }
        p_count->pending = 0;
        p_count->cross_candidate = p_count->cross_ready = false;
        return false;
    }
    if (p_first->conflict && mm_value - p_first->conflict_end <= 12)
    {
        return false;
    }
    p_first->selected = (uint8_t)winner;
    const analog_bank_t * p_value_value = &p_first->bank[p_first->selected];
    p_first->black = p_value_value->stable;
    p_first->accepted = true;
    p_first->accepted_start = p_value_value->vote_start;
    p_first->accepted_end = p_value_value->vote_end;
    ++p_first->selections;
    p_count->cross_candidate = p_count->cross_ready = false;
    barcode_queue_code(p_count, code);
    barcode_event(p_count, B_ANALOG_SELECT, now, mm_value,
                  barcode_analog_threshold(p_first->selected), code,
                  barcode_decode_action(p_count, code), p_first->selected + 1,
                  p_value_value->pattern);
    barcode_event(p_count, B_DECODE, now, mm_value,
                  p_value_value->vote_end - p_value_value->vote_start, code,
                  barcode_decode_action(p_count, code),
                  0x80u | (p_first->selected << 1) | p_value_value->reverse,
                  p_value_value->pattern);
    return true;
}
static inline void barcode_analog_sample (barcode_analog_t * p_first,
                                          barcode_nav_t * p_count, uint16_t adc,
                                          uint64_t now, const double wheel[2])
{
    double mm_value = (wheel[0] + wheel[1]) * .5;
    p_first->adc = adc;
    if (adc < p_first->minimum)
    {
        p_first->minimum = adc;
    }
    if (adc > p_first->maximum)
    {
        p_first->maximum = adc;
    }
    if (p_count->busy)
    {
        if (!p_first->busy)
        {
            barcode_analog_begin(p_first, p_count, adc, now, wheel);
            p_first->busy = true;
        }
        p_first->last_us = now;
        return;
    }
    if (p_first->busy)
    {
        barcode_analog_begin(p_first, p_count, adc, now, wheel);
    }
    if (now > p_first->last_us)
    {
        uint64_t gap = now - p_first->last_us;
        if (gap > p_first->max_gap_us)
        {
            p_first->max_gap_us = (uint32_t)gap;
        }
        if (gap > 10000)
        {
            ++p_first->gap_count;
            if (p_first->gap_count <= 4)
            {
                barcode_event(p_count, B_ANALOG_GAP, now, mm_value,
                              (double)gap / 1000, 0, 0, 0, NULL);
            }
        }
    }
    p_first->last_us = now;
    p_first->scanning = false;
    for (unsigned second = 0; second < AO_BANKS; ++second)
    {
        analog_bank_t * p_value = &p_first->bank[second];
        unsigned sample = barcode_analog_threshold(second);
        p_value->changed = false;
        unsigned hysteresis = barcode_analog_hysteresis(second);
        bool black = p_value->stable ? adc > sample - hysteresis
                                     : adc >= sample + hysteresis;
        if (black != p_value->candidate)
        {
            p_value->candidate = black;
            p_value->candidate_us = now;
            memcpy(p_value->candidate_wheel, wheel,
                   sizeof(p_value->candidate_wheel));
        }
        if (p_value->candidate != p_value->stable &&
            now - p_value->candidate_us >= AO_DEBOUNCE_US)
        {
            p_value->stable = p_value->candidate;
            p_value->changed = true;
            p_value->was_frame = p_value->reader.frame;
            double edge_mm =
                (p_value->candidate_wheel[0] + p_value->candidate_wheel[1]) *
                .5;
            barcode_read_t result =
                barcode_reader_edge(&p_value->reader, p_value->stable,
                                    p_value->candidate_us, edge_mm);
            if (second == p_first->selected)
            {
                if (result.events & (1u << B_RUN))
                {
                    barcode_event(p_count, B_RUN, p_value->candidate_us,
                                  edge_mm, result.width, 0, 0,
                                  result.completed_black, NULL);
                }
                if (result.events & (1u << B_FRAME_START))
                {
                    barcode_event(p_count, B_FRAME_START, p_value->candidate_us,
                                  edge_mm, edge_mm - result.start, 0, 0,
                                  p_value->reader.reverse, NULL);
                }
                if (result.events & (1u << B_BAD_FRAME))
                {
                    ++p_count->rejected;
                    barcode_event(p_count, B_BAD_FRAME, p_value->candidate_us,
                                  edge_mm, edge_mm - result.start, '?', 0,
                                  result.rejection, result.pattern);
                }
            }
            if (result.events & (1u << B_DECODE))
            {
                p_value->vote = result.code;
                p_value->vote_us = p_value->candidate_us;
                p_value->vote_start = result.start;
                p_value->vote_end = edge_mm;
                p_value->reverse = result.reverse;
                memcpy(p_value->pattern, result.pattern,
                       sizeof(p_value->pattern));
                ++p_first->candidates;
                barcode_event(p_count, B_ANALOG_CANDIDATE,
                              p_value->candidate_us, edge_mm, sample,
                              result.code, 0, second + 1, result.pattern);
                if (p_first->accepted &&
                    fabs(result.start - p_first->accepted_start) <= 12 &&
                    fabs(edge_mm - p_first->accepted_end) <= 12 &&
                    result.code != p_count->last_code)
                {
                    p_first->conflict = true;
                    p_first->conflict_end = mm_value;
                    p_count->pending = 0;
                    p_count->cross_candidate = p_count->cross_ready = false;
                    barcode_event(p_count, B_ANALOG_CONFLICT, now, mm_value,
                                  sample, result.code, 0, second + 1,
                                  result.pattern);
                }
            }
        }
        if (barcode_reader_expire(&p_value->reader, now, mm_value) &&
            second == p_first->selected)
        {
            ++p_count->rejected;
            barcode_event(p_count, B_BAD_FRAME, now, mm_value,
                          mm_value - p_value->reader.start_mm, '?', 0,
                          B_REJECT_TIMEOUT, NULL);
        }
        if (p_value->reader.frame || p_value->reader.history_count ||
            (sample > p_first->baseline + 100u && p_value->stable))
        {
            p_first->scanning = true;
        }
    }
    bool selected = barcode_analog_select(p_first, p_count, now, mm_value);
    analog_bank_t * p_value_value = &p_first->bank[p_first->selected];
    p_first->black = p_value_value->stable;
    p_count->external_scanning = p_first->scanning;
    /* A cross is searched only after a verified command. The selecting edge
       is the final stop bar, and is never reused as the next junction. */
    if (p_count->external_cross || selected || !p_count->pending)
    {
        return;
    }
    if (p_value_value->reader.frame)
    {
        p_count->cross_candidate = false;
        return;
    }
    if (p_value_value->changed && !p_value_value->was_frame)
    {
        double edge_mm = (p_value_value->candidate_wheel[0] +
                          p_value_value->candidate_wheel[1]) *
                         .5;
        if (p_value_value->stable)
        {
            barcode_cross_confirm(p_count, p_value_value->candidate_us, edge_mm,
                                  true);
            if (p_count->cross_candidate)
            {
                p_count->cross_candidate = false;
                barcode_event(p_count, B_CROSS_CANCEL,
                              p_value_value->candidate_us, edge_mm,
                              edge_mm - p_count->cross_trailing_mm,
                              p_count->pending, 0, 0, NULL);
            }
            p_count->leading_us = p_value_value->candidate_us;
            memcpy(p_count->leading_wheel, p_value_value->candidate_wheel,
                   sizeof(p_count->leading_wheel));
            p_count->leading_gate = p_count->allow_cross;
        }
        else if (p_count->leading_us >= p_count->start_us &&
                 now >= p_count->leading_us &&
                 now - p_count->leading_us < 20000000)
        {
            double leading =
                       (p_count->leading_wheel[0] + p_count->leading_wheel[1]) *
                       .5,
                   width = edge_mm - leading;
            if (width >= B_CROSS_MIN_MM && width <= B_CROSS_MAX_MM)
            {
                p_count->cross_width = width;
                p_count->cross_us = p_count->leading_us;
                memcpy(p_count->cross_wheel, p_count->leading_wheel,
                       sizeof(p_count->cross_wheel));
                p_count->cross_gate = p_count->leading_gate;
                p_count->cross_candidate = true;
                p_count->cross_trailing_mm = edge_mm;
                barcode_event(
                    p_count, B_CROSS_CANDIDATE, p_value_value->candidate_us,
                    edge_mm, width, p_count->pending,
                    barcode_nav_action(p_count, p_count->pending), 0, NULL);
            }
        }
    }
    barcode_cross_confirm(p_count, now, mm_value, !p_first->black);
}
#endif

/* End of file. */
