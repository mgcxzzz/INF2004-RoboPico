/**
 * @file barcode_scan.h
 * @brief Barcode scan.
 * @details Pure barcode/route logic is called by the ADC owner and emits requests.
 * Native records are not wire packets; preserve both decode directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_SCAN_H
#define BARCODE_SCAN_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

enum
{
    B_RUN = 1,
    B_FRAME_START,
    B_DECODE,
    B_BAD_FRAME,
    B_CROSS,
    B_NO_COMMAND,
    B_DISPATCH,
    B_COMPLETE,
    B_GATE_REJECT,
    B_QUEUE_OVERFLOW,
    B_CROSS_CANDIDATE,
    B_CROSS_CANCEL,
    B_ANALOG_CANDIDATE,
    B_ANALOG_SELECT,
    B_ANALOG_CONFLICT,
    B_ANALOG_GAP,
    B_UTURN_STAGE,
    B_RECOVERY_WAIT,
    B_RECOVERY_SWEEP,
    B_RECOVERY_FOUND,
    B_RECOVERY_FAIL,
    B_HOLD_BEGIN,
    B_HOLD_PATTERN,
    B_HOLD_END,
    B_HOLD_CANCEL,
    B_SOFT_BEGIN,
    B_SOFT_END,
    B_EVENTS = 192,
    B_ROWS =
#ifdef BUDDY_WIFI_LOGGING
        384,
#else
        1000,
#endif
    B_EDGE_QUEUE = 256
};
enum
{
    B_REJECT_WIDTH = 1,
    B_REJECT_SEPARATION = 2,
    B_REJECT_COUNT = 4,
    B_REJECT_GAP = 8,
    B_REJECT_PATTERN = 16,
    B_REJECT_TIMEOUT = 32,
    B_REJECT_RUN = 64
};
typedef struct
{
    uint64_t ms;
    uint32_t path10;
    int32_t width100;
    uint32_t number;
    uint16_t pattern[3];
    uint8_t type;
    uint8_t code;
    uint8_t action;
    uint8_t flags;
    uint8_t map_revision;
    uint8_t reserved;
} barcode_event_t;
_Static_assert(sizeof(barcode_event_t) == 32, "Barcode event ABI");
typedef struct
{
    /* Index0=black bars,1=white gaps. D0 can expand black and shrink white,
       so comparing their widths in a single sorted list is not reliable. */
    double narrow[2];
    double wide[2];
    double threshold[2];
    uint16_t pattern;
    bool valid;
} barcode_group_t;
/* Standard Code39 encodings, checked against the supplied A/Z PDF and
   ZXing's published Code39Reader table. Decoder/classifier below is local. */
static inline char barcode_character (uint16_t pattern)
{
    static const char alphabet[] =
        "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    static const uint16_t table[] = {
        0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124,
        0x064, 0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00d, 0x10c,
        0x04c, 0x01c, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007,
        0x106, 0x046, 0x016, 0x181, 0x0c1, 0x1c0, 0x091, 0x190, 0x0d0,
        0x085, 0x184, 0x0c4, 0x0a8, 0x0a2, 0x08a, 0x02a};
    for (unsigned count = 0; count < 43; ++count)
    {
        if (table[count] == pattern)
        {
            return alphabet[count];
        }
    }
    return pattern == 0x094 ? '*' : '?';
}
static inline uint16_t barcode_reverse9 (uint16_t value_x)
{
    uint16_t value_y = 0;
    for (unsigned count = 0; count < 9; ++count)
    {
        value_y = (uint16_t)((value_y << 1) | (value_x & 1));
        value_x >>= 1;
    }
    return value_y;
}
static inline barcode_group_t barcode_group (const double * p_wheel)
{
    barcode_group_t g_value = {0};
    /* Sentinel * (either direction) has two wide black bars and one wide
       white gap. Learn each colour independently from that structure. */
    for (unsigned colour = 0; colour < 2; ++colour)
    {
        double sorted[5];
        unsigned count = 0;
        for (unsigned count_value = colour; count_value < 9; count_value += 2)
        {
            sorted[count++] = p_wheel[count_value];
        }
        for (unsigned count_value_value = 1; count_value_value < count;
             ++count_value_value)
        {
            double value = sorted[count_value_value];
            unsigned other_index = count_value_value;
            while (other_index && sorted[other_index - 1] > value)
            {
                sorted[other_index] = sorted[other_index - 1];
                --other_index;
            }
            sorted[other_index] = value;
        }
        unsigned wide_count = colour ? 1 : 2, narrow_count = count - wide_count;
        if (sorted[0] < 0.35 || sorted[count - 1] > 40 ||
            sorted[narrow_count] < sorted[narrow_count - 1] * 1.55 ||
            sorted[count - 1] > sorted[narrow_count] * 1.75)
        {
            return g_value;
        }
        for (unsigned count_value_value_value = 0;
             count_value_value_value < narrow_count; ++count_value_value_value)
        {
            g_value.narrow[colour] +=
                sorted[count_value_value_value] / narrow_count;
        }
        for (unsigned count_value_value_value_value = narrow_count;
             count_value_value_value_value < count;
             ++count_value_value_value_value)
        {
            g_value.wide[colour] +=
                sorted[count_value_value_value_value] / wide_count;
        }
        g_value.threshold[colour] =
            (sorted[narrow_count - 1] + sorted[narrow_count]) * 0.5;
        if (g_value.narrow[colour] > 14 ||
            g_value.wide[colour] < g_value.narrow[colour] * 1.8)
        {
            return g_value;
        }
    }
    for (unsigned count_value_value_value_value_value = 0;
         count_value_value_value_value_value < 9;
         ++count_value_value_value_value_value)
    {
        g_value.pattern =
            (uint16_t)((g_value.pattern << 1) |
                       (p_wheel[count_value_value_value_value_value] >
                        g_value.threshold[count_value_value_value_value_value %
                                          2]));
    }
    g_value.valid = true;
    return g_value;
}
static inline bool barcode_classify (const double * p_wheel,
                                     const barcode_group_t * p_reference,
                                     uint16_t * p_pattern,
                                     unsigned * p_rejection)
{
    unsigned wide_count = 0;
    *p_pattern = 0;
    double max_narrow[2] = {0}, min_wide[2] = {1e9, 1e9};
    for (unsigned count = 0; count < 9; ++count)
    {
        unsigned colour = count % 2;
        bool wide = p_wheel[count] > p_reference->threshold[colour];
        *p_pattern = (uint16_t)((*p_pattern << 1) | wide);
        if (p_wheel[count] < 0.35 || p_wheel[count] > 40)
        {
            *p_rejection |= B_REJECT_WIDTH;
        }
        if (wide)
        {
            ++wide_count;
            min_wide[colour] = fmin(min_wide[colour], p_wheel[count]);
            if (p_wheel[count] < p_reference->wide[colour] * .55 ||
                p_wheel[count] > p_reference->wide[colour] * 1.8)
            {
                *p_rejection |= B_REJECT_WIDTH;
            }
        }
        else
        {
            max_narrow[colour] = fmax(max_narrow[colour], p_wheel[count]);
            /* Learned colour threshold already defines the narrow class.
               Do not impose a second limit based on the sentinel's mean:
               partly shortened edge bars can bias that mean substantially. */
        }
    }
    for (unsigned colour = 0; colour < 2; ++colour)
    {
        if (max_narrow[colour] > 0 &&
            min_wide[colour] < max_narrow[colour] * 1.5)
        {
            *p_rejection |= B_REJECT_SEPARATION;
        }
    }
    if (wide_count != 3)
    {
        *p_rejection |= B_REJECT_COUNT;
    }
    return !*p_rejection;
}
typedef struct
{
    double widths[29];
    double history[9];
    double edge_mm;
    double start_mm;
    double max_frame_mm;
    barcode_group_t reference;
    uint64_t edge_us;
    uint64_t frame_us;
    uint8_t count;
    uint8_t history_count;
    bool initialized;
    bool black;
    bool frame;
    bool reverse;
} barcode_reader_t;
typedef struct
{
    unsigned events;
    unsigned rejection;
    bool completed_black;
    bool reverse;
    double width;
    double start;
    double end;
    char code;
    uint16_t pattern[3];
} barcode_read_t;
static inline void barcode_reader_reset (barcode_reader_t * p_result,
                                         bool black, uint64_t now,
                                         double mm_value)
{
    *p_result = (barcode_reader_t){.initialized = true,
                                   .black = black,
                                   .edge_us = now,
                                   .edge_mm = mm_value};
}
/* Completed runs use encoder distance, never a fixed bar time or fixed print
   length. Single-character frames have29 alternating elements: * + gap + X
   + gap + *. Both physical scan directions are accepted. */
static inline barcode_read_t barcode_reader_edge (barcode_reader_t * p_result,
                                                  bool black, uint64_t now,
                                                  double mm_value)
{
    barcode_read_t result = {0};
    if (!p_result->initialized)
    {
        barcode_reader_reset(p_result, black, now, mm_value);
        return result;
    }
    if (black == p_result->black)
    {
        return result;
    }
    bool previous = p_result->black;
    double width = mm_value - p_result->edge_mm;
    result.completed_black = previous;
    result.width = width;
    result.start = p_result->edge_mm;
    result.end = mm_value;
    p_result->black = black;
    p_result->edge_mm = mm_value;
    p_result->edge_us = now;
    result.events = 1u << B_RUN;
    if (width < 0.35 || (width > 40 && p_result->frame))
    {
        if (p_result->frame)
        {
            result.events |= 1u << B_BAD_FRAME;
            result.rejection = B_REJECT_RUN;
        }
        p_result->frame = false;
        p_result->count = p_result->history_count = 0;
        return result;
    }
    if (p_result->frame)
    {
        p_result->widths[p_result->count++] = width;
        if (p_result->count < 29)
        {
            return result;
        }
        bool good = true;
        for (unsigned count = 0; count < 3; ++count)
        {
            uint16_t pattern = 0;
            unsigned rejection = 0;
            if (!barcode_classify(p_result->widths + count * 10,
                                  &p_result->reference, &pattern, &rejection))
            {
                good = false;
            }
            result.rejection |= rejection;
            result.pattern[count] =
                p_result->reverse ? barcode_reverse9(pattern) : pattern;
        }
        for (unsigned count_value = 9; count_value <= 19; count_value += 10)
        {
            if (p_result->widths[count_value] <
                    fmax(.35, p_result->reference.narrow[1] * .25) ||
                p_result->widths[count_value] >
                    p_result->reference.threshold[1])
            {
                good = false;
                result.rejection |= B_REJECT_GAP;
            }
        }
        char code = barcode_character(result.pattern[1]);
        if (result.pattern[0] != 0x094 || result.pattern[2] != 0x094 ||
            code == '?' || code == '*')
        {
            good = false;
            result.rejection |= B_REJECT_PATTERN;
        }
        result.start = p_result->start_mm;
        result.reverse = p_result->reverse;
        result.code = good ? code : '?';
        result.events |= 1u << (good ? B_DECODE : B_BAD_FRAME);
        p_result->frame = false;
        p_result->count = p_result->history_count = 0;
        return result;
    }
    if (width > 40)
    {
        p_result->history_count = 0;
        return result;
    }
    if (p_result->history_count < 9)
    {
        p_result->history[p_result->history_count++] = width;
    }
    else
    {
        memmove(p_result->history, p_result->history + 1, 8 * sizeof(double));
        p_result->history[8] = width;
    }
    if (previous && p_result->history_count == 9)
    {
        barcode_group_t g_value = barcode_group(p_result->history);
        if (g_value.valid && (g_value.pattern == 0x094 ||
                              barcode_reverse9(g_value.pattern) == 0x094))
        {
            memcpy(p_result->widths, p_result->history,
                   sizeof(p_result->history));
            p_result->count = 9;
            p_result->frame = true;
            p_result->reverse = g_value.pattern != 0x094;
            p_result->frame_us = now;
            p_result->reference = g_value;
            double length = 0;
            for (unsigned count_value_value = 0; count_value_value < 9;
                 ++count_value_value)
            {
                length += p_result->history[count_value_value];
            }
            p_result->max_frame_mm =
                fmin(600, length * 4.5 + 2 * g_value.threshold[1]);
            p_result->start_mm = mm_value - length;
            result.start = p_result->start_mm;
            result.events |= 1u << B_FRAME_START;
        }
    }
    return result;
}
static inline bool barcode_reader_expire (barcode_reader_t * p_result,
                                          uint64_t now, double mm_value)
{
    if (p_result->frame &&
        (now - p_result->frame_us >= 20000000 ||
         mm_value - p_result->start_mm > p_result->max_frame_mm))
    {
        p_result->frame = false;
        p_result->count = p_result->history_count = 0;
        return true;
    }
    if (!p_result->frame && !p_result->black &&
        mm_value - p_result->edge_mm > 40)
    {
        p_result->history_count = 0;
    }
    return false;
}
#endif

/* End of file. */
