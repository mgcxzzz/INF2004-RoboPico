/**
 * @file barcode_peak.h
 * @brief Barcode peak.
 * @details Pure barcode/route logic is called by the ADC owner and emits
 * requests. Native records are not wire packets; preserve both decode
 * directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Barcode peak decoder v0.1 (pure C, host-testable; no SDK).
 *
 * Why: on the current car the barcode IR (GP27) resolves 9 mm wide bars as
 * large peaks (~1700-2300 ADC) but 3 mm narrow bars only as small bumps
 * (~205-225 on a ~175 white level), while the gaps next to them dip to
 * ~190-209. A fixed ADC threshold (the original 63-bank decoder) then has no
 * margin. The positions of the bars are still clear, so this decoder:
 *
 *  1. smooths the ADC over ~0.5 mm of travel,
 *  2. finds every local maximum with >= BP_DELTA ADC prominence that rises
 *     >= BP_MIN_RISE above the tracked white baseline (= one bar),
 *  3. groups bars until BP_QUIET_MM of travel without a bar,
 *  4. classifies bars wide/narrow by peak height relative to the group's
 *     tallest bar, and spaces wide/narrow from bar-centre spacing
 *     (in units of the frame's own narrow width, so odometry scale cancels),
 *  5. accepts only a complete *X* Code39 frame (15 bars, 6 wide bars,
 *     unambiguous spaces, narrow inter-character gaps), either direction.
 *
 * Assumes full-size labels as printed (narrow ~3 mm). Characters with no wide
 * bars ($ / + %) are not supported. Diagnostic v0.1: shadow build only logs. */
#ifndef BARCODE_PEAK_H
#define BARCODE_PEAK_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define BP_VERSION 1
#define BP_RING 32
#define BP_SMOOTH_MM 0.5 /* moving average over this much travel ... */
#define BP_SMOOTH_MIN 5  /* ... and at least this many samples */
#define BP_DELTA 4.0     /* ADC: minimum prominence / extremum hysteresis */
#define BP_DELTA_FRAC                                                          \
    0.05 /* ... or 5 % of the peak rise (tall flat wide-bar tops never split)  \
          */
#define BP_MIN_RISE 20.0 /* ADC above white baseline for a bar peak */
#define BP_MIN_SEP_MM                                                          \
    3.0 /* closer peaks are one bar (keep the higher); true bars are >= 2      \
           units (~6 mm) apart */
#define BP_QUIET_MM                                                            \
    24.0 /* no bar for this travel ends a group (max in-frame spacing ~18 mm;  \
            21 if a weak bar is missed) */
#define BP_EARLY_QUIET_MM                                                      \
    12.0 /* with >= 15 bars, try once after this much quiet (low latency) */
#define BP_MAX_BARS 24
#define BP_BASE_BIN_MM 5.0 /* white baseline = min over 12 x 5 mm bins */
#define BP_BASE_BINS 12
#define BP_NARROW_FRAC 0.15 /* bar rise < 15 % of tallest = narrow */
#define BP_WIDE_FRAC                                                           \
    0.50 /* bar rise > 50 % of tallest = wide; between = reject */
#define BP_NARROW_CONSIST                                                      \
    0.35 /* every narrow bar must rise >= 45 % of the frame's median narrow    \
            rise */
#define BP_WIDE_MIN_RISE                                                       \
    300.0 /* tallest bar must rise this much (wide bars exist) */
#define BP_UNIT_MIN_MM 2.0 /* accepted narrow width range */
#define BP_UNIT_MAX_MM 4.5
#define BP_SPACE_CUT 2.0   /* space wide if > 2 narrow units ... */
#define BP_SPACE_AMBIG 0.4 /* ... reject if within +/-0.4 of the cut */
#define BP_REPAIR_MAX_BARS                                                     \
    18 /* groups of 16..18 bars: try dropping the weakest narrow bars */
#define BP_REPAIR_CANDIDATES 4
#define BP_REPORT_MIN_BARS                                                     \
    6 /* groups smaller than this (junctions, noise) are not reported */

enum
{
    BP_OK = 0,
    BP_R_FEW = 1,
    BP_R_WIDTH_CLASS = 2,
    BP_R_WIDE_COUNT = 3,
    BP_R_UNIT = 4,
    BP_R_SPACE = 5,
    BP_R_GAP = 6,
    BP_R_PATTERN = 7,
    BP_R_AMBIGUOUS = 8,
    BP_R_NO_WIDE = 9
};

typedef struct
{
    double mm;
    double peak;
    double rise;
    double prom;
} bp_bar_t; /* prom = min(left, right) drop */

typedef struct
{
    double rmm[BP_RING];
    uint16_t radc[BP_RING];
    uint8_t rhead;
    uint8_t rcount;
    double bin_min[BP_BASE_BINS];
    double bin_start_mm;
    uint8_t bin_head;
    bool base_init;
    bool rising;
    bool last_open;
    bool early_tried;
    double ext;
    double ext_mm;
    double valley;
    bp_bar_t bar[BP_MAX_BARS];
    uint8_t nbars;
    bool overflow;
    double last_mm;
    bool have_mm;
} barcode_peak_t;

typedef struct
{
    bool decoded;
    bool reverse;
    bool overflow;
    char code;
    uint8_t reason;
    uint8_t nbars;
    uint8_t offset;
    uint8_t removed;
    double start_mm;
    double end_mm;
    double unit_mm;
    double base;
    double max_narrow_space;
    double min_wide_space; /* in narrow units */
    double min_narrow_rise;
    double min_narrow_peak;
    double max_wide_peak;
    uint16_t pattern[3];
} bp_result_t;

static inline void bp_reset (barcode_peak_t * p_data)
{
    memset(p_data, 0, sizeof(*p_data));
}

/* Same 9-bit Code39 table/bit order as barcode_scan.h (element 0 = MSB). */
static inline char bp_character (uint16_t pattern)
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
    return 0x094 == pattern ? '*' : '?';
}

static inline double bp_base (const barcode_peak_t * p_data)
{
    double second = p_data->bin_min[0];
    for (unsigned index = 1; index < BP_BASE_BINS; ++index)
    {
        if (p_data->bin_min[index] < second)
        {
            second = p_data->bin_min[index];
        }
    }
    return second;
}

/* Decode bars[off..off+14]. Returns BP_OK and fills r on success. */
static inline uint8_t bp_window (const bp_bar_t * p_bar, unsigned off,
                                 bp_result_t * p_result)
{
    const bp_bar_t * p_second = p_bar + off;
    double maxrise = 0;
    for (unsigned index = 0; index < 15; ++index)
    {
        if (p_second[index].rise > maxrise)
        {
            maxrise = p_second[index].rise;
        }
    }
    if (maxrise < BP_WIDE_MIN_RISE)
    {
        return BP_R_NO_WIDE;
    }
    bool wide[29];
    unsigned nwide = 0;
    double min_nrise = 1e9, min_npeak = 1e9, max_wpeak = 0;
    for (unsigned index_value = 0; index_value < 15; ++index_value)
    {
        double flags = p_second[index_value].rise / maxrise;
        if ((flags > BP_NARROW_FRAC) && (flags < BP_WIDE_FRAC))
        {
            return BP_R_WIDTH_CLASS;
        }
        wide[2 * index_value] = flags >= BP_WIDE_FRAC;
        if (wide[2 * index_value])
        {
            ++nwide;
            if (p_second[index_value].peak > max_wpeak)
            {
                max_wpeak = p_second[index_value].peak;
            }
        }
        else
        {
            if (p_second[index_value].rise < min_nrise)
            {
                min_nrise = p_second[index_value].rise;
            }
            if (p_second[index_value].peak < min_npeak)
            {
                min_npeak = p_second[index_value].peak;
            }
        }
    }
    if (nwide != 6)
    {
        return BP_R_WIDE_COUNT;
    }
    double nr_value[9];
    unsigned nn_value = 0; /* median narrow rise (insertion sort, 9 values) */
    for (unsigned index_value_value = 0; index_value_value < 15;
         ++index_value_value)
    {
        if (!wide[2 * index_value_value])
        {
            unsigned other_index = nn_value++;
            while ((other_index > 0) && (nr_value[other_index - 1] >
                                         p_second[index_value_value].rise))
            {
                nr_value[other_index] = nr_value[other_index - 1];
                --other_index;
            }
            nr_value[other_index] = p_second[index_value_value].rise;
        }
    }
    if (min_nrise < BP_NARROW_CONSIST * nr_value[nn_value / 2])
    {
        return BP_R_WIDTH_CLASS;
    }
    double u_value = (p_second[14].mm - p_second[0].mm) /
                     46.0; /* *X*: 47 units, minus half of the two end bars */
    if ((u_value < BP_UNIT_MIN_MM) || (u_value > BP_UNIT_MAX_MM))
    {
        return BP_R_UNIT;
    }
    double max_ns = 0, min_ws = 1e9;
    for (unsigned index_value_value_value = 0; index_value_value_value < 14;
         ++index_value_value_value)
    {
        double sp_value = (p_second[index_value_value_value + 1].mm -
                           p_second[index_value_value_value].mm) /
                              u_value -
                          ((wide[2 * index_value_value_value] ? 3 : 1) +
                           (wide[2 * index_value_value_value + 2] ? 3 : 1)) *
                              0.5;
        if (((sp_value < 0.2) ||
             (fabs(sp_value - BP_SPACE_CUT) < BP_SPACE_AMBIG)) ||
            (sp_value > 5.0))
        {
            return BP_R_SPACE;
        }
        wide[2 * index_value_value_value + 1] = sp_value > BP_SPACE_CUT;
        if (wide[2 * index_value_value_value + 1])
        {
            if (sp_value < min_ws)
            {
                min_ws = sp_value;
            }
        }
        else if (sp_value > max_ns)
        {
            max_ns = sp_value;
        }
    }
    if ((wide[9]) || (wide[19]))
    {
        return BP_R_GAP;
    }
    char character[2][3];
    uint16_t pat[2][3];
    for (unsigned dir = 0; dir < 2; ++dir)
    {
        for (unsigned ch_value = 0; ch_value < 3; ++ch_value)
        {
            uint16_t data = 0;
            for (unsigned item_index = 0; item_index < 9; ++item_index)
            {
                unsigned idx = ch_value * 10 + item_index;
                data = (uint16_t)((data << 1) |
                                  (wide[dir ? 28 - idx : idx] ? 1u : 0u));
            }
            pat[dir][ch_value] = data;
            character[dir][ch_value] = bp_character(data);
        }
    }
    bool ok_value[2];
    for (unsigned dir = 0; dir < 2; ++dir)
    {
        ok_value[dir] = '*' == character[dir][0] && '*' == character[dir][2] &&
                        (character[dir][1] != '*') &&
                        (character[dir][1] != '?');
    }
    if (ok_value[0] == ok_value[1])
    {
        return ok_value[0] ? BP_R_AMBIGUOUS : BP_R_PATTERN;
    }
    unsigned dir = ok_value[1];
    p_result->decoded = true;
    p_result->reverse = dir;
    p_result->code = character[dir][1];
    p_result->offset = (uint8_t)off;
    p_result->start_mm = p_second[0].mm - 0.5 * u_value;
    p_result->end_mm = p_second[14].mm + 0.5 * u_value;
    p_result->unit_mm = u_value;
    p_result->max_narrow_space = max_ns;
    p_result->min_wide_space = min_ws;
    p_result->min_narrow_rise = min_nrise;
    p_result->min_narrow_peak = min_npeak;
    p_result->max_wide_peak = max_wpeak;
    memcpy(p_result->pattern, pat[dir], sizeof(p_result->pattern));
    return BP_OK;
}

/* Try every 15-bar window of bars[0..n). Collects into *best; returns number of
 * valid windows (sets *conflict if two windows disagree on the code). */
static inline unsigned bp_windows (const bp_bar_t * p_bars, unsigned count,
                                   bp_result_t * p_best,
                                   const bp_result_t * p_proto,
                                   uint8_t * p_first_reason, bool * p_conflict)
{
    unsigned found = 0;
    for (unsigned off = 0; off + 15 <= count; ++off)
    {
        bp_result_t sample = *p_proto;
        uint8_t why = bp_window(p_bars, off, &sample);
        if (BP_OK == why)
        {
            if (found && (sample.code != p_best->code))
            {
                *p_conflict = true;
            }
            if (!found)
            {
                *p_best = sample;
            }
            ++found;
        }
        else if (0 == off && p_first_reason)
        {
            *p_first_reason = why;
        }
    }
    return found;
}
/* Decode the current group. Exactly one code must result. If no window of the
 * raw group decodes and it has 16..18 bars, retry without 1, then 2, of the
 * weakest-prominence narrow bars (noise bumps); every successful removal must
 * agree on the same code. */
static inline void bp_decode (const barcode_peak_t * p_data,
                              bp_result_t * p_result)
{
    memset(p_result, 0, sizeof(*p_result));
    p_result->nbars = p_data->nbars;
    p_result->overflow = p_data->overflow;
    p_result->code = '?';
    p_result->base = bp_base(p_data);
    if (p_data->nbars)
    {
        p_result->start_mm = p_data->bar[0].mm;
        p_result->end_mm = p_data->bar[p_data->nbars - 1].mm;
    }
    if (p_data->nbars < 15)
    {
        p_result->reason = BP_R_FEW;
        return;
    }
    uint8_t first_reason = BP_R_PATTERN;
    bool conflict = false;
    bp_result_t best = *p_result;
    unsigned found = bp_windows(p_data->bar, p_data->nbars, &best, p_result,
                                &first_reason, &conflict);
    if (((!found) && (p_data->nbars > 15)) &&
        (p_data->nbars <= BP_REPAIR_MAX_BARS))
    {
        double maxrise = 0;
        for (unsigned index = 0; index < p_data->nbars; ++index)
        {
            if (p_data->bar[index].rise > maxrise)
            {
                maxrise = p_data->bar[index].rise;
            }
        }
        unsigned cand[BP_REPAIR_CANDIDATES],
            nc_value = 0; /* weakest narrow bars, ascending prominence */
        for (unsigned index_value = 0; index_value < p_data->nbars;
             ++index_value)
        {
            if (p_data->bar[index_value].rise >= BP_NARROW_FRAC * maxrise)
            {
                continue;
            }
            unsigned other_index = nc_value < BP_REPAIR_CANDIDATES
                                       ? nc_value++
                                       : BP_REPAIR_CANDIDATES;
            if (BP_REPAIR_CANDIDATES == other_index)
            {
                if (p_data->bar[index_value].prom >=
                    p_data->bar[cand[other_index - 1]].prom)
                {
                    continue;
                }
                other_index = other_index - 1;
            }
            while ((other_index > 0) &&
                   (p_data->bar[cand[other_index - 1]].prom >
                    p_data->bar[index_value].prom))
            {
                cand[other_index] = cand[other_index - 1];
                --other_index;
            }
            cand[other_index] = index_value;
        }
        for (unsigned drop = 1;
             ((drop <= 2) && (!found)) && (p_data->nbars - drop >= 15); ++drop)
        {
            for (unsigned first = 0; first < nc_value; ++first)
            {
                for (unsigned second = 2 == drop ? first + 1 : nc_value;
                     (second < nc_value) || (1 == drop && (second == nc_value));
                     ++second)
                {
                    static bp_bar_t tmp[BP_MAX_BARS];
                    unsigned m_value = 0; /* static: keeps ~0.8 KB off the
                                             core-0 stack (single caller) */
                    for (unsigned index_value_value = 0;
                         index_value_value < p_data->nbars; ++index_value_value)
                    {
                        if ((index_value_value != cand[first]) &&
                            !(2 == drop && (index_value_value == cand[second])))
                        {
                            tmp[m_value++] = p_data->bar[index_value_value];
                        }
                    }
                    bp_result_t sample = best;
                    unsigned flags = bp_windows(tmp, m_value, &sample, p_result,
                                                NULL, &conflict);
                    if (flags)
                    {
                        if (found && (sample.code != best.code))
                        {
                            conflict = true;
                        }
                        if (!found)
                        {
                            best = sample;
                            best.removed = (uint8_t)drop;
                        }
                        found += flags;
                    }
                    if (1 == drop)
                    {
                        break;
                    }
                }
            }
        }
    }
    if (conflict)
    {
        p_result->reason = BP_R_AMBIGUOUS;
        return;
    }
    if (!found)
    {
        p_result->reason = first_reason;
        return;
    }
    *p_result = best;
    p_result->reason = BP_OK;
}

static inline void bp_add_bar (barcode_peak_t * p_data, double mm_value,
                               double peak, double rise, double prom)
{
    if ((p_data->nbars) &&
        (mm_value - p_data->bar[p_data->nbars - 1].mm < BP_MIN_SEP_MM))
    {
        bp_bar_t * p_l_value = &p_data->bar[p_data->nbars - 1];
        if (peak > p_l_value->peak)
        {
            p_l_value->mm = mm_value;
            p_l_value->peak = peak;
            p_l_value->rise = rise;
            p_l_value->prom = prom;
        }
        p_data->last_open = true;
        return;
    }
    if (p_data->nbars >= BP_MAX_BARS)
    {
        p_data->overflow = true;
        p_data->last_open = false;
        return;
    }
    p_data->bar[p_data->nbars++] = (bp_bar_t){mm_value, peak, rise, prom};
    p_data->last_open = true;
    p_data->early_tried = false;
}
static inline void bp_close_right (barcode_peak_t * p_data, double valley)
{
    if ((!p_data->last_open) || (!p_data->nbars))
    {
        return;
    }
    bp_bar_t * p_l_value = &p_data->bar[p_data->nbars - 1];
    if (p_l_value->peak - valley < p_l_value->prom)
    {
        p_l_value->prom = p_l_value->peak - valley;
    }
    p_data->last_open = false;
}

/* One ADC sample (about every 2 ms) with the mean wheel path in mm.
 * Returns true when a bar group ended; *out then holds the attempt. */
static inline bool bp_sample (barcode_peak_t * p_data, uint16_t adc,
                              double mm_value, bp_result_t * p_out)
{
    if ((p_data->have_mm) && (mm_value < p_data->last_mm))
    {
        mm_value = p_data->last_mm; /* never step backwards */
    }
    p_data->last_mm = mm_value;
    p_data->have_mm = true;
    p_data->rmm[p_data->rhead] = mm_value;
    p_data->radc[p_data->rhead] = adc;
    p_data->rhead = (uint8_t)((p_data->rhead + 1) % BP_RING);
    if (p_data->rcount < BP_RING)
    {
        ++p_data->rcount;
    }
    uint32_t sum = 0;
    unsigned count = 0;
    for (unsigned item_index = 0; item_index < p_data->rcount; ++item_index)
    {
        unsigned index = (p_data->rhead + BP_RING - 1 - item_index) % BP_RING;
        if ((count >= BP_SMOOTH_MIN) &&
            (p_data->rmm[index] < mm_value - BP_SMOOTH_MM))
        {
            break;
        }
        sum += p_data->radc[index];
        ++count;
    }
    double state = (double)sum / count;
    if (!p_data->base_init)
    {
        for (unsigned index_value = 0; index_value < BP_BASE_BINS;
             ++index_value)
        {
            p_data->bin_min[index_value] = state;
        }
        p_data->bin_start_mm = mm_value;
        p_data->base_init = true;
        p_data->rising = false;
        p_data->ext = state;
        p_data->ext_mm = mm_value;
        p_data->valley = state;
    }
    while (mm_value - p_data->bin_start_mm >= BP_BASE_BIN_MM)
    {
        p_data->bin_head = (uint8_t)((p_data->bin_head + 1) % BP_BASE_BINS);
        p_data->bin_min[p_data->bin_head] = state;
        p_data->bin_start_mm += BP_BASE_BIN_MM;
        if (mm_value - p_data->bin_start_mm > BP_BASE_BIN_MM * BP_BASE_BINS)
        {
            p_data->bin_start_mm = mm_value;
        }
    }
    if (state < p_data->bin_min[p_data->bin_head])
    {
        p_data->bin_min[p_data->bin_head] = state;
    }
    double base = bp_base(p_data);
    if (p_data->rising)
    {
        if (state > p_data->ext)
        {
            p_data->ext = state;
            p_data->ext_mm = mm_value;
        }
        else if (state <=
                 p_data->ext -
                     fmax(BP_DELTA, BP_DELTA_FRAC * (p_data->ext - base)))
        {
            if (p_data->ext >= base + BP_MIN_RISE)
            {
                bp_add_bar(p_data, p_data->ext_mm, p_data->ext,
                           p_data->ext - base, p_data->ext - p_data->valley);
            }
            p_data->rising = false;
            p_data->ext = state;
            p_data->ext_mm = mm_value;
        }
    }
    else
    {
        if (state < p_data->ext)
        {
            p_data->ext = state;
            p_data->ext_mm = mm_value;
        }
        else if (state >= p_data->ext + BP_DELTA)
        {
            bp_close_right(p_data, p_data->ext);
            p_data->valley = p_data->ext;
            p_data->rising = true;
            p_data->ext = state;
            p_data->ext_mm = mm_value;
        }
    }
    bool pending =
        (p_data->rising) &&
        (p_data->ext >= base + BP_MIN_RISE); /* a bar peak is still forming */
    if ((p_data->nbars) && (!pending))
    {
        double quiet = mm_value - p_data->bar[p_data->nbars - 1].mm;
        bool end = quiet > BP_QUIET_MM,
             early = (((!end) && (!p_data->early_tried)) &&
                      (p_data->nbars >= 15)) &&
                     (quiet > BP_EARLY_QUIET_MM);
        if (end || early)
        {
            if (!p_data->rising)
            {
                bp_close_right(p_data, p_data->ext);
            }
            bp_decode(p_data, p_out);
            if (early)
            {
                p_data->early_tried = true;
                if (!p_out->decoded)
                {
                    return false;
                }
            } /* not yet: wait for full quiet */
            p_data->nbars = 0;
            p_data->overflow = false;
            p_data->early_tried = false;
            return true;
        }
    }
    return false;
}

/* Source-15 barcode-peak telemetry: frame = [0xF2, 24, seq] + 24-byte payload.
 *  0 u32 uptime_ms   4 u8 subtype 0xB1 (= peak decoder v0.1)
 *  5 u8 flags: bit0 active build, bit1 decoded, bit2 applied (queued by peak),
 *     bit3 original decoder already accepted this frame, bit4 reverse,
 *     bit5 original accepted a DIFFERENT code here, bit6 bar overflow
 *  6 u8 code ('?' none)   7 u8 reason (BP_*)   8 u8 bars in group
 *  9 u8 window offset (bits0-3) | weak bars removed by repair (bits4-5)
 * 10 u16 white baseline ADC  12 u16 lowest narrow-bar peak ADC  14 u16 highest
 * wide-bar peak ADC 16 u8 lowest narrow-bar rise ADC (255 max)  17 u8 max
 * narrow space x20 (units) 18 u8 min wide space x20 (units)  19 u8 narrow unit
 * mm x50 20 u16 frame span mm x10   22 u16 groups reported since run start */
#define BP_TLM_PAYLOAD 24
#define BP_TLM_FRAME (3 + BP_TLM_PAYLOAD)
static inline uint8_t bp_u8 (double value)
{
    return (uint8_t)(value < 0 ? 0 : value > 255 ? 255 : value + 0.5);
}
static inline uint16_t bp_u16 (double value)
{
    return (uint16_t)(value < 0 ? 0 : value > 65535 ? 65535 : value + 0.5);
}
static inline void bp_put16 (uint8_t * p_second, uint16_t value)
{
    p_second[0] = (uint8_t)value;
    p_second[1] = (uint8_t)(value >> 8);
}
static inline void bp_tlm_pack (const bp_result_t * p_result,
                                uint32_t uptime_ms, uint8_t flags,
                                uint16_t groups, uint8_t seq,
                                uint8_t flags_value[BP_TLM_FRAME])
{
    uint8_t * p_second = flags_value + 3;
    memset(flags_value, 0, BP_TLM_FRAME);
    flags_value[0] = 0xF2;
    flags_value[1] = BP_TLM_PAYLOAD;
    flags_value[2] = seq;
    p_second[0] = (uint8_t)uptime_ms;
    p_second[1] = (uint8_t)(uptime_ms >> 8);
    p_second[2] = (uint8_t)(uptime_ms >> 16);
    p_second[3] = (uint8_t)(uptime_ms >> 24);
    p_second[4] = 0xB0 | BP_VERSION;
    p_second[5] = flags;
    p_second[6] = (uint8_t)(p_result->decoded ? p_result->code : '?');
    p_second[7] = p_result->reason;
    p_second[8] = p_result->nbars;
    p_second[9] =
        (uint8_t)((p_result->offset & 15u) | ((p_result->removed & 3u) << 4));
    bp_put16(p_second + 10, bp_u16(p_result->base));
    bp_put16(p_second + 12,
             bp_u16(p_result->decoded ? p_result->min_narrow_peak : 0));
    bp_put16(p_second + 14,
             bp_u16(p_result->decoded ? p_result->max_wide_peak : 0));
    p_second[16] = bp_u8(p_result->decoded ? p_result->min_narrow_rise : 0);
    p_second[17] =
        bp_u8(p_result->decoded ? p_result->max_narrow_space * 20 : 0);
    p_second[18] = bp_u8(p_result->decoded ? p_result->min_wide_space * 20 : 0);
    p_second[19] = bp_u8(p_result->decoded ? p_result->unit_mm * 50 : 0);
    bp_put16(p_second + 20,
             bp_u16((p_result->end_mm - p_result->start_mm) * 10));
    bp_put16(p_second + 22, groups);
}
#endif

/* End of file. */
