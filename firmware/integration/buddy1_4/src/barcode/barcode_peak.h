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
#define BP_SMOOTH_MM 0.5   /* moving average over this much travel ... */
#define BP_SMOOTH_MIN 5    /* ... and at least this many samples */
#define BP_DELTA 4.0       /* ADC: minimum prominence / extremum hysteresis */
#define BP_DELTA_FRAC 0.05 /* ... or 5 % of the peak rise (tall flat wide-bar tops never split) */
#define BP_MIN_RISE 20.0   /* ADC above white baseline for a bar peak */
#define BP_MIN_SEP_MM                                                                              \
    3.0 /* closer peaks are one bar (keep the higher); true bars are >= 2 units (~6 mm) apart */
#define BP_QUIET_MM                                                                                \
    24.0 /* no bar for this travel ends a group (max in-frame spacing ~18 mm; 21 if a weak bar is missed) */
#define BP_EARLY_QUIET_MM 12.0 /* with >= 15 bars, try once after this much quiet (low latency) */
#define BP_MAX_BARS 24
#define BP_BASE_BIN_MM 5.0 /* white baseline = min over 12 x 5 mm bins */
#define BP_BASE_BINS 12
#define BP_NARROW_FRAC 0.15 /* bar rise < 15 % of tallest = narrow */
#define BP_WIDE_FRAC 0.50   /* bar rise > 50 % of tallest = wide; between = reject */
#define BP_NARROW_CONSIST                                                                          \
    0.35 /* every narrow bar must rise >= 45 % of the frame's median narrow rise */
#define BP_WIDE_MIN_RISE 300.0 /* tallest bar must rise this much (wide bars exist) */
#define BP_UNIT_MIN_MM 2.0     /* accepted narrow width range */
#define BP_UNIT_MAX_MM 4.5
#define BP_SPACE_CUT 2.0      /* space wide if > 2 narrow units ... */
#define BP_SPACE_AMBIG 0.4    /* ... reject if within +/-0.4 of the cut */
#define BP_REPAIR_MAX_BARS 18 /* groups of 16..18 bars: try dropping the weakest narrow bars */
#define BP_REPAIR_CANDIDATES 4
#define BP_REPORT_MIN_BARS 6 /* groups smaller than this (junctions, noise) are not reported */

enum {
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

typedef struct {
    double mm;
    double peak;
    double rise;
    double prom;
} BpBar; /* prom = min(left, right) drop */

typedef struct {
    double rmm[BP_RING];
    uint16_t radc[BP_RING];
    uint8_t rhead, rcount;
    double bin_min[BP_BASE_BINS], bin_start_mm;
    uint8_t bin_head;
    bool base_init;
    bool rising, last_open, early_tried;
    double ext, ext_mm, valley;
    BpBar bar[BP_MAX_BARS];
    uint8_t nbars;
    bool overflow;
    double last_mm;
    bool have_mm;
} BarcodePeak;

typedef struct {
    bool decoded, reverse, overflow;
    char code;
    uint8_t reason, nbars, offset, removed;
    double start_mm, end_mm, unit_mm, base;
    double max_narrow_space, min_wide_space; /* in narrow units */
    double min_narrow_rise, min_narrow_peak, max_wide_peak;
    uint16_t pattern[3];
} BpResult;

static inline void bp_reset(BarcodePeak *p) {
    memset(p, 0, sizeof(*p));
}

/* Same 9-bit Code39 table/bit order as barcode_scan.h (element 0 = MSB). */
static inline char bp_character(uint16_t pattern) {
    static const char alphabet[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    static const uint16_t table[] = {0x034, 0x121, 0x061, 0x160, 0x031, 0x130, 0x070, 0x025, 0x124,
                                     0x064, 0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00d, 0x10c,
                                     0x04c, 0x01c, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007,
                                     0x106, 0x046, 0x016, 0x181, 0x0c1, 0x1c0, 0x091, 0x190, 0x0d0,
                                     0x085, 0x184, 0x0c4, 0x0a8, 0x0a2, 0x08a, 0x02a};
    for (unsigned n = 0; n < 43; ++n)
        if (table[n] == pattern)
            return alphabet[n];
    return pattern == 0x094 ? '*' : '?';
}

static inline double bp_base(const BarcodePeak *p) {
    double b = p->bin_min[0];
    for (unsigned i = 1; i < BP_BASE_BINS; ++i)
        if (p->bin_min[i] < b)
            b = p->bin_min[i];
    return b;
}

/* Decode bars[off..off+14]. Returns BP_OK and fills r on success. */
static inline uint8_t bp_window(const BpBar *bar, unsigned off, BpResult *r) {
    const BpBar *b = bar + off;
    double maxrise = 0;
    for (unsigned i = 0; i < 15; ++i)
        if (b[i].rise > maxrise)
            maxrise = b[i].rise;
    if (maxrise < BP_WIDE_MIN_RISE)
        return BP_R_NO_WIDE;
    bool wide[29];
    unsigned nwide = 0;
    double min_nrise = 1e9, min_npeak = 1e9, max_wpeak = 0;
    for (unsigned i = 0; i < 15; ++i) {
        double f = b[i].rise / maxrise;
        if (f > BP_NARROW_FRAC && f < BP_WIDE_FRAC)
            return BP_R_WIDTH_CLASS;
        wide[2 * i] = f >= BP_WIDE_FRAC;
        if (wide[2 * i]) {
            ++nwide;
            if (b[i].peak > max_wpeak)
                max_wpeak = b[i].peak;
        } else {
            if (b[i].rise < min_nrise)
                min_nrise = b[i].rise;
            if (b[i].peak < min_npeak)
                min_npeak = b[i].peak;
        }
    }
    if (nwide != 6)
        return BP_R_WIDE_COUNT;
    double nr[9];
    unsigned nn = 0; /* median narrow rise (insertion sort, 9 values) */
    for (unsigned i = 0; i < 15; ++i)
        if (!wide[2 * i]) {
            unsigned j = nn++;
            while (j > 0 && nr[j - 1] > b[i].rise) {
                nr[j] = nr[j - 1];
                --j;
            }
            nr[j] = b[i].rise;
        }
    if (min_nrise < BP_NARROW_CONSIST * nr[nn / 2])
        return BP_R_WIDTH_CLASS;
    double u = (b[14].mm - b[0].mm) / 46.0; /* *X*: 47 units, minus half of the two end bars */
    if (u < BP_UNIT_MIN_MM || u > BP_UNIT_MAX_MM)
        return BP_R_UNIT;
    double max_ns = 0, min_ws = 1e9;
    for (unsigned i = 0; i < 14; ++i) {
        double sp =
            (b[i + 1].mm - b[i].mm) / u - ((wide[2 * i] ? 3 : 1) + (wide[2 * i + 2] ? 3 : 1)) * 0.5;
        if (sp < 0.2 || fabs(sp - BP_SPACE_CUT) < BP_SPACE_AMBIG || sp > 5.0)
            return BP_R_SPACE;
        wide[2 * i + 1] = sp > BP_SPACE_CUT;
        if (wide[2 * i + 1]) {
            if (sp < min_ws)
                min_ws = sp;
        } else if (sp > max_ns)
            max_ns = sp;
    }
    if (wide[9] || wide[19])
        return BP_R_GAP;
    char c[2][3];
    uint16_t pat[2][3];
    for (unsigned dir = 0; dir < 2; ++dir)
        for (unsigned ch = 0; ch < 3; ++ch) {
            uint16_t p = 0;
            for (unsigned k = 0; k < 9; ++k) {
                unsigned idx = ch * 10 + k;
                p = (uint16_t)((p << 1) | (wide[dir ? 28 - idx : idx] ? 1u : 0u));
            }
            pat[dir][ch] = p;
            c[dir][ch] = bp_character(p);
        }
    bool ok[2];
    for (unsigned dir = 0; dir < 2; ++dir)
        ok[dir] = c[dir][0] == '*' && c[dir][2] == '*' && c[dir][1] != '*' && c[dir][1] != '?';
    if (ok[0] == ok[1])
        return ok[0] ? BP_R_AMBIGUOUS : BP_R_PATTERN;
    unsigned dir = ok[1];
    r->decoded = true;
    r->reverse = dir;
    r->code = c[dir][1];
    r->offset = (uint8_t)off;
    r->start_mm = b[0].mm - 0.5 * u;
    r->end_mm = b[14].mm + 0.5 * u;
    r->unit_mm = u;
    r->max_narrow_space = max_ns;
    r->min_wide_space = min_ws;
    r->min_narrow_rise = min_nrise;
    r->min_narrow_peak = min_npeak;
    r->max_wide_peak = max_wpeak;
    memcpy(r->pattern, pat[dir], sizeof(r->pattern));
    return BP_OK;
}

/* Try every 15-bar window of bars[0..n). Collects into *best; returns number of valid windows
 * (sets *conflict if two windows disagree on the code). */
static inline unsigned bp_windows(const BpBar *bars, unsigned n, BpResult *best,
                                  const BpResult *proto, uint8_t *first_reason, bool *conflict) {
    unsigned found = 0;
    for (unsigned off = 0; off + 15 <= n; ++off) {
        BpResult t = *proto;
        uint8_t why = bp_window(bars, off, &t);
        if (why == BP_OK) {
            if (found && t.code != best->code)
                *conflict = true;
            if (!found)
                *best = t;
            ++found;
        } else if (off == 0 && first_reason)
            *first_reason = why;
    }
    return found;
}
/* Decode the current group. Exactly one code must result. If no window of the raw group decodes
 * and it has 16..18 bars, retry without 1, then 2, of the weakest-prominence narrow bars (noise
 * bumps); every successful removal must agree on the same code. */
static inline void bp_decode(const BarcodePeak *p, BpResult *r) {
    memset(r, 0, sizeof(*r));
    r->nbars = p->nbars;
    r->overflow = p->overflow;
    r->code = '?';
    r->base = bp_base(p);
    if (p->nbars) {
        r->start_mm = p->bar[0].mm;
        r->end_mm = p->bar[p->nbars - 1].mm;
    }
    if (p->nbars < 15) {
        r->reason = BP_R_FEW;
        return;
    }
    uint8_t first_reason = BP_R_PATTERN;
    bool conflict = false;
    BpResult best = *r;
    unsigned found = bp_windows(p->bar, p->nbars, &best, r, &first_reason, &conflict);
    if (!found && p->nbars > 15 && p->nbars <= BP_REPAIR_MAX_BARS) {
        double maxrise = 0;
        for (unsigned i = 0; i < p->nbars; ++i)
            if (p->bar[i].rise > maxrise)
                maxrise = p->bar[i].rise;
        unsigned cand[BP_REPAIR_CANDIDATES], nc = 0; /* weakest narrow bars, ascending prominence */
        for (unsigned i = 0; i < p->nbars; ++i) {
            if (p->bar[i].rise >= BP_NARROW_FRAC * maxrise)
                continue;
            unsigned j = nc < BP_REPAIR_CANDIDATES ? nc++ : BP_REPAIR_CANDIDATES;
            if (j == BP_REPAIR_CANDIDATES) {
                if (p->bar[i].prom >= p->bar[cand[j - 1]].prom)
                    continue;
                j = j - 1;
            }
            while (j > 0 && p->bar[cand[j - 1]].prom > p->bar[i].prom) {
                cand[j] = cand[j - 1];
                --j;
            }
            cand[j] = i;
        }
        for (unsigned drop = 1; drop <= 2 && !found && p->nbars - drop >= 15; ++drop)
            for (unsigned a = 0; a < nc; ++a)
                for (unsigned b = drop == 2 ? a + 1 : nc; b < nc || (drop == 1 && b == nc); ++b) {
                    static BpBar tmp[BP_MAX_BARS];
                    unsigned m = 0; /* static: keeps ~0.8 KB off the core-0 stack (single caller) */
                    for (unsigned i = 0; i < p->nbars; ++i)
                        if (i != cand[a] && !(drop == 2 && i == cand[b]))
                            tmp[m++] = p->bar[i];
                    BpResult t = best;
                    unsigned f = bp_windows(tmp, m, &t, r, NULL, &conflict);
                    if (f) {
                        if (found && t.code != best.code)
                            conflict = true;
                        if (!found) {
                            best = t;
                            best.removed = (uint8_t)drop;
                        }
                        found += f;
                    }
                    if (drop == 1)
                        break;
                }
    }
    if (conflict) {
        r->reason = BP_R_AMBIGUOUS;
        return;
    }
    if (!found) {
        r->reason = first_reason;
        return;
    }
    *r = best;
    r->reason = BP_OK;
}

static inline void bp_add_bar(BarcodePeak *p, double mm, double peak, double rise, double prom) {
    if (p->nbars && mm - p->bar[p->nbars - 1].mm < BP_MIN_SEP_MM) {
        BpBar *l = &p->bar[p->nbars - 1];
        if (peak > l->peak) {
            l->mm = mm;
            l->peak = peak;
            l->rise = rise;
            l->prom = prom;
        }
        p->last_open = true;
        return;
    }
    if (p->nbars >= BP_MAX_BARS) {
        p->overflow = true;
        p->last_open = false;
        return;
    }
    p->bar[p->nbars++] = (BpBar){mm, peak, rise, prom};
    p->last_open = true;
    p->early_tried = false;
}
static inline void bp_close_right(BarcodePeak *p, double valley) {
    if (!p->last_open || !p->nbars)
        return;
    BpBar *l = &p->bar[p->nbars - 1];
    if (l->peak - valley < l->prom)
        l->prom = l->peak - valley;
    p->last_open = false;
}

/* One ADC sample (about every 2 ms) with the mean wheel path in mm.
 * Returns true when a bar group ended; *out then holds the attempt. */
static inline bool bp_sample(BarcodePeak *p, uint16_t adc, double mm, BpResult *out) {
    if (p->have_mm && mm < p->last_mm)
        mm = p->last_mm; /* never step backwards */
    p->last_mm = mm;
    p->have_mm = true;
    p->rmm[p->rhead] = mm;
    p->radc[p->rhead] = adc;
    p->rhead = (uint8_t)((p->rhead + 1) % BP_RING);
    if (p->rcount < BP_RING)
        ++p->rcount;
    uint32_t sum = 0;
    unsigned n = 0;
    for (unsigned k = 0; k < p->rcount; ++k) {
        unsigned i = (p->rhead + BP_RING - 1 - k) % BP_RING;
        if (n >= BP_SMOOTH_MIN && p->rmm[i] < mm - BP_SMOOTH_MM)
            break;
        sum += p->radc[i];
        ++n;
    }
    double s = (double)sum / n;
    if (!p->base_init) {
        for (unsigned i = 0; i < BP_BASE_BINS; ++i)
            p->bin_min[i] = s;
        p->bin_start_mm = mm;
        p->base_init = true;
        p->rising = false;
        p->ext = s;
        p->ext_mm = mm;
        p->valley = s;
    }
    while (mm - p->bin_start_mm >= BP_BASE_BIN_MM) {
        p->bin_head = (uint8_t)((p->bin_head + 1) % BP_BASE_BINS);
        p->bin_min[p->bin_head] = s;
        p->bin_start_mm += BP_BASE_BIN_MM;
        if (mm - p->bin_start_mm > BP_BASE_BIN_MM * BP_BASE_BINS)
            p->bin_start_mm = mm;
    }
    if (s < p->bin_min[p->bin_head])
        p->bin_min[p->bin_head] = s;
    double base = bp_base(p);
    if (p->rising) {
        if (s > p->ext) {
            p->ext = s;
            p->ext_mm = mm;
        } else if (s <= p->ext - fmax(BP_DELTA, BP_DELTA_FRAC * (p->ext - base))) {
            if (p->ext >= base + BP_MIN_RISE)
                bp_add_bar(p, p->ext_mm, p->ext, p->ext - base, p->ext - p->valley);
            p->rising = false;
            p->ext = s;
            p->ext_mm = mm;
        }
    } else {
        if (s < p->ext) {
            p->ext = s;
            p->ext_mm = mm;
        } else if (s >= p->ext + BP_DELTA) {
            bp_close_right(p, p->ext);
            p->valley = p->ext;
            p->rising = true;
            p->ext = s;
            p->ext_mm = mm;
        }
    }
    bool pending = p->rising && p->ext >= base + BP_MIN_RISE; /* a bar peak is still forming */
    if (p->nbars && !pending) {
        double quiet = mm - p->bar[p->nbars - 1].mm;
        bool end = quiet > BP_QUIET_MM,
             early = !end && !p->early_tried && p->nbars >= 15 && quiet > BP_EARLY_QUIET_MM;
        if (end || early) {
            if (!p->rising)
                bp_close_right(p, p->ext);
            bp_decode(p, out);
            if (early) {
                p->early_tried = true;
                if (!out->decoded)
                    return false;
            } /* not yet: wait for full quiet */
            p->nbars = 0;
            p->overflow = false;
            p->early_tried = false;
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
 * 10 u16 white baseline ADC  12 u16 lowest narrow-bar peak ADC  14 u16 highest wide-bar peak ADC
 * 16 u8 lowest narrow-bar rise ADC (255 max)  17 u8 max narrow space x20 (units)
 * 18 u8 min wide space x20 (units)  19 u8 narrow unit mm x50
 * 20 u16 frame span mm x10   22 u16 groups reported since run start */
#define BP_TLM_PAYLOAD 24
#define BP_TLM_FRAME (3 + BP_TLM_PAYLOAD)
static inline uint8_t bp_u8(double v) {
    return (uint8_t)(v < 0 ? 0 : v > 255 ? 255 : v + 0.5);
}
static inline uint16_t bp_u16(double v) {
    return (uint16_t)(v < 0 ? 0 : v > 65535 ? 65535 : v + 0.5);
}
static inline void bp_put16(uint8_t *b, uint16_t v) {
    b[0] = (uint8_t)v;
    b[1] = (uint8_t)(v >> 8);
}
static inline void bp_tlm_pack(const BpResult *r, uint32_t uptime_ms, uint8_t flags,
                               uint16_t groups, uint8_t seq, uint8_t f[BP_TLM_FRAME]) {
    uint8_t *b = f + 3;
    memset(f, 0, BP_TLM_FRAME);
    f[0] = 0xF2;
    f[1] = BP_TLM_PAYLOAD;
    f[2] = seq;
    b[0] = (uint8_t)uptime_ms;
    b[1] = (uint8_t)(uptime_ms >> 8);
    b[2] = (uint8_t)(uptime_ms >> 16);
    b[3] = (uint8_t)(uptime_ms >> 24);
    b[4] = 0xB0 | BP_VERSION;
    b[5] = flags;
    b[6] = (uint8_t)(r->decoded ? r->code : '?');
    b[7] = r->reason;
    b[8] = r->nbars;
    b[9] = (uint8_t)((r->offset & 15u) | ((r->removed & 3u) << 4));
    bp_put16(b + 10, bp_u16(r->base));
    bp_put16(b + 12, bp_u16(r->decoded ? r->min_narrow_peak : 0));
    bp_put16(b + 14, bp_u16(r->decoded ? r->max_wide_peak : 0));
    b[16] = bp_u8(r->decoded ? r->min_narrow_rise : 0);
    b[17] = bp_u8(r->decoded ? r->max_narrow_space * 20 : 0);
    b[18] = bp_u8(r->decoded ? r->min_wide_space * 20 : 0);
    b[19] = bp_u8(r->decoded ? r->unit_mm * 50 : 0);
    bp_put16(b + 20, bp_u16((r->end_mm - r->start_mm) * 10));
    bp_put16(b + 22, groups);
}
#endif
