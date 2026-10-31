#ifndef BUDDY_DRIVE_DIAG_H
#define BUDDY_DRIVE_DIAG_H
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
/* Controller-private canonical bit stream; never used as a motor input.
 * 0.5-unit quantisation for speeds/PI, integer ADC/PWM/ticks. */
#define DIAG_FIELDS(X)                                                                             \
    X(US, 32, 0)                                                                                   \
    X(DT, 17, 0)                                                                                   \
    X(COST, 17, 0) X(RAW_L, 12, 0) X(RAW_R, 12, 0) X(ADC_L, 12, 0) X(ADC_R, 12, 0)                 \
        X(LEVEL_L, 10, 0) X(LEVEL_R, 10, 0) X(OPT_L, 9, 1) X(OPT_R, 9, 1) X(SOFT_L, 9, 1)          \
            X(SOFT_R, 9, 1) X(TARGET_L, 9, 1) X(TARGET_R, 9, 1) X(SPEED_L, 12, 1)                  \
                X(SPEED_R, 12, 1) X(REF_BEFORE_L, 9, 0) X(REF_BEFORE_R, 9, 0) X(REF_L, 9, 0)       \
                    X(REF_R, 9, 0) X(I_BEFORE_L, 11, 1) X(I_BEFORE_R, 11, 1) X(I_L, 11, 1)         \
                        X(I_R, 11, 1) X(FF_L, 10, 0) X(FF_R, 10, 0) X(REQUEST_L, 11, 1)            \
                            X(REQUEST_R, 11, 1) X(PWM_L, 11, 1) X(PWM_R, 11, 1) X(TICKS_L, 12, 1)  \
                                X(TICKS_R, 12, 1) X(PHASE, 4, 0) X(OPT_DECISION, 5, 0)             \
                                    X(DECISION, 5, 0) X(PROFILE, 2, 0) X(ASSIST, 3, 0)             \
                                        X(FEEDBACK, 2, 0) X(DRIVER, 2, 0) X(EXTERNAL, 1, 0)        \
                                            X(FLAGS, 19, 0) X(MEAN_BEFORE, 9, 0)                   \
                                                X(HALF_BEFORE, 9, 1) X(MEAN, 9, 0) X(HALF, 9, 1)   \
                                                    X(GAIN, 3, 0) X(DAMP, 2, 0)
enum {
#define DIAG_ENUM(n, w, s) D_##n,
    DIAG_FIELDS(DIAG_ENUM)
#undef DIAG_ENUM
        D_FIELDS
};
static const uint8_t diag_width[D_FIELDS] = {
#define DIAG_WIDTH(n, w, s) w,
    DIAG_FIELDS(DIAG_WIDTH)
#undef DIAG_WIDTH
};
static const uint8_t diag_signed[D_FIELDS] = {
#define DIAG_SIGN(n, w, s) s,
    DIAG_FIELDS(DIAG_SIGN)
#undef DIAG_SIGN
};
enum {
    DIAG_ROWS = 256,
    DIAG_ROW_BYTES = 58,
    DIAG_POST = 128,
    DIAG_HEADER = 256,
    DIAG_BANK = 16384,
    DF_FILTER = 1,
    DF_URGENT = 2,
    DF_SCAN = 4,
    DF_ACTIVE_BEFORE = 8,
    DF_ACTIVE = 16,
    DF_WHITE = 32,
    DF_PREPARE = 64,
    DF_LEAVE_FAST = 128,
    DF_KICK_L = 256,
    DF_KICK_R = 512,
    DF_PEAK_L = 1024,
    DF_PEAK_R = 2048,
    DF_LOADED_L = 4096,
    DF_LOADED_R = 8192,
    DF_COMMIT = 16384,
    DF_CLIPPED = 32768,
    DF_OPTICAL = 65536,
    DF_RUN_ACTIVE = 131072,
    DF_HANDOVER_KEEP = 262144
};
typedef struct {
    uint8_t bytes[DIAG_ROW_BYTES];
} DiagRow;
typedef struct {
    int64_t v[D_FIELDS];
} DiagValues;
typedef struct {
    DiagRow rows[DIAG_ROWS];
    uint64_t start_us, duration_us;
    int64_t ticks[2];
    uint32_t total, max_gap, max_cost, max_pack, late20, trigger_us, overwritten;
    uint16_t count, head, post;
    uint16_t white[2], black[2];
    uint8_t profile, reason, trigger;
    bool begun, frozen, triggered, previous_valid;
    int16_t previous_pwm[2], previous_half;
} DriveDiag;
_Static_assert(DIAG_ROWS * sizeof(DiagRow) + DIAG_HEADER <= DIAG_BANK, "Diagnostic bank overflow");
_Static_assert(DIAG_ROWS * sizeof(DiagRow) % 256 == 0, "Diagnostic payload must be page aligned");
static inline int64_t diag_half(float x) {
    return isfinite(x) ? (int64_t)lroundf(x * 2) : INT64_MAX;
}
static inline void diag_set_bits(DiagRow *r, unsigned bit, unsigned width, uint32_t x) {
    for (unsigned k = 0; k < width; ++k) {
        unsigned p = bit + k;
        uint8_t mask = (uint8_t)(1u << (p % 8));
        if ((x >> k) & 1u)
            r->bytes[p / 8] |= mask;
        else
            r->bytes[p / 8] &= (uint8_t)~mask;
    }
}
static inline uint32_t diag_get_bits(const DiagRow *r, unsigned bit, unsigned width) {
    uint32_t x = 0;
    for (unsigned k = 0; k < width; ++k)
        x |= (uint32_t)((r->bytes[(bit + k) / 8] >> ((bit + k) % 8)) & 1u) << k;
    return x;
}
static inline void diag_pack(DiagRow *r, const DiagValues *values) {
    DiagValues a = *values;
    bool clipped = false;
    for (unsigned n = 0; n < D_FIELDS; ++n) {
        int64_t lo = diag_signed[n] ? -(INT64_C(1) << (diag_width[n] - 1)) : 0;
        int64_t hi = (INT64_C(1) << (diag_width[n] - diag_signed[n])) - 1;
        if (a.v[n] < lo) {
            a.v[n] = lo;
            clipped = true;
        }
        if (a.v[n] > hi) {
            a.v[n] = hi;
            clipped = true;
        }
    }
    if (clipped)
        a.v[D_FLAGS] |= DF_CLIPPED;
    memset(r, 0, sizeof(*r));
    unsigned bit = 0;
    for (unsigned n = 0; n < D_FIELDS; ++n) {
        diag_set_bits(r, bit, diag_width[n], (uint32_t)a.v[n]);
        bit += diag_width[n];
    }
}
static inline void diag_unpack(const DiagRow *r, DiagValues *v) {
    unsigned bit = 0;
    for (unsigned n = 0; n < D_FIELDS; ++n) {
        uint32_t x = diag_get_bits(r, bit, diag_width[n]);
        bit += diag_width[n];
        v->v[n] = diag_signed[n] && (x & (1u << (diag_width[n] - 1)))
                      ? (int64_t)x - (INT64_C(1) << diag_width[n])
                      : x;
    }
}
static inline unsigned diag_bits(void) {
    unsigned n = 0;
    for (unsigned k = 0; k < D_FIELDS; ++k)
        n += diag_width[k];
    return n;
}
static inline void diag_begin(DriveDiag *d, uint64_t start, unsigned profile,
                              const uint16_t white[2], const uint16_t black[2]) {
    memset(d, 0, sizeof(*d));
    d->start_us = start;
    d->profile = (uint8_t)profile;
    d->begun = true;
    for (unsigned w = 0; w < 2; ++w) {
        d->white[w] = white[w];
        d->black[w] = black[w];
    }
}
/* Always observe timing, even after the capture freezes. Never change targets.
 * Trigger thresholds select a diagnostic window; they are NOT control gates. */
static inline int diag_push(DriveDiag *d, const DiagValues *v) {
    if (!d->begun)
        return -1;
    if (d->total < UINT32_MAX)
        ++d->total;
    uint32_t gap = (uint32_t)v->v[D_DT];
    if (gap > d->max_gap)
        d->max_gap = gap;
    if (gap >= 20000 && d->late20 < UINT32_MAX)
        ++d->late20;
    if (d->frozen)
        return -1;
    if (v->v[D_US] > UINT32_MAX) {
        d->frozen = true;
        return -1;
    }
    int slot = d->head;
    diag_pack(&d->rows[slot], v);
    d->head = (uint16_t)((d->head + 1) % DIAG_ROWS);
    if (d->count < DIAG_ROWS)
        ++d->count;
    else if (d->overwritten < UINT32_MAX)
        ++d->overwritten;
    int l = (int)v->v[D_PWM_L], r = (int)v->v[D_PWM_R],
        half = (int)(v->v[D_TARGET_R] - v->v[D_TARGET_L]);
    unsigned trigger = 0;
    if (d->previous_valid && v->v[D_US] >= 2000000 && v->v[D_PHASE] == 10 && !v->v[D_EXTERNAL]) {
        if (v->v[D_FLAGS] & DF_HANDOVER_KEEP)
            trigger |= 4;
        /* Smooth output may never cross the old jump trigger: capture a repeatable
   * ordinary-driving window around3s rather than only the end of a long run. */
        if (v->v[D_DRIVER] == 3 && v->v[D_US] >= 3000000)
            trigger |= 4;
        if (abs(l - d->previous_pwm[0]) >= 80 || abs(r - d->previous_pwm[1]) >= 80)
            trigger |= 1;
        if ((half >= 10 && d->previous_half <= -10) || (half <= -10 && d->previous_half >= 10))
            trigger |= 2;
    }
    if (!d->triggered && trigger) {
        d->triggered = true;
        d->trigger = (uint8_t)trigger;
        d->trigger_us = (uint32_t)v->v[D_US];
        d->post = DIAG_POST;
    } else if (d->triggered && d->post && !--d->post)
        d->frozen = true;
    d->previous_valid = true;
    d->previous_pwm[0] = (int16_t)l;
    d->previous_pwm[1] = (int16_t)r;
    d->previous_half = (int16_t)half;
    return slot;
}
static inline void diag_cost(DriveDiag *d, int slot, uint32_t cost) {
    if (!d->begun)
        return;
    if (cost > d->max_cost)
        d->max_cost = cost;
    if (slot < 0)
        return;
    /* COST immediately follows US32/DT17; update only this observation. */
    diag_set_bits(&d->rows[slot], 49, 17, cost > 131071 ? 131071 : cost);
    if (cost > 131071) {
        DiagValues v;
        diag_unpack(&d->rows[slot], &v);
        v.v[D_FLAGS] |= DF_CLIPPED;
        diag_pack(&d->rows[slot], &v);
    }
}
#endif
