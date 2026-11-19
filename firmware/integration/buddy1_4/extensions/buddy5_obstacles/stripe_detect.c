#include "extensions/buddy5_obstacles/stripe_detect.h"
#include <math.h>
#include <string.h>

static uint16_t clamp_level(uint16_t v) { return v > 1000 ? 1000 : v; }
static uint16_t rise_level(uint16_t bg) {
    unsigned t = (unsigned)bg + B5S_CONTRAST;
    return (uint16_t)(t > B5S_RISE_ABS ? t : B5S_RISE_ABS);
}
uint16_t b5_stripe_threshold(const B5Stripe *d, unsigned w) {
    return rise_level(d->bg[w & 1u]);
}
static void window_reset(B5Stripe *d, float p) {
    d->q_start = p;
    d->q_n = 0;
    d->q_sum[0] = d->q_sum[1] = 0;
}
static void window_add(B5Stripe *d, float p, const uint16_t v[2]) {
    if (d->q_n) {
        for (unsigned w = 0; w < 2; ++w) {
            uint16_t lo = v[w] < d->q_min[w] ? v[w] : d->q_min[w];
            uint16_t hi = v[w] > d->q_max[w] ? v[w] : d->q_max[w];
            if (hi - lo > B5S_QUIET_SPREAD) { window_reset(d, p); break; }
        }
    }
    if (!d->q_n) {
        d->q_start = p;
        for (unsigned w = 0; w < 2; ++w) d->q_min[w] = d->q_max[w] = v[w];
    }
    for (unsigned w = 0; w < 2; ++w) {
        if (v[w] < d->q_min[w]) d->q_min[w] = v[w];
        if (v[w] > d->q_max[w]) d->q_max[w] = v[w];
        d->q_sum[w] += v[w];
    }
    ++d->q_n;
    if (d->q_n >= B5S_QUIET_SAMPLES && p - d->q_start >= B5S_QUIET_MM &&
        d->q_max[0] <= B5S_BG_MAX && d->q_max[1] <= B5S_BG_MAX) {
        d->snap_ok = true;
        d->snap_p = p;
        for (unsigned w = 0; w < 2; ++w)
            d->snap_bg[w] = (uint16_t)((d->q_sum[w] + d->q_n / 2) / d->q_n);
    }
}
void b5_stripe_init(B5Stripe *d, float search_mm) {
    memset(d, 0, sizeof(*d));
    d->search_mm = search_mm;
}
static void reject(B5Stripe *d, unsigned why, float p) {
    ++d->rejects;
    d->last_reject = (uint8_t)why;
    d->phase = 0;
    memset(d->s, 0, sizeof(d->s));
    d->post_cnt = 0;
    d->snap_ok = false;          /* a fresh quiet background is required */
    window_reset(d, p);
}
/* Sustained above-threshold run for one sensor; returns true when it starts. */
static bool sustained_rise(B5StripeSensor *s, float p, uint16_t v, uint16_t th) {
    if (v >= th) {
        if (!s->cnt) s->run = p;
        if (s->cnt < 255) ++s->cnt;
        if (s->cnt >= B5S_SUSTAIN_N && p - s->run >= B5S_SUSTAIN_MM) {
            s->st = 1; s->rise = s->run; s->cnt = 0; s->fcnt = 0;
            return true;
        }
    } else s->cnt = 0;
    return false;
}
unsigned b5_stripe_step(B5Stripe *d, float p, uint16_t left, uint16_t right) {
    if (d->result != B5S_PENDING) return d->result;
    uint16_t v[2] = {clamp_level(left), clamp_level(right)};
    ++d->samples;
    d->last_p = p;
    if (d->phase == 0) {
        bool fresh = d->snap_ok && p - d->snap_p <= B5S_SNAP_GAP_MM;
        bool counting = false;
        if (fresh) {
            for (unsigned w = 0; w < 2; ++w) {
                uint8_t before = d->s[w].cnt;
                if (sustained_rise(&d->s[w], p, v[w], rise_level(d->snap_bg[w]))) {
                    d->phase = 1;
                    d->bg[0] = d->snap_bg[0]; d->bg[1] = d->snap_bg[1];
                    d->first_rise = d->s[w].rise;
                } else if (before && !d->s[w].cnt) {
                    ++d->spikes; /* a short excursion ended: isolated spike, ignored */
                }
                counting |= d->s[w].cnt != 0;
            }
        }
        if (d->phase == 0) {
            if (!counting) window_add(d, p, v);
            if (p > d->search_mm) return d->result = B5S_NO_LINE;
            return B5S_PENDING;
        }
        /* fall through: evaluate the other sensor on this same sample */
    }
    if (d->phase == 1) {
        for (unsigned w = 0; w < 2; ++w) {
            B5StripeSensor *s = &d->s[w];
            if (s->st == 0) {
                (void)sustained_rise(s, p, v[w], rise_level(d->bg[w]));
            } else if (s->st == 1) {
                if (v[w] <= d->bg[w] + B5S_FALL_MARGIN) {
                    if (!s->fcnt) s->frun = p;
                    if (s->fcnt < 255) ++s->fcnt;
                    if (s->fcnt >= B5S_SUSTAIN_N && p - s->frun >= B5S_SUSTAIN_MM) {
                        s->st = 2; s->fall = s->frun;
                    }
                } else s->fcnt = 0;
            }
        }
        for (unsigned w = 0; w < 2; ++w) {
            B5StripeSensor *s = &d->s[w];
            if (s->st == 1 && p - s->rise > B5S_WIDTH_MAX_MM + B5S_SUSTAIN_MM) {
                reject(d, B5SR_WIDE_OR_STEP, p); return B5S_PENDING;
            }
        }
        if ((d->s[0].st == 0 || d->s[1].st == 0) && p - d->first_rise > B5S_SKEW_MM) {
            reject(d, B5SR_SINGLE_SENSOR, p); return B5S_PENDING;
        }
        if (d->s[0].st == 2 && d->s[1].st == 2) {
            float c[2];
            for (unsigned w = 0; w < 2; ++w) {
                d->width[w] = d->s[w].fall - d->s[w].rise;
                c[w] = (d->s[w].fall + d->s[w].rise) * .5f;
                if (d->width[w] < B5S_WIDTH_MIN_MM) { reject(d, B5SR_NARROW, p); return B5S_PENDING; }
                if (d->width[w] > B5S_WIDTH_MAX_MM) { reject(d, B5SR_WIDE_OR_STEP, p); return B5S_PENDING; }
            }
            if (fabsf(c[0] - c[1]) > B5S_SKEW_MM) { reject(d, B5SR_SKEW, p); return B5S_PENDING; }
            d->centre = (c[0] + c[1]) * .5f;
            d->phase = 2;
            d->post_start = d->s[0].fall > d->s[1].fall ? d->s[0].fall : d->s[1].fall;
            d->post_cnt = 0;
        }
        return B5S_PENDING;
    }
    /* phase 2: clean background after the stripe */
    bool dirty = v[0] > d->bg[0] + B5S_FALL_MARGIN || v[1] > d->bg[1] + B5S_FALL_MARGIN;
    if (dirty) {
        if (!d->post_cnt) d->post_run = p;
        if (d->post_cnt < 255) ++d->post_cnt;
        if (d->post_cnt >= B5S_SUSTAIN_N && p - d->post_run >= B5S_SUSTAIN_MM) {
            reject(d, B5SR_NOT_CLEAN_AFTER, p); return B5S_PENDING;
        }
    } else d->post_cnt = 0;
    if (p - d->post_start >= B5S_POST_MM && !d->post_cnt) return d->result = B5S_FOUND;
    return B5S_PENDING;
}
bool b5_stripe_evidence(const B5Stripe *d, uint16_t left, uint16_t right) {
    if (d->result != B5S_FOUND) return false;
    return clamp_level(left) >= rise_level(d->bg[0]) || clamp_level(right) >= rise_level(d->bg[1]);
}
const char *b5_stripe_result_name(unsigned r) {
    static const char *const n[] = {"PENDING", "FOUND", "NO_LINE"};
    return r < sizeof(n) / sizeof(n[0]) ? n[r] : "INVALID";
}
const char *b5_stripe_reject_name(unsigned r) {
    static const char *const n[] = {"NONE", "SINGLE_SENSOR", "WIDE_OR_STEP", "NARROW", "SKEW",
        "NOT_CLEAN_AFTER"};
    return r < sizeof(n) / sizeof(n[0]) ? n[r] : "INVALID";
}
