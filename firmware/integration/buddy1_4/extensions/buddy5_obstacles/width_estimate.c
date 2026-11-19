#include "extensions/buddy5_obstacles/width_estimate.h"

#include <string.h>

/* Raw class bits 0..1: 0 = ECHO (as decoded by decode_width_probe.py RAW[]). The
 * replay counts every ECHO value regardless of its reason bits; so does this. */
B5WEPoint b5_we_point(uint8_t side, bool centre, const uint8_t cls[3], const uint16_t value[3],
                      uint16_t expected_mm, int16_t axis_x_mm10) {
    B5WEPoint p = {.side = side, .centre = centre ? 1u : 0u, .kind = B5WE_K_NONE};
    int32_t ax = axis_x_mm10;
    p.absx10 = (uint16_t)(ax < 0 ? -ax : ax);
    uint16_t v[3];
    unsigned n = 0;
    for (unsigned i = 0; i < 3; ++i)
        if ((cls[i] & 3u) == 0u) v[n++] = value[i];
    for (unsigned i = 1; i < n; ++i)            /* sort ascending */
        for (unsigned j = i; j && v[j - 1] > v[j]; --j) {
            uint16_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
        }
    if (n >= 2u) {
        int32_t med2 = n == 3u ? 2 * (int32_t)v[1] : (int32_t)v[0] + (int32_t)v[1];
        p.res4 = 2 * med2 - 4 * (int32_t)expected_mm;
        p.kind = p.res4 > B5WE_FAR_Q ? B5WE_K_FAR : B5WE_K_NUM;
    }
    bool all_far = true, no_return = false;
    for (unsigned i = 0; i < 3u; ++i) {
        uint8_t raw = cls[i];
        if (raw == 1u) no_return = true;        /* clean NO_RETURN */
        else if (raw != 0u || (uint32_t)value[i] <= (uint32_t)expected_mm + 60u)
            all_far = false;                    /* mixed IN/FAR, fault or bad reason */
    }
    if (all_far && no_return) {
        p.legacy_kind = p.kind;
        p.kind = B5WE_K_CLEAN_FAR;
    }
    return p;
}

/* statistics.median of K_NUM values (all even in quarter-mm, so exact). */
static bool median4(const int32_t *x, unsigned n, int32_t *out) {
    if (!n) return false;
    int32_t v[B5WE_MAX_POINTS];
    memcpy(v, x, n * sizeof v[0]);
    for (unsigned i = 1; i < n; ++i)
        for (unsigned j = i; j && v[j - 1] > v[j]; --j) {
            int32_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
        }
    *out = n & 1u ? v[n / 2u] : (v[n / 2u - 1u] + v[n / 2u]) / 2;
    return true;
}

/* Python sorts (abs x, residual) tuples: by x, then numerically. An equal x with
 * one F and one number raises TypeError in the reference; we order F last and flag it. */
static bool before(const B5WEPoint *a, const B5WEPoint *b, bool *tie) {
    if (a->absx10 != b->absx10) return a->absx10 < b->absx10;
    uint8_t ak = a->kind == B5WE_K_CLEAN_FAR ? B5WE_K_FAR : a->kind;
    uint8_t bk = b->kind == B5WE_K_CLEAN_FAR ? B5WE_K_FAR : b->kind;
    if (ak != bk) { *tie = true; return ak == B5WE_K_NUM; }
    if (a->kind == B5WE_K_NUM) return a->res4 < b->res4;
    return false;
}

static void side_onset(const B5WEInput *in, uint8_t want, int32_t base4, B5WESide *s,
                       bool *tie, bool *extended) {
    B5WEPoint p[B5WE_MAX_POINTS];
    unsigned n = 0;
    bool corroborated = in->side[want == B5WE_PSIDE_R ? 1u : 0u].boundary_ok;
    uint16_t anchor = (uint16_t)B5WE_NONE;
    for (unsigned i = 0; i < in->n && i < B5WE_MAX_POINTS; ++i) {
        B5WEPoint q = in->pt[i];
        if (q.side != want) continue;
        if (q.centre) {
            if (corroborated && q.kind == B5WE_K_NUM && q.absx10 < anchor) anchor = q.absx10;
            continue;                         /* baseline / optional anchor, never a side sample */
        }
        if (q.kind == B5WE_K_CLEAN_FAR && !corroborated) q.kind = q.legacy_kind;
        if (q.kind != B5WE_K_NONE) p[n++] = q;
    }
    for (unsigned i = 1; i < n; ++i)            /* stable insertion sort */
        for (unsigned j = i; j && before(&p[j], &p[j - 1], tie); --j) {
            B5WEPoint t = p[j]; p[j] = p[j - 1]; p[j - 1] = t;
        }
    memset(s, 0, sizeof *s);
    s->n = (uint8_t)n;
    s->base4 = base4;
    s->a10 = s->b10 = (uint16_t)B5WE_NONE;
    if (!n) { s->status = B5WE_NO_DATA; return; }
    int last_low = -1;                          /* last index that is not HIGH */
    for (unsigned i = 0; i < n; ++i) {
        bool high = p[i].kind == B5WE_K_FAR || p[i].kind == B5WE_K_CLEAN_FAR ||
                    p[i].res4 - base4 >= B5WE_HIGH_Q;
        if (!high) last_low = (int)i;
    }
    unsigned i = (unsigned)(last_low + 1);
    if (i == 0u) {
        s->status = B5WE_HIGH_FROM_START; s->a10 = 0; s->b10 = p[0].absx10;
        if (anchor != B5WE_NONE && anchor < p[0].absx10) {
            s->status = B5WE_ONSET; s->a10 = anchor; *extended = true;
        }
    }
    else if (i == n) { s->status = B5WE_WALK_FREE; s->a10 = p[n - 1u].absx10; }
    else { s->status = B5WE_ONSET; s->a10 = p[i - 1u].absx10; s->b10 = p[i].absx10; }
    if (i < n && p[i].kind == B5WE_K_CLEAN_FAR) *extended = true;
}

static uint16_t u16sat(int32_t v) { return (uint16_t)(v < 0 ? 0 : v > 0xFFFE ? 0xFFFE : v); }

void b5_we_estimate(const B5WEInput *in, B5WEResult *out) {
    memset(out, 0, sizeof *out);
    out->raw_mm100 = out->corrected_mm100 = out->sampled_mm10 = (uint16_t)B5WE_NONE;
    out->span_lo_mm10 = out->span_hi_mm10 = (uint16_t)B5WE_NONE;
    for (unsigned k = 0; k < 2u; ++k) {
        out->side[k].a10 = out->side[k].b10 = (uint16_t)B5WE_NONE;
        const B5WESideIn *si = &in->side[k];
        if (si->cap_budget) out->flags |= (uint16_t)(B5WE_F_L_BUDGET << k);
        if (si->return_ok) out->flags |= (uint16_t)(B5WE_F_L_RETURN_OK << k);
        if (si->not_reached) out->flags |= (uint16_t)(B5WE_F_L_NOT_REACHED << k);
        if (si->last_out_in) out->flags |= (uint16_t)(B5WE_F_L_LAST_OUT_IN << k);
    }
    if (in->mid_in) out->flags |= B5WE_F_MID_IN;
    if (in->final_in) out->flags |= B5WE_F_FINAL_IN;
    if (in->consistent) out->flags |= B5WE_F_CONSISTENT;

    /* Default (C): project range midpoint, never a hold override. */
    out->basis = B5WE_BASIS_PROJECT_RANGE_ASSUMPTION;
    out->status = B5WE_STATUS_INVALID;
    out->display_mm100 = B5WE_MID_MM100;

    /* Brackets (also reported, as information only, for invalid runs). */
    int32_t centre[B5WE_MAX_POINTS], right[B5WE_MAX_POINTS];
    unsigned nc = 0, nr = 0;
    bool extended = false;
    for (unsigned i = 0; i < in->n && i < B5WE_MAX_POINTS; ++i) {
        const B5WEPoint *p = &in->pt[i];
        uint8_t kind = p->kind == B5WE_K_CLEAN_FAR ? p->legacy_kind : p->kind;
        if (kind != B5WE_K_NUM) continue;
        if (p->centre) centre[nc++] = p->res4;
        else if (p->side == B5WE_PSIDE_R && p->absx10 < B5WE_RIGHT_BASE_X10) right[nr++] = p->res4;
    }
    int32_t cbase = 0, rbase = 0;
    bool have = median4(centre, nc, &cbase);
    if (have) {
        if (!median4(right, nr, &rbase)) rbase = cbase;
        bool tie = false;
        side_onset(in, B5WE_PSIDE_L, cbase, &out->side[0], &tie, &extended);
        side_onset(in, B5WE_PSIDE_R, rbase, &out->side[1], &tie, &extended);
        if (tie) out->flags |= B5WE_F_SORT_TIE;
    }

    if (!in->records) out->reason = B5WE_REASON_NO_RECORDS;
    else if (!in->consistent) out->reason = B5WE_REASON_INCONSISTENT;
    else if (in->end != 1u || !in->mid_in || !in->final_in || !have)
        out->reason = B5WE_REASON_RUN_OR_REFERENCE;   /* !have is unreachable when MID/FINAL are IN */
    if (out->reason != B5WE_REASON_NONE) {
        if (have) out->flags |= B5WE_F_SIDES_INCOMPLETE_RUN;   /* brackets: information only */
        return;
    }

    const B5WESide *L = &out->side[0], *R = &out->side[1];
    bool extended_ok = !extended || (in->side[0].boundary_ok && in->side[1].boundary_ok);
    if (L->status == B5WE_ONSET && R->status == B5WE_ONSET && extended_ok) {
        int32_t lo = (int32_t)L->a10 + R->a10, hi = (int32_t)L->b10 + R->b10;
        int32_t raw = (lo + hi) * 5;                         /* mean of the sums, mm100 */
        int32_t corrected = raw - B5WE_OFFSET_MM100;
        int32_t shown = corrected < B5WE_MIN_MM100 ? B5WE_MIN_MM100 :
                        corrected > B5WE_MAX_MM100 ? B5WE_MAX_MM100 : corrected;
        out->basis = extended ? B5WE_BASIS_CORROBORATED_EDGE_ESTIMATE : B5WE_BASIS_CALIBRATED_RANGE_RISE;
        out->status = B5WE_STATUS_ESTIMATE;
        out->reason = B5WE_REASON_NONE;
        out->span_lo_mm10 = u16sat(lo);
        out->span_hi_mm10 = u16sat(hi);
        out->raw_mm100 = u16sat(raw);
        out->corrected_mm100 = u16sat(corrected);
        out->display_mm100 = (uint16_t)shown;
        if (shown != corrected) out->flags |= B5WE_F_LIMIT_APPLIED;
        return;
    }
    /* Not both bracketed: REACH_LIMIT. Sampled span exactly as two_sided(). */
    out->status = B5WE_STATUS_REACH_LIMIT;
    out->reason = B5WE_REASON_EDGES_UNRESOLVED;
    int32_t sampled = -1;
    if (L->status == B5WE_ONSET && R->status == B5WE_WALK_FREE) sampled = (int32_t)L->a10 + R->a10;
    if (R->status == B5WE_ONSET && L->status == B5WE_WALK_FREE) sampled = (int32_t)R->a10 + L->a10;
    if (L->status == B5WE_WALK_FREE && R->status == B5WE_WALK_FREE)
        sampled = (int32_t)L->a10 + R->a10;
    if (sampled >= 0) out->sampled_mm10 = u16sat(sampled);
    unsigned walk_free = (L->status == B5WE_WALK_FREE) + (R->status == B5WE_WALK_FREE);
    if (walk_free == 1u && sampled >= 0) {
        unsigned far = L->status == B5WE_WALK_FREE ? 0u : 1u, near = 1u - far;
        const B5WESideIn *fs = &in->side[far];
        bool in_range = sampled >= B5WE_MIN_MM100 / 10 && sampled <= B5WE_MAX_MM100 / 10;
        if (!in_range) out->flags |= B5WE_F_SAMPLED_OUT_OF_RANGE;
        if (out->side[near].status == B5WE_ONSET && fs->cap_budget && fs->return_ok &&
            fs->not_reached && fs->last_out_in && in_range) {
            out->basis = B5WE_BASIS_EMPIRICAL_REACH_COMPLETION;
            out->display_mm100 = (uint16_t)(sampled * 5 + B5WE_MAX_MM100 / 2);
            out->flags |= B5WE_F_COMPLETION_MET;
        }
    }
}
