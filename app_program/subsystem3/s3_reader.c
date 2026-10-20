/**
 * @file    s3_reader.c
 * @brief   Read Barcode: edges -> bar and space widths (see s3_reader.h).
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#include "s3_reader.h"

#define D_MAX_Q         (65535)         /* 16-bit distance field */
#define MIN_SPAN_Q      (20 * S3_RD_Q)  /* below 20 mm of travel over the
                                         * whole barcode, use time widths */

void
s3_rd_reset (s3_reader_t * p_rd)
{
    uint32_t t = p_rd->t1_us;
    int32_t  d = p_rd->d1_q;
    bool     b_anchored = (p_rd->b_anchored != 0U);

    /* Keep the anchors: the car's position is still known. */
    p_rd->count      = 0U;
    p_rd->b_black    = 0U;
    p_rd->b_overflow = 0U;
    p_rd->start_t_us = 0U;
    p_rd->start_d_q  = 0;
    p_rd->t0_us      = t;
    p_rd->d0_q       = d;
    p_rd->t1_us      = t;
    p_rd->d1_q       = d;
    p_rd->b_anchored = b_anchored ? 1U : 0U;
}

void
s3_rd_anchor (s3_reader_t * p_rd, uint32_t t_us, int32_t d_q, bool b_dist_ok)
{
    p_rd->b_dist_ok = b_dist_ok ? 1U : 0U;

    if (p_rd->b_anchored == 0U)
    {
        p_rd->t0_us = t_us;
        p_rd->d0_q  = d_q;
        p_rd->b_anchored = 1U;
    }
    else
    {
        p_rd->t0_us = p_rd->t1_us;
        p_rd->d0_q  = p_rd->d1_q;
    }

    p_rd->t1_us = t_us;
    p_rd->d1_q  = d_q;
}

/* Distance at time t, interpolated between the two latest anchors. An
 * edge outside that window is clamped to its nearer end. */
static int32_t
dist_at (s3_reader_t const * p_rd, uint32_t t_us)
{
    uint32_t span = p_rd->t1_us - p_rd->t0_us;
    uint32_t into = t_us - p_rd->t0_us;

    if (span == 0U)
    {
        return p_rd->d1_q;
    }

    if ((int32_t)into <= 0)
    {
        return p_rd->d0_q;          /* before the window (late cycle) */
    }

    if (into >= span)
    {
        return p_rd->d1_q;
    }

    return p_rd->d0_q +
           (int32_t)(((int64_t)(p_rd->d1_q - p_rd->d0_q) * (int64_t)into) /
                     (int64_t)span);
}

void
s3_rd_edge (s3_reader_t * p_rd, uint32_t t_us, bool b_black)
{
    int32_t d = dist_at(p_rd, t_us);

    if (p_rd->count == 0U)
    {
        if (!b_black)
        {
            return;                 /* a barcode starts with a bar */
        }

        p_rd->start_t_us = t_us;
        p_rd->start_d_q  = d;
    }
    else if ((p_rd->b_black != 0U) == b_black)
    {
        return;                     /* no level change: bounce, ignore */
    }
    else
    {
        /* normal alternation */
    }

    if (p_rd->count >= S3_RD_MAX_EDGES)
    {
        p_rd->b_overflow = 1U;
        return;
    }

    {
        int32_t rel = d - p_rd->start_d_q;

        rel = (rel < 0) ? 0 : ((rel > D_MAX_Q) ? D_MAX_Q : rel);
        p_rd->edge_t[p_rd->count] = t_us - p_rd->start_t_us;
        p_rd->edge_d[p_rd->count] = (uint16_t)rel;
    }

    p_rd->count++;
    p_rd->b_black = b_black ? 1U : 0U;
}

s3_rd_status_t
s3_rd_poll (s3_reader_t const * p_rd)
{
    uint32_t last_t;
    int32_t  last_d;
    uint32_t quiet_us;
    int32_t  quiet_q;

    if (p_rd->b_overflow != 0U)
    {
        return S3_RD_OVERFLOW;
    }

    if (p_rd->count == 0U)
    {
        return S3_RD_IDLE;
    }

    if ((p_rd->t1_us - p_rd->start_t_us) > (S3_BC_TIMEOUT_MS * 1000U))
    {
        return S3_RD_TIMEOUT;
    }

    if (p_rd->b_black != 0U)
    {
        return S3_RD_READING;       /* still on a bar */
    }

    last_t   = p_rd->start_t_us + p_rd->edge_t[p_rd->count - 1U];
    last_d   = p_rd->start_d_q + (int32_t)p_rd->edge_d[p_rd->count - 1U];
    quiet_us = p_rd->t1_us - last_t;
    quiet_q  = p_rd->d1_q - last_d;

    /* The barcode is over after S3_BC_QUIET_MM of white travel. Only when
     * there is no real distance is it judged by time (S3_BC_QUIET_MS):
     * with a real distance, a car that stops on a space in the middle of
     * a barcode just waits, and the read carries on when it moves. */
    if (p_rd->b_dist_ok != 0U)
    {
        if (quiet_q >= (S3_BC_QUIET_MM * S3_RD_Q))
        {
            return S3_RD_READY;
        }
    }
    else if (((int32_t)quiet_us > 0) &&
             (quiet_us >= (S3_BC_QUIET_MS * 1000U)))
    {
        return S3_RD_READY;
    }
    else
    {
        /* still in the barcode */
    }

    return S3_RD_READING;
}

uint32_t
s3_rd_widths (s3_reader_t const * p_rd, uint32_t * p_w, uint32_t w_max,
              bool b_dist)
{
    uint32_t n;
    uint32_t i;

    if (p_rd->count < 2U)
    {
        return 0U;
    }

    n = p_rd->count - 1U;

    if (b_dist && ((int32_t)p_rd->edge_d[n] < MIN_SPAN_Q))
    {
        return 0U;                  /* the car hardly moved: no distance */
    }

    n = (n > w_max) ? w_max : n;

    for (i = 0U; i < n; i++)
    {
        p_w[i] = b_dist
               ? (uint32_t)(p_rd->edge_d[i + 1U] - p_rd->edge_d[i])
               : (p_rd->edge_t[i + 1U] - p_rd->edge_t[i]);
    }

    return n;
}

static bool
final (s3_bc_result_t r)
{
    return (r == S3_BC_OK) || (r == S3_BC_BACKWARDS);
}

s3_bc_result_t
s3_rd_decode (s3_reader_t const * p_rd, char * p_text, uint32_t text_max,
              bool * p_b_dist)
{
    uint32_t       w[S3_BC_MAX_ELEMENTS];
    uint32_t       n;
    s3_bc_result_t r_dist = S3_BC_BAD_LENGTH;
    s3_bc_result_t r_time;

    *p_b_dist = false;
    n = s3_rd_widths(p_rd, w, S3_BC_MAX_ELEMENTS, true);

    if ((n != 0U) && (p_rd->b_dist_ok != 0U))
    {
        r_dist = s3_bc_decode(w, n, p_text, text_max);

        if (final(r_dist))
        {
            *p_b_dist = true;
            return r_dist;
        }
    }

    n      = s3_rd_widths(p_rd, w, S3_BC_MAX_ELEMENTS, false);
    r_time = s3_bc_decode(w, n, p_text, text_max);

    if (final(r_time) || (p_rd->b_dist_ok == 0U))
    {
        return r_time;
    }

    p_text[0] = '\0';
    *p_b_dist = true;
    return r_dist;                  /* report the distance read's reason */
}
