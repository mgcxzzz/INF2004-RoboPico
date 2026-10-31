/**
 * @file    obstacle_profile.c
 * @brief   Obstacle profiling and bypass planning (pure logic, no RTOS).
 *
 * Implements the Scanning and Planning states of statechart 6.5:
 *  - Edge Refinement: bisect each near/far transition until the bracket
 *    is no wider than the beam half-angle alpha.
 *  - Planning: build the obstacle profile, correct the width for beam
 *    spread (the beam reads an obstacle about 2*d*tan(alpha) too wide),
 *    measure the clearance on each side and choose the side with more room
 *    (FR-USO-03, FR-USO-04, NFR-USO-02, NFR-USO-03).
 *
 * Integer only: the RP2040 has no FPU, so all trigonometry is Q15.
 *
 * Angles are signed tenths of a degree, + = right (see the header). Each
 * result is computed in a full-width local first and only clamped into
 * its bit-field when it is stored, so no intermediate value is truncated.
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#include <stddef.h>

#include "obstacle_profile.h"
#include "b5_config.h"

/* The bit-field records must stay as small as designed, on every compiler
 * this code is built with (arm-none-eabi-gcc on target, gcc on the host). */
_Static_assert(sizeof(b5_sample_t) == 4U, "scan sample must fit one word");
_Static_assert(sizeof(b5_profile_t) <= 32U, "profile must fit 8 words");

#define Q15_ONE         (32768)
#define DD_PER_DEG      (10)
#define DD_HALF_TURN    (1800)
#define DD_FULL_TURN    (3600)
#define DD_QUARTER      (900)

/* cos(d) for d = 0..90 whole degrees, Q15. */
static int16_t const g_cos_table[91] =
{
    32767, 32762, 32747, 32722, 32687, 32642, 32587, 32523,
    32448, 32364, 32269, 32165, 32051, 31927, 31794, 31650,
    31498, 31335, 31163, 30982, 30791, 30591, 30381, 30162,
    29934, 29697, 29451, 29196, 28932, 28659, 28377, 28087,
    27788, 27481, 27165, 26841, 26509, 26169, 25821, 25465,
    25101, 24730, 24351, 23964, 23571, 23170, 22762, 22347,
    21925, 21497, 21062, 20621, 20173, 19720, 19260, 18794,
    18323, 17846, 17364, 16876, 16384, 15886, 15383, 14876,
    14364, 13848, 13328, 12803, 12275, 11743, 11207, 10668,
    10126,  9580,  9032,  8481,  7927,  7371,  6813,  6252,
     5690,  5126,  4560,  3993,  3425,  2856,  2286,  1715,
     1144,   572,     0
};

/* ---------------------------------------------------------------------
 * Trigonometry
 * ------------------------------------------------------------------ */

/* cos for 0..900 dd with linear interpolation between whole degrees. */
static int32_t
cos_first_quadrant (int32_t angle_dd)
{
    int32_t deg  = angle_dd / DD_PER_DEG;
    int32_t frac = angle_dd % DD_PER_DEG;
    int32_t c0;
    int32_t c1;

    if (deg >= 90)
    {
        return 0;
    }

    c0 = g_cos_table[deg];
    c1 = g_cos_table[deg + 1];

    return (c0 + (((c1 - c0) * frac) / DD_PER_DEG));
}

/**
 * @brief  cos(angle) in Q15.
 * @param  angle_dd  angle in tenths of a degree, any value.
 */
int32_t
b5_cos_q15 (int32_t angle_dd)
{
    int32_t a = angle_dd % DD_FULL_TURN;

    if (a < 0)
    {
        a += DD_FULL_TURN;
    }

    if (a > DD_HALF_TURN)
    {
        a = DD_FULL_TURN - a;           /* cos is even about 180 deg  */
    }

    if (a <= DD_QUARTER)
    {
        return cos_first_quadrant(a);
    }

    return -cos_first_quadrant(DD_HALF_TURN - a);
}

/**
 * @brief  sin(angle) in Q15.
 */
int32_t
b5_sin_q15 (int32_t angle_dd)
{
    return b5_cos_q15(DD_QUARTER - angle_dd);
}

/* r * trig(angle), rounded, without overflowing int32 for r < 65 m. */
static int32_t
mul_q15 (int32_t range_mm, int32_t q15)
{
    int32_t product = range_mm * q15;

    return ((product >= 0) ? (product + (Q15_ONE / 2)) / Q15_ONE
                           : (product - (Q15_ONE / 2)) / Q15_ONE);
}

/* ---------------------------------------------------------------------
 * Scan bookkeeping
 * ------------------------------------------------------------------ */

void
b5_scan_clear (b5_scan_t * p_scan)
{
    p_scan->count = 0U;
}

/**
 * @brief  Insert a sample, keeping the array sorted by angle.
 *         A sample at an angle already present replaces the old one.
 * @return false if the scan is full.
 */
bool
b5_scan_add (b5_scan_t * p_scan, int16_t angle_dd, int32_t range_mm,
             bool b_valid)
{
    b5_sample_t s = { 0 };

    /* Fit the 11-bit angle and 12-bit range fields before storing. A
     * plain valid sample is a measured, stable echo (pre-rule-B meaning). */
    s.angle_dd = (int16_t)b5_clamp(angle_dd, B5_ANGLE_MIN_DD, B5_ANGLE_MAX_DD);
    s.range_mm = (uint32_t)b5_clamp(range_mm, 0, B5_MM_MAX);
    s.b_valid  = b_valid ? 1U : 0U;
    s.b_stable = s.b_valid;

    return b5_scan_add_sample(p_scan, &s);
}

/**
 * @brief  Insert a fully classified sample (rule B flags kept), sorted by
 *         angle; a sample at an angle already present replaces the old one.
 * @return false if the scan is full.
 */
bool
b5_scan_add_sample (b5_scan_t * p_scan, b5_sample_t const * p_s)
{
    uint8_t idx = 0U;
    uint8_t move;

    while ((idx < p_scan->count) &&
           (p_scan->sample[idx].angle_dd < p_s->angle_dd))
    {
        idx++;
    }

    if ((idx < p_scan->count) && (p_scan->sample[idx].angle_dd == p_s->angle_dd))
    {
        p_scan->sample[idx] = *p_s;
        return true;
    }

    if (p_scan->count >= B5_MAX_SAMPLES)
    {
        return false;
    }

    for (move = p_scan->count; move > idx; move--)
    {
        p_scan->sample[move] = p_scan->sample[move - 1U];
    }

    p_scan->sample[idx] = *p_s;
    p_scan->count++;

    return true;
}

/* ---------------------------------------------------------------------
 * Rule B: ping classes and per-angle combination
 * ------------------------------------------------------------------ */

#define B5_PING_WIDTH_MAX   (131071U)       /* 17-bit diagnostic field */

/**
 * @brief  Classify one ping from what the driver observed.
 * @param  b_pretrig_low  ECHO read low before TRIG was pulsed
 * @param  b_rise         a rising edge of this ping was seen
 * @param  b_fall         ... and the matching falling edge
 * @param  b_short_seen   a pulse shorter than B5_ECHO_MIN_US was dropped
 * @param  width_us       rise-to-fall width of the accepted pulse
 */
b5_ping_t
b5_classify_ping (bool b_pretrig_low, bool b_rise, bool b_fall,
                  bool b_short_seen, uint32_t width_us)
{
    b5_ping_t p = { 0 };

    p.cls      = (uint32_t)B5_PING_FAULT;
    p.range_mm = (uint32_t)B5_RANGE_FAR_MM;
    p.width_us = (width_us > B5_PING_WIDTH_MAX) ? B5_PING_WIDTH_MAX : width_us;

    if (!b_pretrig_low)
    {
        p.why = (uint32_t)B5_PING_WHY_STUCK_HIGH;   /* never a distance */
    }
    else if (b_rise && b_fall && (width_us >= B5_ECHO_MIN_US))
    {
        if (width_us <= B5_ECHO_MAX_US)
        {
            p.cls      = (uint32_t)B5_PING_ECHO;
            p.range_mm = (uint32_t)b5_clamp((int32_t)((width_us * 343U) / 2000U),
                                            0, B5_MM_MAX);
        }
        else
        {
            p.cls = (uint32_t)B5_PING_NO_RETURN;    /* completed, nothing heard */
        }
    }
    else if (b_rise && !b_fall)
    {
        p.why = (uint32_t)B5_PING_WHY_NO_FALL;
    }
    else if (b_short_seen || (b_rise && b_fall))
    {
        p.why = (uint32_t)B5_PING_WHY_SHORT;
    }
    else
    {
        p.why = (uint32_t)B5_PING_WHY_NO_RISE;
    }

    return p;
}

/**
 * @brief  Combine the pings taken at one angle into a scan sample.
 *
 * Any echo   -> occupied at the NEAREST echo. Stable only when every ping
 *               echoed within B5_SAMPLE_SPREAD_MAX_MM. With ONE fault
 *               among the pings the sample is partial: it blocks at that
 *               range and the space nearer is only weak evidence. With two
 *               or more faults it is block-only: it blocks, else unknown.
 * No echo    -> B5_WEAK_CLEAR_PINGS clean no-returns = weak clear (flagged,
 *               range FAR) when B5_NORETURN_WEAK_CLEAR; otherwise unknown.
 */
void
b5_combine_pings (int16_t angle_dd, b5_ping_t const * p_ping,
                  uint8_t n_pings, b5_sample_t * p_out,
                  b5_ping_counts_t * p_counts)
{
    b5_ping_counts_t c = { 0U, 0U, 0U };
    int32_t lo = B5_MM_MAX;
    int32_t hi = 0;
    uint8_t i;

    *p_out = (b5_sample_t){ 0 };
    p_out->angle_dd = (int16_t)b5_clamp(angle_dd, B5_ANGLE_MIN_DD, B5_ANGLE_MAX_DD);
    p_out->range_mm = (uint32_t)B5_RANGE_FAR_MM;

    for (i = 0U; i < n_pings; i++)
    {
        if (p_ping[i].cls == (uint32_t)B5_PING_ECHO)
        {
            int32_t r = (int32_t)p_ping[i].range_mm;
            c.n_echo++;
            if (r < lo) { lo = r; }
            if (r > hi) { hi = r; }
        }
        else if (p_ping[i].cls == (uint32_t)B5_PING_NO_RETURN)
        {
            c.n_no_return++;
        }
        else
        {
            c.n_fault++;
        }
    }

    if (c.n_echo > 0U)
    {
        p_out->range_mm  = (uint32_t)lo;    /* credible echo blocks here */
        p_out->b_valid   = 1U;
        p_out->b_partial = (c.n_fault == 1U) ? 1U : 0U;
        p_out->b_block_only = (c.n_fault >= 2U) ? 1U : 0U;
        p_out->b_stable  = ((c.n_echo == n_pings) && (n_pings >= 2U) &&
                            ((hi - lo) <= B5_SAMPLE_SPREAD_MAX_MM)) ? 1U : 0U;
    }
    else if ((B5_NORETURN_WEAK_CLEAR != 0) && (c.n_fault == 0U) &&
             (c.n_no_return == n_pings) && (n_pings >= B5_WEAK_CLEAR_PINGS))
    {
        p_out->b_valid = 1U;
        p_out->b_weak  = 1U;
    }
    else
    {
        /* faults or too few pings: unknown, never clear */
    }

    if (p_counts != NULL)
    {
        *p_counts = c;
    }
}

/**
 * @brief  Median of three readings (Assumption A6).
 */
int32_t
b5_median3 (int32_t a, int32_t b, int32_t c)
{
    if (a > b)
    {
        int32_t t = a;
        a = b;
        b = t;
    }

    if (b > c)
    {
        b = c;
    }

    return ((a > b) ? a : b);
}

/*
 * Midpoint of two angles, rounded up (towards +900). Rounding the same way
 * everywhere keeps left and right edges treated alike; it matches the
 * results validated before the switch to signed angles.
 */
static int16_t
mid_up (int32_t a, int32_t b)
{
    int32_t sum = a + b + 1;

    return (int16_t)((sum >= 0) ? (sum / 2) : -((-sum + 1) / 2));
}

static bool
is_near (b5_sample_t const * p_s, int32_t near_mm)
{
    return ((p_s->b_valid != 0U) && ((int32_t)p_s->range_mm < near_mm));
}

/**
 * @brief  Coarse Sweep exit test: "near points found".
 */
bool
b5_scan_has_near (b5_scan_t const * p_scan, int32_t near_mm)
{
    uint8_t i;

    for (i = 0U; i < p_scan->count; i++)
    {
        if (is_near(&p_scan->sample[i], near_mm))
        {
            return true;
        }
    }

    return false;
}

/**
 * @brief  Edge Refinement: next angle to sample, if any.
 *
 * Looks for two neighbouring valid samples where one is near and the other
 * far, and whose angular gap is still wider than alpha. Sampling the
 * midpoint halves that bracket. When no such pair is left, every edge is
 * known to within the beam half-angle and refinement stops ("bracket > alpha"
 * self-transition in the statechart). The flat face between the edges is
 * never re-sampled because it has no near/far transition.
 *
 * @return true and the angle in *p_angle_dd if another sample is needed.
 */
bool
b5_next_refine_angle (b5_scan_t const * p_scan, int32_t near_mm,
                      int16_t alpha_dd, int16_t * p_angle_dd)
{
    uint8_t i;

    for (i = 0U; (i + 1U) < p_scan->count; i++)
    {
        b5_sample_t const * p_a = &p_scan->sample[i];
        b5_sample_t const * p_b = &p_scan->sample[i + 1U];
        int16_t gap = (int16_t)(p_b->angle_dd - p_a->angle_dd);

        if ((p_a->b_valid == 0U) || (p_b->b_valid == 0U))
        {
            continue;
        }

        if ((is_near(p_a, near_mm) != is_near(p_b, near_mm)) &&
            (gap > alpha_dd) && (gap >= 2))
        {
            *p_angle_dd = mid_up(p_a->angle_dd, p_b->angle_dd);
            return true;
        }
    }

    return false;
}

/* ---------------------------------------------------------------------
 * Planning
 * ------------------------------------------------------------------ */

/* Index of the closest valid near sample, or -1. */
static int
closest_near (b5_scan_t const * p_scan, int32_t near_mm)
{
    int     best = -1;
    uint8_t i;

    for (i = 0U; i < p_scan->count; i++)
    {
        if (is_near(&p_scan->sample[i], near_mm) &&
            ((best < 0) ||
             (p_scan->sample[i].range_mm <
              p_scan->sample[best].range_mm)))
        {
            best = (int)i;
        }
    }

    return best;
}

/*
 * Free lateral space beside one edge of the obstacle: the distance from the
 * corrected edge to the nearest other object inside the bypass corridor.
 * dir = +1 looks right (x increasing), -1 looks left.
 */
static int32_t
side_clearance (b5_scan_t const * p_scan, int lo, int hi, int dir,
                int32_t edge_x, int32_t corridor_y, int32_t open_mm)
{
    int32_t best = open_mm;
    uint8_t i;

    for (i = 0U; i < p_scan->count; i++)
    {
        b5_sample_t const * p_s = &p_scan->sample[i];
        int32_t x;
        int32_t y;
        int32_t gap;

        if (((int)i >= lo) && ((int)i <= hi))
        {
            continue;                   /* part of the obstacle itself */
        }

        if (p_s->b_valid == 0U)
        {
            continue;
        }

        /* + = right: x across (sin), y ahead (cos). */
        x = mul_q15((int32_t)p_s->range_mm, b5_sin_q15(p_s->angle_dd));
        y = mul_q15((int32_t)p_s->range_mm, b5_cos_q15(p_s->angle_dd));

        if ((y <= 0) || (y > corridor_y))
        {
            continue;                   /* behind the car or beyond it */
        }

        gap = (dir > 0) ? (x - edge_x) : (edge_x - x);

        if ((gap > 0) && (gap < best))
        {
            best = gap;
        }
    }

    return best;
}

/* Store helpers: clamp into the field width (b5_pack.h). */
static uint32_t
mm_u12 (int32_t mm)
{
    return (uint32_t)b5_clamp(mm, 0, B5_MM_MAX);
}

static int32_t
mm_s12 (int32_t mm)
{
    return b5_clamp(mm, B5_SMM_MIN, B5_SMM_MAX);
}

/**
 * @brief  Planning: obstacle profile, width correction, side choice.
 *
 * The obstacle is the contiguous run of near samples around the closest
 * reading. Samples are sorted by angle, so the low end of the run is the
 * LEFT edge and the high end the RIGHT edge. Each raw edge is the middle
 * of the bracket between the outermost near sample and its far neighbour.
 * Because a near reading means the obstacle is somewhere inside the beam,
 * each raw edge is moved inward by alpha before computing the width; at
 * range d this removes about d*tan(alpha) per edge, the 2*d*tan(alpha)
 * beam-spread error. An edge that reaches the end of the scan arc is not
 * corrected and that side is treated as having no clearance, because its
 * extent is unknown.
 */
void
b5_build_profile (b5_scan_t const * p_scan, b5_params_t const * p_param,
                  b5_profile_t * p_prof)
{
    int       centre;
    int       lo;
    int       hi;
    int16_t   right_dd;
    int16_t   left_dd;
    int32_t   r_right;
    int32_t   r_left;
    int32_t   closest;
    int32_t   x_right;
    int32_t   x_left;
    int32_t   width;
    int32_t   clear_right;
    int32_t   clear_left;
    int32_t   lateral;
    int32_t   corridor_y;
    int32_t   need_mm;
    uint8_t   valid = 0U;
    uint8_t   i;
    bool      b_right_lim;
    bool      b_left_lim;
    bool      b_right_ok;
    bool      b_left_ok;
    b5_side_t side;

    *p_prof = (b5_profile_t){ 0 };

    for (i = 0U; i < p_scan->count; i++)
    {
        if (p_scan->sample[i].b_valid != 0U)
        {
            valid++;
        }
    }

    p_prof->confidence_pct = (p_scan->count == 0U) ? 0U :
        (uint32_t)((valid * 100U) / p_scan->count);

    centre = closest_near(p_scan, p_param->near_mm);

    if (centre < 0)
    {
        p_prof->side = B5_SIDE_NONE;
        return;                         /* no obstacle: back to Guard  */
    }

    closest            = (int32_t)p_scan->sample[centre].range_mm;
    p_prof->b_found    = 1U;
    p_prof->closest_mm = mm_u12(closest);
    p_prof->closest_dd = p_scan->sample[centre].angle_dd;

    /* Grow the run of near samples outwards from the closest one. */
    lo = centre;
    hi = centre;

    while ((lo > 0) && is_near(&p_scan->sample[lo - 1], p_param->near_mm))
    {
        lo--;
    }

    while (((hi + 1) < (int)p_scan->count) &&
           is_near(&p_scan->sample[hi + 1], p_param->near_mm))
    {
        hi++;
    }

    /* Each edge lies inside the bracket between the outermost near sample
     * and its far neighbour; take the middle of that bracket. */
    left_dd  = (int16_t)p_scan->sample[lo].angle_dd;
    right_dd = (int16_t)p_scan->sample[hi].angle_dd;
    r_left   = (int32_t)p_scan->sample[lo].range_mm;
    r_right  = (int32_t)p_scan->sample[hi].range_mm;

    if ((lo > 0) && (p_scan->sample[lo - 1].b_valid != 0U))
    {
        left_dd = mid_up(left_dd, p_scan->sample[lo - 1].angle_dd);
    }

    if (((hi + 1) < (int)p_scan->count) &&
        (p_scan->sample[hi + 1].b_valid != 0U))
    {
        right_dd = mid_up(right_dd, p_scan->sample[hi + 1].angle_dd);
    }

    b_left_lim  = (lo == 0) ||
        (p_scan->sample[lo].angle_dd <= p_param->scan_min_dd);
    b_right_lim = (hi == ((int)p_scan->count - 1)) ||
        (p_scan->sample[hi].angle_dd >= p_param->scan_max_dd);

    p_prof->right_edge_dd    = right_dd;
    p_prof->left_edge_dd     = left_dd;
    p_prof->b_right_at_limit = b_right_lim ? 1U : 0U;
    p_prof->b_left_at_limit  = b_left_lim ? 1U : 0U;
    p_prof->width_raw_mm     = mm_u12(mul_q15(r_right, b5_sin_q15(right_dd)) -
                                      mul_q15(r_left,  b5_sin_q15(left_dd)));

    /* Beam-spread correction: move each known edge inward by alpha. */
    if (!b_right_lim)
    {
        right_dd = (int16_t)(right_dd - p_param->alpha_dd);
    }

    if (!b_left_lim)
    {
        left_dd = (int16_t)(left_dd + p_param->alpha_dd);
    }

    if (left_dd > right_dd)
    {
        /* Narrower than the beam: treat it as a point at the middle. */
        int16_t mid = mid_up(p_prof->right_edge_dd, p_prof->left_edge_dd);
        right_dd = mid;
        left_dd  = mid;
    }

    x_right = mul_q15(r_right, b5_sin_q15(right_dd));
    x_left  = mul_q15(r_left,  b5_sin_q15(left_dd));
    width   = x_right - x_left;

    p_prof->x_right_mm = mm_s12(x_right);
    p_prof->x_left_mm  = mm_s12(x_left);
    p_prof->width_mm   = mm_u12(width);     /* negative -> 0 */

    /* Anything closer than this, beside the obstacle, blocks the bypass. */
    corridor_y = closest + p_param->obst_depth_mm + p_param->car_length_mm;

    clear_right = b_right_lim ? 0 :
        side_clearance(p_scan, lo, hi, +1, x_right, corridor_y,
                       p_param->open_clearance_mm);

    clear_left = b_left_lim ? 0 :
        side_clearance(p_scan, lo, hi, -1, x_left, corridor_y,
                       p_param->open_clearance_mm);

    p_prof->clear_right_mm = mm_u12(clear_right);
    p_prof->clear_left_mm  = mm_u12(clear_left);

    /* A side is passable if the car plus a margin each side fits. */
    need_mm    = p_param->car_width_mm + (2 * p_param->margin_mm);
    b_right_ok = (clear_right >= need_mm);
    b_left_ok  = (clear_left  >= need_mm);

    if (b_right_ok && b_left_ok)
    {
        if (clear_right == clear_left)
        {
            side = p_param->b_prefer_right ? B5_SIDE_RIGHT : B5_SIDE_LEFT;
        }
        else
        {
            side = (clear_right > clear_left) ? B5_SIDE_RIGHT : B5_SIDE_LEFT;
        }
    }
    else if (b_right_ok)
    {
        side = B5_SIDE_RIGHT;
    }
    else if (b_left_ok)
    {
        side = B5_SIDE_LEFT;
    }
    else
    {
        side = B5_SIDE_NONE;            /* -> Blocked                  */
    }

    p_prof->side = (uint32_t)side;

    /* Sideways shift that puts the car's near flank margin_mm past the
     * corrected edge, and forward travel that clears the obstacle. */
    if (side == B5_SIDE_RIGHT)
    {
        lateral = x_right + (p_param->car_width_mm / 2) + p_param->margin_mm;
    }
    else if (side == B5_SIDE_LEFT)
    {
        lateral = -x_left + (p_param->car_width_mm / 2) + p_param->margin_mm;
    }
    else
    {
        lateral = 0;
    }

    p_prof->lateral_mm = mm_u12(lateral);   /* already clear -> 0 */
    p_prof->forward_mm = mm_u12(closest + p_param->obst_depth_mm +
                                p_param->car_length_mm + p_param->margin_mm);
}

/* Conservative navigation profiler. The angular allowances below are model
 * bounds (servo repeatability plus half the local sample bracket), not a
 * calibrated beam cone or a claim of physical width accuracy. */
static int32_t
nav_allowance (int32_t range_mm, int32_t angle_dd)
{
    int32_t s = b5_sin_q15(angle_dd);
    int32_t a = (range_mm * (s < 0 ? -s : s)) / Q15_ONE;
    return a + B5_PROFILE_RANGE_ALLOW_MM;
}

/* Intersect a ray with the forward-depth plane. The ray range is radial, so
 * x at y=corridor_y is y*tan(angle), not y*sin(angle). */
static bool
nav_x_on_plane (int32_t corridor_y, int32_t angle_dd, int32_t * p_x)
{
    int32_t c = b5_cos_q15(angle_dd);
    int32_t s = b5_sin_q15(angle_dd);

    if (c <= 0)
    {
        return false;
    }
    *p_x = (int32_t)(((int64_t)corridor_y * s) / c);
    return true;
}

/* Return the observed free-space bound on one side, or zero if evidence is
 * incomplete. Two adjacent valid rays must reach past the whole corridor. */
/* Rule B: unknown and block-only samples never support clearance. A
 * partial sample (echo + one fault) supports clearance only up to its
 * nearest echo, and only as weak evidence, like a no-return sample. */
static bool
nav_clear_usable (b5_sample_t const * p_s)
{
    return (p_s->b_valid != 0U) && (p_s->b_block_only == 0U);
}

static bool
nav_is_weak (b5_sample_t const * p_s)
{
    return (p_s->b_weak != 0U) || (p_s->b_partial != 0U);
}

/* A sample whose range is a measured return (echo, possibly partial). */
static bool
nav_has_echo (b5_sample_t const * p_s)
{
    return (p_s->b_valid != 0U) && (p_s->b_weak == 0U);
}

/* lim_dd: only samples with |angle| <= lim_dd are used, so both sides are
 * measured over the same angles when the sweep is asymmetric (2026-11-01). */
static int32_t
nav_side_clearance (b5_scan_t const * p_scan, int32_t near_edge_x,
                    int32_t edge_angle, int32_t corridor_y, int dir,
                    int32_t lim_dd, bool * p_b_weak)
{
    int32_t best = 0;
    int32_t start = 0;
    int32_t i;
    bool    b_pair = false;

    if (dir > 0)
    {
        while ((start < (int32_t)p_scan->count) &&
               ((int32_t)p_scan->sample[start].angle_dd <= edge_angle))
        {
            start++;
        }
    }
    else
    {
        start = (int32_t)p_scan->count - 1;
        while ((start >= 0) &&
               ((int32_t)p_scan->sample[start].angle_dd >= edge_angle))
        {
            start--;
        }
    }

    *p_b_weak = false;
    if ((start < 0) || (start >= (int32_t)p_scan->count) ||
        !nav_clear_usable(&p_scan->sample[start]) ||
        ((int32_t)p_scan->sample[start].angle_dd > lim_dd) ||
        ((int32_t)p_scan->sample[start].angle_dd < -lim_dd) ||
        ((dir > 0) ?
         ((int32_t)p_scan->sample[start].angle_dd - edge_angle >
          B5_PROFILE_MAX_GAP_DD) :
         (edge_angle - (int32_t)p_scan->sample[start].angle_dd >
          B5_PROFILE_MAX_GAP_DD)))
    {
        return 0;                       /* unknown wedge beside this edge */
    }

    /* Walk outward from the obstacle. An invalid return, large gap or
     * nearer return ends coverage; later samples cannot bridge that unknown. */
    for (i = start; (dir > 0) ? (i + 1 < (int32_t)p_scan->count) : (i > 0);
         i += dir)
    {
        int32_t ia = (dir > 0) ? i : i - 1;
        int32_t ib = (dir > 0) ? i + 1 : i;
        b5_sample_t const * a = &p_scan->sample[ia];
        b5_sample_t const * b = &p_scan->sample[ib];
        int32_t gap = (int32_t)b->angle_dd - a->angle_dd;
        int32_t ya;
        int32_t yb;
        int32_t outer_x;
        int32_t outer_angle;
        int32_t allowance;
        int32_t clear;

        if (!nav_clear_usable(a) || !nav_clear_usable(b) ||
            (gap <= 0) || (gap > B5_PROFILE_MAX_GAP_DD) ||
            ((int32_t)a->angle_dd < -lim_dd) || ((int32_t)b->angle_dd > lim_dd))
        {
            break;                      /* evidence or comparison window ends */
        }
        ya = mul_q15((int32_t)a->range_mm, b5_cos_q15(a->angle_dd));
        yb = mul_q15((int32_t)b->range_mm, b5_cos_q15(b->angle_dd));
        if ((ya < (corridor_y + B5_PROFILE_RANGE_ALLOW_MM)) ||
            (yb < (corridor_y + B5_PROFILE_RANGE_ALLOW_MM)))
        {
            break;
        }

        outer_angle = (dir > 0) ? (b->angle_dd - B5_PROFILE_SERVO_ALLOW_DD -
                                   (gap / 2)) :
                                 (a->angle_dd + B5_PROFILE_SERVO_ALLOW_DD +
                                  (gap / 2));
        if (!nav_x_on_plane(corridor_y, outer_angle, &outer_x))
        {
            break;
        }
        allowance = B5_PROFILE_RANGE_ALLOW_MM;
        clear = (dir > 0) ? outer_x - near_edge_x - allowance :
                            near_edge_x - outer_x - allowance;
        if (clear < 0) { clear = 0; }
        best = clear;
        b_pair = true;
        if (nav_is_weak(a) || nav_is_weak(b))
        {
            *p_b_weak = true;           /* tentative: no-return / partial */
        }
    }

    if (!b_pair) { return 0; }

    /* Any second return inside the bypass corridor limits the measured gap. */
    for (i = 0U; i < p_scan->count; i++)
    {
        b5_sample_t const * s = &p_scan->sample[i];
        int32_t x;
        int32_t y;
        int32_t gap;

        if (!nav_has_echo(s) ||
            ((int32_t)s->angle_dd > lim_dd) || ((int32_t)s->angle_dd < -lim_dd))
        {
            continue;
        }
        if ((dir > 0) ? ((int32_t)s->angle_dd <= edge_angle) :
                        ((int32_t)s->angle_dd >= edge_angle))
        {
            continue;
        }
        x = mul_q15((int32_t)s->range_mm, b5_sin_q15(s->angle_dd));
        y = mul_q15((int32_t)s->range_mm, b5_cos_q15(s->angle_dd));
        if ((y <= 0) || (y > corridor_y))
        {
            continue;
        }
        gap = (dir > 0) ? (x - near_edge_x) : (near_edge_x - x);
        if (gap <= 0)
        {
            return 0;                   /* an outer return overlaps the envelope */
        }
        if (gap < best)
        {
            best = gap;
        }
    }
    return best;
}

/* ---------------------------------------------------------------------
 * Swept-footprint check of the bypass manoeuvre (rule B condition 4)
 * ------------------------------------------------------------------ */

enum { PT_FREE = 0, PT_WEAK, PT_ASSUMED, PT_FAIL };

typedef struct
{
    b5_scan_t const *   p_scan;
    b5_sweep_report_t * p_rep;
    int32_t env_xl;             /* planning envelope, sensor frame */
    int32_t env_xr;
    int32_t env_y0;
    int32_t env_y1;
    int32_t obst_lo_dd;         /* angular bracket of the main obstacle */
    int32_t obst_hi_dd;
    int32_t hw;                 /* car half-width + margin            */
    int32_t vf;                 /* pivot to front + margin            */
    int32_t vr;                 /* pivot to rear + margin (negative)  */
    uint8_t leg;
    bool    b_fail;
} sweep_ctx_t;

/* Which side of sample ray i the point lies: > 0 right (larger angle).
 * Footprint coordinates stay within +/-4096 mm, so x * Q15 fits 32 bits
 * (no 64-bit multiply on the Cortex-M0+). */
static int32_t
sweep_side (b5_sample_t const * p_s, int32_t x, int32_t y)
{
    return (x * b5_cos_q15(p_s->angle_dd)) - (y * b5_sin_q15(p_s->angle_dd));
}

/* What one scan ray says about a point at squared distance rho2. */
static uint8_t
sweep_ray (sweep_ctx_t const * p_c, b5_sample_t const * p_s, int32_t rho2,
           uint8_t * p_why)
{
    int32_t r;

    if (p_s->b_valid == 0U)
    {
        *p_why = (uint8_t)B5_SWEEP_WHY_UNKNOWN;
        return PT_FAIL;                 /* scanned, but no usable evidence */
    }
    if (p_s->b_weak != 0U)
    {
        return PT_WEAK;                 /* clean no-returns: tentative */
    }
    r = (int32_t)p_s->range_mm - B5_PROFILE_RANGE_ALLOW_MM;
    if (r < 0) { r = 0; }
    if (rho2 < (r * r))
    {
        if (p_s->b_block_only != 0U)
        {
            *p_why = (uint8_t)B5_SWEEP_WHY_UNKNOWN;
            return PT_FAIL;             /* mostly faults: unknown nearer */
        }
        return (p_s->b_partial != 0U) ? PT_WEAK : PT_FREE;
    }
    if (((int32_t)p_s->angle_dd >= p_c->obst_lo_dd) &&
        ((int32_t)p_s->angle_dd <= p_c->obst_hi_dd))
    {
        return PT_ASSUMED;              /* occluded behind the obstacle */
    }
    *p_why = (uint8_t)B5_SWEEP_WHY_BEYOND;
    return PT_FAIL;                     /* in or behind a side return */
}

/* Classify one footprint outline point (sensor frame). */
static uint8_t
sweep_point (sweep_ctx_t const * p_c, int32_t x, int32_t y, uint8_t * p_why)
{
    b5_scan_t const * p_scan = p_c->p_scan;
    int32_t a;
    int32_t b;
    uint8_t ca;
    uint8_t cb;
    int32_t rho2;

    if ((x >= p_c->env_xl) && (x <= p_c->env_xr) &&
        (y >= p_c->env_y0) && (y <= p_c->env_y1))
    {
        *p_why = (uint8_t)B5_SWEEP_WHY_ENVELOPE;
        return PT_FAIL;
    }
    if (y <= 0)
    {
        return PT_ASSUMED;              /* behind the sensor: never scanned */
    }
    if ((x > B5_MM_MAX) || (x < -B5_MM_MAX) || (y > B5_MM_MAX))
    {
        return PT_ASSUMED;              /* beyond sensor range: not scanned */
    }
    if (p_scan->count == 0U)
    {
        *p_why = (uint8_t)B5_SWEEP_WHY_UNKNOWN;
        return PT_FAIL;
    }
    if ((sweep_side(&p_scan->sample[0], x, y) < 0) ||
        (sweep_side(&p_scan->sample[p_scan->count - 1U], x, y) > 0))
    {
        return PT_ASSUMED;              /* outside the scanned arc */
    }

    a = 0;
    while (((a + 1) < (int32_t)p_scan->count) &&
           (sweep_side(&p_scan->sample[a + 1], x, y) >= 0))
    {
        a++;
    }
    b = (sweep_side(&p_scan->sample[a], x, y) == 0) ? a :
        (((a + 1) < (int32_t)p_scan->count) ? (a + 1) : a);
    if (((int32_t)p_scan->sample[b].angle_dd -
         (int32_t)p_scan->sample[a].angle_dd) > B5_PROFILE_MAX_GAP_DD)
    {
        *p_why = (uint8_t)B5_SWEEP_WHY_UNKNOWN;
        return PT_FAIL;                 /* evidence gap in the arc */
    }

    rho2 = (x * x) + (y * y);
    ca = sweep_ray(p_c, &p_scan->sample[a], rho2, p_why);
    if (ca == PT_FAIL) { return PT_FAIL; }
    cb = sweep_ray(p_c, &p_scan->sample[b], rho2, p_why);
    if (cb == PT_FAIL) { return PT_FAIL; }
    return (ca > cb) ? ca : cb;
}

static void
sweep_fail (sweep_ctx_t * p_c, uint8_t why, int32_t x, int32_t y)
{
    if (!p_c->b_fail)
    {
        p_c->b_fail = true;
        p_c->p_rep->why = why;
        p_c->p_rep->fail_leg = p_c->leg;
        p_c->p_rep->fail_x_mm = (int16_t)b5_clamp(x, -32768, 32767);
        p_c->p_rep->fail_y_mm = (int16_t)b5_clamp(y, -32768, 32767);
    }
}

/* Is the sensor-frame point inside the grown car at pose (px, py, phi)? */
static bool
sweep_inside (sweep_ctx_t const * p_c, int32_t px, int32_t py,
              int32_t c, int32_t s, int32_t x, int32_t y)
{
    int32_t dx = x - px;
    int32_t dy = y - py;
    int32_t u = ((dx * c) - (dy * s)) / Q15_ONE;
    int32_t v = ((dx * s) + (dy * c)) / Q15_ONE;

    return (u >= -p_c->hw) && (u <= p_c->hw) && (v >= p_c->vr) && (v <= p_c->vf);
}

/* Check the grown car rectangle at one pose: pivot (px, py), heading phi
 * (+ = right). Outline points every B5_SWEEP_EDGE_STEP_MM. */
static void
sweep_pose (sweep_ctx_t * p_c, int32_t px, int32_t py, int32_t phi_dd)
{
    int32_t c = b5_cos_q15(phi_dd);
    int32_t s = b5_sin_q15(phi_dd);
    int32_t cu[5];
    int32_t cv[5];
    uint8_t e;
    uint8_t i;

    if (p_c->b_fail) { return; }

    /* Measured returns and envelope corners inside the car: fail. */
    for (i = 0U; i < p_c->p_scan->count; i++)
    {
        b5_sample_t const * p_s = &p_c->p_scan->sample[i];
        int32_t x;
        int32_t y;

        if (!nav_has_echo(p_s)) { continue; }
        x = mul_q15((int32_t)p_s->range_mm, b5_sin_q15(p_s->angle_dd));
        y = mul_q15((int32_t)p_s->range_mm, b5_cos_q15(p_s->angle_dd));
        if (sweep_inside(p_c, px, py, c, s, x, y))
        {
            sweep_fail(p_c, (uint8_t)B5_SWEEP_WHY_RETURN, x, y);
            return;
        }
    }
    {
        int32_t ex[4] = { p_c->env_xl, p_c->env_xr, p_c->env_xr, p_c->env_xl };
        int32_t ey[4] = { p_c->env_y0, p_c->env_y0, p_c->env_y1, p_c->env_y1 };
        for (i = 0U; i < 4U; i++)
        {
            if (sweep_inside(p_c, px, py, c, s, ex[i], ey[i]))
            {
                sweep_fail(p_c, (uint8_t)B5_SWEEP_WHY_ENVELOPE, ex[i], ey[i]);
                return;
            }
        }
    }

    /* Outline: front-left, front-right, rear-right, rear-left, back. */
    cu[0] = -p_c->hw; cv[0] = p_c->vf;
    cu[1] =  p_c->hw; cv[1] = p_c->vf;
    cu[2] =  p_c->hw; cv[2] = p_c->vr;
    cu[3] = -p_c->hw; cv[3] = p_c->vr;
    cu[4] = cu[0];    cv[4] = cv[0];

    for (e = 0U; e < 4U; e++)
    {
        int32_t du = cu[e + 1U] - cu[e];
        int32_t dv = cv[e + 1U] - cv[e];
        int32_t len = (du < 0 ? -du : du) + (dv < 0 ? -dv : dv);
        int32_t n = (len + B5_SWEEP_EDGE_STEP_MM - 1) / B5_SWEEP_EDGE_STEP_MM;
        int32_t k;

        for (k = 0; k < n; k++)
        {
            int32_t u = cu[e] + ((du * k) / n);
            int32_t v = cv[e] + ((dv * k) / n);
            int32_t x = px + (((u * c) + (v * s)) / Q15_ONE);
            int32_t y = py + (((v * c) - (u * s)) / Q15_ONE);
            uint8_t why = (uint8_t)B5_SWEEP_WHY_NONE;
            uint8_t cls = sweep_point(p_c, x, y, &why);

            p_c->p_rep->n_points++;
            if (cls == PT_FAIL)
            {
                sweep_fail(p_c, why, x, y);
                return;
            }
            if (cls == PT_ASSUMED) { p_c->p_rep->n_assumed++; }
            if (cls == PT_WEAK)    { p_c->p_rep->n_weak++; }
        }
    }
}

static void
sweep_spin (sweep_ctx_t * p_c, int32_t px, int32_t py, int32_t from_dd,
            int32_t to_dd)
{
    int32_t step = (to_dd >= from_dd) ? B5_SWEEP_STEP_DD : -B5_SWEEP_STEP_DD;
    int32_t phi = from_dd;

    for (;;)
    {
        sweep_pose(p_c, px, py, phi);
        if ((phi == to_dd) || p_c->b_fail) { break; }
        phi += step;
        if (((step > 0) && (phi > to_dd)) || ((step < 0) && (phi < to_dd)))
        {
            phi = to_dd;
        }
    }
}

/* Drive dist_mm along heading phi from (*p_x, *p_y); updates the pivot. */
static void
sweep_drive (sweep_ctx_t * p_c, int32_t * p_x, int32_t * p_y,
             int32_t phi_dd, int32_t dist_mm)
{
    int32_t x0 = *p_x;
    int32_t y0 = *p_y;
    int32_t s = b5_sin_q15(phi_dd);
    int32_t c = b5_cos_q15(phi_dd);
    int32_t d = 0;

    if (dist_mm <= 0) { return; }
    for (;;)
    {
        *p_x = x0 + mul_q15(d, s);
        *p_y = y0 + mul_q15(d, c);
        sweep_pose(p_c, *p_x, *p_y, phi_dd);
        if ((d == dist_mm) || p_c->b_fail) { break; }
        d += B5_SWEEP_STEP_MM;
        if (d > dist_mm) { d = dist_mm; }
    }
}

/**
 * @brief  Check the full bypass footprint on one side.
 *
 * Replays the Navigation Coordinator's manoeuvre (spin dir*45 on the spot,
 * lateral*sqrt2 diagonal, spin back, forward-lateral pass, spin -dir*45,
 * lateral*sqrt2 + search rejoin, capped) with the car rectangle grown by
 * the margin. FAIL if it overlaps the planning envelope, contains a
 * measured return, reaches behind a side return or crosses scanned space
 * with no usable evidence. PASS_ASSUMED if parts lie where the scan cannot
 * see (behind the sensor, outside the arc, occluded behind the obstacle).
 */
uint8_t
b5_check_sweep (b5_scan_t const * p_scan, b5_params_t const * p_param,
                b5_profile_t const * p_prof, b5_side_t side,
                int32_t lateral_mm, int32_t forward_mm, int32_t search_mm,
                b5_sweep_report_t * p_rep)
{
    sweep_ctx_t c;
    int32_t dir = (side == B5_SIDE_LEFT) ? -1 : 1;
    int32_t turn = dir * B5_SWEEP_TURN_DD;
    int32_t px = 0;
    int32_t py = B5_SENSOR_BEHIND_FRONT_MM - B5_PIVOT_TO_FRONT_MM;
    int32_t diag = (lateral_mm * 1414) / 1000;
    int32_t rejoin = diag + search_mm;

    *p_rep = (b5_sweep_report_t){ 0 };
    if ((side == B5_SIDE_NONE) || (lateral_mm <= 0))
    {
        p_rep->result = (uint8_t)B5_SWEEP_NOT_RUN;
        return p_rep->result;
    }
    if (rejoin > B5_SWEEP_REJOIN_MAX_MM) { rejoin = B5_SWEEP_REJOIN_MAX_MM; }

    c.p_scan = p_scan;
    c.p_rep = p_rep;
    c.env_xl = (int32_t)p_prof->x_left_mm;
    c.env_xr = (int32_t)p_prof->x_right_mm;
    c.env_y0 = (int32_t)p_prof->closest_mm - B5_PROFILE_RANGE_ALLOW_MM;
    c.env_y1 = (int32_t)p_prof->closest_mm + p_param->obst_depth_mm;
    c.obst_lo_dd = (int32_t)p_prof->left_edge_dd;
    c.obst_hi_dd = (int32_t)p_prof->right_edge_dd;
    c.hw = (p_param->car_width_mm / 2) + p_param->margin_mm;
    c.vf = B5_PIVOT_TO_FRONT_MM + p_param->margin_mm;
    c.vr = -((p_param->car_length_mm - B5_PIVOT_TO_FRONT_MM) + p_param->margin_mm);
    c.b_fail = false;

    c.leg = 1U; sweep_spin(&c, px, py, 0, turn);
    c.leg = 2U; sweep_drive(&c, &px, &py, turn, diag);
    c.leg = 3U; sweep_spin(&c, px, py, turn, 0);
    c.leg = 4U; sweep_drive(&c, &px, &py, 0, forward_mm - lateral_mm);
    c.leg = 5U; sweep_spin(&c, px, py, 0, -turn);
    c.leg = 6U; sweep_drive(&c, &px, &py, -turn, rejoin);

    p_rep->b_weak = (p_rep->n_weak > 0U) ? 1U : 0U;
    if (c.b_fail)
    {
        p_rep->result = (uint8_t)B5_SWEEP_FAIL;
    }
    else if (((B5_SWEEP_ALLOW_ASSUMED == 0) && (p_rep->n_assumed > 0U)) ||
             ((B5_SWEEP_ALLOW_WEAK == 0) && (p_rep->n_weak > 0U)))
    {
        p_rep->result = (uint8_t)B5_SWEEP_FAIL;
        p_rep->why = (uint8_t)B5_SWEEP_WHY_POLICY;
    }
    else
    {
        p_rep->result = (p_rep->n_assumed > 0U) ? (uint8_t)B5_SWEEP_PASS_ASSUMED
                                                : (uint8_t)B5_SWEEP_PASS;
    }
    return p_rep->result;
}

void
b5_build_navigation_profile (b5_scan_t const * p_scan,
                             b5_params_t const * p_param,
                             b5_profile_t * p_prof)
{
    b5_build_navigation_profile_ex(p_scan, p_param, p_prof, NULL, NULL);
}

/**
 * @brief  As b5_build_navigation_profile, also returning each side's
 *         swept-footprint report (either pointer may be NULL).
 */
void
b5_build_navigation_profile_ex (b5_scan_t const * p_scan,
                                b5_params_t const * p_param,
                                b5_profile_t * p_prof,
                                b5_sweep_report_t * p_rep_l,
                                b5_sweep_report_t * p_rep_r)
{
    int32_t core_count = 0;
    int32_t lo = -1;
    int32_t hi = -1;
    int32_t closest = B5_MM_MAX;
    int32_t closest_i = -1;
    int32_t max_range = 0;
    int32_t min_x = B5_SMM_MAX;
    int32_t max_x = B5_SMM_MIN;
    int32_t plan_min_x;
    int32_t plan_max_x;
    int32_t uncertainty;
    int32_t corridor_y;
    int32_t need_mm;
    int32_t clear_l;
    int32_t clear_r;
    int32_t prev_support = -1;
    int32_t st_min_x = B5_SMM_MAX;
    int32_t st_max_x = B5_SMM_MIN;
    int32_t n_stable = 0;
    int32_t lat_l;
    int32_t lat_r;
    int32_t sym_dd;
    bool    b_supported = false;
    bool    b_weak_l = false;
    bool    b_weak_r = false;
    bool    b_pass_l;
    bool    b_pass_r;
    uint8_t valid = 0U;
    uint8_t measured = 0U;
    uint8_t i;
    b5_sweep_report_t rep_l;
    b5_sweep_report_t rep_r;

    *p_prof = (b5_profile_t){ 0 };
    p_prof->side = B5_SIDE_NONE;
    if (p_rep_l != NULL) { *p_rep_l = (b5_sweep_report_t){ 0 }; }
    if (p_rep_r != NULL) { *p_rep_r = (b5_sweep_report_t){ 0 }; }

    for (i = 0U; i < p_scan->count; i++)
    {
        b5_sample_t const * s = &p_scan->sample[i];
        int32_t range = (int32_t)s->range_mm;

        if (s->b_valid == 0U)
        {
            prev_support = -1;
            continue;
        }
        if (s->b_block_only == 0U)
        {
            valid++;                    /* usable for decisions */
        }
        if (s->b_weak == 0U)
        {
            measured++;                 /* weak clears never count here */
        }
        if (range < (p_param->near_mm - B5_PROFILE_NEAR_HYST_MM))
        {
            core_count++;
        }
        if (range < (p_param->near_mm + B5_PROFILE_NEAR_HYST_MM))
        {
            if (lo < 0) { lo = (int32_t)i; }
            hi = (int32_t)i;
            if (range < closest) { closest = range; closest_i = (int32_t)i; }
            if (range > max_range) { max_range = range; }
            {
                int32_t x = mul_q15(range, b5_sin_q15(s->angle_dd));
                if (x < min_x) { min_x = x; }
                if (x > max_x) { max_x = x; }
                if (s->b_stable != 0U)
                {
                    n_stable++;
                    if (x < st_min_x) { st_min_x = x; }
                    if (x > st_max_x) { st_max_x = x; }
                }
            }
            if ((prev_support >= 0) &&
                (((int32_t)s->angle_dd - p_scan->sample[prev_support].angle_dd) <=
                 B5_PROFILE_MAX_GAP_DD))
            {
                b_supported = true;
            }
            prev_support = (int32_t)i;
        }
        else if (prev_support >= 0)
        {
            prev_support = -1;
        }
    }

    /* Measured confidence counts echoes only; coverage adds weak clears. */
    p_prof->confidence_pct = (p_scan->count == 0U) ? 0U :
        (uint32_t)((measured * 100U) / p_scan->count);
    p_prof->coverage_pct = (p_scan->count == 0U) ? 0U :
        (uint32_t)((valid * 100U) / p_scan->count);
    if (lo < 0)
    {
        return;                         /* absence is not clearance evidence */
    }

    p_prof->b_found = 1U;
    p_prof->closest_mm = mm_u12(closest);
    p_prof->closest_dd = (closest_i >= 0) ? p_scan->sample[closest_i].angle_dd :
                                            p_scan->sample[lo].angle_dd;
    p_prof->left_edge_dd = p_scan->sample[lo].angle_dd;
    p_prof->right_edge_dd = p_scan->sample[hi].angle_dd;
    p_prof->width_raw_mm = mm_u12(max_x - min_x);
    p_prof->width_mm = mm_u12(max_x - min_x); /* observed span: never beam-shrunk */
    p_prof->stable_span_mm = (n_stable >= 2) ? mm_u12(st_max_x - st_min_x) : 0U;
    p_prof->b_left_at_limit = (p_scan->sample[lo].angle_dd <= p_param->scan_min_dd);
    p_prof->b_right_at_limit = (p_scan->sample[hi].angle_dd >= p_param->scan_max_dd);

    if ((core_count < 2) || !b_supported)
    {
        return;                         /* isolated/noisy hit: unknown, blocked */
    }
    p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_SUPPORTED;

    /* Use the widest local bracket near the observed edges. */
    {
        int32_t half_bracket = 0;
        for (i = 0U; (i + 1U) < p_scan->count; i++)
        {
            int32_t gap = p_scan->sample[i + 1U].angle_dd - p_scan->sample[i].angle_dd;
            if ((gap > half_bracket) && (gap <= B5_PROFILE_MAX_GAP_DD))
            {
                half_bracket = gap;
            }
        }
        uncertainty = nav_allowance(max_range,
                                    B5_PROFILE_SERVO_ALLOW_DD + (half_bracket / 2));
    }

    /* Total span growth: one modeled allowance at each observed edge. */
    p_prof->span_uncertainty_mm = mm_u12(2 * uncertainty);
    plan_min_x = min_x;
    plan_max_x = max_x;
    if ((plan_max_x - plan_min_x) < B5_EXPECTED_MAX_WIDTH_MM)
    {
        int32_t centre_x = (plan_min_x + plan_max_x) / 2;
        plan_min_x = centre_x - (B5_EXPECTED_MAX_WIDTH_MM / 2);
        plan_max_x = centre_x + (B5_EXPECTED_MAX_WIDTH_MM / 2);
    }
    p_prof->x_left_mm = mm_s12(plan_min_x - uncertainty);
    p_prof->x_right_mm = mm_s12(plan_max_x + uncertainty);
    p_prof->planning_width_mm = mm_u12((plan_max_x - plan_min_x) +
                                        (2 * uncertainty));

    corridor_y = closest + p_param->obst_depth_mm + p_param->car_length_mm;
    /* Side comparison over the same angles on both sides: with an
     * asymmetric sweep the longer side would otherwise always show more
     * room (2026-11-01 bench: LEFT in 17 of 18 scans whatever the offset).
     * The swept-footprint check below still uses every sample. */
    sym_dd = -(int32_t)p_param->scan_min_dd;
    if ((int32_t)p_param->scan_max_dd < sym_dd)
    {
        sym_dd = (int32_t)p_param->scan_max_dd;
    }
    if (B5_SIDE_COMPARE_SYMMETRIC == 0)
    {
        sym_dd = B5_ANGLE_MAX_DD;
    }
    clear_l = (p_prof->b_left_at_limit != 0U) ? 0 :
        nav_side_clearance(p_scan, plan_min_x - uncertainty,
                           p_scan->sample[lo].angle_dd, corridor_y, -1,
                           sym_dd, &b_weak_l);
    clear_r = (p_prof->b_right_at_limit != 0U) ? 0 :
        nav_side_clearance(p_scan, plan_max_x + uncertainty,
                           p_scan->sample[hi].angle_dd, corridor_y, +1,
                           sym_dd, &b_weak_r);
    p_prof->clear_left_mm = mm_u12(clear_l);
    p_prof->clear_right_mm = mm_u12(clear_r);
    need_mm = p_param->car_width_mm + (2 * p_param->margin_mm);

    if ((clear_l >= need_mm) || (clear_r >= need_mm))
    {
        p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_CLEAR_SIDE;
    }
    if (clear_l > 0) { p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_SCAN_LEFT; }
    if (clear_r > 0) { p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_SCAN_RIGHT; }

    p_prof->forward_mm = mm_u12(closest + p_param->obst_depth_mm +
                                p_param->car_length_mm + p_param->margin_mm);

    /* Sideways move per side: past the envelope edge + half car + margin,
     * never less than the worst-case floor (2026-10-31). */
    lat_r = plan_max_x + uncertainty + (p_param->car_width_mm / 2) + p_param->margin_mm;
    lat_l = -plan_min_x + uncertainty + (p_param->car_width_mm / 2) + p_param->margin_mm;
    if (lat_r < B5_LATERAL_MIN_MM) { lat_r = B5_LATERAL_MIN_MM; }
    if (lat_l < B5_LATERAL_MIN_MM) { lat_l = B5_LATERAL_MIN_MM; }

    /* A side is passable only if the corridor gap is wide enough AND the
     * whole manoeuvre footprint (turns, diagonal, pass, rejoin) does not
     * fail against the scan. The gap test alone does not validate it. */
    rep_l = (b5_sweep_report_t){ 0 };
    rep_r = (b5_sweep_report_t){ 0 };
    if (clear_l >= need_mm)
    {
        (void)b5_check_sweep(p_scan, p_param, p_prof, B5_SIDE_LEFT, lat_l,
                             (int32_t)p_prof->forward_mm, B5_SEARCH_WIDTH_MM,
                             &rep_l);
    }
    if (clear_r >= need_mm)
    {
        (void)b5_check_sweep(p_scan, p_param, p_prof, B5_SIDE_RIGHT, lat_r,
                             (int32_t)p_prof->forward_mm, B5_SEARCH_WIDTH_MM,
                             &rep_r);
    }
    p_prof->sweep_left = rep_l.result;
    p_prof->sweep_right = rep_r.result;
    if (p_rep_l != NULL) { *p_rep_l = rep_l; }
    if (p_rep_r != NULL) { *p_rep_r = rep_r; }
    if (rep_l.result == (uint8_t)B5_SWEEP_FAIL)
    {
        p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_SWEEP_FAIL_L;
    }
    if (rep_r.result == (uint8_t)B5_SWEEP_FAIL)
    {
        p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_SWEEP_FAIL_R;
    }
    b_pass_l = (rep_l.result == (uint8_t)B5_SWEEP_PASS) ||
               (rep_l.result == (uint8_t)B5_SWEEP_PASS_ASSUMED);
    b_pass_r = (rep_r.result == (uint8_t)B5_SWEEP_PASS) ||
               (rep_r.result == (uint8_t)B5_SWEEP_PASS_ASSUMED);

    /* Tie band only among sides that passed every check. */
    if (b_pass_l && b_pass_r)
    {
        int32_t diff = clear_l - clear_r;
        if ((diff <= B5_SIDE_TIE_MM) && (diff >= -B5_SIDE_TIE_MM))
        {   /* 2026-10-31: near-tie -> preferred side, not noise */
            p_prof->side = p_param->b_prefer_right ? B5_SIDE_RIGHT : B5_SIDE_LEFT;
        }
        else
        {
            p_prof->side = (clear_r > clear_l) ? B5_SIDE_RIGHT : B5_SIDE_LEFT;
        }
    }
    else if (b_pass_l)
    {
        p_prof->side = B5_SIDE_LEFT;
    }
    else if (b_pass_r)
    {
        p_prof->side = B5_SIDE_RIGHT;
    }

    if (p_prof->side == B5_SIDE_RIGHT)
    {
        p_prof->lateral_mm = mm_u12(lat_r);
        if (b_weak_r || (rep_r.b_weak != 0U))
        {
            p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_WEAK_CLEAR;
        }
        if (rep_r.result == (uint8_t)B5_SWEEP_PASS_ASSUMED)
        {
            p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_SWEEP_ASSUMED;
        }
    }
    else if (p_prof->side == B5_SIDE_LEFT)
    {
        p_prof->lateral_mm = mm_u12(lat_l);
        if (b_weak_l || (rep_l.b_weak != 0U))
        {
            p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_WEAK_CLEAR;
        }
        if (rep_l.result == (uint8_t)B5_SWEEP_PASS_ASSUMED)
        {
            p_prof->evidence_flags |= B5_PROFILE_EVIDENCE_SWEEP_ASSUMED;
        }
    }
}
