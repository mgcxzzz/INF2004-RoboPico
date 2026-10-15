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
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#include "obstacle_profile.h"

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
    uint8_t idx = 0U;
    uint8_t move;

    while ((idx < p_scan->count) && (p_scan->sample[idx].angle_dd < angle_dd))
    {
        idx++;
    }

    if ((idx < p_scan->count) && (p_scan->sample[idx].angle_dd == angle_dd))
    {
        p_scan->sample[idx].range_mm = range_mm;
        p_scan->sample[idx].b_valid  = b_valid;
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

    p_scan->sample[idx].angle_dd = angle_dd;
    p_scan->sample[idx].range_mm = range_mm;
    p_scan->sample[idx].b_valid  = b_valid;
    p_scan->count++;

    return true;
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

static bool
is_near (b5_sample_t const * p_s, int32_t near_mm)
{
    return (p_s->b_valid && (p_s->range_mm < near_mm));
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

        if (!p_a->b_valid || !p_b->b_valid)
        {
            continue;
        }

        if ((is_near(p_a, near_mm) != is_near(p_b, near_mm)) &&
            (gap > alpha_dd) && (gap >= 2))
        {
            *p_angle_dd = (int16_t)(p_a->angle_dd + (gap / 2));
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
             (p_scan->sample[i].range_mm < p_scan->sample[best].range_mm)))
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

        if (!p_s->b_valid)
        {
            continue;
        }

        x = mul_q15(p_s->range_mm, b5_cos_q15(p_s->angle_dd));
        y = mul_q15(p_s->range_mm, b5_sin_q15(p_s->angle_dd));

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

/**
 * @brief  Planning: obstacle profile, width correction, side choice.
 *
 * The obstacle is the contiguous run of near samples around the closest
 * reading. Each raw edge is the middle of the bracket between the outermost
 * near sample and its far neighbour. Because a near
 * reading means the obstacle is somewhere inside the beam, each raw edge is
 * moved inward by alpha before computing the width; at range d this removes
 * about d*tan(alpha) per edge, the 2*d*tan(alpha) beam-spread error.
 * An edge that reaches the end of the scan arc is not corrected and that
 * side is treated as having no clearance, because its extent is unknown.
 */
void
b5_build_profile (b5_scan_t const * p_scan, b5_params_t const * p_param,
                  b5_profile_t * p_prof)
{
    int     centre;
    int     lo;
    int     hi;
    int16_t right_dd;
    int16_t left_dd;
    int32_t r_right;
    int32_t r_left;
    int32_t corridor_y;
    int32_t need_mm;
    uint8_t valid = 0U;
    uint8_t i;
    bool    b_right_ok;
    bool    b_left_ok;

    *p_prof = (b5_profile_t){ 0 };

    for (i = 0U; i < p_scan->count; i++)
    {
        if (p_scan->sample[i].b_valid)
        {
            valid++;
        }
    }

    p_prof->confidence_pct = (p_scan->count == 0U) ? 0U :
        (uint8_t)((valid * 100U) / p_scan->count);

    centre = closest_near(p_scan, p_param->near_mm);

    if (centre < 0)
    {
        p_prof->side = B5_SIDE_NONE;
        return;                         /* no obstacle: back to Guard  */
    }

    p_prof->b_found    = true;
    p_prof->closest_mm = p_scan->sample[centre].range_mm;
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
    right_dd = p_scan->sample[lo].angle_dd;
    left_dd  = p_scan->sample[hi].angle_dd;
    r_right  = p_scan->sample[lo].range_mm;
    r_left   = p_scan->sample[hi].range_mm;

    if ((lo > 0) && p_scan->sample[lo - 1].b_valid)
    {
        right_dd = (int16_t)((right_dd + p_scan->sample[lo - 1].angle_dd) / 2);
    }

    if (((hi + 1) < (int)p_scan->count) && p_scan->sample[hi + 1].b_valid)
    {
        left_dd = (int16_t)((left_dd + p_scan->sample[hi + 1].angle_dd) / 2);
    }

    p_prof->right_edge_dd    = right_dd;
    p_prof->left_edge_dd     = left_dd;
    p_prof->b_right_at_limit = (lo == 0) ||
        (p_scan->sample[lo].angle_dd <= p_param->scan_min_dd);
    p_prof->b_left_at_limit  = (hi == ((int)p_scan->count - 1)) ||
        (p_scan->sample[hi].angle_dd >= p_param->scan_max_dd);

    p_prof->width_raw_mm =
        mul_q15(r_right, b5_cos_q15(right_dd)) -
        mul_q15(r_left,  b5_cos_q15(left_dd));

    /* Beam-spread correction: move each known edge inward by alpha. */
    if (!p_prof->b_right_at_limit)
    {
        right_dd = (int16_t)(right_dd + p_param->alpha_dd);
    }

    if (!p_prof->b_left_at_limit)
    {
        left_dd = (int16_t)(left_dd - p_param->alpha_dd);
    }

    if (right_dd > left_dd)
    {
        /* Narrower than the beam: treat it as a point at the middle. */
        int16_t mid = (int16_t)((p_prof->right_edge_dd +
                                 p_prof->left_edge_dd) / 2);
        right_dd = mid;
        left_dd  = mid;
    }

    p_prof->x_right_mm = mul_q15(r_right, b5_cos_q15(right_dd));
    p_prof->x_left_mm  = mul_q15(r_left,  b5_cos_q15(left_dd));
    p_prof->width_mm   = p_prof->x_right_mm - p_prof->x_left_mm;

    if (p_prof->width_mm < 0)
    {
        p_prof->width_mm = 0;
    }

    /* Anything closer than this, beside the obstacle, blocks the bypass. */
    corridor_y = p_prof->closest_mm + p_param->obst_depth_mm +
                 p_param->car_length_mm;

    p_prof->clear_right_mm = p_prof->b_right_at_limit ? 0 :
        side_clearance(p_scan, lo, hi, +1, p_prof->x_right_mm,
                       corridor_y, p_param->open_clearance_mm);

    p_prof->clear_left_mm = p_prof->b_left_at_limit ? 0 :
        side_clearance(p_scan, lo, hi, -1, p_prof->x_left_mm,
                       corridor_y, p_param->open_clearance_mm);

    /* A side is passable if the car plus a margin each side fits. */
    need_mm    = p_param->car_width_mm + (2 * p_param->margin_mm);
    b_right_ok = (p_prof->clear_right_mm >= need_mm);
    b_left_ok  = (p_prof->clear_left_mm  >= need_mm);

    if (b_right_ok && b_left_ok)
    {
        if (p_prof->clear_right_mm == p_prof->clear_left_mm)
        {
            p_prof->side = p_param->b_prefer_right ? B5_SIDE_RIGHT
                                                   : B5_SIDE_LEFT;
        }
        else
        {
            p_prof->side = (p_prof->clear_right_mm > p_prof->clear_left_mm)
                           ? B5_SIDE_RIGHT : B5_SIDE_LEFT;
        }
    }
    else if (b_right_ok)
    {
        p_prof->side = B5_SIDE_RIGHT;
    }
    else if (b_left_ok)
    {
        p_prof->side = B5_SIDE_LEFT;
    }
    else
    {
        p_prof->side = B5_SIDE_NONE;    /* -> Blocked                  */
    }

    /* Sideways shift that puts the car's near flank margin_mm past the
     * corrected edge, and forward travel that clears the obstacle. */
    if (p_prof->side == B5_SIDE_RIGHT)
    {
        p_prof->lateral_mm = p_prof->x_right_mm +
                             (p_param->car_width_mm / 2) + p_param->margin_mm;
    }
    else if (p_prof->side == B5_SIDE_LEFT)
    {
        p_prof->lateral_mm = -p_prof->x_left_mm +
                             (p_param->car_width_mm / 2) + p_param->margin_mm;
    }
    else
    {
        p_prof->lateral_mm = 0;
    }

    if (p_prof->lateral_mm < 0)
    {
        p_prof->lateral_mm = 0;         /* already clear on that side  */
    }

    p_prof->forward_mm = p_prof->closest_mm + p_param->obst_depth_mm +
                         p_param->car_length_mm + p_param->margin_mm;
}
