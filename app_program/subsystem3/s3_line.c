/**
 * @file    s3_line.c
 * @brief   Line position, steering proposal and line events.
 *
 * Two on/off sensors give four readings, so steering is "bang-bang" with
 * memory (Week 6 feedback): a hard correction while a sensor is on the
 * line, then a gentle lean back towards the side the line was last seen,
 * which also catches a sharp curve that carries the line past a sensor.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#include "s3_line.h"
#include "s3_config.h"

#define MM_MAX      (4095U)     /* 12-bit distance fields saturate here */

static uint32_t
add_sat (uint32_t a, uint32_t b)
{
    uint32_t s = a + b;

    return (s > MM_MAX) ? MM_MAX : s;
}

void
s3_line_reset (s3_line_t * p_ln)
{
    p_ln->last_side  = (uint32_t)S3_SIDE_NONE;
    p_ln->b_lost     = 0U;
    p_ln->b_junction = 0U;
    p_ln->b_stuck    = 0U;
    p_ln->no_line_mm = 0U;
    p_ln->both_mm    = 0U;
}

static void
set_speeds (s3_line_out_t * p_out, int32_t left, int32_t right)
{
    p_out->left_mm_s  = (int16_t)left;
    p_out->right_mm_s = (int16_t)right;
}

void
s3_line_step (s3_line_t * p_ln, uint32_t sensors, uint32_t moved_mm,
              s3_line_out_t * p_out)
{
    bool b_left  = ((sensors & S3_SENSE_LEFT) != 0U);
    bool b_right = ((sensors & S3_SENSE_RIGHT) != 0U);

    p_out->events = 0U;

    if (b_left && b_right)
    {
        p_out->side = (uint8_t)S3_SIDE_BOTH;
    }
    else if (b_left)
    {
        p_out->side = (uint8_t)S3_SIDE_LEFT;
    }
    else if (b_right)
    {
        p_out->side = (uint8_t)S3_SIDE_RIGHT;
    }
    else
    {
        p_out->side = (uint8_t)S3_SIDE_NONE;
    }

    /* --- contact and line lost (FR-LINE-02) ------------------------- */
    if (b_left || b_right)
    {
        if (p_ln->b_lost != 0U)
        {
            p_out->events |= S3_LEV_FOUND;
            p_ln->b_lost   = 0U;
        }

        p_ln->no_line_mm = 0U;
    }
    else
    {
        p_ln->no_line_mm = add_sat(p_ln->no_line_mm, moved_mm);

        if ((p_ln->b_lost == 0U) &&
            (p_ln->no_line_mm >= (uint32_t)S3_LOST_MM))
        {
            p_out->events |= S3_LEV_LOST;
            p_ln->b_lost   = 1U;
        }
    }

    /* --- junction vs curve, and a stuck sensor ----------------------- */
    if (b_left && b_right)
    {
        p_ln->both_mm = add_sat(p_ln->both_mm, moved_mm);

        if ((p_ln->b_junction == 0U) &&
            (p_ln->both_mm >= (uint32_t)S3_JUNCTION_MM))
        {
            p_out->events   |= S3_LEV_JUNCTION;
            p_ln->b_junction = 1U;
        }

        if ((p_ln->b_stuck == 0U) &&
            (p_ln->both_mm >= (uint32_t)S3_STUCK_MM))
        {
            p_out->events |= S3_LEV_STUCK;
            p_ln->b_stuck  = 1U;
        }
    }
    else
    {
        p_ln->both_mm    = 0U;
        p_ln->b_junction = 0U;          /* re-arm: one event per junction */
        p_ln->b_stuck    = 0U;
    }

    /* --- steering proposal -------------------------------------------- */
    if (b_left && !b_right)
    {
        p_ln->last_side = (uint32_t)S3_SIDE_LEFT;
        set_speeds(p_out, S3_TURN_INNER_MM_S, S3_TURN_OUTER_MM_S);
    }
    else if (b_right && !b_left)
    {
        p_ln->last_side = (uint32_t)S3_SIDE_RIGHT;
        set_speeds(p_out, S3_TURN_OUTER_MM_S, S3_TURN_INNER_MM_S);
    }
    else if (b_left && b_right)
    {
        set_speeds(p_out, S3_BASE_MM_S, S3_BASE_MM_S);   /* across a junction */
    }
    else if (p_ln->last_side == (uint32_t)S3_SIDE_LEFT)
    {
        set_speeds(p_out, S3_SOFT_INNER_MM_S, S3_SOFT_OUTER_MM_S);
    }
    else if (p_ln->last_side == (uint32_t)S3_SIDE_RIGHT)
    {
        set_speeds(p_out, S3_SOFT_OUTER_MM_S, S3_SOFT_INNER_MM_S);
    }
    else
    {
        set_speeds(p_out, S3_BASE_MM_S, S3_BASE_MM_S);
    }
}
