/**
 * @file width_retry_policy.h
 * @brief RTOS policy for borderline references and stationary retries.
 */
#ifndef WIDTH_RETRY_POLICY_H
#define WIDTH_RETRY_POLICY_H

#include "extensions/buddy5_obstacles/width_probe.h"

enum
{
    WIDTH_POSE_ECHOS = 3,
    WIDTH_ECHO_AGREE_MM = 15,
    WIDTH_REF_SHORT_MARGIN_MM = 10
};

/* Use a nominal-range median and check the initial turn with the nearest
 * echo. The probe's 25 mm face margin covers the permitted 15 mm spread. */
static inline bool width_ref_borderline (uint32_t near_mm, uint16_t median_mm,
                                         uint32_t far_mm)
{
    return (near_mm < B5WP_D0_MIN_MM) &&
           (near_mm >= B5WP_D0_MIN_MM - WIDTH_REF_SHORT_MARGIN_MM) &&
           (median_mm >= B5WP_D0_MIN_MM) && (far_mm <= B5WP_D0_MAX_MM) &&
           ((far_mm - near_mm) <= WIDTH_ECHO_AGREE_MM) &&
           (b5_wp_turn_front_gap((float)near_mm, 0.0f) >= B5WP_FRONT_MARGIN_MM);
}

/* Two in-window echoes must agree. Only one clean echo just outside the
 * reference window may use this path. The new triplet must pass the normal
 * probe reference rule. */
static inline bool
width_ref_retry_outlier (const b5_ping_t p_ping[WIDTH_POSE_ECHOS])
{
    uint32_t lower = UINT32_MAX;
    uint32_t upper = 0U;
    unsigned agreeing = 0U;
    for (unsigned index = 0U; index < WIDTH_POSE_ECHOS; ++index)
    {
        uint32_t range = p_ping[index].range_mm;
        if ((B5_PING_ECHO != p_ping[index].cls) ||
            (B5_PING_WHY_OK != p_ping[index].why) ||
            (range < B5WP_D0_MIN_MM - WIDTH_ECHO_AGREE_MM) ||
            (range > B5WP_D0_MAX_MM + WIDTH_ECHO_AGREE_MM))
        {
            return false;
        }
        if ((range >= B5WP_D0_MIN_MM) && (range <= B5WP_D0_MAX_MM))
        {
            lower = (range < lower) ? range : lower;
            upper = (range > upper) ? range : upper;
            ++agreeing;
        }
    }
    return (WIDTH_POSE_ECHOS - 1U == agreeing) &&
           ((upper - lower) <= WIDTH_ECHO_AGREE_MM);
}

/* An in-window echo plus a disagreeing short echo or missed rise allows a
 * stationary recheck. Treat the shortest echo as moving the modelled face
 * closer by its full range deficit for the clearance check. None of these
 * readings can establish the edge or authorize a movement. */
static inline bool
width_pose_retry_plausible (const b5_ping_t p_ping[WIDTH_POSE_ECHOS],
                            uint16_t expected_mm, float turn_gap_mm)
{
    int32_t lower = (int32_t)expected_mm - B5WP_NEAR_MM;
    uint32_t upper = (uint32_t)expected_mm + B5WP_FAR_MM;
    unsigned missed_rises = 0U;
    unsigned in_window = 0U;
    uint32_t nearest_mm = expected_mm;
    for (unsigned index = 0U; index < WIDTH_POSE_ECHOS; ++index)
    {
        const b5_ping_t * p_echo = &p_ping[index];
        if ((B5_PING_FAULT == p_echo->cls) &&
            (B5_PING_WHY_NO_RISE == p_echo->why))
        {
            ++missed_rises;
        }
        else if ((B5_PING_ECHO != p_echo->cls) ||
                 (B5_PING_WHY_OK != p_echo->why) || (p_echo->range_mm > upper))
        {
            return false;
        }
        else
        {
            if ((int32_t)p_echo->range_mm >= lower)
            {
                ++in_window;
            }
            if (p_echo->range_mm < nearest_mm)
            {
                nearest_mm = p_echo->range_mm;
            }
        }
    }
    float conservative_gap = turn_gap_mm - (float)(expected_mm - nearest_mm);
    return (0U < in_window) && (in_window < WIDTH_POSE_ECHOS) &&
           (missed_rises <= 1U) && (conservative_gap >= B5WP_FRONT_MARGIN_MM);
}

#endif

/* End of file. */
