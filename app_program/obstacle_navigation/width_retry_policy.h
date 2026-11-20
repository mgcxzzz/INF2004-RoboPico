/**
 * @file width_retry_policy.h
 * @brief RTOS policy for resampling suspect stationary width triplets.
 * @details The probe owns the stopped pose and retry budget. This policy
 * only authorizes resampling and provides no accepted width evidence.
 */
#ifndef WIDTH_RETRY_POLICY_H
#define WIDTH_RETRY_POLICY_H

#include "extensions/buddy5_obstacles/width_probe.h"

enum
{
    WIDTH_POSE_ECHOS = 3,
    WIDTH_REF_AGREE_MM = 15,
    WIDTH_OUT_NEAR_MARGIN_MM = 5
};

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
            (range < B5WP_D0_MIN_MM - WIDTH_REF_AGREE_MM) ||
            (range > B5WP_D0_MAX_MM + WIDTH_REF_AGREE_MM))
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
           ((upper - lower) <= WIDTH_REF_AGREE_MM);
}

/* At an outward pose, a clean in-window echo can corroborate one missed
 * rise and/or one marginal early echo. Only a fresh triplet may be used. */
static inline bool
width_out_retry_plausible (const b5_ping_t p_ping[WIDTH_POSE_ECHOS],
                           uint16_t expected_mm)
{
    int32_t lower = (int32_t)expected_mm - B5WP_NEAR_MM;
    uint32_t upper = (uint32_t)expected_mm + B5WP_FAR_MM;
    unsigned missed_rises = 0U;
    unsigned early_echoes = 0U;
    unsigned in_window = 0U;
    for (unsigned index = 0U; index < WIDTH_POSE_ECHOS; ++index)
    {
        const b5_ping_t * p_echo = &p_ping[index];
        if ((B5_PING_FAULT == p_echo->cls) &&
            (B5_PING_WHY_NO_RISE == p_echo->why))
        {
            ++missed_rises;
        }
        else if ((B5_PING_ECHO != p_echo->cls) ||
                 (B5_PING_WHY_OK != p_echo->why) ||
                 (p_echo->range_mm > upper) ||
                 ((int32_t)p_echo->range_mm < lower - WIDTH_OUT_NEAR_MARGIN_MM))
        {
            return false;
        }
        else if ((int32_t)p_echo->range_mm < lower)
        {
            ++early_echoes;
        }
        else
        {
            ++in_window;
        }
    }
    return (0U < in_window) && (missed_rises <= 1U) && (early_echoes <= 1U) &&
           (0U < missed_rises + early_echoes);
}

#endif

/* End of file. */
