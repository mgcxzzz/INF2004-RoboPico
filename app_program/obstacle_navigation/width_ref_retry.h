/**
 * @file width_ref_retry.h
 * @brief RTOS policy for resampling one outlying stationary width echo.
 * @details The probe owns the stopped pose and retry budget. This policy
 * only authorizes resampling and provides no accepted width evidence.
 */
#ifndef WIDTH_REF_RETRY_H
#define WIDTH_REF_RETRY_H

#include "extensions/buddy5_obstacles/width_probe.h"

enum
{
    WIDTH_REF_ECHOS = 3,
    WIDTH_REF_AGREE_MM = 15
};

/* Two in-window echoes must agree. Only one clean echo just outside the
 * reference window may use this path. The new triplet must pass the normal
 * probe reference rule. */
static inline bool
width_ref_retry_outlier (const b5_ping_t p_ping[WIDTH_REF_ECHOS])
{
    uint32_t lower = UINT32_MAX;
    uint32_t upper = 0U;
    unsigned agreeing = 0U;
    for (unsigned index = 0U; index < WIDTH_REF_ECHOS; ++index)
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
    return (WIDTH_REF_ECHOS - 1U == agreeing) &&
           ((upper - lower) <= WIDTH_REF_AGREE_MM);
}

#endif

/* End of file. */
