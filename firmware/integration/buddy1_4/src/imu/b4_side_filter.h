/**
 * @file b4_side_filter.h
 * @brief B4 side filter.
 * @details Sensor processing owns calibration/terrain state; RTOS readers use copies.
 * Height acceptance and motor assistance are separate from end-run safety.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Robust per-FIFO gravity for side height only. Raw samples still feed shocks. */
#ifndef B4_SIDE_FILTER_H
#define B4_SIDE_FILTER_H
#include "src/imu/imu_proc.h"
static inline int32_t b4_median (int32_t * p_value, unsigned count)
{
    for (unsigned index = 1; index < count; index++)
    {
        int32_t value_x = p_value[index];
        unsigned other_index = index;
        while (other_index && p_value[other_index - 1] > value_x)
        {
            p_value[other_index] = p_value[other_index - 1];
            other_index--;
        }
        p_value[other_index] = value_x;
    }
    return count & 1U ? p_value[count / 2U]
                      : (p_value[count / 2U - 1U] + p_value[count / 2U]) / 2;
}
static inline bool b4_side_filter (b4_vec_t const * p_samples, unsigned count,
                                   int16_t gravity[3], uint16_t * p_max_mad,
                                   uint16_t axis_mad[3])
{
    int32_t value[B4_FIFO_DEPTH];
    *p_max_mad = 0;
    axis_mad[0] = axis_mad[1] = axis_mad[2] = 0;
    if (count < 3U || count > B4_FIFO_DEPTH)
    {
        return false;
    }
    for (unsigned axis = 0; axis < 3; axis++)
    {
        for (unsigned index = 0; index < count; index++)
        {
            value[index] = axis == 0   ? p_samples[index].fwd_mg
                           : axis == 1 ? p_samples[index].lat_mg
                                       : p_samples[index].up_mg;
        }
        int32_t median = b4_median(value, count);
        gravity[axis] = (int16_t)median;
        for (unsigned index_value = 0; index_value < count; index_value++)
        {
            int32_t demand = value[index_value] - median;
            value[index_value] = demand < 0 ? -demand : demand;
        }
        uint16_t mad = (uint16_t)b4_median(value, count);
        axis_mad[axis] = mad; /* v0.6 diagnostic: 0 fwd, 1 lat, 2 up */
        if (mad > *p_max_mad)
        {
            *p_max_mad = mad;
        }
    }
    return true;
}
#endif

/* End of file. */
