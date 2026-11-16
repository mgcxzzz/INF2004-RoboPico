/**
 * @file b4_diagnostics.h
 * @brief B4 diagnostics.
 * @details Sensor processing owns calibration/terrain state; RTOS readers use copies.
 * Height acceptance and motor assistance are separate from end-run safety.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Interval diagnostics only: does not change measurement or acceptance. */
#ifndef B4_DIAGNOSTICS_H
#define B4_DIAGNOSTICS_H
#include "src/imu/imu_proc.h"
#include <stdint.h>
typedef struct
{
    uint8_t reasons;
    uint8_t bad;
    uint8_t total;
    uint8_t rate_samples;
    uint16_t spread_mg;
    uint16_t accel_mm_s2;
    uint16_t turn_dd_s;
    uint16_t mad_mg;
    uint16_t axis_mad_mg[3]; /* v0.6: per-axis interval maxima, fwd/lat/up */
    uint8_t min_fifo_samples;
} b4_diagnostics_t;
static inline uint16_t b4_diag_magnitude (int32_t value)
{
    int64_t value_value = value;
    if (value_value < 0)
    {
        value_value = -value_value;
    }
    return (uint16_t)((value_value > UINT16_MAX) ? UINT16_MAX : value_value);
}
static inline void b4_diag_accumulate (b4_diagnostics_t * p_demand,
                                       b4_ctx_t const * p_character)
{
    uint16_t first, sample;
    p_demand->reasons |= p_character->side_quality_reason;
    if (!p_demand->total ||
        p_character->side_fifo_samples < p_demand->min_fifo_samples)
    {
        p_demand->min_fifo_samples = p_character->side_fifo_samples;
    }
    if (p_demand->total < UINT8_MAX)
    {
        p_demand->total++;
    }
    if (p_character->side_quality_reason && p_demand->bad < UINT8_MAX)
    {
        p_demand->bad++;
    }
    if (p_character->side_spread_mg > p_demand->spread_mg)
    {
        p_demand->spread_mg = p_character->side_spread_mg;
    }
    if (p_character->side_mad_mg > p_demand->mad_mg)
    {
        p_demand->mad_mg = p_character->side_mad_mg;
    }
    for (unsigned index = 0U; index < 3U; index++)
    {
        if (p_character->side_axis_mad_mg[index] > p_demand->axis_mad_mg[index])
        {
            p_demand->axis_mad_mg[index] = p_character->side_axis_mad_mg[index];
        }
    }
    if (!p_character->side_rates_valid)
    {
        return;
    }
    if (p_demand->rate_samples < UINT8_MAX)
    {
        p_demand->rate_samples++;
    }
    first = b4_diag_magnitude(p_character->accel_mm_s2);
    sample = b4_diag_magnitude(p_character->turn_dd_s);
    if (first > p_demand->accel_mm_s2)
    {
        p_demand->accel_mm_s2 = first;
    }
    if (sample > p_demand->turn_dd_s)
    {
        p_demand->turn_dd_s = sample;
    }
}
#endif

/* End of file. */
