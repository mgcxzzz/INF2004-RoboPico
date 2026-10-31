/* Interval diagnostics only: does not change measurement or acceptance. */
#ifndef B4_DIAGNOSTICS_H
#define B4_DIAGNOSTICS_H
#include "src/imu/imu_proc.h"
#include <stdint.h>
typedef struct {
    uint8_t reasons, bad, total, rate_samples;
    uint16_t spread_mg, accel_mm_s2, turn_dd_s;
    uint16_t mad_mg;
    uint16_t axis_mad_mg[3]; /* v0.6: per-axis interval maxima, fwd/lat/up */
    uint8_t min_fifo_samples;
} b4_diagnostics_t;
static inline uint16_t b4_diag_magnitude(int32_t value) {
    int64_t v = value;
    if (v < 0)
        v = -v;
    return (uint16_t)((v > UINT16_MAX) ? UINT16_MAX : v);
}
static inline void b4_diag_accumulate(b4_diagnostics_t *d, b4_ctx_t const *c) {
    uint16_t a, t;
    d->reasons |= c->side_quality_reason;
    if (!d->total || c->side_fifo_samples < d->min_fifo_samples)
        d->min_fifo_samples = c->side_fifo_samples;
    if (d->total < UINT8_MAX)
        d->total++;
    if (c->side_quality_reason && d->bad < UINT8_MAX)
        d->bad++;
    if (c->side_spread_mg > d->spread_mg)
        d->spread_mg = c->side_spread_mg;
    if (c->side_mad_mg > d->mad_mg)
        d->mad_mg = c->side_mad_mg;
    for (unsigned i = 0U; i < 3U; i++)
        if (c->side_axis_mad_mg[i] > d->axis_mad_mg[i])
            d->axis_mad_mg[i] = c->side_axis_mad_mg[i];
    if (!c->side_rates_valid)
        return;
    if (d->rate_samples < UINT8_MAX)
        d->rate_samples++;
    a = b4_diag_magnitude(c->accel_mm_s2);
    t = b4_diag_magnitude(c->turn_dd_s);
    if (a > d->accel_mm_s2)
        d->accel_mm_s2 = a;
    if (t > d->turn_dd_s)
        d->turn_dd_s = t;
}
#endif
