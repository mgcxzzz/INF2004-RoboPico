/**
 * @file b4_side_ramp.h
 * @brief Integer geometry for a one-sided (right wheel) ramp.
 */
#ifndef B4_SIDE_RAMP_H
#define B4_SIDE_RAMP_H

#include <stdbool.h>
#include <stdint.h>

#include "b4_config.h"

/*
 * Project normalized gravity onto the car-right direction. Axes are
 * [forward, right, up]. Right is cross(baseline-up, projected-forward),
 * which is +right for the current +Y/+Z sensor mounting. Magnitude is
 * checked against the measured calibration gravity before normalization.
 */
bool b4_side_geometry(int16_t const baseline[3], int16_t const sample[3],
                      int32_t g_mg, int16_t *height_mm, int16_t *roll_dd);

#endif
