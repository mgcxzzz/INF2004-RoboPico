/**
 * @file calibration_presets.h
 * @brief Calibration presets.
 * @details Build-time configuration for the tested application.
 * Coding cleanup does not change calibration, timing, motor or sensor
 * constants.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef SUCCESSFUL_CONTINUOUS_V2_CALIBRATION_H
#define SUCCESSFUL_CONTINUOUS_V2_CALIBRATION_H
#include "src/line_following/line_logic.h"
/* idk_v2.txt: successful Continuous V2 run, profile 1. These are ADC
 * references, not required live centre readings. User captures remain valid. */
enum
{
    CALIBRATION_RECORD_VERSION = 28
};
static inline line_calibration_t successful_calibration (void)
{
    return (line_calibration_t){
        .white = {163, 160}, .black = {3457, 3700}, .mask = 7};
}
/* Previous-firmware records may contain the unwanted recalibration. Only
 * this build's CRC-validated saved record can supersede its starting preset. */
static inline bool successful_calibration_boot (line_calibration_t * p_current,
                                                unsigned version, bool valid)
{
    if (valid && CALIBRATION_RECORD_VERSION == version)
    {
        return false;
    }
    *p_current = successful_calibration();
    return true;
}
#endif

/* End of file. */
