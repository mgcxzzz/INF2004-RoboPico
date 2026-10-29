#ifndef SUCCESSFUL_CONTINUOUS_V2_CALIBRATION_H
#define SUCCESSFUL_CONTINUOUS_V2_CALIBRATION_H
#include "v12_2/baseline/testing/subsystem2/analog_line_follow/line_logic.h"
/* Raised front sensors (~10 mm): measured 2026-10-29 w/l/r/k endpoints.
 * These reproduce all 256 recorded ADC/level pairs, not a validated new
 * smooth-running sweet spot. Manual captures remain supported. */
enum { CALIBRATION_RECORD_VERSION=29 };
static inline LineCalibration successful_calibration(void) {
    return (LineCalibration){.white={191,154},.black={1940,2306},.mask=7};
}
/* Previous-firmware records may contain the unwanted recalibration. Only
 * this build's CRC-validated saved record can supersede its starting preset. */
static inline bool successful_calibration_boot(LineCalibration *current,
                                               unsigned version,bool valid) {
    if(valid && version==CALIBRATION_RECORD_VERSION)return false;
    *current=successful_calibration();return true;
}
#endif
