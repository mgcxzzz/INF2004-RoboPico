#ifndef SUCCESSFUL_CONTINUOUS_V2_CALIBRATION_H
#define SUCCESSFUL_CONTINUOUS_V2_CALIBRATION_H
#include "v12_2/baseline/testing/subsystem2/analog_line_follow/line_logic.h"
/* idk_v2.txt: successful Continuous V2 run, profile 1. These are ADC
 * references, not required live centre readings. User captures remain valid. */
enum { CALIBRATION_RECORD_VERSION=28 };
static inline LineCalibration successful_calibration(void) {
    return (LineCalibration){.white={163,160},.black={3457,3700},.mask=7};
}
/* Previous-firmware records may contain the unwanted recalibration. Only
 * this build's CRC-validated saved record can supersede its starting preset. */
static inline bool successful_calibration_boot(LineCalibration *current,
                                               unsigned version,bool valid) {
    if(valid && (version==CALIBRATION_RECORD_VERSION || version==30))return false;
    *current=successful_calibration();return true;
}
#endif
