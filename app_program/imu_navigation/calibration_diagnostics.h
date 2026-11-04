#ifndef RTOS_IMU_CALIBRATION_DIAGNOSTICS_H
#define RTOS_IMU_CALIBRATION_DIAGNOSTICS_H
#include "src/imu/imu_proc.h"
/* Exact restart reasons from the existing guard; observations only. */
enum { RI_CAL_FWD = 1, RI_CAL_LAT = 2, RI_CAL_UP = 4, RI_CAL_SPEED = 8 };
typedef struct {
    uint32_t restarts, axis_restarts[3], speed_restarts;
    uint32_t failed_reads, empty_reads, late_gaps, fifo_overruns;
    uint16_t last_span_mg[3];
    uint16_t age_ms; /* capped at65535 after a delayed deadline observation */
    unsigned progress : 7, best : 7, last_reason : 4, state : 3, fault : 3;
} RiCalibration;
/* Called only by the sole IMU owner; mailbox copies cross task boundaries. */
void ri_calibration_restart(uint8_t reasons, const b4_ctx_t *ctx, const int16_t mean[3]);
void b4v23_calibration(RiCalibration *out);
#endif
