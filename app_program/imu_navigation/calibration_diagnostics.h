/**
 * @file calibration_diagnostics.h
 * @brief Calibration diagnostics.
 * @details The IMU task owns sensor acquisition and calibration; readers use snapshots.
 * Ramp requests expire through the coordinator; lift safety may remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_IMU_CALIBRATION_DIAGNOSTICS_H
#define RTOS_IMU_CALIBRATION_DIAGNOSTICS_H
#include "src/imu/imu_proc.h"
/* Exact restart reasons from the existing guard; observations only. */
enum
{
    RI_CAL_FWD = 1,
    RI_CAL_LAT = 2,
    RI_CAL_UP = 4,
    RI_CAL_SPEED = 8
};
typedef struct
{
    uint32_t restarts;
    uint32_t axis_restarts[3];
    uint32_t speed_restarts;
    uint32_t failed_reads;
    uint32_t empty_reads;
    uint32_t late_gaps;
    uint32_t fifo_overruns;
    uint16_t last_span_mg[3];
    uint16_t age_ms; /* capped at65535 after a delayed deadline observation */
    unsigned progress : 7;
    unsigned best : 7;
    unsigned last_reason : 4;
    unsigned state : 3;
    unsigned fault : 3;
} ri_calibration_t;
/* Called only by the sole IMU owner; mailbox copies cross task boundaries. */
void ri_calibration_restart (uint8_t reasons, const b4_ctx_t * p_ctx,
                             const int16_t mean[3]);
void b4v23_calibration (ri_calibration_t * p_out);
#endif

/* End of file. */
