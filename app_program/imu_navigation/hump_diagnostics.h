/**
 * @file hump_diagnostics.h
 * @brief Hump diagnostics.
 * @details The IMU task owns sensor acquisition and calibration; readers use
 * snapshots. Ramp requests expire through the coordinator; lift safety may
 * remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_IMU_HUMP_DIAGNOSTICS_H
#define RTOS_IMU_HUMP_DIAGNOSTICS_H
#include <stdint.h>
/* Sole IMU writer; copied into the existing protected task mailbox.
 * Counts wrap modulo32 and restart with physical IMU initialization. */
typedef struct
{
    uint32_t holds;
    uint32_t recoveries;
    uint32_t disarms;
    uint32_t early_clears;
    unsigned flags : 4;
    unsigned hold_reason : 2;
    unsigned clear_reason : 4;
    unsigned failure_reason : 4;
    unsigned gap_ms : 7;
    unsigned last_gap_ms : 7;
} ri_hump_qualification_t;
void b4v23_hump_qualification (ri_hump_qualification_t * p_out);
#endif

/* End of file. */
