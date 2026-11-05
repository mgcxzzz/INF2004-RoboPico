#ifndef RTOS_IMU_HUMP_DIAGNOSTICS_H
#define RTOS_IMU_HUMP_DIAGNOSTICS_H
#include <stdint.h>
/* Sole IMU writer; copied into the existing protected task mailbox.
 * Counts wrap modulo32 and restart with physical IMU initialization. */
typedef struct {
    uint32_t holds, recoveries, disarms, early_clears;
    unsigned flags : 4, hold_reason : 2, clear_reason : 4;
    unsigned failure_reason : 4;
    unsigned gap_ms : 7, last_gap_ms : 7;
} RiHumpQualification;
void b4v23_hump_qualification(RiHumpQualification *out);
#endif
