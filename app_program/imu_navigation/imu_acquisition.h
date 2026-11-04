#ifndef RTOS_IMU_ACQUISITION_H
#define RTOS_IMU_ACQUISITION_H
#include <stdbool.h>
#include <stdint.h>

/* Sole IMU-task writer; consumers receive a mailbox copy. Counters are
 * lifetime uint32 modulo counters, not the six-bit legacy telemetry view. */
enum { RI_I2C_NONE, RI_I2C_TX, RI_I2C_STOP, RI_I2C_RX };
typedef struct {
    uint32_t transfers, aborts, timeout_tx, timeout_stop, timeout_rx, rescued;
    uint32_t max_transfer_us, last_abort_source;
    uint32_t reads, failed_reads, late_gaps, fifo_overruns, max_gap_us;
    unsigned last_phase : 2;
    unsigned last_address : 7;
} RiAcquisition;
void ri_acquisition_record(uint32_t gap_us, bool ok, bool overrun);
void ri_acquisition_snapshot(RiAcquisition *out);
#endif
