#ifndef RTOS_IMU_ACQUISITION_H
#define RTOS_IMU_ACQUISITION_H
#include <stdbool.h>
#include <stdint.h>

/* Sole IMU-task writer; consumers receive a mailbox copy. Counters are
 * lifetime uint32 modulo counters, not the six-bit legacy telemetry view. */
#define RI_FIFO_BUDGET_US (8000U)
#define RI_INIT_BUDGET_US (20000U)
#define RI_RESET_READY_TIMEOUT_US (2000U)
enum {
    RI_I2C_NONE,
    RI_I2C_TX,
    RI_I2C_STOP,
    RI_I2C_RX,
    RI_I2C_BUDGET,
    RI_I2C_INIT_BUDGET,
    RI_I2C_RESET_TIMEOUT
};
typedef struct {
    uint32_t transfers, aborts, timeout_tx, timeout_stop, timeout_rx, rescued;
    uint32_t max_transfer_us, last_abort_source;
    uint32_t reads, failed_reads, late_gaps, fifo_overruns, max_gap_us;
    uint32_t last_gap_us, last_late_us, last_late_read;
    uint32_t window_seq, window_max_gap_us, previous_max_gap_us;
    uint32_t last_fifo_read_wall_us, max_fifo_read_wall_us;
    uint32_t last_processor_wall_us, max_processor_wall_us;
    uint32_t last_tick_wall_us, max_tick_wall_us;
    uint32_t fifo_budget_overruns, last_fifo_budget_overrun_us, max_fifo_budget_overrun_us;
    uint32_t init_budget_overruns, last_init_budget_overrun_us, max_init_budget_overrun_us;
    uint32_t reset_timeouts;
    uint32_t ticks_over_8ms;
    /* Ten-second buckets at50Hz hold at most501 releases (9 bits). */
    unsigned window_reads : 9, window_late : 9, previous_reads : 9, previous_late : 9;
    /* FIFO depth is32 (6 bits); phase4 is the aggregate FIFO deadline. */
    unsigned fifo_batch_n : 6, last_phase : 3;
    unsigned last_address : 7;
} RiAcquisition;
void ri_acquisition_record(uint32_t gap_us, bool ok, bool overrun);
void ri_acquisition_fifo_begin(void);
bool ri_acquisition_fifo_end(uint8_t batch_n);
void ri_acquisition_init_begin(void);
bool ri_acquisition_init_end(void);
void ri_acquisition_record_reset_timeout(void);
void ri_acquisition_record_processor(uint32_t wall_us);
void ri_acquisition_record_tick(uint32_t wall_us);
void ri_acquisition_snapshot(RiAcquisition *out);
#endif
