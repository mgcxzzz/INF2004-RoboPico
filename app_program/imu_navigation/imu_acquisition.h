/**
 * @file imu_acquisition.h
 * @brief Imu acquisition.
 * @details The IMU task owns sensor acquisition and calibration; readers use snapshots.
 * Ramp requests expire through the coordinator; lift safety may remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_IMU_ACQUISITION_H
#define RTOS_IMU_ACQUISITION_H
#include <stdbool.h>
#include <stdint.h>

/* Sole IMU-task writer; consumers receive a mailbox copy. Counters are
 * lifetime uint32 modulo counters, not the six-bit legacy telemetry view. */
#define RI_FIFO_BUDGET_US (8000U)
#define RI_INIT_BUDGET_US (20000U)
#define RI_RESET_READY_TIMEOUT_US (2000U)
enum
{
    RI_I2C_NONE,
    RI_I2C_TX,
    RI_I2C_STOP,
    RI_I2C_RX,
    RI_I2C_BUDGET,
    RI_I2C_INIT_BUDGET,
    RI_I2C_RESET_TIMEOUT
};
typedef struct
{
    uint32_t transfers;
    uint32_t aborts;
    uint32_t timeout_tx;
    uint32_t timeout_stop;
    uint32_t timeout_rx;
    uint32_t rescued;
    uint32_t max_transfer_us;
    uint32_t last_abort_source;
    uint32_t reads;
    uint32_t failed_reads;
    uint32_t late_gaps;
    uint32_t fifo_overruns;
    uint32_t max_gap_us;
    uint32_t last_gap_us;
    uint32_t last_late_us;
    uint32_t last_late_read;
    uint32_t window_seq;
    uint32_t window_max_gap_us;
    uint32_t previous_max_gap_us;
    uint32_t last_fifo_read_wall_us;
    uint32_t max_fifo_read_wall_us;
    uint32_t last_processor_wall_us;
    uint32_t max_processor_wall_us;
    uint32_t last_tick_wall_us;
    uint32_t max_tick_wall_us;
    uint32_t fifo_budget_overruns;
    uint32_t last_fifo_budget_overrun_us;
    uint32_t max_fifo_budget_overrun_us;
    uint32_t init_budget_overruns;
    uint32_t last_init_budget_overrun_us;
    uint32_t max_init_budget_overrun_us;
    uint32_t reset_timeouts;
    uint32_t ticks_over_8ms;
    /* Ten-second buckets at50Hz hold at most501 releases (9 bits). */
    unsigned window_reads : 9;
    unsigned window_late : 9;
    unsigned previous_reads : 9;
    unsigned previous_late : 9;
    /* FIFO depth is32 (6 bits); phase4 is the aggregate FIFO deadline. */
    unsigned fifo_batch_n : 6;
    unsigned last_phase : 3;
    unsigned last_address : 7;
} ri_acquisition_t;
void ri_acquisition_record (uint32_t gap_us, bool ok_value, bool overrun);
void ri_acquisition_fifo_begin (void);
bool ri_acquisition_fifo_end (uint8_t batch_n);
void ri_acquisition_init_begin (void);
bool ri_acquisition_init_end (void);
void ri_acquisition_record_reset_timeout (void);
void ri_acquisition_record_processor (uint32_t wall_us);
void ri_acquisition_record_tick (uint32_t wall_us);
void ri_acquisition_snapshot (ri_acquisition_t * p_out);
#endif

/* End of file. */
