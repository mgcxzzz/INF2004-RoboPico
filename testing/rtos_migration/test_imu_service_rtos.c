/* Same service/processor fixtures as the SDK, with the RTOS TIMER port. */
#include <stdint.h>
#include <stdbool.h>
#include "app_program/imu_navigation/imu_acquisition.h"
/* Service fixture uses a synthetic FIFO, not the physical transport. */
static RiAcquisition acq;
static uint32_t fifo_start_us, init_start_us;
void ri_acquisition_record(uint32_t gap_us, bool ok, bool overrun) {
    ++acq.reads;
    acq.failed_reads += !ok;
    acq.late_gaps += gap_us > 30000U;
    acq.fifo_overruns += overrun;
}
extern uint32_t host_now_us;
uint64_t board_time_us(void) {
    return host_now_us;
}
void ri_acquisition_fifo_begin(void) {
    fifo_start_us = host_now_us;
}
bool ri_acquisition_fifo_end(uint8_t batch_n) {
    uint32_t elapsed = host_now_us - fifo_start_us;
    acq.fifo_batch_n = batch_n > 32U ? 32U : batch_n;
    acq.last_fifo_read_wall_us = elapsed;
    if (elapsed > acq.max_fifo_read_wall_us)
        acq.max_fifo_read_wall_us = elapsed;
    return elapsed < RI_FIFO_BUDGET_US;
}
void ri_acquisition_init_begin(void) {
    init_start_us = host_now_us;
}
bool ri_acquisition_init_end(void) {
    return host_now_us - init_start_us < RI_INIT_BUDGET_US;
}
void ri_acquisition_record_reset_timeout(void) {
    ++acq.reset_timeouts;
}
void ri_acquisition_record_processor(uint32_t wall_us) {
    acq.last_processor_wall_us = wall_us;
    if (wall_us > acq.max_processor_wall_us)
        acq.max_processor_wall_us = wall_us;
}
void ri_acquisition_record_tick(uint32_t wall_us) {
    acq.last_tick_wall_us = wall_us;
    if (wall_us > acq.max_tick_wall_us)
        acq.max_tick_wall_us = wall_us;
    acq.ticks_over_8ms += wall_us > RI_FIFO_BUDGET_US;
}
void ri_acquisition_snapshot(RiAcquisition *out) {
    *out = acq;
}
#include "tests/imu/test_buddy4_v23_service.c"
