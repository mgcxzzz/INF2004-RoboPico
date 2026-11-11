/* Actual IMU task/service/estimator with deterministic RTOS and FIFO. */
#define APP_RTOS_IMU 1
#define APP_RTOS_LINE 1
#define APP_RTOS_NETWORK 1
#define RTOS_BRINGUP_H
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include "app_program/imu_navigation/imu_acquisition.h"
#include "src/motion/motion_port.h"
typedef unsigned UINT;
typedef unsigned char UB;
typedef unsigned T_SPLOCK;
typedef int INT;
static unsigned depth;
uint32_t host_now_us = 1000000;
static jmp_buf finished;
static bool requested, checked_calibrating;
static unsigned sleeps;
static RiAcquisition acq;
static uint32_t fifo_start_us, init_start_us;
static int ISpinLock(T_SPLOCK *p, UINT *key) {
    (void)p;
    *key = 0;
    ++depth;
    return 0;
}
static int ISpinUnlock(T_SPLOCK *p, UINT key) {
    (void)p;
    (void)key;
    assert(depth);
    --depth;
    return 0;
}
static void tm_printf(UB *format, ...) {
    (void)format;
    assert(!depth);
}
uint64_t board_time_us(void) {
    return host_now_us;
}
uint32_t board_kernel_ms(void) {
    return host_now_us / 1000;
}
static int tk_dly_tsk(uint32_t ms);
static int tk_get_prc(void) {
    return 1;
}
bool rn_drive_telemetry_enabled(void) {
    return true;
}
bool rn_drive_online(void) {
    return true;
}
bool rn_tx_push(uint8_t topic, const uint8_t *data, uint8_t len, uint32_t now) {
    (void)topic;
    (void)data;
    (void)len;
    (void)now;
    assert(!depth);
    return true;
}
void ri_ramp_halt(uint64_t now) {
    (void)now;
    assert(!depth);
}
void ri_ramp_report(void) {
    assert(!depth);
}
#ifdef APP_RTOS_RELEASE
typedef struct { uint32_t control_count, late_count; unsigned flags, reason; } BringupStatus;
static void bringup_get_status(BringupStatus *s) {
    assert(!depth); *s=(BringupStatus){.control_count=1234,.late_count=0,.flags=12};
}
#endif
#include "app_program/imu_navigation/imu_service.c"
void ri_acquisition_record(uint32_t gap, bool ok, bool overrun) {
    assert(!depth);
    ++acq.reads;
    acq.failed_reads += !ok;
    acq.late_gaps += gap > 30000;
    acq.fifo_overruns += overrun;
}
void ri_acquisition_snapshot(RiAcquisition *out) {
    assert(!depth);
    *out = acq;
}
void ri_acquisition_fifo_begin(void) {
    assert(!depth);
    fifo_start_us = host_now_us;
}
bool ri_acquisition_fifo_end(uint8_t batch_n) {
    assert(!depth);
    uint32_t elapsed = host_now_us - fifo_start_us;
    acq.fifo_batch_n = batch_n > 32U ? 32U : batch_n;
    acq.last_fifo_read_wall_us = elapsed;
    if (elapsed > acq.max_fifo_read_wall_us)
        acq.max_fifo_read_wall_us = elapsed;
    return elapsed < RI_FIFO_BUDGET_US;
}
void ri_acquisition_init_begin(void) {
    assert(!depth);
    init_start_us = host_now_us;
}
bool ri_acquisition_init_end(void) {
    assert(!depth);
    return host_now_us - init_start_us < RI_INIT_BUDGET_US;
}
void ri_acquisition_record_reset_timeout(void) {
    assert(!depth);
    ++acq.reset_timeouts;
}
void ri_acquisition_record_processor(uint32_t wall_us) {
    assert(!depth);
    acq.last_processor_wall_us = wall_us;
    if (wall_us > acq.max_processor_wall_us)
        acq.max_processor_wall_us = wall_us;
}
void ri_acquisition_record_tick(uint32_t wall_us) {
    assert(!depth);
    acq.last_tick_wall_us = wall_us;
    if (wall_us > acq.max_tick_wall_us)
        acq.max_tick_wall_us = wall_us;
    acq.ticks_over_8ms += wall_us > RI_FIFO_BUDGET_US;
}
static int tk_dly_tsk(uint32_t ms) {
    assert(!depth && ms >= 1 && ms <= 20);
    ++sleeps;
    host_now_us += (ms + 1) * 1000; /* real kernel's extra timer tick */
    if (!requested && host_now_us >= 4000000) {
        assert(shared.ready_epoch == 1 && shared.tlm.state == IMU_ST_IDLE);
        ri_reset_request();
        requested = true;
    }
    if (host_now_us >= 5000000 && !checked_calibrating) {
        assert(shared.init_epoch == 2 && shared.ready_epoch == 1);
        assert(shared.tlm.state == IMU_ST_CALIBRATING);
        checked_calibrating = true;
    }
    if (host_now_us >= 7500000) {
        assert(checked_calibrating && shared.ready_epoch == 2 && shared.init_epoch == 2);
        assert(shared.core == 0);
        assert(shared.tlm.state == IMU_ST_IDLE && !shared.tlm.fault);
        assert(!acq.failed_reads && !acq.late_gaps && !acq.fifo_overruns);
        assert(acq.reads > 300 && shared.ticks > 300 && sleeps < 400);
        assert(acq.fifo_batch_n == 8 && acq.max_tick_wall_us >= acq.last_tick_wall_us);
        assert(!acq.fifo_budget_overruns && !acq.init_budget_overruns && !acq.reset_timeouts);
        assert(!acq.ticks_over_8ms); /* host fixture advances only between releases */
        assert(!shared.running);
        ri_report();
        longjmp(finished, 1);
    }
    return 0;
}
int main(void) {
    ri_init();
    if (!setjmp(finished))
        ri_task(0, NULL);
#ifdef APP_RTOS_RELEASE
    uint8_t packet[63];
    ri_release_diagnostics(17, host_now_us/1000, packet);
    assert(packet[0]==0xf2 && packet[1]==60 && packet[3]==0xc3 && packet[62]==12);
    FILE *file=fopen("testing/rtos_migration/build-host/release_runtime.bin","wb");
    assert(file && fwrite(packet,1,63,file)==63); fclose(file);
#endif
    puts(
        "PASS actual IMU task/service: stationary calibration, fresh stopped Reset calibration epoch, lifetime counters, deadline waits and unlocked I/O.");
    return 0;
}
