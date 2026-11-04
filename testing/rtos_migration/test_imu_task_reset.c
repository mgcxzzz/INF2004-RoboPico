/* Actual IMU task/service/estimator with deterministic RTOS and FIFO. */
#define APP_RTOS_IMU 1
#define APP_RTOS_LINE 1
#define APP_RTOS_NETWORK 1
#define RTOS_BRINGUP_H
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
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
#include "app_program/imu_navigation/imu_service.c"
static RiAcquisition acq;
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
        assert(shared.tlm.state == IMU_ST_IDLE && !shared.tlm.fault);
        assert(!acq.failed_reads && !acq.late_gaps && !acq.fifo_overruns);
        assert(acq.reads > 300 && shared.ticks > 300 && sleeps < 400);
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
    puts(
        "PASS actual IMU task/service: stationary calibration, fresh stopped Reset calibration epoch, lifetime counters, deadline waits and unlocked I/O.");
}
