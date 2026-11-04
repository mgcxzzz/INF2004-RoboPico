/* Actual RTOS transport, deterministic register/time substitution. */
#define APP_RTOS_IMU 1
#include "src/imu/pico_registers.h"
#include <assert.h>
#include <stdio.h>
#include "app_program/imu_navigation/acquisition_schedule.h"

enum { NORMAL, PREEMPT_STOP, ABORTED, STUCK_STOP, STUCK_TX, STUCK_RX, PREEMPT_RX };
static unsigned mode, now_us, queued, remaining, pending_raw, rx_polls, enabled;
static unsigned complete_at;
static uint32_t fake_time(void) {
    now_us += 10;
    return now_us;
}
static uint32_t fake_in(uint32_t a) {
    if (a == 0x40048034U) {
        if (mode == ABORTED)
            return 1U << 6;
        if (!complete_at || mode == STUCK_STOP)
            return 0;
        if (now_us >= complete_at)
            return 1U << 9;
        if (mode == PREEMPT_STOP && ++pending_raw == 2)
            now_us += 3500;
        return 0;
    }
    if (a == 0x40048080U)
        return 8; /* injected address NACK */
    if (a == 0x40048074U)
        return mode == STUCK_TX && queued ? 16 : 0;
    if (a == 0x40048078U) {
        if (mode == STUCK_RX)
            return 0;
        if (mode == PREEMPT_RX && !rx_polls++) {
            now_us += 3500;
            return 0;
        }
        return remaining;
    }
    if (a == 0x40048010U) {
        assert(remaining);
        --remaining;
        return 0x33;
    }
    return 0;
}
static void fake_out(uint32_t a, uint32_t v) {
    if (a == 0x4004806CU)
        enabled = v;
    if (a == 0x40048010U) {
        ++queued;
        if (v & (1U << 8))
            ++remaining;
        if (v & (1U << 9))
            complete_at = now_us + 250;
    }
}
#define in_w fake_in
#define out_w fake_out
#define sys_time_us fake_time
#include "app_program/imu_navigation/i2c_transport.c"

int main(void) {
    for (mode = NORMAL; mode <= PREEMPT_RX; ++mode) {
        now_us = queued = remaining = complete_at = pending_raw = rx_polls = enabled = 0;
        uint8_t value = 0;
        RiAcquisition before, after;
        ri_acquisition_snapshot(&before);
        bool ok = b4_i2c_read_reg(0x19, 0x0f, &value, 1);
        ri_acquisition_snapshot(&after);
        assert(ok == (mode == NORMAL || mode == PREEMPT_STOP || mode == PREEMPT_RX));
        assert(!enabled && after.transfers == before.transfers + 1);
        if (ok)
            assert(value == 0x33 && !remaining);
        if (mode == PREEMPT_STOP || mode == PREEMPT_RX)
            assert(after.rescued == before.rescued + 1);
        if (mode == ABORTED)
            assert(after.aborts == before.aborts + 1 && after.last_abort_source == 8);
        if (!ok)
            assert(after.last_address == 0x19);
        if (mode == STUCK_STOP)
            assert(after.timeout_stop == before.timeout_stop + 1);
        if (mode == STUCK_RX)
            assert(after.timeout_rx == before.timeout_rx + 1);
        if (mode == STUCK_TX) {
            assert(after.timeout_tx == before.timeout_tx + 1);
            assert(queued == 1); /* no READ/STOP queued after the failed wait */
        }
        assert(now_us < 8000); /* model bounds, including injected suspension */
    }
    RiAcquisition before, after;
    ri_acquisition_snapshot(&before);
    ri_acquisition_record(20000, true, false);
    ri_acquisition_record(31000, false, false);
    ri_acquisition_record(20000, true, true);
    ri_acquisition_snapshot(&after);
    assert(after.reads == before.reads + 3 && after.failed_reads == before.failed_reads + 1);
    assert(after.late_gaps == before.late_gaps + 1 &&
           after.fifo_overruns == before.fifo_overruns + 1);
    assert(after.max_gap_us == 31000);
    assert(ri_next_release(100000, 101000) == 120000);
    assert(ri_next_release(100000, 141000) == 161000);
    assert(ri_release_delay_ms(120000, 103000) == 16);
    assert(ri_release_delay_ms(120000, 119900) == 1);
    puts(
        "PASS IMU transport: STOP/RX preemption rescue; genuine NACK and TX/STOP/RX deadlines; no post-timeout commands; split acquisition counters; fixed-rate schedule.");
}
