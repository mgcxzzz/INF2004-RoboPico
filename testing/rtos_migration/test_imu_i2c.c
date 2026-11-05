/* Actual RTOS transport, deterministic register/time substitution. */
#define APP_RTOS_IMU 1
#include "src/imu/pico_registers.h"
#include "src/imu/b4_config.h"
#include <assert.h>
#include <stdio.h>
#include "app_program/imu_navigation/acquisition_schedule.h"

enum {
    NORMAL,
    PREEMPT_STOP,
    ABORTED,
    STUCK_STOP,
    STUCK_TX,
    STUCK_RX,
    PREEMPT_RX,
    FIFO_BUDGET_READY,
    STUCK_RESET,
    INIT_PROBE_BUDGET
};
static unsigned mode, now_us, queued, remaining, pending_raw, rx_polls, enabled;
static unsigned complete_at;
static unsigned current_address, current_reg, budget_injected, queued_at_budget;
static void fake_set(uint32_t a, uint32_t v);
static void fake_clr(uint32_t a, uint32_t v);
static uint32_t fake_time(void) {
    now_us += 10;
    return now_us;
}
static uint32_t fake_in(uint32_t a) {
    if (a == RESETS_RESET_DONE)
        return mode == STUCK_RESET ? 0U : RESETS_RESET_I2C1;
    if (a == 0x40048034U && !budget_injected &&
        ((mode == FIFO_BUDGET_READY && current_address == 0x19U && current_reg == 0x0FU) ||
         (mode == INIT_PROBE_BUDGET && current_address == B4_MAG_ADDR && current_reg == 0x0AU))) {
        now_us += 25000U; /* status will report ready, after the aggregate deadline */
        budget_injected = 1U;
        queued_at_budget = queued;
        return 1U << 9;
    }
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
        if (current_address == 0x19U && current_reg == 0x20U)
            return 0x77; /* accelerometer control-register readback */
        return 0x33; /* accelerometer ID; optional magnetometer probes fail */
    }
    return 0;
}
static void fake_out(uint32_t a, uint32_t v) {
    if (a == 0x4004806CU)
        enabled = v;
    if (a == 0x40048004U)
        current_address = v;
    if (a == 0x40048010U) {
        ++queued;
        if (v & (1U << 8))
            ++remaining;
        else
            current_reg = v & 0xFFU;
        if (v & (1U << 9))
            complete_at = now_us + 250;
    }
}
static void fake_set(uint32_t a, uint32_t v) {
    (void)a;
    (void)v;
}
static void fake_clr(uint32_t a, uint32_t v) {
    (void)a;
    (void)v;
}
#define in_w fake_in
#define out_w fake_out
#define set_w fake_set
#define clr_w fake_clr
#define sys_time_us fake_time
#include "app_program/imu_navigation/i2c_transport.c"
#include "firmware/integration/buddy1_4/src/imu/lsm303_driver.c"

int main(void) {
    for (mode = NORMAL; mode <= PREEMPT_RX; ++mode) {
        now_us = queued = remaining = complete_at = pending_raw = rx_polls = enabled = 0;
        current_address = current_reg = budget_injected = queued_at_budget = 0;
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
    assert(after.last_gap_us == 20000 && after.last_late_us == 31000);
    assert(after.last_late_read == after.reads - 1);
    assert(after.window_seq == 1 && after.window_max_gap_us == 31000);
    now_us += 10000000;
    ri_acquisition_record(20000, true, false);
    ri_acquisition_snapshot(&after);
    assert(after.window_seq == 2 && after.previous_max_gap_us == 31000);
    assert(after.previous_late == 1 && after.previous_reads == 3);
    assert(after.window_max_gap_us == 20000 && after.window_reads == 1 && !after.window_late);
    for (unsigned i = 0; i < 499; ++i) {
        now_us += 19990; /* fake_time adds10us: exactly20ms per call */
        ri_acquisition_record(20000, true, false);
    }
    ri_acquisition_snapshot(&after);
    assert(after.window_seq == 2 && after.window_reads == 500);
    now_us += 19990;
    ri_acquisition_record(20000, true, false);
    ri_acquisition_snapshot(&after);
    assert(after.window_seq == 3 && after.previous_reads == 500 && after.window_reads == 1);
    assert(ri_next_release(100000, 101000) == 120000);
    assert(ri_next_release(100000, 141000) == 161000);
    assert(ri_release_delay_ms(120000, 103000) == 16);
    assert(ri_release_delay_ms(120000, 119900) == 1);

    /* A status-ready result after the absolute FIFO deadline still fails and
     * leaves the command FIFO untouched from the instant expiry is observed. */
    mode = FIFO_BUDGET_READY;
    now_us = queued = remaining = complete_at = pending_raw = rx_polls = enabled = 0;
    current_address = current_reg = budget_injected = queued_at_budget = 0;
    uint8_t budget_value = 0;
    ri_acquisition_snapshot(&before);
    ri_acquisition_fifo_begin();
    bool fifo_ok = b4_i2c_read_reg(0x19, 0x0f, &budget_value, 1);
    bool fifo_timely = ri_acquisition_fifo_end(1);
    ri_acquisition_snapshot(&after);
    assert(budget_injected && !fifo_ok && !fifo_timely && !enabled);
    assert(queued == queued_at_budget && after.fifo_budget_overruns == before.fifo_budget_overruns + 1);
    assert(after.last_phase == RI_I2C_BUDGET && after.last_address == 0x19);
    assert(after.last_fifo_budget_overrun_us >= 17000U);

    /* Startup reset-ready is independently bounded at the per-transfer limit. */
    mode = STUCK_RESET;
    now_us = queued = remaining = complete_at = pending_raw = rx_polls = enabled = 0;
    current_address = current_reg = budget_injected = queued_at_budget = 0;
    b4_chip_t chip = B4_CHIP_AGR;
    bool mag = true;
    ri_acquisition_snapshot(&before);
    ri_acquisition_init_begin();
    bool init_ok = b4_drv_init(&chip, &mag);
    bool init_timely = ri_acquisition_init_end();
    ri_acquisition_snapshot(&after);
    assert(!init_ok && init_timely && chip == B4_CHIP_NONE && !mag);
    assert(after.reset_timeouts == before.reset_timeouts + 1);
    assert(after.init_budget_overruns == before.init_budget_overruns);

    /* A ready optional magnetometer probe that crosses 20 ms expires the
     * aggregate init budget, with no follow-on command queued. */
    mode = INIT_PROBE_BUDGET;
    now_us = queued = remaining = complete_at = pending_raw = rx_polls = enabled = 0;
    current_address = current_reg = budget_injected = queued_at_budget = 0;
    ri_acquisition_snapshot(&before);
    ri_acquisition_init_begin();
    (void)b4_drv_init(&chip, &mag);
    init_timely = ri_acquisition_init_end();
    ri_acquisition_snapshot(&after);
    assert(budget_injected && !init_timely && after.init_budget_overruns == before.init_budget_overruns + 1);
    assert(after.last_phase == RI_I2C_INIT_BUDGET && queued == queued_at_budget);
    assert(after.last_init_budget_overrun_us >= 5000U);
    puts(
        "PASS IMU transport: phase timeout/rescue, 8 ms FIFO budget, 20 ms init budget, reset-ready timeout, no post-deadline commands, counters and fixed-rate schedule.");
}
