#ifdef APP_RTOS_IMU
#include "src/imu/pico_registers.h"
#include "src/imu/drv_lsm303.h"
#include "imu_acquisition.h"

#define RI_REG(off) (0x40048000U + (off))
#define RI_TAR RI_REG(0x04U)
#define RI_DATA RI_REG(0x10U)
#define RI_RAW RI_REG(0x34U)
#define RI_CLEAR RI_REG(0x40U)
#define RI_CLEAR_ABORT RI_REG(0x54U)
#define RI_CLEAR_STOP RI_REG(0x60U)
#define RI_ENABLE RI_REG(0x6CU)
#define RI_TX_LEVEL RI_REG(0x74U)
#define RI_RX_LEVEL RI_REG(0x78U)
#define RI_ABORT_SOURCE RI_REG(0x80U)
#define RI_ABORT (1U << 6)
#define RI_STOP (1U << 9)
#define RI_READ (1U << 8)
#define RI_RESTART (1U << 10)

static RiAcquisition diagnostics;
static uint32_t window_start_us;
static uint32_t budget_start_us;
static struct {
    unsigned phase : 3, active : 1, expired : 1;
} budget_state;
void ri_acquisition_snapshot(RiAcquisition *out) {
    *out = diagnostics;
}
void ri_acquisition_record(uint32_t gap_us, bool ok, bool overrun) {
    uint32_t now = sys_time_us();
    if (!diagnostics.reads) {
        window_start_us = now;
        diagnostics.window_seq = 1;
    } else if ((uint32_t)(now - window_start_us) >= 10000000U) {
        ++diagnostics.window_seq;
        diagnostics.previous_max_gap_us = diagnostics.window_max_gap_us;
        diagnostics.previous_reads = diagnostics.window_reads;
        diagnostics.previous_late = diagnostics.window_late;
        diagnostics.window_reads = diagnostics.window_late = diagnostics.window_max_gap_us = 0;
        window_start_us = now;
    }
    ++diagnostics.reads;
    ++diagnostics.window_reads;
    diagnostics.last_gap_us = gap_us;
    if (gap_us > B4_LATE_US) {
        diagnostics.last_late_us = gap_us;
        diagnostics.last_late_read = diagnostics.reads;
        ++diagnostics.window_late;
    }
    if (gap_us > diagnostics.window_max_gap_us)
        diagnostics.window_max_gap_us = gap_us;
    diagnostics.failed_reads += !ok;
    diagnostics.late_gaps += gap_us > B4_LATE_US;
    diagnostics.fifo_overruns += overrun;
    if (gap_us > diagnostics.max_gap_us)
        diagnostics.max_gap_us = gap_us;
}

static uint32_t budget_limit_us(void) {
    return budget_state.phase == RI_I2C_BUDGET ? RI_FIFO_BUDGET_US : RI_INIT_BUDGET_US;
}

static void record_budget_expiry(uint32_t elapsed) {
    uint32_t limit_us = budget_limit_us();
    uint32_t overrun = elapsed > limit_us ? elapsed - limit_us : 0U;
    if (!budget_state.expired) {
        budget_state.expired = true;
        if (budget_state.phase == RI_I2C_BUDGET) {
            ++diagnostics.fifo_budget_overruns;
            diagnostics.last_phase = RI_I2C_BUDGET;
        } else {
            ++diagnostics.init_budget_overruns;
            diagnostics.last_phase = RI_I2C_INIT_BUDGET;
        }
    }
    if (budget_state.phase == RI_I2C_BUDGET) {
        diagnostics.last_fifo_budget_overrun_us = overrun;
        if (overrun > diagnostics.max_fifo_budget_overrun_us)
            diagnostics.max_fifo_budget_overrun_us = overrun;
    } else {
        diagnostics.last_init_budget_overrun_us = overrun;
        if (overrun > diagnostics.max_init_budget_overrun_us)
            diagnostics.max_init_budget_overrun_us = overrun;
    }
}

static bool budget_check(void) {
    if (!budget_state.active)
        return false;
    uint32_t elapsed = sys_time_us() - budget_start_us;
    if (elapsed < budget_limit_us())
        return false;
    record_budget_expiry(elapsed);
    return true;
}

static void budget_begin(unsigned phase) {
    budget_state.phase = phase;
    budget_start_us = sys_time_us();
    budget_state.expired = false;
    budget_state.active = true;
}

void ri_acquisition_fifo_begin(void) {
    budget_begin(RI_I2C_BUDGET);
}

bool ri_acquisition_fifo_end(uint8_t batch_n) {
    uint32_t elapsed = sys_time_us() - budget_start_us;
    if (budget_state.active && budget_state.phase == RI_I2C_BUDGET &&
        elapsed >= budget_limit_us())
        record_budget_expiry(elapsed);
    bool ok = !budget_state.expired;
    budget_state.active = false;
    budget_state.expired = false;
    diagnostics.fifo_batch_n = batch_n > 32U ? 32U : batch_n;
    diagnostics.last_fifo_read_wall_us = elapsed;
    if (elapsed > diagnostics.max_fifo_read_wall_us)
        diagnostics.max_fifo_read_wall_us = elapsed;
    return ok;
}

void ri_acquisition_init_begin(void) {
    budget_begin(RI_I2C_INIT_BUDGET);
}

bool ri_acquisition_init_end(void) {
    uint32_t elapsed = sys_time_us() - budget_start_us;
    if (budget_state.active && budget_state.phase == RI_I2C_INIT_BUDGET &&
        elapsed >= budget_limit_us())
        record_budget_expiry(elapsed);
    bool ok = !budget_state.expired;
    budget_state.active = false;
    budget_state.expired = false;
    return ok;
}

void ri_acquisition_record_reset_timeout(void) {
    ++diagnostics.reset_timeouts;
    diagnostics.last_phase = RI_I2C_RESET_TIMEOUT;
}

void ri_acquisition_record_processor(uint32_t wall_us) {
    diagnostics.last_processor_wall_us = wall_us;
    if (wall_us > diagnostics.max_processor_wall_us)
        diagnostics.max_processor_wall_us = wall_us;
}

void ri_acquisition_record_tick(uint32_t wall_us) {
    diagnostics.last_tick_wall_us = wall_us;
    if (wall_us > diagnostics.max_tick_wall_us)
        diagnostics.max_tick_wall_us = wall_us;
    diagnostics.ticks_over_8ms += wall_us > RI_FIFO_BUDGET_US;
}

static bool aborted(unsigned phase) {
    if (!(in_w(RI_RAW) & RI_ABORT))
        return false;
    ++diagnostics.aborts;
    diagnostics.last_phase = phase;
    diagnostics.last_abort_source = in_w(RI_ABORT_SOURCE);
    (void)in_w(RI_CLEAR_ABORT);
    return true;
}
static bool ready(unsigned phase) {
    if (phase == RI_I2C_TX)
        return in_w(RI_TX_LEVEL) < 16U;
    if (phase == RI_I2C_RX)
        return in_w(RI_RX_LEVEL) != 0U;
    return (in_w(RI_RAW) & RI_STOP) != 0U;
}
/* Each wait has a bounded no-progress deadline. Register status is sampled
 * again after the deadline: a task preemption between the first status load
 * and timer load must not turn an already-complete transfer into a timeout.
 * Interrupts and task dispatch remain enabled throughout. */
static bool wait_ready(unsigned phase) {
    uint32_t start = sys_time_us();
    for (;;) {
        if (budget_check())
            return false;
        if (aborted(phase)) {
            (void)budget_check();
            return false;
        }
        if (ready(phase)) {
            /* A just-completed status phase must not hide a crossed deadline. */
            return !budget_check();
        }
        if (budget_check())
            return false;
        if ((uint32_t)(sys_time_us() - start) > B4_I2C_TIMEOUT_US) {
            if (aborted(phase)) {
                (void)budget_check();
                return false;
            }
            if (ready(phase) && !budget_check()) {
                ++diagnostics.rescued;
                return true;
            }
            if (budget_check())
                return false;
            diagnostics.last_phase = phase;
            if (phase == RI_I2C_TX)
                ++diagnostics.timeout_tx;
            else if (phase == RI_I2C_RX)
                ++diagnostics.timeout_rx;
            else
                ++diagnostics.timeout_stop;
            return false;
        }
    }
}
static bool transfer(uint8_t address, const uint8_t *write, uint32_t wn, uint8_t *read,
                     uint32_t rn) {
    /* All callers are the sole IMU owner. Hardware setup/reset remains in the
     * reference sensor driver. Its transfers fit the 16-byte RX FIFO. */
    if (!wn || wn > 2U || rn > 16U)
        return false;
    ++diagnostics.transfers;
    if (budget_check()) {
        diagnostics.last_address = address;
        return false;
    }
    uint32_t start = sys_time_us();
    out_w(RI_ENABLE, 0U);
    out_w(RI_TAR, address);
    (void)in_w(RI_CLEAR);
    out_w(RI_ENABLE, 1U);
    bool ok = true;
    for (uint32_t i = 0; i < wn && ok; ++i) {
        ok = wait_ready(RI_I2C_TX);
        if (ok && budget_check())
            ok = false;
        if (ok)
            out_w(RI_DATA, write[i] | ((!rn && i + 1U == wn) ? RI_STOP : 0U));
    }
    for (uint32_t i = 0; i < rn && ok; ++i) {
        ok = wait_ready(RI_I2C_TX);
        if (ok && budget_check())
            ok = false;
        if (ok)
            out_w(RI_DATA, RI_READ | (!i ? RI_RESTART : 0U) | (i + 1U == rn ? RI_STOP : 0U));
    }
    if (ok)
        ok = wait_ready(RI_I2C_STOP);
    for (uint32_t i = 0; i < rn && ok; ++i) {
        ok = wait_ready(RI_I2C_RX);
        if (ok && budget_check())
            ok = false;
        if (ok)
            read[i] = (uint8_t)in_w(RI_DATA);
    }
    if (ok && budget_check())
        ok = false;
    (void)in_w(RI_CLEAR_STOP);
    out_w(RI_ENABLE, 0U);
    if (!ok)
        diagnostics.last_address = address;
    uint32_t elapsed = sys_time_us() - start;
    if (elapsed > diagnostics.max_transfer_us)
        diagnostics.max_transfer_us = elapsed;
    return ok;
}
bool b4_i2c_read_reg(uint8_t address, uint8_t reg, uint8_t *out, uint32_t count) {
    return transfer(address, &reg, 1U, out, count);
}
bool b4_i2c_write_reg(uint8_t address, uint8_t reg, uint8_t value) {
    const uint8_t bytes[2] = {reg, value};
    return transfer(address, bytes, 2U, NULL, 0U);
}
#endif
