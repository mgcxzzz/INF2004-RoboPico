/**
 * @file i2c_transport.c
 * @brief I2c transport.
 * @details The IMU task owns sensor acquisition and calibration; readers use
 * snapshots. Ramp requests expire through the coordinator; lift safety may
 * remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
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

static ri_acquisition_t g_diagnostics;
static uint32_t g_window_start_us;
static uint32_t g_budget_start_us;
static struct
{
    unsigned phase : 3;
    unsigned active : 1;
    unsigned expired : 1;
} g_budget_state;
void ri_acquisition_snapshot (ri_acquisition_t * p_out_value)
{
    *p_out_value = g_diagnostics;
}
void ri_acquisition_record (uint32_t gap_us, bool ok_value_value, bool overrun)
{
    uint32_t now = sys_time_us();
    if (!g_diagnostics.reads)
    {
        g_window_start_us = now;
        g_diagnostics.window_seq = 1;
    }
    else if ((uint32_t)(now - g_window_start_us) >= 10000000U)
    {
        ++g_diagnostics.window_seq;
        g_diagnostics.previous_max_gap_us = g_diagnostics.window_max_gap_us;
        g_diagnostics.previous_reads = g_diagnostics.window_reads;
        g_diagnostics.previous_late = g_diagnostics.window_late;
        g_diagnostics.window_reads = g_diagnostics.window_late =
            g_diagnostics.window_max_gap_us = 0;
        g_window_start_us = now;
    }
    ++g_diagnostics.reads;
    ++g_diagnostics.window_reads;
    g_diagnostics.last_gap_us = gap_us;
    if (gap_us > B4_LATE_US)
    {
        g_diagnostics.last_late_us = gap_us;
        g_diagnostics.last_late_read = g_diagnostics.reads;
        ++g_diagnostics.window_late;
    }
    if (gap_us > g_diagnostics.window_max_gap_us)
    {
        g_diagnostics.window_max_gap_us = gap_us;
    }
    g_diagnostics.failed_reads += !ok_value_value;
    g_diagnostics.late_gaps += gap_us > B4_LATE_US;
    g_diagnostics.fifo_overruns += overrun;
    if (gap_us > g_diagnostics.max_gap_us)
    {
        g_diagnostics.max_gap_us = gap_us;
    }
}

static uint32_t budget_limit_us (void)
{
    return RI_I2C_BUDGET == g_budget_state.phase ? RI_FIFO_BUDGET_US
                                                 : RI_INIT_BUDGET_US;
}

static void record_budget_expiry (uint32_t elapsed)
{
    uint32_t limit_us = budget_limit_us();
    uint32_t overrun = elapsed > limit_us ? elapsed - limit_us : 0U;
    if (!g_budget_state.expired)
    {
        g_budget_state.expired = true;
        if (RI_I2C_BUDGET == g_budget_state.phase)
        {
            ++g_diagnostics.fifo_budget_overruns;
            g_diagnostics.last_phase = RI_I2C_BUDGET;
        }
        else
        {
            ++g_diagnostics.init_budget_overruns;
            g_diagnostics.last_phase = RI_I2C_INIT_BUDGET;
        }
    }
    if (RI_I2C_BUDGET == g_budget_state.phase)
    {
        g_diagnostics.last_fifo_budget_overrun_us = overrun;
        if (overrun > g_diagnostics.max_fifo_budget_overrun_us)
        {
            g_diagnostics.max_fifo_budget_overrun_us = overrun;
        }
    }
    else
    {
        g_diagnostics.last_init_budget_overrun_us = overrun;
        if (overrun > g_diagnostics.max_init_budget_overrun_us)
        {
            g_diagnostics.max_init_budget_overrun_us = overrun;
        }
    }
}

static bool budget_check (void)
{
    if (!g_budget_state.active)
    {
        return false;
    }
    uint32_t elapsed = sys_time_us() - g_budget_start_us;
    if (elapsed < budget_limit_us())
    {
        return false;
    }
    record_budget_expiry(elapsed);
    return true;
}

static void budget_begin (unsigned phase)
{
    g_budget_state.phase = phase;
    g_budget_start_us = sys_time_us();
    g_budget_state.expired = false;
    g_budget_state.active = true;
}

void ri_acquisition_fifo_begin (void)
{
    budget_begin(RI_I2C_BUDGET);
}

bool ri_acquisition_fifo_end (uint8_t batch_n)
{
    uint32_t elapsed = sys_time_us() - g_budget_start_us;
    if ((g_budget_state.active) && RI_I2C_BUDGET == g_budget_state.phase &&
        (elapsed >= budget_limit_us()))
    {
        record_budget_expiry(elapsed);
    }
    bool ok_value = !g_budget_state.expired;
    g_budget_state.active = false;
    g_budget_state.expired = false;
    g_diagnostics.fifo_batch_n = batch_n > 32U ? 32U : batch_n;
    g_diagnostics.last_fifo_read_wall_us = elapsed;
    if (elapsed > g_diagnostics.max_fifo_read_wall_us)
    {
        g_diagnostics.max_fifo_read_wall_us = elapsed;
    }
    return ok_value;
}

void ri_acquisition_init_begin (void)
{
    budget_begin(RI_I2C_INIT_BUDGET);
}

bool ri_acquisition_init_end (void)
{
    uint32_t elapsed = sys_time_us() - g_budget_start_us;
    if ((g_budget_state.active) && RI_I2C_INIT_BUDGET == g_budget_state.phase &&
        (elapsed >= budget_limit_us()))
    {
        record_budget_expiry(elapsed);
    }
    bool ok_value = !g_budget_state.expired;
    g_budget_state.active = false;
    g_budget_state.expired = false;
    return ok_value;
}

void ri_acquisition_record_reset_timeout (void)
{
    ++g_diagnostics.reset_timeouts;
    g_diagnostics.last_phase = RI_I2C_RESET_TIMEOUT;
}

void ri_acquisition_record_processor (uint32_t wall_us)
{
    g_diagnostics.last_processor_wall_us = wall_us;
    if (wall_us > g_diagnostics.max_processor_wall_us)
    {
        g_diagnostics.max_processor_wall_us = wall_us;
    }
}

void ri_acquisition_record_tick (uint32_t wall_us)
{
    g_diagnostics.last_tick_wall_us = wall_us;
    if (wall_us > g_diagnostics.max_tick_wall_us)
    {
        g_diagnostics.max_tick_wall_us = wall_us;
    }
    g_diagnostics.ticks_over_8ms += wall_us > RI_FIFO_BUDGET_US;
}

static bool aborted (unsigned phase)
{
    if (!(in_w(RI_RAW) & RI_ABORT))
    {
        return false;
    }
    ++g_diagnostics.aborts;
    g_diagnostics.last_phase = phase;
    g_diagnostics.last_abort_source = in_w(RI_ABORT_SOURCE);
    (void)in_w(RI_CLEAR_ABORT);
    return true;
}
static bool ready (unsigned phase)
{
    if (RI_I2C_TX == phase)
    {
        return in_w(RI_TX_LEVEL) < 16U;
    }
    if (RI_I2C_RX == phase)
    {
        return in_w(RI_RX_LEVEL) != 0U;
    }
    return (in_w(RI_RAW) & RI_STOP) != 0U;
}
/* Each wait has a bounded no-progress deadline. Register status is sampled
 * again after the deadline: a task preemption between the first status load
 * and timer load must not turn an already-complete transfer into a timeout.
 * Interrupts and task dispatch remain enabled throughout. */
static bool wait_ready (unsigned phase)
{
    uint32_t start = sys_time_us();
    for (;;)
    {
        if (budget_check())
        {
            return false;
        }
        if (aborted(phase))
        {
            (void)budget_check();
            return false;
        }
        if (ready(phase))
        {
            /* A just-completed status phase must not hide a crossed deadline.
             */
            return !budget_check();
        }
        if (budget_check())
        {
            return false;
        }
        if ((uint32_t)(sys_time_us() - start) > B4_I2C_TIMEOUT_US)
        {
            if (aborted(phase))
            {
                (void)budget_check();
                return false;
            }
            if ((ready(phase)) && (!budget_check()))
            {
                ++g_diagnostics.rescued;
                return true;
            }
            if (budget_check())
            {
                return false;
            }
            g_diagnostics.last_phase = phase;
            if (RI_I2C_TX == phase)
            {
                ++g_diagnostics.timeout_tx;
            }
            else if (RI_I2C_RX == phase)
            {
                ++g_diagnostics.timeout_rx;
            }
            else
            {
                ++g_diagnostics.timeout_stop;
            }
            return false;
        }
    }
}
static bool transfer (uint8_t address, const uint8_t * p_write,
                      uint32_t wn_value, uint8_t * p_read, uint32_t rn_value)
{
    /* All callers are the sole IMU owner. Hardware setup/reset remains in the
     * reference sensor driver. Its transfers fit the 16-byte RX FIFO. */
    if (((!wn_value) || (wn_value > 2U)) || (rn_value > 16U))
    {
        return false;
    }
    ++g_diagnostics.transfers;
    if (budget_check())
    {
        g_diagnostics.last_address = address;
        return false;
    }
    uint32_t start = sys_time_us();
    out_w(RI_ENABLE, 0U);
    out_w(RI_TAR, address);
    (void)in_w(RI_CLEAR);
    out_w(RI_ENABLE, 1U);
    bool ok_value = true;
    for (uint32_t index = 0; (index < wn_value) && ok_value; ++index)
    {
        ok_value = wait_ready(RI_I2C_TX);
        if (ok_value && (budget_check()))
        {
            ok_value = false;
        }
        if (ok_value)
        {
            out_w(
                RI_DATA,
                p_write[index] |
                    (((!rn_value) && (index + 1U == wn_value)) ? RI_STOP : 0U));
        }
    }
    for (uint32_t index_value = 0; (index_value < rn_value) && ok_value;
         ++index_value)
    {
        ok_value = wait_ready(RI_I2C_TX);
        if (ok_value && (budget_check()))
        {
            ok_value = false;
        }
        if (ok_value)
        {
            out_w(RI_DATA, RI_READ | (!index_value ? RI_RESTART : 0U) |
                               (index_value + 1U == rn_value ? RI_STOP : 0U));
        }
    }
    if (ok_value)
    {
        ok_value = wait_ready(RI_I2C_STOP);
    }
    for (uint32_t index_value_value = 0;
         (index_value_value < rn_value) && ok_value; ++index_value_value)
    {
        ok_value = wait_ready(RI_I2C_RX);
        if (ok_value && (budget_check()))
        {
            ok_value = false;
        }
        if (ok_value)
        {
            p_read[index_value_value] = (uint8_t)in_w(RI_DATA);
        }
    }
    if (ok_value && (budget_check()))
    {
        ok_value = false;
    }
    (void)in_w(RI_CLEAR_STOP);
    out_w(RI_ENABLE, 0U);
    if (!ok_value)
    {
        g_diagnostics.last_address = address;
    }
    uint32_t elapsed = sys_time_us() - start;
    if (elapsed > g_diagnostics.max_transfer_us)
    {
        g_diagnostics.max_transfer_us = elapsed;
    }
    return ok_value;
}
bool b4_i2c_read_reg (uint8_t address, uint8_t reg, uint8_t * p_out,
                      uint32_t count)
{
    return transfer(address, &reg, 1U, p_out, count);
}
bool b4_i2c_write_reg (uint8_t address, uint8_t reg, uint8_t value)
{
    const uint8_t bytes[2] = {reg, value};
    return transfer(address, bytes, 2U, NULL, 0U);
}
#endif

/* End of file. */
