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
void ri_acquisition_snapshot(RiAcquisition *out) {
    *out = diagnostics;
}
void ri_acquisition_record(uint32_t gap_us, bool ok, bool overrun) {
    ++diagnostics.reads;
    diagnostics.failed_reads += !ok;
    diagnostics.late_gaps += gap_us > B4_LATE_US;
    diagnostics.fifo_overruns += overrun;
    if (gap_us > diagnostics.max_gap_us)
        diagnostics.max_gap_us = gap_us;
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
        if (aborted(phase))
            return false;
        if (ready(phase))
            return true;
        if ((uint32_t)(sys_time_us() - start) > B4_I2C_TIMEOUT_US) {
            if (aborted(phase))
                return false;
            if (ready(phase)) {
                ++diagnostics.rescued;
                return true;
            }
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
    uint32_t start = sys_time_us();
    ++diagnostics.transfers;
    out_w(RI_ENABLE, 0U);
    out_w(RI_TAR, address);
    (void)in_w(RI_CLEAR);
    out_w(RI_ENABLE, 1U);
    bool ok = true;
    for (uint32_t i = 0; i < wn && ok; ++i) {
        ok = wait_ready(RI_I2C_TX);
        if (ok)
            out_w(RI_DATA, write[i] | ((!rn && i + 1U == wn) ? RI_STOP : 0U));
    }
    for (uint32_t i = 0; i < rn && ok; ++i) {
        ok = wait_ready(RI_I2C_TX);
        if (ok)
            out_w(RI_DATA, RI_READ | (!i ? RI_RESTART : 0U) | (i + 1U == rn ? RI_STOP : 0U));
    }
    if (ok)
        ok = wait_ready(RI_I2C_STOP);
    for (uint32_t i = 0; i < rn && ok; ++i) {
        ok = wait_ready(RI_I2C_RX);
        if (ok)
            read[i] = (uint8_t)in_w(RI_DATA);
    }
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
