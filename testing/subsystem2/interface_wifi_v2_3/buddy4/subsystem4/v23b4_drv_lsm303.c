/**
 * @file    drv_lsm303.c
 * @brief   Buddy 4: GY-511 (LSM303) on RP2040 I2C1, polled with timeouts.
 *
 * Register addresses: RP2040 datasheet section 4.3 (DW_apb_i2c), LSM303DLHC
 * and LSM303AGR datasheets. Bus timing values for 400 kHz from a 125 MHz
 * clk_sys are the same as the kernel's own I2C driver and the Pico SDK.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

/* v2.3+Buddy4: identical to app_program/subsystem4/drv_lsm303.c except these
 * include lines (kernel/sys_hal replaced by the SDK register shim). */
#include "../v23b4_rp2040_shim.h"
#include "drv_lsm303.h"

/* ------------------------------------------------------------------------
 * RP2040 I2C1
 * --------------------------------------------------------------------- */
#define I2C1_REG(off)           (0x40048000U + (off))
#define IC_CON                  I2C1_REG(0x00U)
#define IC_TAR                  I2C1_REG(0x04U)
#define IC_DATA_CMD             I2C1_REG(0x10U)
#define IC_FS_SCL_HCNT          I2C1_REG(0x1CU)
#define IC_FS_SCL_LCNT          I2C1_REG(0x20U)
#define IC_INTR_MASK            I2C1_REG(0x30U)
#define IC_RAW_INTR_STAT        I2C1_REG(0x34U)
#define IC_RX_TL                I2C1_REG(0x38U)
#define IC_TX_TL                I2C1_REG(0x3CU)
#define IC_CLR_INTR             I2C1_REG(0x40U)
#define IC_CLR_TX_ABRT          I2C1_REG(0x54U)
#define IC_CLR_STOP_DET         I2C1_REG(0x60U)
#define IC_ENABLE               I2C1_REG(0x6CU)
#define IC_TXFLR                I2C1_REG(0x74U)
#define IC_RXFLR                I2C1_REG(0x78U)
#define IC_SDA_HOLD             I2C1_REG(0x7CU)
#define IC_FS_SPKLEN            I2C1_REG(0xA0U)

#define CON_MASTER              (1U << 0)
#define CON_SPEED_FAST          (2U << 1)
#define CON_RESTART_EN          (1U << 5)
#define CON_SLAVE_DISABLE       (1U << 6)
#define CON_TX_EMPTY_CTRL       (1U << 8)

#define CMD_READ                (1U << 8)
#define CMD_STOP                (1U << 9)
#define CMD_RESTART             (1U << 10)

#define INTR_TX_ABRT            (1U << 6)
#define INTR_STOP_DET           (1U << 9)

#define FIFO_DEPTH_HW           (16U)

/* 400 kHz from clk_sys = 125 MHz (SYSCLK in the BSP). */
#define SCL_HCNT                (126U)
#define SCL_LCNT                (187U)
#define SCL_SPKLEN              (SCL_LCNT / 16U)
#define SDA_HOLD                (38U)

/* ------------------------------------------------------------------------
 * LSM303 accelerometer (same map on DLHC and AGR)
 * --------------------------------------------------------------------- */
#define ACC_WHO_AM_I            (0x0FU)     /* 0x33 on both              */
#define ACC_ID                  (0x33U)
#define ACC_CTRL_REG1           (0x20U)
#define ACC_CTRL_REG4           (0x23U)
#define ACC_CTRL_REG5           (0x24U)
#define ACC_OUT_X_L             (0x28U)
#define ACC_FIFO_CTRL           (0x2EU)
#define ACC_FIFO_SRC            (0x2FU)
#define AUTO_INC                (0x80U)     /* register auto-increment   */

#define REG1_400HZ_XYZ          (0x77U)     /* ODR 400 Hz, all axes on   */
#define REG4_BDU_4G_HR          (0x98U)     /* BDU, +-4 g, high res.     */
#define REG5_FIFO_EN            (0x40U)
#define FIFO_BYPASS             (0x00U)
#define FIFO_STREAM             (0x80U)
#define FIFO_SRC_OVRN           (0x40U)
#define FIFO_SRC_EMPTY          (0x20U)
#define FIFO_SRC_FSS            (0x1FU)

#if ((B4_ODR_HZ != 400U) || (B4_FS_G != 4))
#error "drv_lsm303.c is written for 400 Hz and +-4 g (b4_config.h)"
#endif

/* LSM303DLHC magnetometer */
#define DLHC_CRA                (0x00U)
#define DLHC_CRB                (0x01U)
#define DLHC_MR                 (0x02U)
#define DLHC_OUT_X_H            (0x03U)     /* X, Z, Y, high byte first  */
#define DLHC_IRA                (0x0AU)     /* reads 'H' '4' '3'         */

/* LSM303AGR magnetometer */
#define AGR_WHO_AM_I_M          (0x4FU)     /* 0x40                      */
#define AGR_CFG_A               (0x60U)
#define AGR_CFG_C               (0x62U)
#define AGR_OUT_X_L             (0x68U)

static b4_chip_t g_chip = B4_CHIP_NONE;
static bool      g_b_mag = false;

/* =====================================================================
 * I2C1, polled
 * ================================================================== */

static bool
wait_until (uint32_t addr, uint32_t mask, uint32_t want, uint32_t start_us)
{
    while ((in_w(addr) & mask) != want)
    {
        if ((sys_time_us() - start_us) > B4_I2C_TIMEOUT_US)
        {
            return false;
        }
    }

    return true;
}

static void
bus_setup (void)
{
    /* Reset the block so a bus left half-way by a fault starts clean. */
    set_w(RESETS_RESET, RESETS_RESET_I2C1);
    clr_w(RESETS_RESET, RESETS_RESET_I2C1);

    while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_I2C1) == 0U)
    {
        /* a few clk_sys cycles */
    }

    out_w(GPIO_CTRL(B4_I2C_SDA_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO(B4_I2C_SDA_PIN),
          GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(B4_I2C_SCL_PIN), GPIO_CTRL_FUNCSEL_I2C);
    out_w(GPIO(B4_I2C_SCL_PIN),
          GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);

    out_w(IC_ENABLE, 0U);
    out_w(IC_CON, CON_MASTER | CON_SPEED_FAST | CON_RESTART_EN |
                  CON_SLAVE_DISABLE | CON_TX_EMPTY_CTRL);
    out_w(IC_FS_SCL_HCNT, SCL_HCNT);
    out_w(IC_FS_SCL_LCNT, SCL_LCNT);
    out_w(IC_FS_SPKLEN, SCL_SPKLEN);
    out_w(IC_SDA_HOLD, SDA_HOLD);
    out_w(IC_RX_TL, 0U);
    out_w(IC_TX_TL, 0U);
    out_w(IC_INTR_MASK, 0U);            /* polled: no interrupts      */
}

/**
 * Write wlen bytes then (repeated start) read rlen bytes, STOP at the end.
 * rlen <= 16 (hardware FIFO).
 */
static bool
xfer (uint8_t addr, uint8_t const * p_w, uint32_t wlen, uint8_t * p_r,
      uint32_t rlen)
{
    uint32_t start = sys_time_us();
    uint32_t i;
    bool     b_ok  = true;

    out_w(IC_ENABLE, 0U);
    out_w(IC_TAR, (uint32_t)addr);
    (void)in_w(IC_CLR_INTR);
    out_w(IC_ENABLE, 1U);

    for (i = 0U; (i < wlen) && b_ok; i++)
    {
        uint32_t cmd = p_w[i];

        if ((i == (wlen - 1U)) && (rlen == 0U))
        {
            cmd |= CMD_STOP;
        }

        b_ok = wait_until(IC_TXFLR, 0x1FU, 0U, start);  /* room */
        out_w(IC_DATA_CMD, cmd);
    }

    for (i = 0U; (i < rlen) && b_ok; i++)
    {
        uint32_t cmd = CMD_READ;

        if ((i == 0U) && (wlen > 0U))
        {
            cmd |= CMD_RESTART;
        }

        if (i == (rlen - 1U))
        {
            cmd |= CMD_STOP;
        }

        while (in_w(IC_TXFLR) >= FIFO_DEPTH_HW)
        {
            if ((sys_time_us() - start) > B4_I2C_TIMEOUT_US)
            {
                b_ok = false;
                break;
            }
        }

        out_w(IC_DATA_CMD, cmd);
    }

    /* Wait for the STOP (or an abort: no ACK, lost arbitration). */
    while (b_ok && ((in_w(IC_RAW_INTR_STAT) &
                     (INTR_STOP_DET | INTR_TX_ABRT)) == 0U))
    {
        if ((sys_time_us() - start) > B4_I2C_TIMEOUT_US)
        {
            b_ok = false;
        }
    }

    if ((in_w(IC_RAW_INTR_STAT) & INTR_TX_ABRT) != 0U)
    {
        (void)in_w(IC_CLR_TX_ABRT);
        b_ok = false;
    }

    for (i = 0U; (i < rlen) && b_ok; i++)
    {
        while (in_w(IC_RXFLR) == 0U)
        {
            if ((sys_time_us() - start) > B4_I2C_TIMEOUT_US)
            {
                b_ok = false;
                break;
            }
        }

        if (b_ok)
        {
            p_r[i] = (uint8_t)(in_w(IC_DATA_CMD) & 0xFFU);
        }
    }

    (void)in_w(IC_CLR_STOP_DET);
    out_w(IC_ENABLE, 0U);

    return b_ok;
}

bool
b4_i2c_read_reg (uint8_t addr, uint8_t reg, uint8_t * p_buf, uint32_t len)
{
    return xfer(addr, &reg, 1U, p_buf, len);
}

bool
b4_i2c_write_reg (uint8_t addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];

    buf[0] = reg;
    buf[1] = value;
    return xfer(addr, buf, 2U, NULL, 0U);
}

/* =====================================================================
 * Sensor
 * ================================================================== */

static bool
mag_init (void)
{
    uint8_t id[3] = { 0U, 0U, 0U };

    if (b4_i2c_read_reg(B4_MAG_ADDR, DLHC_IRA, id, 3U) &&
        (id[0] == (uint8_t)'H') && (id[1] == (uint8_t)'4') &&
        (id[2] == (uint8_t)'3'))
    {
        /* 75 Hz, +-1.3 gauss, continuous */
        return (b4_i2c_write_reg(B4_MAG_ADDR, DLHC_CRA, 0x18U) &&
                b4_i2c_write_reg(B4_MAG_ADDR, DLHC_CRB, 0x20U) &&
                b4_i2c_write_reg(B4_MAG_ADDR, DLHC_MR, 0x00U));
    }

    if (b4_i2c_read_reg(B4_MAG_ADDR, AGR_WHO_AM_I_M, id, 1U) &&
        (id[0] == 0x40U))
    {
        /* 100 Hz continuous, block data update */
        return (b4_i2c_write_reg(B4_MAG_ADDR, AGR_CFG_A, 0x0CU) &&
                b4_i2c_write_reg(B4_MAG_ADDR, AGR_CFG_C, 0x10U));
    }

    return false;
}

bool
b4_drv_init (b4_chip_t * p_chip, bool * p_b_mag)
{
    uint8_t id = 0U;
    uint8_t check = 0U;
    bool    b_ok;

    bus_setup();
    g_chip = B4_CHIP_NONE;

    b_ok = b4_i2c_read_reg(B4_ACC_ADDR, ACC_WHO_AM_I, &id, 1U) &&
           (id == ACC_ID);

    if (b_ok)
    {
        b_ok = b4_i2c_write_reg(B4_ACC_ADDR, ACC_CTRL_REG1, REG1_400HZ_XYZ) &&
               b4_i2c_write_reg(B4_ACC_ADDR, ACC_CTRL_REG4, REG4_BDU_4G_HR) &&
               b4_i2c_write_reg(B4_ACC_ADDR, ACC_CTRL_REG5, REG5_FIFO_EN) &&
               b4_i2c_write_reg(B4_ACC_ADDR, ACC_FIFO_CTRL, FIFO_BYPASS) &&
               b4_i2c_write_reg(B4_ACC_ADDR, ACC_FIFO_CTRL, FIFO_STREAM) &&
               b4_i2c_read_reg(B4_ACC_ADDR, ACC_CTRL_REG1, &check, 1U) &&
               (check == REG1_400HZ_XYZ);
    }

    g_b_mag = b_ok && mag_init();

    if (b_ok)
    {
        /* The magnetometer tells the two chips apart. */
        uint8_t m = 0U;

        g_chip = (b4_i2c_read_reg(B4_MAG_ADDR, AGR_WHO_AM_I_M, &m, 1U) &&
                  (m == 0x40U)) ? B4_CHIP_AGR : B4_CHIP_DLHC;
    }

    *p_chip  = g_chip;
    *p_b_mag = g_b_mag;
    return b_ok;
}

bool
b4_drv_read_fifo (b4_raw_t * p_s, uint8_t max, uint8_t * p_n,
                  bool * p_b_overrun)
{
    uint8_t src = 0U;
    uint8_t n;
    uint8_t i;

    *p_n         = 0U;
    *p_b_overrun = false;

    if (!b4_i2c_read_reg(B4_ACC_ADDR, ACC_FIFO_SRC, &src, 1U))
    {
        return false;
    }

    if ((src & FIFO_SRC_EMPTY) != 0U)
    {
        return true;                    /* nothing new                 */
    }

    n = src & FIFO_SRC_FSS;

    if ((src & FIFO_SRC_OVRN) != 0U)
    {
        *p_b_overrun = true;
        n = (uint8_t)B4_FIFO_DEPTH;
    }

    if (n == 0U)
    {
        n = 1U;                         /* not empty: at least one     */
    }

    if (n > max)
    {
        n = max;
    }

    for (i = 0U; i < n; i++)
    {
        uint8_t b[6];

        if (!b4_i2c_read_reg(B4_ACC_ADDR, (uint8_t)(ACC_OUT_X_L | AUTO_INC),
                             b, 6U))
        {
            return false;
        }

        /* 12-bit, left-justified, little-endian; 2 mg per LSB at +-4 g */
        p_s[i].x = (int16_t)(((int32_t)(int16_t)((uint16_t)b[0] |
                                                 ((uint16_t)b[1] << 8)) / 16) *
                             B4_MG_PER_LSB);
        p_s[i].y = (int16_t)(((int32_t)(int16_t)((uint16_t)b[2] |
                                                 ((uint16_t)b[3] << 8)) / 16) *
                             B4_MG_PER_LSB);
        p_s[i].z = (int16_t)(((int32_t)(int16_t)((uint16_t)b[4] |
                                                 ((uint16_t)b[5] << 8)) / 16) *
                             B4_MG_PER_LSB);
    }

    *p_n = n;
    return true;
}

bool
b4_drv_read_mag (b4_raw_t * p_m)
{
    uint8_t b[6];

    if (!g_b_mag)
    {
        return false;
    }

    if (g_chip == B4_CHIP_AGR)
    {
        if (!b4_i2c_read_reg(B4_MAG_ADDR, (uint8_t)(AGR_OUT_X_L | AUTO_INC),
                             b, 6U))
        {
            return false;
        }

        p_m->x = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
        p_m->y = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
        p_m->z = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
        return true;
    }

    if (!b4_i2c_read_reg(B4_MAG_ADDR, DLHC_OUT_X_H, b, 6U))
    {
        return false;
    }

    p_m->x = (int16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
    p_m->z = (int16_t)(((uint16_t)b[2] << 8) | (uint16_t)b[3]);
    p_m->y = (int16_t)(((uint16_t)b[4] << 8) | (uint16_t)b[5]);
    return true;
}
