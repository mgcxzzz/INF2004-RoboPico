/**
 * @file    sys_hal.c
 * @brief   Microsecond clock and shared GPIO interrupt dispatcher (RP2040).
 *
 * Kernel time uses SysTick. The BSP now establishes the independent RP2040
 * TIMER's 1 MHz source before services start. sys_hal_init verifies the
 * registers again; the driving task is gated on a timed cross-check.
 *
 * Register addresses are from the RP2040 datasheet (sections 2.19, 4.6, 4.7).
 *
 * Group IS28 (shared, written by Buddy 5). Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "sys_hal.h"
#include <sys/sysdepend/pico_rp2040/timebase.h>

/* RP2040 atomic register aliases: write 1s to set / clear bits. */
#define REG_ALIAS_SET           (0x2000U)
#define REG_ALIAS_CLR           (0x3000U)

static uint32_t timebase_kernel_ms,timebase_timer_us;
static uint8_t timebase_flags; /* bit0 checked, bit1 passed */

/* IO_BANK0 interrupt registers, 8 GPIOs x 4 event bits per register. */
#define IO_INTR(n)              (IO_BANK0_BASE + 0x0F0U + (4U * (n)))
/* Each core has its own enable/status set: PROC1's is 0x30 above PROC0's. */
#define IO_INTE(core, n)        (IO_BANK0_BASE + 0x100U + (0x30U * (core)) + (4U * (n)))
#define IO_INTS(core, n)        (IO_BANK0_BASE + 0x120U + (0x30U * (core)) + (4U * (n)))
#define SIO_CPUID               (0xD0000000U)   /* 0 on core 0, 1 on core 1 */
#define IO_IRQ_REGS             (4U)
#define INTNO_IO_BANK0          (13U)           /* IO_IRQ_BANK0 */

#define GPIO_COUNT              (30U)
#define EDGE_MASK               (SYS_GPIO_EDGE_FALL | SYS_GPIO_EDGE_RISE)

static sys_gpio_cb_t g_gpio_cb[GPIO_COUNT];
static BOOL          g_b_irq_defined = FALSE;
static uint32_t      g_irq_core = 0U;   /* core whose NVIC owns IRQ 13 */

/**
 * @brief  Release a peripheral from reset if it is still held there.
 *         Uses the atomic clear alias so other bits are never disturbed.
 */
void
sys_unreset (uint32_t mask)
{
    if ((in_w(RESETS_RESET) & mask) != 0U)
    {
        out_w(RESETS_RESET + REG_ALIAS_CLR, mask);

        while ((in_w(RESETS_RESET_DONE) & mask) != mask)
        {
            /* Reset release takes a few clk_sys cycles. */
        }
    }
}

/**
 * @brief  Establish the required divider even if the tick is running.
 */
ER
sys_hal_init (void)
{
    if(!utk_timebase_program()) {timebase_flags=1u;return E_SYS;}
    sys_unreset(RESETS_RESET_TIMER);
    return E_OK;
}

ER sys_timebase_validate(void)
{
    SYSTIM start,end;
    timebase_flags=1u;
    if(tk_get_otm(&start)<E_OK)return E_SYS;
    uint32_t timer_start=sys_time_us();
    if(tk_dly_tsk(250)<E_OK)return E_SYS;
    uint32_t timer_end=sys_time_us();
    if(tk_get_otm(&end)<E_OK)return E_SYS;
    timebase_kernel_ms=(uint32_t)(end.lo-start.lo);
    timebase_timer_us=timer_end-timer_start;
    if(!utk_timebase_elapsed_ok(timebase_timer_us,timebase_kernel_ms) ||
       !utk_timebase_registers_ok(UTK_TB_READ(UTK_TB_TICK_ADDR),
           UTK_TB_READ(UTK_TB_REF_SELECTED),UTK_TB_READ(UTK_TB_REF_DIV_ADDR)))return E_SYS;
    timebase_flags=3u;
    return E_OK;
}

void sys_timebase_report(void)
{
    uint32_t tick=UTK_TB_READ(UTK_TB_TICK_ADDR);
    uint32_t selected=UTK_TB_READ(UTK_TB_REF_SELECTED);
    uint32_t divider=UTK_TB_READ(UTK_TB_REF_DIV_ADDR);
    bool ok=timebase_flags==3u && utk_timebase_registers_ok(tick,selected,divider);
    tm_printf((UB*)"TIMEBASE: %s cycles=%u enable=%u running=%u ref_selected=0x%x ref_div=0x%x kernel_ms=%u timer_us=%u\n",
        ok?"PASS":timebase_flags?"FAIL":"PENDING",(unsigned)(tick&UTK_TB_CYCLES_MASK),
        (unsigned)((tick&UTK_TB_ENABLE)!=0),(unsigned)((tick&UTK_TB_RUNNING)!=0),
        (unsigned)selected,(unsigned)divider,(unsigned)timebase_kernel_ms,(unsigned)timebase_timer_us);
}

/**
 * @brief  Microseconds since TIMER start. Wraps every ~71 minutes, so
 *         always compare with unsigned subtraction.
 */
uint32_t
sys_time_us (void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

/**
 * @brief  Spin for a few microseconds. Only for the 10 us HC-SR04 trigger
 *         pulse; never use it for anything longer.
 */
void
sys_busy_wait_us (uint32_t us)
{
    uint32_t start = sys_time_us();

    while ((sys_time_us() - start) < us)
    {
        /* spin */
    }
}

/* Single owner of IO_IRQ_BANK0: fans each pin's edges out to its owner. */
static void
gpio_dispatch (UINT intno)
{
    uint32_t t_us = sys_time_us();
    uint32_t reg;

    (void)intno;

    for (reg = 0U; reg < IO_IRQ_REGS; reg++)
    {
        uint32_t status = (uint32_t)in_w(IO_INTS(g_irq_core, reg));
        uint32_t pin;

        if (status == 0U)
        {
            continue;
        }

        /* Edge events are latched: acknowledge before handling. */
        out_w(IO_INTR(reg), status);

        for (pin = 0U; pin < 8U; pin++)
        {
            uint32_t gpio  = (reg * 8U) + pin;
            uint32_t edges = (status >> (4U * pin)) & EDGE_MASK;

            if ((edges != 0U) && (gpio < GPIO_COUNT) &&
                (g_gpio_cb[gpio] != NULL))
            {
                g_gpio_cb[gpio](gpio, edges, t_us);
            }
        }
    }
}

/**
 * @brief  Register an edge callback for one pin and enable its interrupt.
 *         The first call defines and enables IO_IRQ_BANK0 at irq_level.
 *         Call from task context during initialisation (subsystemN_init),
 *         i.e. from usermain(), so every attach runs on the same core.
 */
ER
sys_gpio_irq_attach (uint32_t gpio, uint32_t edges, sys_gpio_cb_t cb,
                    INT irq_level)
{
    uint32_t reg;
    uint32_t shift;
    ER       er = E_OK;

    if ((gpio >= GPIO_COUNT) || (cb == NULL) || ((edges & EDGE_MASK) == 0U))
    {
        return E_PAR;
    }

    reg   = gpio / 8U;
    shift = 4U * (gpio % 8U);

    /* The interrupt is enabled in the NVIC of the core that makes the
     * first attach (on the dual-core build that may be either core), so
     * every pin must be enabled for that same core. */
    if (!g_b_irq_defined)
    {
        g_irq_core = (uint32_t)in_w(SIO_CPUID) & 1U;
    }

    g_gpio_cb[gpio] = cb;
    out_w(IO_INTR(reg), EDGE_MASK << shift);            /* clear stale */
    out_w(IO_INTE(g_irq_core, reg) + REG_ALIAS_SET,
          (edges & EDGE_MASK) << shift);

    if (!g_b_irq_defined)
    {
        T_DINT dint = { 0 };

        dint.intatr = TA_HLNG;
        dint.inthdr = gpio_dispatch;
        er = tk_def_int(INTNO_IO_BANK0, &dint);

        if (er == E_OK)
        {
            ClearInt(INTNO_IO_BANK0);
            EnableInt(INTNO_IO_BANK0, irq_level);
            g_b_irq_defined = TRUE;
        }
    }

    return er;
}
