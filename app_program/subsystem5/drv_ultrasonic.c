/**
 * @file    drv_ultrasonic.c
 * @brief   HC-SR04+ driver: TRIG on GP16, ECHO on GP17 (edge interrupt).
 *
 * One ping:
 *   1. wait until at least B5_PING_PERIOD_MS has passed since the last
 *      trigger (NFR-USO-06), sleeping rather than spinning;
 *   2. pulse TRIG high for ~12 us;
 *   3. the ECHO rising and falling edges are timestamped in the GPIO
 *      interrupt (system/sys_hal.c), and the falling edge sets an event flag;
 *   4. the task sleeps on that flag with a timeout.
 *
 * Sharing with the interrupt (Tribunal 1: safe access to packed data).
 * Every shared word has exactly ONE writer:
 *   - g_arm_seq   written only by the task: the sequence number of the
 *                 ping in flight;
 *   - g_rise_word written only by the ISR: "a rising edge was seen for
 *                 ping N";
 *   - g_result    written only by the ISR: echo width (16 bits), the
 *                 ping's sequence number (8 bits) and flags, packed into
 *                 one 32-bit word (b5_pack.h) and stored with one aligned
 *                 write, which the Cortex-M0+ does atomically.
 * The rise/fall bookkeeping itself is private to the ISR. The task never
 * writes anything the ISR writes, so there is no read-modify-write race
 * on either core, and a result tagged with an old sequence number (a late
 * echo, or a fall after the task gave up) is simply ignored.
 *
 * Rule B (2026-10-31): every ping is classified (b5_classify_ping):
 *   - ECHO must read LOW before TRIG (polled up to B5_PRETRIG_LOW_WAIT_MS);
 *     still high = FAULT (stuck high), and TRIG is not pulsed;
 *   - rise and fall seen, width in [ECHO_MIN, ECHO_MAX] = ECHO (range);
 *   - rise and fall seen, width > B5_ECHO_MAX_US = NO_RETURN (the sensor
 *     completed a measurement and heard nothing in range);
 *   - rise but no fall within B5_NORETURN_WAIT_MS = FAULT (no fall);
 *   - only pulses shorter than B5_ECHO_MIN_US = FAULT (noise);
 *   - no rise = FAULT (silent / disconnected).
 * A still-high ECHO is never reported as "no target" any more.
 * Range = echo width * 343 m/s / 2.
 *
 * B5_US_DIAG = 1 (diagnostic image only) also polls ECHO in the task the
 * way the working SDK bench does and prints one [us diag] line per ping
 * with the pad / function registers of TRIG and ECHO.
 *
 * The HC-SR04+ runs from 3V3, so ECHO is 3.3 V and goes straight to GP17
 * (Assumption A5). A 5 V HC-SR04 must NOT be connected without a divider.
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "b5_config.h"
#include "../system/sys_hal.h"
#include "b5_pack.h"
#include "drv_ultrasonic.h"

#ifndef B5_US_DIAG
#define B5_US_DIAG          (0)
#endif

#if B5_US_DIAG
#include <tm/tmonitor.h>
#endif

#define FLG_ECHO_DONE       (1U << 0)
#define RISE_FLAG_SEEN      (1UL << 8)      /* g_rise_word: bit 8 = seen */

/* mm = us * 343 / 2000  (343 m/s, there and back) */
#define SOUND_NUM           (343U)
#define SOUND_DEN           (2000U)

/* Data memory barrier: make the data visible before the wake-up. */
#define B5_DMB()            __asm volatile ("dmb" ::: "memory")

static ID                g_echo_flg = 0;

/* Written only by the task. */
static volatile uint32_t g_arm_seq  = 0U;
static uint8_t           g_seq      = 0U;
static uint32_t          g_last_trig_us = 0U;
static BOOL              g_b_triggered  = FALSE;

/* Written only by the ISR (published, one aligned word each). */
static volatile uint32_t g_rise_word  = 0U;
static volatile uint32_t g_result     = 0U;
static volatile uint32_t g_short_word = 0U;   /* short pulse seen for ping N */

/* Private to the ISR. */
static BOOL              g_isr_high     = FALSE;
static uint32_t          g_isr_rise_us  = 0U;
static uint8_t           g_isr_rise_seq = 0U;

/* GPIO interrupt callback for ECHO (interrupt context). */
static void
echo_edge (uint32_t gpio, uint32_t edges, uint32_t t_us)
{
    (void)gpio;

    if ((edges & SYS_GPIO_EDGE_RISE) != 0U)
    {
        /* Start (or restart, after a missed fall) a measurement and tag
         * it with the ping that is in flight now. */
        g_isr_high     = TRUE;
        g_isr_rise_us  = t_us;
        g_isr_rise_seq = (uint8_t)g_arm_seq;
        g_rise_word    = RISE_FLAG_SEEN | g_isr_rise_seq;
    }

    if (((edges & SYS_GPIO_EDGE_FALL) != 0U) && g_isr_high)
    {
        /* A fall is only accepted after a matching rise. */
        uint32_t width_us = t_us - g_isr_rise_us;

        g_isr_high = FALSE;

        /* A pulse shorter than any real echo is noise: drop it and wait
         * for the real echo of the same ping. */
        if (width_us >= B5_ECHO_MIN_US)
        {
            g_result = b5_echo_pack(width_us, g_isr_rise_seq);
            B5_DMB();                   /* one aligned 32-bit store above */
            (void)tk_set_flg(g_echo_flg, FLG_ECHO_DONE);
        }
        else
        {
            g_short_word = RISE_FLAG_SEEN | g_isr_rise_seq;
        }
    }
}

/**
 * @brief  Configure TRIG/ECHO pins, the echo flag and the edge interrupt.
 */
ER
b5_us_init (void)
{
    T_CFLG cflg = { 0 };
    ER     er;

    cflg.flgatr  = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0U;
    cflg.exinf   = NULL;
    g_echo_flg   = tk_cre_flg(&cflg);

    if (g_echo_flg < E_OK)
    {
        return g_echo_flg;
    }

    er = gpio_set_pin(B5_PIN_US_TRIG, GPIO_MODE_OUT);

    if (er == E_OK)
    {
        er = gpio_set_val(B5_PIN_US_TRIG, 0U);
    }

    if (er == E_OK)
    {
        er = gpio_set_pin(B5_PIN_US_ECHO, GPIO_MODE_IN);
    }

    if (er == E_OK)
    {
        /* Explicit pull-down on ECHO (as the SDK bench): a floating or
         * disconnected input reads low = no rise, never a stuck high. */
        out_w(GPIO(B5_PIN_US_ECHO),
              (in_w(GPIO(B5_PIN_US_ECHO)) | GPIO_PDE) & ~(UW)GPIO_PUE);
    }

    if (er == E_OK)
    {
        er = sys_gpio_irq_attach(B5_PIN_US_ECHO,
                                SYS_GPIO_EDGE_RISE | SYS_GPIO_EDGE_FALL,
                                echo_edge, B5_ECHO_IRQ_LEVEL);
    }

    return er;
}

/* Sleep until the 60 ms acoustic cycle since the last trigger is over. */
static void
pace_pings (void)
{
    uint32_t period_us = B5_PING_PERIOD_MS * 1000U;
    uint32_t since_us;

    if (!g_b_triggered)
    {
        return;
    }

    since_us = sys_time_us() - g_last_trig_us;

    if (since_us < period_us)
    {
        /* tk_dly_tsk() never returns early, so this is a lower bound. */
        (void)tk_dly_tsk((TMO)(((period_us - since_us) + 999U) / 1000U));
    }
}

#if B5_US_DIAG
#define IO_GPIO_CTRL(n)     (IO_BANK0_BASE + 0x004U + (8U * (n)))
#define IO_GPIO_STATUS(n)   (IO_BANK0_BASE + 0x000U + (8U * (n)))

/* Poll-based measurement, same sequence as the SDK bench, run alongside
 * the interrupt capture. Returns widths in *p_* (0 if not seen). */
static void
diag_poll (uint32_t trig_us, uint32_t * p_rise_us, uint32_t * p_fall_us)
{
    uint32_t t;

    *p_rise_us = 0U;
    *p_fall_us = 0U;
    for (;;)
    {
        t = sys_time_us();
        if (gpio_get_val(B5_PIN_US_ECHO) != 0U) { *p_rise_us = t; break; }
        if ((t - trig_us) > 30000U) { return; }
    }
    for (;;)
    {
        t = sys_time_us();
        if (gpio_get_val(B5_PIN_US_ECHO) == 0U) { *p_fall_us = t; break; }
        if ((t - *p_rise_us) > ((uint32_t)B5_NORETURN_WAIT_MS * 1000U)) { return; }
    }
}

static void
diag_print (b5_ping_t const * p, uint32_t pre_wait_us, uint32_t trig_us,
            uint32_t prise, uint32_t pfall, uint32_t isr_w)
{
    static char const * const cls[3] = { "ECHO", "NO_RETURN", "FAULT" };
    static char const * const why[5] = { "-", "stuck-high", "no-rise",
                                         "no-fall", "short" };

    tm_printf((UB*)"[us diag] seq %d %s(%s) w=%d us r=%d mm | pre-low wait %d us | "
                    "poll rise@+%d fall@+%d w=%d | isr w=%d | "
                    "TRIG ctrl 0x%x pad 0x%x | ECHO ctrl 0x%x pad 0x%x st 0x%x\n",
              (INT)g_seq, cls[p->cls % 3U], why[(p->why < 5U) ? p->why : 0U],
              (INT)p->width_us, (INT)p->range_mm, (INT)pre_wait_us,
              (INT)((prise != 0U) ? (prise - trig_us) : 0U),
              (INT)((pfall != 0U) ? (pfall - trig_us) : 0U),
              (INT)(((prise != 0U) && (pfall != 0U)) ? (pfall - prise) : 0U),
              (INT)isr_w,
              (UINT)in_w(IO_GPIO_CTRL(B5_PIN_US_TRIG)), (UINT)in_w(GPIO(B5_PIN_US_TRIG)),
              (UINT)in_w(IO_GPIO_CTRL(B5_PIN_US_ECHO)), (UINT)in_w(GPIO(B5_PIN_US_ECHO)),
              (UINT)in_w(IO_GPIO_STATUS(B5_PIN_US_ECHO)));
}
#endif

/**
 * @brief  Take one classified reading (rule B).
 * @return E_OK with *p_ping filled (cls may be FAULT), or a kernel error.
 */
ER
b5_us_ping_ex (b5_ping_t * p_ping)
{
    uint32_t wait_us = (uint32_t)B5_NORETURN_WAIT_MS * 1000U;
    uint32_t start_us;
    uint32_t word;
    uint32_t pre_us;
    bool     b_rise = false;
    bool     b_fall = false;
    bool     b_short = false;
    uint32_t width_us = 0U;
#if B5_US_DIAG
    uint32_t prise = 0U;
    uint32_t pfall = 0U;
#endif

    pace_pings();

    /* 1. ECHO must be low before the trigger (the SDK bench sequence). */
    pre_us = sys_time_us();
    while (gpio_get_val(B5_PIN_US_ECHO) != 0U)
    {
        if ((sys_time_us() - pre_us) >= ((uint32_t)B5_PRETRIG_LOW_WAIT_MS * 1000U))
        {
            *p_ping = b5_classify_ping(false, false, false, false, 0U);
            g_last_trig_us = sys_time_us();     /* keep the pacing */
            g_b_triggered  = TRUE;
#if B5_US_DIAG
            diag_print(p_ping, sys_time_us() - pre_us, 0U, 0U, 0U, 0U);
#endif
            return E_OK;
        }
        (void)tk_dly_tsk(1);
    }
    pre_us = sys_time_us() - pre_us;

    g_seq++;                            /* new ping, new sequence number */
    g_arm_seq = g_seq;
    B5_DMB();
    (void)tk_clr_flg(g_echo_flg, ~FLG_ECHO_DONE);

    (void)gpio_set_val(B5_PIN_US_TRIG, 1U);
    sys_busy_wait_us(B5_TRIG_PULSE_US);
    (void)gpio_set_val(B5_PIN_US_TRIG, 0U);

    start_us       = sys_time_us();
    g_last_trig_us = start_us;
    g_b_triggered  = TRUE;

#if B5_US_DIAG
    diag_poll(start_us, &prise, &pfall);
#endif

    for (;;)
    {
        UINT     ptn     = 0U;
        uint32_t used_us = sys_time_us() - start_us;
        TMO      tmo;
        ER       er;

        tmo = (used_us >= wait_us) ? TMO_POL :
              (TMO)(((wait_us - used_us) + 999U) / 1000U);
        er  = tk_wai_flg(g_echo_flg, FLG_ECHO_DONE, TWF_ORW | TWF_CLR, &ptn,
                         tmo);

        if (er == E_OK)
        {
            word = g_result;            /* one aligned 32-bit load */

            if (b5_echo_seq(word) == g_seq)
            {
                b_rise = true;
                b_fall = true;
                width_us = ((word & B5_ECHO_FLAG_SAT) != 0U) ?
                           0x1FFFFU : b5_echo_width_us(word);
                break;                  /* the echo of this ping */
            }

            continue;                   /* stale echo: keep waiting */
        }

        if (er != E_TMOUT)
        {
            return er;
        }

        /* Timed out: classify what was seen for THIS ping. */
        word = g_rise_word;
        b_rise = (((word & RISE_FLAG_SEEN) != 0U) &&
                  ((uint8_t)(word & 0xFFU) == g_seq));
        word = g_short_word;
        b_short = (((word & RISE_FLAG_SEEN) != 0U) &&
                   ((uint8_t)(word & 0xFFU) == g_seq));
        if (b_short && b_rise)
        {
            /* the rise belonged to a dropped short pulse, unless ECHO is
             * still high now (a real pulse that never fell) */
            b_rise = (gpio_get_val(B5_PIN_US_ECHO) != 0U);
        }
        break;
    }

    *p_ping = b5_classify_ping(true, b_rise, b_fall, b_short, width_us);
#if B5_US_DIAG
    diag_print(p_ping, pre_us, start_us, prise, pfall, width_us);
#endif
    return E_OK;
}

/**
 * @brief  Take one range reading (legacy view of b5_us_ping_ex).
 * @param  p_range_mm  range in mm, or B5_RANGE_FAR_MM for a no-return.
 * @return E_OK, or E_TMOUT for any fault (stuck high, silent, no fall,
 *         noise only), so the caller counts it towards a sensor fault.
 */
ER
b5_us_ping (int32_t * p_range_mm)
{
    b5_ping_t p;
    ER        er = b5_us_ping_ex(&p);

    if (er != E_OK)
    {
        return er;
    }
    if (p.cls == (uint32_t)B5_PING_FAULT)
    {
        return E_TMOUT;
    }
    *p_range_mm = (int32_t)p.range_mm;
    return E_OK;
}
