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
 * Range = echo width * 343 m/s / 2. A pulse longer than B5_ECHO_MAX_US, or
 * one still high at the timeout, means "no target" and returns
 * B5_RANGE_FAR_MM. No rising edge at all means the sensor did not respond,
 * and the call returns E_TMOUT so the task can count it as a fault.
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
static volatile uint32_t g_rise_word = 0U;
static volatile uint32_t g_result    = 0U;

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

/**
 * @brief  Take one range reading.
 * @param  p_range_mm  range in mm, or B5_RANGE_FAR_MM for no target.
 * @return E_OK, or E_TMOUT if the sensor did not respond at all.
 */
ER
b5_us_ping (int32_t * p_range_mm)
{
    uint32_t wait_us = (uint32_t)B5_ECHO_WAIT_MS * 1000U;
    uint32_t start_us;
    uint32_t word;

    pace_pings();

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
                break;                  /* the echo of this ping */
            }

            continue;                   /* stale echo: keep waiting */
        }

        if (er != E_TMOUT)
        {
            return er;
        }

        /* Timed out. If this ping's echo rose but never fell, there is
         * no target in range; if it never rose, the sensor is silent. */
        word = g_rise_word;

        if (((word & RISE_FLAG_SEEN) != 0U) &&
            ((uint8_t)(word & 0xFFU) == g_seq))
        {
            *p_range_mm = B5_RANGE_FAR_MM;
            return E_OK;
        }

        return E_TMOUT;
    }

    if (((word & B5_ECHO_FLAG_SAT) != 0U) ||
        (b5_echo_width_us(word) > B5_ECHO_MAX_US))
    {
        *p_range_mm = B5_RANGE_FAR_MM;
    }
    else
    {
        *p_range_mm = (int32_t)((b5_echo_width_us(word) * SOUND_NUM) /
                                SOUND_DEN);
    }

    return E_OK;
}
