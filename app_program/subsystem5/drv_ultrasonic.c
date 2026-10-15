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
#include "drv_ultrasonic.h"

#define FLG_ECHO_DONE       (1U << 0)

/* Measurement phase, shared between task and interrupt. */
#define PHASE_IDLE          (0U)
#define PHASE_ARMED         (1U)    /* trigger sent, waiting for rise */
#define PHASE_HIGH          (2U)    /* echo is high                   */

/* mm = us * 343 / 2000  (343 m/s, there and back) */
#define SOUND_NUM           (343U)
#define SOUND_DEN           (2000U)

static ID                g_echo_flg = 0;
static volatile uint32_t g_phase    = PHASE_IDLE;
static volatile uint32_t g_rise_us  = 0U;
static volatile uint32_t g_width_us = 0U;
static uint32_t          g_last_trig_us = 0U;
static BOOL              g_b_triggered  = FALSE;

/* GPIO interrupt callback for ECHO (interrupt context). */
static void
echo_edge (uint32_t gpio, uint32_t edges, uint32_t t_us)
{
    (void)gpio;

    if (((edges & SYS_GPIO_EDGE_RISE) != 0U) && (g_phase == PHASE_ARMED))
    {
        g_rise_us = t_us;
        g_phase   = PHASE_HIGH;
    }

    if (((edges & SYS_GPIO_EDGE_FALL) != 0U) && (g_phase == PHASE_HIGH))
    {
        g_width_us = t_us - g_rise_us;
        g_phase    = PHASE_IDLE;
        (void)tk_set_flg(g_echo_flg, FLG_ECHO_DONE);
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
    UINT     ptn = 0U;
    ER       er;
    uint32_t width_us;

    pace_pings();

    (void)tk_clr_flg(g_echo_flg, ~FLG_ECHO_DONE);
    g_phase = PHASE_ARMED;

    (void)gpio_set_val(B5_PIN_US_TRIG, 1U);
    sys_busy_wait_us(B5_TRIG_PULSE_US);
    (void)gpio_set_val(B5_PIN_US_TRIG, 0U);

    g_last_trig_us = sys_time_us();
    g_b_triggered  = TRUE;

    er = tk_wai_flg(g_echo_flg, FLG_ECHO_DONE, TWF_ORW | TWF_CLR, &ptn,
                    B5_ECHO_WAIT_MS);

    if (er == E_TMOUT)
    {
        uint32_t phase = g_phase;

        g_phase = PHASE_IDLE;

        if (phase == PHASE_HIGH)
        {
            *p_range_mm = B5_RANGE_FAR_MM;  /* echo never fell: no target */
            return E_OK;
        }

        return E_TMOUT;                     /* no echo at all: sensor fault */
    }

    if (er < E_OK)
    {
        return er;
    }

    width_us = g_width_us;

    if (width_us > B5_ECHO_MAX_US)
    {
        *p_range_mm = B5_RANGE_FAR_MM;
    }
    else
    {
        *p_range_mm = (int32_t)((width_us * SOUND_NUM) / SOUND_DEN);
    }

    return E_OK;
}
