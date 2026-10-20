/**
 * @file    drv_servo.c
 * @brief   SG90 scanning servo on GP12 (Robo Pico servo port 1, PWM 6A).
 *
 * 50 Hz servo PWM from the 125 MHz system clock:
 *   divider 125   -> 1 MHz counter (1 count = 1 us)
 *   TOP 19999     -> 20 ms period
 *   compare value -> pulse width in us (B5_SERVO_MIN_US .. B5_SERVO_MAX_US)
 *
 * The servo is powered from VIN through the servo header, never from 3V3
 * (Assumption A8). b5_servo_move() sleeps for the settle time after each
 * step so the next ping is taken with the horn at rest.
 *
 * The SG90 has no position feedback. The only "servo timeout" this driver
 * can detect is the PWM slice not running or not holding the commanded
 * pulse, which it checks by reading the registers back after each move.
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#include "b5_config.h"
#include "../system/sys_hal.h"
#include "drv_servo.h"
#include "b5_pack.h"

#define PWM_SLICE               ((B5_PIN_SERVO_PWM >> 1) & 0x7U)
#define PWM_SLICE_REG(off)      (PWM_BASE + (off) + (PWM_SLICE * 0x14U))
#define PWM_DIV_INT_SHIFT       (4U)
#define PWM_CLK_DIV             (125U)          /* 125 MHz / 125 = 1 MHz */
#define PWM_TOP_20MS            (19999U)
#define PWM_IS_CHAN_B           ((B5_PIN_SERVO_PWM & 1U) != 0U)

#define ANGLE_SPAN_DD           (1800)  /* -900 .. +900                  */
#define ANGLE_STEP_REF_DD       (300)

static int16_t g_angle_dd = B5_AHEAD_DD;

/*
 * Signed angle (-900 left .. +900 right) to pulse width. The horn turns
 * towards smaller pulses for "right", so +900 maps to B5_SERVO_MIN_US.
 */
static uint32_t
angle_to_pulse_us (int16_t angle_dd)
{
    int32_t from_right = B5_ANGLE_MAX_DD - angle_dd;    /* 0 .. 1800 */

    if (B5_SERVO_REVERSED != 0)
    {
        from_right = ANGLE_SPAN_DD - from_right;
    }

    return B5_SERVO_MIN_US +
           (((B5_SERVO_MAX_US - B5_SERVO_MIN_US) * (uint32_t)from_right) /
            (uint32_t)ANGLE_SPAN_DD);
}

static uint32_t
read_back_pulse (void)
{
    uint32_t cc = (uint32_t)in_w(PWM_SLICE_REG(PWM_CHx_CC));

    return PWM_IS_CHAN_B ? (cc >> 16) : (cc & 0xFFFFU);
}

/**
 * @brief  Start 50 Hz PWM on the servo pin, horn pointing ahead.
 */
ER
b5_servo_init (void)
{
    ER er;

    sys_unreset(RESETS_RESET_PWM);

    er = pwm_set_pin(B5_PIN_SERVO_PWM);

    if (er == E_OK)
    {
        out_w(PWM_SLICE_REG(PWM_CHx_DIV), PWM_CLK_DIV << PWM_DIV_INT_SHIFT);
        er = pwm_set_wrap(B5_PIN_SERVO_PWM, PWM_TOP_20MS);
    }

    if (er == E_OK)
    {
        er = pwm_set_cc(B5_PIN_SERVO_PWM, angle_to_pulse_us(B5_AHEAD_DD));
    }

    if (er == E_OK)
    {
        er = pwm_set_enabled(B5_PIN_SERVO_PWM, TRUE);
    }

    g_angle_dd = B5_AHEAD_DD;

    return er;
}

/**
 * @brief  Move to an angle and sleep until it has settled.
 * @param  angle_dd  -900 = left, 0 = ahead, +900 = right (0.1 deg);
 *                   clamped to B5_SCAN_START_DD .. B5_SCAN_END_DD.
 * @return E_OK, or E_IO if the PWM slice is not holding the pulse.
 */
ER
b5_servo_move (int16_t angle_dd)
{
    uint32_t pulse;
    uint32_t delta;
    uint32_t settle_ms;

    /* Never past the sweep ends: beyond about 55 deg the sensor bracket
     * hits the car's casing (mechanical limit, see b5_config.h). */
    angle_dd = (int16_t)b5_clamp(angle_dd, B5_SCAN_START_DD, B5_SCAN_END_DD);

    pulse = angle_to_pulse_us(angle_dd);
    (void)pwm_set_cc(B5_PIN_SERVO_PWM, pulse);

    if (((in_w(PWM_SLICE_REG(PWM_CHx_CSR)) & PWM_CH_CSR_EN) == 0U) ||
        (read_back_pulse() != pulse))
    {
        return E_IO;
    }

    delta = (uint32_t)((angle_dd > g_angle_dd) ? (angle_dd - g_angle_dd)
                                                : (g_angle_dd - angle_dd));
    settle_ms = (B5_SERVO_SETTLE_MS_30 * delta) / ANGLE_STEP_REF_DD;

    if (settle_ms < B5_SERVO_SETTLE_MIN_MS)
    {
        settle_ms = B5_SERVO_SETTLE_MIN_MS;
    }

    g_angle_dd = angle_dd;
    (void)tk_dly_tsk((TMO)settle_ms);

    return E_OK;
}

/**
 * @brief  Last commanded angle.
 */
int16_t
b5_servo_angle (void)
{
    return g_angle_dd;
}
