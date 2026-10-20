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
 * (Assumption A8). b5_servo_move() turns the horn slowly, in small steps
 * at B5_SERVO_SPEED_DD_S (an SG90 left alone swings fast enough to snap
 * the sensor bracket into the wiring), then rests for B5_SERVO_SETTLE_MS
 * so the next ping is taken with the horn still.
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
#define SLEW_STEP_DD            ((int32_t)((B5_SERVO_SPEED_DD_S *        \
                                            B5_SERVO_STEP_MS) / 1000U))

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

/* Command one pulse and check the PWM slice is holding it. */
static ER
set_angle (int16_t angle_dd)
{
    uint32_t pulse = angle_to_pulse_us(angle_dd);

    (void)pwm_set_cc(B5_PIN_SERVO_PWM, pulse);

    if (((in_w(PWM_SLICE_REG(PWM_CHx_CSR)) & PWM_CH_CSR_EN) == 0U) ||
        (read_back_pulse() != pulse))
    {
        return E_IO;
    }

    g_angle_dd = angle_dd;
    return E_OK;
}

/**
 * @brief  Turn slowly to an angle, then sleep until it has settled.
 * @param  angle_dd  -900 = left, 0 = ahead, +900 = right (0.1 deg);
 *                   clamped to B5_SCAN_START_DD .. B5_SCAN_END_DD.
 * @return E_OK, or E_IO if the PWM slice is not holding the pulse.
 *
 * NOTE: blocks for the whole move: about 1 s for 55 deg at 60 deg/s.
 */
ER
b5_servo_move (int16_t angle_dd)
{
    int32_t target;
    int32_t now;
    int32_t diff;
    ER      er = E_OK;

    /* Never past the sweep ends: beyond about 55 deg the sensor bracket
     * hits the car's casing (mechanical limit, see b5_config.h). */
    target = b5_clamp(angle_dd, B5_SCAN_START_DD, B5_SCAN_END_DD);
    now    = (int32_t)g_angle_dd;

    /* One step of at most SLEW_STEP_DD per PWM period. */
    while ((er == E_OK) && (now != target))
    {
        diff = target - now;

        if (diff > SLEW_STEP_DD)
        {
            diff = SLEW_STEP_DD;
        }
        else if (diff < -SLEW_STEP_DD)
        {
            diff = -SLEW_STEP_DD;
        }
        else
        {
            /* last, shorter step */
        }

        now += diff;
        er   = set_angle((int16_t)now);   /* within +/-55 deg: fits */

        if ((er == E_OK) && (now != target))
        {
            (void)tk_dly_tsk((TMO)B5_SERVO_STEP_MS);
        }
    }

    if (er == E_OK)
    {
        /* A zero-length move still re-checks the PWM slice. */
        er = set_angle((int16_t)target);
    }

    if (er == E_OK)
    {
        (void)tk_dly_tsk((TMO)B5_SERVO_SETTLE_MS);
    }

    return er;
}

/**
 * @brief  Last commanded angle.
 */
int16_t
b5_servo_angle (void)
{
    return g_angle_dd;
}
