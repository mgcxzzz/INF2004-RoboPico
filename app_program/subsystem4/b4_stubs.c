/**
 * @file    b4_stubs.c
 * @brief   Weak defaults for the functions other buddies provide to
 *          Buddy 4. A normal definition with the same name anywhere else
 *          replaces each one at link time; nothing here needs editing
 *          during integration.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#include "../subsystem5/b5_iface.h"
#include "../system/sys_config.h"
#include "imu_iface.h"

#define B4_WEAK     __attribute__((weak))

/* Buddy 2: until Motion Control offers its estimate to the IMU directly,
 * read the same shared estimate everyone else reads (IF-16), as Buddy 3
 * does. That interface has no age: it returns false when it has nothing
 * current, so a true answer counts as fresh (age 0). */
B4_WEAK bool
imu_motion_get (imu_motion_in_t * p_est)
{
    b5_motion_est_t est;

    if (!b5_motion_get_estimate(&est))
    {
        return false;
    }

    p_est->distance_mm = est.distance_mm;
    p_est->heading_dd  = est.heading_dd;
    p_est->age_ms      = 0U;
    return true;
}

/* Anyone on the chip: nobody listens yet. */
B4_WEAK void
imu_event_notify (imu_event_t const * p_evt)
{
    (void)p_evt;
}

/* Buddy 1: queue the frame for MQTT. Until then, with SYS_CONSOLE_STATUS
 * set, decode it the way the PC will and print it. */
B4_WEAK void
imu_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
#if SYS_CONSOLE_STATUS
    static uint8_t last_state = 0xFFU;
    imu_tlm_t      t;

    if (!imu_tlm_unpack(p_frame, len, &t))
    {
        tm_printf((UB *)"[imu] bad telemetry frame\n");
        return;
    }

    if (t.state != last_state)
    {
        tm_printf((UB *)"[imu] %s%s\n",
                  (UB *)imu_state_name((imu_state_t)t.state),
                  t.b_valid ? "" : " (data not valid)");
        last_state = t.state;
    }

    switch ((imu_evt_type_t)t.evt)
    {
        case IMU_EVT_MOTION:
            tm_printf((UB *)"[imu] motion: %s\n",
                      (UB *)imu_motion_name((imu_motion_t)t.motion));
            break;

        case IMU_EVT_HUMP_START:
            tm_printf((UB *)"[imu] hump start, pitch %d.%d deg%s\n",
                      (INT)(t.pitch_dd / 10),
                      (INT)(((t.pitch_dd < 0) ? -t.pitch_dd : t.pitch_dd) %
                            10),
                      t.b_odo ? "" : " (no odometry)");
            break;

        case IMU_EVT_HUMP_PEAK:
            tm_printf((UB *)"[imu] HUMP %d mm (highest this run %d mm, "
                            "%d valid)\n",
                      (INT)t.height_mm, (INT)t.max_height_mm,
                      (INT)t.humps);
            break;

        case IMU_EVT_HUMP_REJECTED:
            tm_printf((UB *)"[imu] hump rejected: %s (peak %d mm)\n",
                      (UB *)imu_reject_name((imu_reject_t)t.detail),
                      (INT)t.height_mm);
            break;

        case IMU_EVT_IMPACT:
            tm_printf((UB *)"[imu] IMPACT %d mg, Safety warned %d ms after "
                            "the sample (limit 50)\n",
                      (INT)t.shock_mg, (INT)t.shock_ms);
            break;

        case IMU_EVT_FAULT:
            tm_printf((UB *)"[imu] FAULT 0x%x, IMU data invalid until a "
                            "reset\n", (UINT)t.detail);
            break;

        case IMU_EVT_RUN_SUMMARY:
            tm_printf((UB *)"[imu] run summary: highest hump %d mm, %d "
                            "valid, %d rejected, %d frames missed\n",
                      (INT)t.height_mm, (INT)t.humps, (INT)t.rejected,
                      (INT)t.missed);
            break;

        default:
            break;                      /* periodic status */
    }
#else
    (void)p_frame;
    (void)len;
#endif
}
