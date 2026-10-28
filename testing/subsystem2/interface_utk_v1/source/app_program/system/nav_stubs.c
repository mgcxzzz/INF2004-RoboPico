/**
 * @file    nav_stubs.c
 * @brief   Weak defaults for the functions other buddies provide.
 *
 * Each is replaced at link time by a normal definition with the same name
 * in that buddy's code. The defaults do nothing, and Motion Control
 * commands are never answered, so a manoeuvre times out and the car goes
 * to Safe Stop: fail safe.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#include "nav_iface.h"

#define NAV_WEAK    __attribute__((weak))

NAV_WEAK ER
mc_set_wheel_speeds (int16_t left_mm_s, int16_t right_mm_s)
{
    (void)left_mm_s;
    (void)right_mm_s;
    return E_OK;
}

NAV_WEAK ER
mc_drive_distance (int32_t distance_mm, int16_t speed_mm_s, uint32_t cmd_id)
{
    (void)distance_mm;
    (void)speed_mm_s;
    (void)cmd_id;
    return E_OK;
}

NAV_WEAK ER
mc_turn (int32_t angle_dd, int16_t speed_mm_s, uint32_t cmd_id)
{
    (void)angle_dd;
    (void)speed_mm_s;
    (void)cmd_id;
    return E_OK;
}

NAV_WEAK ER
mc_stop (uint32_t cmd_id)
{
    (void)cmd_id;
    return E_OK;
}

NAV_WEAK void line_set_control (bool b_granted)      { (void)b_granted; }
NAV_WEAK void line_notify_state (nav_state_t state)  { (void)state; }
NAV_WEAK void imu_notify_state (nav_state_t state)   { (void)state; }
NAV_WEAK void wifi_notify_state (nav_state_t state)  { (void)state; }
NAV_WEAK void nav_report_fault (uint8_t code)        { (void)code; }

NAV_WEAK void
nav_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    (void)p_frame;
    (void)len;
}
NAV_WEAK void nav_reinitialise (void)                { }
NAV_WEAK bool nav_start_allowed (void)              { return true; }
NAV_WEAK void nav_heartbeat (void)                  { }
