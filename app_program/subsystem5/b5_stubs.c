/**
 * @file    b5_stubs.c
 * @brief   Weak default implementations of the other buddies' interfaces.
 *
 * These let Buddy 5 build and run on its own. Each one is replaced at link
 * time by a normal (strong) definition with the same name in another
 * buddy's code, so nothing here needs editing during integration.
 *
 * The defaults fail safe: with no Navigation Coordinator, a stop request is
 * never accepted, so Halting times out into Fault instead of the car
 * carrying on into an obstacle.
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#include "b5_iface.h"

#define B5_WEAK     __attribute__((weak))

B5_WEAK void
b5_nav_request_slow (bool b_slow)
{
    (void)b_slow;
}

B5_WEAK void
b5_nav_request_stop (void)
{
}

B5_WEAK void
b5_nav_send_plan (b5_plan_t const * p_plan)
{
    (void)p_plan;
}

B5_WEAK void
b5_nav_request_rejoin (b5_rejoin_t const * p_req)
{
    (void)p_req;
}

B5_WEAK void
b5_nav_request_reverse (int32_t distance_mm)
{
    (void)distance_mm;
}

B5_WEAK void
b5_nav_blocked_route (b5_profile_t const * p_prof)
{
    (void)p_prof;
}

B5_WEAK void
b5_nav_release_motion (void)
{
}

B5_WEAK bool
b5_motion_get_estimate (b5_motion_est_t * p_est)
{
    p_est->distance_mm = 0;
    p_est->heading_dd  = 0;
    return false;                       /* no estimate available        */
}

B5_WEAK bool
b5_line_detected (void)
{
    return false;
}

B5_WEAK void
b5_telemetry_publish (b5_status_t const * p_status)
{
    (void)p_status;
}

B5_WEAK void
b5_report_fault (b5_fault_t fault)
{
    (void)fault;
}
