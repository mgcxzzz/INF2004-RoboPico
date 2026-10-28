/**
 * @file    nav_b5.c
 * @brief   Glue: Buddy 5's requests (b5_iface.h) into the coordinator inbox.
 *
 * These are the real (strong) definitions of the b5_nav_*() functions, so
 * they replace the weak placeholders in buddy5/src/b5_stubs.c. Each one
 * only queues a message; the coordinator task does the work and answers
 * with b5_post_event().
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#include "../subsystem5/b5_iface.h"
#include "nav_iface.h"
#include "nav_internal.h"

void
b5_nav_request_slow (bool b_slow)
{
    (void)nav_post_msg(NAV_MSG_B5_SLOW, b_slow ? 1 : 0, 0, 0);
}

void
b5_nav_request_stop (void)
{
    (void)nav_post_msg(NAV_MSG_B5_STOP, 0, 0, 0);
}

void
b5_nav_send_plan (b5_plan_t const * p_plan)
{
    /* The plan is a bit-field record; widen each field for the inbox. */
    (void)nav_post_msg(NAV_MSG_B5_PLAN, (int32_t)p_plan->side,
                       (int32_t)p_plan->lateral_mm,
                       (int32_t)p_plan->forward_mm);
}

void
b5_nav_request_rejoin (b5_rejoin_t const * p_req)
{
    (void)nav_post_msg(NAV_MSG_B5_REJOIN, (int32_t)p_req->lateral_mm,
                       (int32_t)p_req->heading_dd, (int32_t)p_req->search_mm);
}

void
b5_nav_request_reverse (int32_t distance_mm)
{
    (void)nav_post_msg(NAV_MSG_B5_REVERSE, distance_mm, 0, 0);
}

void
b5_nav_blocked_route (b5_profile_t const * p_prof)
{
    (void)p_prof;
    (void)nav_post_msg(NAV_MSG_B5_BLOCKED, 0, 0, 0);
}

void
b5_nav_release_motion (void)
{
    /* Tell the coordinator whether this release comes from Buddy 5's
     * Fault state, so it stops the car instead of resuming the line. */
    int32_t b_fault = (b5_get_state() == B5_ST_FAULT) ? 1 : 0;

    (void)nav_post_msg(NAV_MSG_B5_RELEASE, b_fault, 0, 0);
}
