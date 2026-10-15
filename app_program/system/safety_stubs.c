/**
 * @file    safety_stubs.c
 * @brief   Weak defaults for functions other buddies provide.
 *
 * Replaced at link time by a normal definition with the same name.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#include "safety_iface.h"

#define SAFETY_WEAK __attribute__((weak))

/* Without Motion Control there is no drive to remove, and no one calls
 * safety_ack_drive_off(): stops are then reported as slow (> 100 ms). */
SAFETY_WEAK void
mc_inhibit (bool b_inhibit)
{
    (void)b_inhibit;
}

SAFETY_WEAK void
safety_telemetry_publish (safety_status_t const * p_status)
{
    (void)p_status;
}
