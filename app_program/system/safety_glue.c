/**
 * @file    safety_glue.c
 * @brief   Connects Buddy 5 and the Navigation Coordinator to the supervisor.
 *
 * Real (strong) definitions of functions those modules call, replacing
 * their weak placeholders:
 *   b5_report_fault()     Buddy 5 fault       -> safety_report_fault()
 *   nav_report_fault()    coordinator fault   -> safety_report_fault()
 *   nav_start_allowed()   start guard         -> no latched stop or fault
 *   nav_heartbeat()       coordinator alive   -> safety_kick()
 *
 * Which faults are critical (stop and latch until reset) and which are
 * warnings (logged; the reporting task already stopped what it was doing)
 * is decided here, in one place.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#include "../subsystem5/b5_iface.h"
#include "nav_config.h"
#include "nav_iface.h"
#include "safety_iface.h"

void
b5_report_fault (b5_fault_t fault)
{
    bool b_critical;

    switch (fault)
    {
        case B5_FAULT_ECHO_TIMEOUT:     /* no forward sensing          */
        case B5_FAULT_SERVO:            /* scanner broken              */
        case B5_FAULT_STOP_TIMEOUT:     /* the car did not stop        */
        case B5_FAULT_MOTION:           /* a bypass move failed        */
            b_critical = true;
            break;

        default:                        /* LINE_LOST, BLOCKED: the car is
                                         * stopped safely, nothing broken */
            b_critical = false;
            break;
    }

    (void)safety_report_fault(SAFETY_SRC_OBSTACLE, (uint8_t)fault,
                              b_critical);
}

void
nav_report_fault (uint8_t code)
{
    /* A failed Motion Control command is hardware trouble; a lost line
     * or Buddy 5 stopping is not (Buddy 5 reports its own cause). */
    bool b_critical = (code == NAV_FAULT_MOTION);

    (void)safety_report_fault(SAFETY_SRC_NAV, code, b_critical);
}

bool
nav_start_allowed (void)
{
    return ((safety_get_state() == SAFETY_ST_NORMAL) &&
            !safety_drive_inhibited());
}

void
nav_heartbeat (void)
{
    safety_kick(SAFETY_SRC_NAV);
}
