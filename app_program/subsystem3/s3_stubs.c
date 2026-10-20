/**
 * @file    s3_stubs.c
 * @brief   Weak defaults for the functions other buddies provide to
 *          Buddy 3. A normal definition with the same name anywhere else
 *          replaces each one at link time.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#include "../subsystem5/b5_iface.h"
#include "s3_iface.h"

#define S3_WEAK __attribute__((weak))

/* Buddy 2: net signed distance travelled, int16 mm, may wrap. Until
 * Buddy 2 provides it, use the same motion estimate Buddy 5 reads, so
 * there is one source of distance for the whole car. */
S3_WEAK bool
s3_motion_get_distance (int16_t * p_mm)
{
    b5_motion_est_t est;

    if (!b5_motion_get_estimate(&est))
    {
        return false;
    }

    *p_mm = est.distance_mm;
    return true;
}

/* Buddy 1: queue the binary frame for MQTT. Nothing until then. */
S3_WEAK void
line_telemetry_publish (uint8_t const * p_frame, uint32_t len)
{
    (void)p_frame;
    (void)len;
}
