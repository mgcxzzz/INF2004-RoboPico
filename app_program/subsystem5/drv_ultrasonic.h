/**
 * @file    drv_ultrasonic.h
 * @brief   HC-SR04+ driver: TRIG on GP16, ECHO on GP17 (edge interrupt).
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#ifndef B5_ULTRASONIC_H
#define B5_ULTRASONIC_H

#include <stdint.h>
#include <tk/tkernel.h>

#include "obstacle_profile.h"

ER b5_us_init(void);

/* Rule B: one classified ping (ECHO / NO_RETURN / FAULT + reason). */
ER b5_us_ping_ex(b5_ping_t * p_ping);

/* Legacy range view: ECHO -> mm, NO_RETURN -> B5_RANGE_FAR_MM,
 * FAULT -> E_TMOUT. */
ER b5_us_ping(int32_t * p_range_mm);

#endif /* B5_ULTRASONIC_H */
