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

ER b5_us_init(void);
ER b5_us_ping(int32_t * p_range_mm);

#endif /* B5_ULTRASONIC_H */
