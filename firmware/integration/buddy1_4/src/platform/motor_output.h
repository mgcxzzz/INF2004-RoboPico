/**
 * @file motor_output.h
 * @brief Motor output.
 * @details Platform access obeys the configured motor/flash/interrupt owner.
 * Hardware and kernel calling conventions are retained under CEX-02/CEX-07.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef ROBOPICO_MOTOR_OUTPUT_H
#define ROBOPICO_MOTOR_OUTPUT_H
#include <stdint.h>
/* Private platform port. Only the application commit/safety owner calls it. */
void platform_motor_init (void);
void platform_motor_write (unsigned wheel, uint16_t channel_a,
                           uint16_t channel_b);
void platform_motor_off (void);
int16_t platform_motor_read (unsigned wheel);
#endif

/* End of file. */
