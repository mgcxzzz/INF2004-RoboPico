#ifndef ROBOPICO_MOTOR_OUTPUT_H
#define ROBOPICO_MOTOR_OUTPUT_H
#include <stdint.h>
/* Private platform port. Only the application commit/safety owner calls it. */
void platform_motor_init(void);
void platform_motor_write(unsigned wheel, uint16_t channel_a, uint16_t channel_b);
void platform_motor_off(void);
int16_t platform_motor_read(unsigned wheel);
#endif
