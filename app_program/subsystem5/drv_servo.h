/**
 * @file    drv_servo.h
 * @brief   SG90 scanning servo on GP12 (Robo Pico servo port 1, PWM 6A).
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#ifndef B5_SERVO_H
#define B5_SERVO_H

#include <stdint.h>
#include <tk/tkernel.h>

ER      b5_servo_init(void);
ER      b5_servo_move(int16_t angle_dd);
int16_t b5_servo_angle(void);

#endif /* B5_SERVO_H */
