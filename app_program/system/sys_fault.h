/**
 * @file    sys_fault.h
 * @brief   Shared fault bitmask (Tribunal 1: minimum-bit data).
 *
 * One bit per fault condition instead of separate integer variables. The
 * mask lives in a µT-Kernel event flag, so setting or clearing a bit is
 * done by the kernel as one operation: it is safe from any task or
 * interrupt handler on either RP2040 core. A plain `faults |= bit` would
 * not be: the Cortex-M0+ has no atomic read-modify-write instruction, so a
 * second writer could overwrite the first and a fault would be lost.
 *
 * Each subsystem sets only its own bits. The Safety Supervisor still
 * decides what is critical (safety_glue.c); this mask is the shared,
 * compact record of which faults are active, e.g. for telemetry.
 *
 * Group IS28 (team). Coding standard: Barr-C:2018.
 */

#ifndef SYS_FAULT_H
#define SYS_FAULT_H

#include <stdint.h>
#include <tk/tkernel.h>

/* One bit each. Owner in brackets; only the owner sets or clears it. */
#define SYS_FAULT_MOTOR_STALL   (1U << 0)   /* [S2] wheel not turning       */
#define SYS_FAULT_ENCODER       (1U << 1)   /* [S2] encoder not counting    */
#define SYS_FAULT_IMU           (1U << 2)   /* [S4] GY-511 not answering    */
#define SYS_FAULT_NO_ECHO       (1U << 3)   /* [S5] ultrasonic silent       */
#define SYS_FAULT_WIFI_LOSS     (1U << 4)   /* [S1] WiFi or broker lost     */
#define SYS_FAULT_SERVO         (1U << 5)   /* [S5] servo PWM not holding   */
#define SYS_FAULT_LINE_SENSOR   (1U << 6)   /* [S3] IR sensor implausible   */
#define SYS_FAULT_OBSTACLE      (1U << 7)   /* [S5] stop/motion/line/blocked */
#define SYS_FAULT_ALL           (0xFFU)

ER       sys_fault_init(void);              /* system_init() calls this    */
ER       sys_fault_set(uint32_t bits);      /* any task or interrupt       */
ER       sys_fault_clear(uint32_t bits);    /* any task or interrupt       */
uint32_t sys_fault_get(void);               /* snapshot, never blocks      */

#endif /* SYS_FAULT_H */
