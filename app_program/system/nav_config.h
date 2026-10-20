/**
 * @file    nav_config.h
 * @brief   Navigation Coordinator build-time configuration.
 *
 * Values marked TUNE are starting points to adjust on the real car.
 * Units: mm, mm/s, tenths of a degree (dd), ms.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#ifndef NAV_CONFIG_H
#define NAV_CONFIG_H

/* RTOS. Event-driven, high priority (Section 3). Lower number = higher;
 * 1-4 belong to the kernel (USB console 3, WiFi 4). */
#define NAV_TASK_PRIORITY       (10)    /* system band, TEAM.md section 4 */
#define NAV_TASK_STACK_BYTES    (4096)
#define NAV_INBOX_MSGS          (16U)   /* queued messages before drops  */
#define NAV_IDLE_WAKE_MS        (100)   /* watchdog tick when no message */

/* Speeds. */
#define NAV_SLOW_PERCENT        (50)    /* TUNE: line speed when slowed  */
#define NAV_MANOEUVRE_MM_S      (120)   /* TUNE: bypass / search speed   */
#define NAV_TURN_MM_S           (100)   /* TUNE: wheel speed in turns    */
#define NAV_REVERSE_MM_S        (80)    /* TUNE: reversing speed         */

/* Bypass geometry: leave and rejoin the line at 45 degrees. */
#define NAV_BYPASS_ANGLE_DD     (450)
#define NAV_SQRT2_X1000         (1414)  /* 1 / sin(45 deg), x1000        */
#define NAV_REJOIN_MIN_MM       (20)    /* closer than this = on line    */
#define NAV_REJOIN_MAX_MM       (600)   /* cap on one rejoin leg         */

/* Barcode turns (FR-LINE-04). Positive = right. */
#define NAV_TURN_LEFT_DD        (-900)
#define NAV_TURN_RIGHT_DD       (900)
#define NAV_TURN_UTURN_DD       (1800)

/* Line search after an unexpected line loss: alternate sweeps of
 * growing size. After the last one fails the car goes to Safe Stop. */
#define NAV_SEARCH_STEPS        (4U)
#define NAV_SEARCH_STEP1_DD     (150)   /* then -300, +450, -600 ...     */

/* A bounded Motion Control command that takes longer than this is
 * treated as failed. */
#define NAV_MOTION_TMO_MS       (8000)

/* Longest single drive or turn a manoeuvre step can hold: steps store
 * the value in 13 signed bits (+/-4095 mm or dd). The longest real step
 * is a bypass leg, at most the 12-bit obstacle distance (4095 mm). */
#define NAV_STEP_VALUE_MAX      (4095)

/* Fault codes passed to nav_report_fault(): 8 bits, the coordinator's own
 * numbering (the Safety Supervisor records the source separately). */
#define NAV_FAULT_MOTION        (0x01U)
#define NAV_FAULT_LINE_LOST     (0x02U)
#define NAV_FAULT_OBSTACLE      (0x03U)     /* Buddy 5 released from Fault */

#endif /* NAV_CONFIG_H */
