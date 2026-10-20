/**
 * @file    b5_config.h
 * @brief   Buddy 5 build-time configuration: pins, timing and tuning.
 *
 * Every number the Obstacle Scan and Recovery task depends on lives here,
 * so tuning on the real track never needs a change anywhere else.
 * Values marked "TUNE" are design choices from the Week 6 report that must
 * be confirmed by the bench tests in Assumptions A5 to A8.
 *
 * Units: distance in mm, angle in tenths of a degree ("dd"), time in ms
 * unless the name ends in _US.
 *
 * Group IS28, Buddy 5 (2500339). Coding standard: Barr-C:2018.
 */

#ifndef B5_CONFIG_H
#define B5_CONFIG_H

/* ------------------------------------------------------------------------
 * Pins (Section 2 pin table, after the approved swap)
 * --------------------------------------------------------------------- */
#define B5_PIN_US_TRIG          (16U)   /* Grove 4, Pico W pin 21        */
#define B5_PIN_US_ECHO          (17U)   /* Grove 4, Pico W pin 22, IRQ   */
#define B5_PIN_SERVO_PWM        (12U)   /* Robo Pico servo port 1, PWM 6A */

/* ------------------------------------------------------------------------
 * RTOS
 * --------------------------------------------------------------------- */
#define B5_TASK_PRIORITY        (20)    /* S5 band 20-21 (TEAM.md section 4) */
#define B5_TASK_STACK_BYTES     (4096)  /* SMP kernel path needs 4 KiB   */
#define B5_ECHO_IRQ_LEVEL       (1)     /* highest external IRQ level    */

/* ------------------------------------------------------------------------
 * HC-SR04+ ranging
 * --------------------------------------------------------------------- */
#define B5_PING_PERIOD_MS       (60U)   /* NFR-USO-06: >= 60 ms apart    */
#define B5_TRIG_PULSE_US        (12U)   /* datasheet minimum is 10 us    */
#define B5_ECHO_WAIT_MS         (50)    /* longest echo (~38 ms) + slack */
#define B5_ECHO_MAX_US          (23300U) /* beyond this = no target (4 m) */
#define B5_ECHO_MIN_US          (200U)  /* shorter echo pulses (34 mm) are
                                         * glitches (the sensor's minimum is
                                         * about 2 cm), e.g. the trigger pulse
                                         * coupling onto ECHO in the cable;
                                         * ignored, the real echo follows */
#define B5_RANGE_FAR_MM         (4000)  /* value reported for no target  */
#define B5_SAMPLES_PER_ANGLE    (3U)    /* median-of-three (A6)          */
#define B5_ECHO_FAIL_LIMIT      (3U)    /* no-response pings before Fault (fits 2 bits) */

/* ------------------------------------------------------------------------
 * Guard and Confirm (statechart 6.5)
 * --------------------------------------------------------------------- */
#define B5_D_STOP_MM            (300)   /* TUNE: stop distance D_stop    */
#define B5_D_SLOW_MM            (500)   /* TUNE: ask Nav to slow (FR-USO-02) */
#define B5_SLOW_HYST_MM         (50)    /* hysteresis on the slow request */
#define B5_CONFIRM_COUNT        (3U)    /* consecutive readings < D_stop */

/* ------------------------------------------------------------------------
 * SG90 servo
 * --------------------------------------------------------------------- */
#define B5_SERVO_MIN_US         (500U)  /* pulse at full right (+900)     */
#define B5_SERVO_MAX_US         (2400U) /* pulse at full left  (-900); 0 =
                                         * 1450 us faces straight ahead
                                         * (S5-C01, checked by eye twice;
                                         * the scans vary +/-4 deg with the
                                         * box placement)                */
#define B5_SERVO_REVERSED       (0)     /* 1 if +900 points left         */
#define B5_SERVO_SPEED_DD_S     (6000U) /* TUNE: horn speed, 0.1 deg/s:
                                         * 600 deg/s, about the SG90's own
                                         * full speed (the original
                                         * behaviour). Lower it (e.g. 600 =
                                         * 60 deg/s) if the bracket hits
                                         * the wiring.                   */
#define B5_SERVO_STEP_MS        (20U)   /* one small step per PWM period */
#define B5_SERVO_SETTLE_MS      (40U)   /* TUNE (A8, S5-C02): rest after
                                         * a move, before the next ping  */

#if ((B5_SERVO_SPEED_DD_S * B5_SERVO_STEP_MS) < 1000U)
#error "B5_SERVO_SPEED_DD_S too low: the step would be under 0.1 deg"
#endif

/* ------------------------------------------------------------------------
 * Scanning. Angles are signed tenths of a degree, 11 bits, with the same
 * sign rule as Motion Control's heading: -900 = left, 0 = straight ahead,
 * +900 = right (Tribunal 1 decision; was 0 = right .. 1800 = left).
 * --------------------------------------------------------------------- */
#define B5_AHEAD_DD             (0)
#define B5_SCAN_START_DD        (-550)  /* coarse sweep 55 deg left ...  */
#define B5_SCAN_END_DD          (550)   /* ... to 55 deg right ...       */
#define B5_SCAN_STEP_DD         (275)   /* ... in 27.5 deg steps (5 pts) */
/* 55 deg is the mechanical limit on our car: at 60 deg the sensor bracket
 * hits the casing. Keep END - START a multiple of STEP so the sweep
 * includes both ends and 0 (straight ahead). */
#define B5_NEAR_MM              (B5_D_STOP_MM + 250) /* near/far split   */
#define B5_ALPHA_DD             (0)     /* beam-spread correction, measured
                                         * (S5-C04): a 230 mm box read its
                                         * true width uncorrected, a 100 mm
                                         * box ~3x too wide (a flat face
                                         * echoes from wherever it is in the
                                         * beam). 0 never makes an obstacle
                                         * look narrower than it is.     */
#define B5_REFINE_STOP_DD       (10)    /* stop bisecting below 1 deg, see README */
#define B5_REFINE_PINGS         (1U)    /* pings per refinement sample   */
#define B5_MAX_REFINE_SAMPLES   (12U)   /* bound on Edge Refinement      */

/* ------------------------------------------------------------------------
 * Planning and bypass (car geometry: TUNE on the real car)
 * --------------------------------------------------------------------- */
#define B5_CAR_WIDTH_MM         (155)   /* measured, widest point (S5-C05) */
#define B5_CAR_LENGTH_MM        (190)   /* measured, front to back (S5-C05) */
#define B5_MARGIN_MM            (50)    /* clearance margin each side    */
#define B5_OBST_DEPTH_MM        (200)   /* assumed obstacle depth        */
#define B5_OPEN_CLEARANCE_MM    (1000)  /* reported when a side is open  */
#define B5_MIN_CONFIDENCE_PCT   (50U)   /* below this, return to Guard   */
#define B5_PREFER_RIGHT         (1)     /* tie-break between equal sides */

/* ------------------------------------------------------------------------
 * Blocked and Recovering
 * --------------------------------------------------------------------- */
#define B5_BLOCKED_RETRIES      (2U)    /* reverse-and-rescan, FR-USO-07 */
#define B5_REVERSE_MM           (150)   /* how far to back up            */
#define B5_RECOVER_ATTEMPTS     (3U)    /* failed line searches -> Fault */
#define B5_SEARCH_WIDTH_MM      (100)   /* first search leg, widened per try */
#define B5_LINE_POLL_MS         (20U)   /* line sensor poll in Recovering */

/* ------------------------------------------------------------------------
 * Handshake timeouts with Navigation Coordinator
 * --------------------------------------------------------------------- */
#define B5_STOP_ACCEPT_TMO_MS   (1000)  /* no stop accept -> Fault       */
#define B5_MOTION_TMO_MS        (8000)  /* longest bypass/rejoin leg     */
#define B5_STATUS_PERIOD_MS     (500U)  /* telemetry rate while active   */

#endif /* B5_CONFIG_H */
