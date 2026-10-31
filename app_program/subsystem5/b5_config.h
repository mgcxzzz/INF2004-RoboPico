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
#define B5_D_STOP_MM            (150)   /* TUNE: stop distance D_stop; 2026-10-31 bench: 150 (was 300) */
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
#define B5_SCAN_START_DD        (-460)  /* coarse sweep 46 deg left ...  */
#define B5_SCAN_END_DD          (460)   /* ... to 46 deg right ...       */
#define B5_SCAN_STEP_DD         (46)    /* ... in 4.6 deg steps (21 pts) */
/* Mechanical limits on our car (2026-11-01 end-stop check, servo centre
 * 1450 us): left about -60 deg; right clear to +47.7 deg, contact at about
 * +48.2 deg (chip U1 on the back of the sensor board meets the battery
 * mount). v0.7.4: symmetric +/-46 deg sweep, ~2 deg clear on the right
 * (was -55..+44 in v0.7.1-v0.7.3, +/-55 before 2026-10-31). A spacer behind
 * the sensor board or moving the mount would allow +/-55 again.
 * Keep END - START a multiple of STEP so the sweep includes both ends and
 * 0 (straight ahead). */
#define B5_NEAR_MM              (550)   /* near/far split; kept at the 10-28/bench value when D_stop moved to 150 */
#define B5_ALPHA_DD             (0)     /* beam-spread correction, measured
                                         * (S5-C04): a 230 mm box read its
                                         * true width uncorrected, a 100 mm
                                         * box ~3x too wide (a flat face
                                         * echoes from wherever it is in the
                                         * beam). 0 never makes an obstacle
                                         * look narrower than it is.     */
#define B5_REFINE_STOP_DD       (10)    /* stop bisecting below 1 deg, see README */
#define B5_REFINE_PINGS         (3U)    /* pings per refinement sample   */
#define B5_SAMPLE_SPREAD_MAX_MM (20) /* reject unstable angle triplets */
#define B5_MAX_REFINE_SAMPLES   (12U)   /* bound on Edge Refinement      */

/* ------------------------------------------------------------------------
 * Planning and bypass (car geometry: TUNE on the real car)
 * --------------------------------------------------------------------- */
#define B5_CAR_WIDTH_MM         (135)   /* measured 2026-11-01: widest point incl.
                                         * the barcode IR (left) to the far wheel;
                                         * S5-C05 had 155 */
#define B5_CAR_LENGTH_MM        (190)   /* front IR bar to rear: 130 + <60 (2026-11-01); S5-C05 190 */
#define B5_MARGIN_MM            (50)    /* clearance margin each side    */
#define B5_OBST_DEPTH_MM        (200)   /* assumed obstacle depth        */
#define B5_OPEN_CLEARANCE_MM    (1000)  /* reported when a side is open  */
#define B5_MIN_CONFIDENCE_PCT   (50U)   /* below this, return to Guard   */
#define B5_PREFER_RIGHT         (1)     /* tie-break between equal sides */

/* Navigation-profile uncertainty assumptions, pending physical validation. */
#define B5_PROFILE_NEAR_HYST_MM      (20)
#define B5_PROFILE_SERVO_ALLOW_DD    (40)    /* ~4 deg measured repeatability */
#define B5_PROFILE_RANGE_ALLOW_MM   (10)    /* conservative range allowance */
#define B5_PROFILE_MAX_GAP_DD        (110)   /* max adjacent evidence gap */
#define B5_EXPECTED_MAX_WIDTH_MM    (200)    /* planning floor, never a cap */
/* Rule B (2026-10-31 review, supervised open-course testing). Each ping
 * ends in one of three classes (obstacle_profile.h b5_classify_ping):
 *   ECHO       rise and fall seen, B5_ECHO_MIN_US <= width <= B5_ECHO_MAX_US
 *   NO_RETURN  ECHO was low before TRIG, rose, and FELL after a width
 *              > B5_ECHO_MAX_US: the sensor completed a measurement and
 *              heard nothing in range
 *   FAULT      anything else: ECHO already high before TRIG and never low
 *              within B5_PRETRIG_LOW_WAIT_MS, no rise, rise without a fall
 *              within B5_NORETURN_WAIT_MS, or only short noise pulses.
 *              A fault is never a distance and never clearance.
 * Per angle (b5_combine_pings): any echo -> occupied at the NEAREST echo
 * (stable only if every ping echoed within B5_SAMPLE_SPREAD_MAX_MM);
 * an echo plus ONE fault -> blocks at that range, nearer space weak only;
 * an echo plus two or more faults -> blocks, the angle otherwise unknown;
 * B5_WEAK_CLEAR_PINGS clean no-returns -> tentative WEAK clear (flagged,
 * excluded from measured confidence); anything else -> unknown. */
#define B5_NORETURN_WAIT_MS         (100)    /* wait for the FALL of a no-target pulse */
#define B5_PRETRIG_LOW_WAIT_MS      (100)    /* ECHO must read low before TRIG */
#define B5_NORETURN_WEAK_CLEAR      (1)      /* 0: no-return triplets stay unknown */
#define B5_WEAK_CLEAR_PINGS         (3U)
/* Swept-footprint check of the actual bypass manoeuvre (Navigation
 * Coordinator on_b5_plan / on_b5_rejoin): spin dir*45 deg on the spot,
 * drive lateral*sqrt2, spin back, drive forward-lateral, then spin -dir*45
 * and drive lateral*sqrt2 + search (capped). The car rectangle grown by
 * B5_MARGIN_MM is checked at every pose against the planning envelope,
 * measured returns and the scan's free space. Regions the scan cannot see
 * (behind the sensor, outside the arc, behind the obstacle) are ASSUMED
 * free and flagged; points in or behind a measured side return FAIL. */
#define B5_PIVOT_TO_FRONT_MM        (130)    /* measured 2026-11-01: drive-axle mid to the front IR bar tip */
#define B5_SENSOR_BEHIND_FRONT_MM   (65)     /* measured 2026-11-01: sensor face to the front IR bar tip
                                                  * (the IR bar sticks out 20 mm past the bumper) */
#define B5_SWEEP_TURN_DD            (450)    /* must equal NAV_BYPASS_ANGLE_DD */
#define B5_SWEEP_REJOIN_MAX_MM      (600)    /* must equal NAV_REJOIN_MAX_MM */
#define B5_SWEEP_STEP_MM            (20)     /* pose step along drive legs */
#define B5_SWEEP_STEP_DD            (50)     /* pose step while spinning */
#define B5_SWEEP_EDGE_STEP_MM       (20)     /* outline point spacing */
#define B5_SWEEP_ALLOW_ASSUMED      (1)      /* 0: any unscanned footprint point fails the side */
#define B5_SWEEP_ALLOW_WEAK         (1)      /* 0: footprint relying on no-return rays fails */
/* 1: measure the side clearances for the left/right comparison only over
 * |angle| <= min(-B5_SCAN_START_DD, B5_SCAN_END_DD), so the asymmetric
 * sweep (-55..+44) does not favour the left. Safety checks use all samples.
 * (2026-11-01: as recorded the comparison picked LEFT in 17 of 18 scans;
 * re-scored with this window, 12/12 correct for a 180-185 mm box shifted
 * 60 mm at 150 mm.) */
#define B5_SIDE_COMPARE_SYMMETRIC   (1)
/* Both sides passable and clearances within this of each other: take the
 * preferred side (B5_PREFER_RIGHT) instead of a noise-level difference. */
#define B5_SIDE_TIE_MM              (50)
/* Sideways move never less than: largest expected obstacle (it may stick out
 * entirely to the pass side; a flat face hides its true edges) + half the
 * car + margin. */
#define B5_LATERAL_MIN_MM           (B5_EXPECTED_MAX_WIDTH_MM + (B5_CAR_WIDTH_MM / 2) + B5_MARGIN_MM)
#ifndef B5_SCAN_DUMP
#define B5_SCAN_DUMP                (1)      /* 1: print every scan + decision (test evidence) */
#endif

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
