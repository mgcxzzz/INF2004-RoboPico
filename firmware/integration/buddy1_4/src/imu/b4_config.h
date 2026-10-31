/**
 * @file    b4_config.h
 * @brief   Buddy 4 (IMU Terrain): pins, timing and every tuning value.
 *
 * Values marked TUNE are starting points to set on the real car with the
 * bench tests in testing/subsystem4/HOW_TO_TEST.md (assumption A4).
 * Units: mm, mm/s, mm/s^2, mg (1/1000 g), 0.1 degree ("dd"), ms.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#ifndef B4_CONFIG_H
#define B4_CONFIG_H

/* One-sided drive-wheel ramp geometry is the production default. Set to 0
 * only for the legacy longitudinal hump model and its regression tests. */
#ifndef B4_SIDE_RAMP_MODE
#define B4_SIDE_RAMP_MODE (1)
#endif

/* ------------------------------------------------------------------------
 * Hardware: GY-511 (LSM303 accelerometer + magnetometer, NO gyroscope) on
 * I2C1, Robo Pico Grove 2 / Maker port (Week 6 pin table). IMU Terrain is
 * the only user of this bus (Week 6 Section 3).
 * --------------------------------------------------------------------- */
#define B4_I2C_SDA_PIN (2U) /* GP2 */
#define B4_I2C_SCL_PIN (3U) /* GP3 */
#define B4_I2C_TIMEOUT_US                                                                          \
    (2000U)                 /* one register transfer (a 6-byte
                                         * read takes about 0.2 ms at
                                         * 400 kHz)                      */
#define B4_ACC_ADDR (0x19U) /* LSM303 accelerometer          */
#define B4_MAG_ADDR (0x1EU) /* LSM303 magnetometer           */

/* ------------------------------------------------------------------------
 * Mounting (TUNE, record with S4-C01): which GY-511 axis points to the
 * car's FRONT and which points UP, and their signs. 0 = X, 1 = Y, 2 = Z.
 * The board's silkscreen shows X and Y; Z is out of the component side.
 * Calibration checks that the UP axis really carries gravity.
 * --------------------------------------------------------------------- */
#ifndef B4_AXIS_FWD /* (the bench tool may set them) */
#define B4_AXIS_FWD                                                                                \
    (1)                 /* +Y: prior car mounting check;
                                         * reconfirm with S4-C01 */
#define B4_SIGN_FWD (1) /* TUNE: +1 or -1 */
#define B4_AXIS_UP (2)  /* TUNE */
#define B4_SIGN_UP (1)  /* TUNE: +1 or -1 */
#endif

#if (B4_AXIS_FWD == B4_AXIS_UP)
#error "B4_AXIS_FWD and B4_AXIS_UP must be different axes"
#endif

/* ------------------------------------------------------------------------
 * RTOS (TEAM.md section 4: band 18-19, Week 6 priority "Medium")
 * --------------------------------------------------------------------- */
#define B4_TASK_PRIORITY (18)
#define B4_TASK_STACK_BYTES (2048)
#define B4_PERIOD_MS (20U)         /* NFR-IMU-01: 50 Hz            */
#define B4_WATCH_MS (200U)         /* Safety Supervisor health check */
#define B4_STATUS_PERIOD_MS (500U) /* telemetry at least this often */
#define B4_NAV_QUEUE                                                                               \
    (8U) /* operating-state changes queued
                                         * between two samples           */

/* ------------------------------------------------------------------------
 * Accelerometer set-up. The sensor samples at 400 Hz into its 32-entry
 * FIFO; every 20 ms the task empties it (about 8 samples). Their average
 * is the 50 Hz sample (no aliasing of short bumps), and each one is also
 * checked for a shock, so a spike shorter than 20 ms is not missed.
 * --------------------------------------------------------------------- */
#define B4_ODR_HZ (400U)
#define B4_FS_G (4)       /* full scale +-4 g             */
#define B4_MG_PER_LSB (2) /* +-4 g, 12-bit high-resolution */
#define B4_FIFO_DEPTH (32U)
#define B4_SAMPLE_US (1000000U / B4_ODR_HZ) /* 2500 us     */

/* ------------------------------------------------------------------------
 * Calibrating (statechart 6.4): stationary gravity baseline.
 * Tribunal 1 kept a 2.0 s stationary window.
 * --------------------------------------------------------------------- */
#define B4_CAL_WINDOW_MS (2000U)
#define B4_CAL_DEADLINE_MS (10000U) /* not stable by then = fault  */
#define B4_CAL_STABLE_MG                                                                           \
    (40)                       /* TUNE: max peak-to-peak of each
                                         * axis inside the window        */
#define B4_CAL_G_MIN_MG (850)  /* |g| outside this = sensor or */
#define B4_CAL_G_MAX_MG (1150) /* scale wrong                   */
#define B4_CAL_UP_MIN_PCT                                                                          \
    (90) /* UP axis carries >= 90 % of g,
                                         * i.e. board tilted < ~25 deg;
                                         * otherwise the mounting settings
                                         * above are wrong               */

/* ------------------------------------------------------------------------
 * Filtering. The 50 Hz sample is already the average of the ~8 FIFO
 * samples of the period (a 20 ms box filter: noise / 2.8). An extra
 * exponential low-pass, alpha = 1 / 2^B4_LP_SHIFT, can be added if the
 * motors' vibration needs it (TUNE with S4-C02), but every step adds lag,
 * and lag between pitch and distance shows up as hump-height error
 * (host test: shift 1 = up to +4 mm on a sharp 30 mm hump).
 * --------------------------------------------------------------------- */
#define B4_LP_SHIFT (0)

/* ------------------------------------------------------------------------
 * Motion classification (FR-IMU-03). Tribunal 1: stationary, accelerating
 * and turning come from Buddy 2's motion estimate (wheel distance and
 * heading), not re-derived from the accelerometer. Climbing, descending
 * and impact come from the IMU itself.
 *
 * Freshness: the same limit as every other reader of the shared motion
 * estimate (IF-19, NAV_LINE_STALE_MS = 50 ms). Checked in task_imu.c.
 * --------------------------------------------------------------------- */
#define B4_MOTION_FRESH_MS (50U)
#define B4_RATE_WINDOW_MS                                                                          \
    (200U)                   /* speed, acceleration and turn
                                         * rate measured over this       */
#define B4_STILL_MM_S (10)   /* slower = stationary          */
#define B4_ACCEL_MM_S2 (200) /* TUNE: |dv/dt| above = accel. */
#define B4_TURN_DD_S                                                                               \
    (200) /* TUNE: |turn rate| above (20
                                         * deg/s) = turning              */
#define B4_CLASS_HOLD_MS                                                                           \
    (100U) /* a new class must last this
                                         * long before it is published
                                         * (hysteresis; impact excepted) */
#define B4_QUIET_MG                                                                                \
    (30) /* IMU-only fallback: deviation
                                         * below this = not moving       */

/* Tilt (pitch, + = nose up). Climbing above ON, back to level below OFF. */
#define B4_TILT_ON_DD (40) /* TUNE: 4.0 deg                */
#define B4_TILT_OFF_DD (20)

/* ------------------------------------------------------------------------
 * Hump Processing (FR-IMU-02, NFR-IMU-02).
 *
 * Height comes from body pitch and odometry, not from double-integrating
 * acceleration (Tribunal 1: h = v^2/2g drifts). With the rear contact
 * following the front one at a wheelbase L behind it:
 *     h_front(s) = h_front(s - L) + L * sin(pitch(s))
 * so the front contact's height is rebuilt every B4_HUMP_BIN_MM of travel
 * from Buddy 2's distance. See imu_proc.c.
 * --------------------------------------------------------------------- */
#ifndef B4_WHEELBASE_MM
#define B4_WHEELBASE_MM                                                                            \
    (84) /* S4-C05: user measured ~85 mm;
                                         * nearest 4 mm profile bin.
                                         * Block check pending.          */
#endif
#define B4_TRACK_MM                                                                                \
    (113)                                       /* User measured drive-wheel
                                         * centre spacing, 28 Oct.
                                         * Static side bench only; the
                                         * production ramp model below
                                         * remains under review.        */
#define B4_SIDE_NORM_TOL_PCT (8U)               /* gravity magnitude tolerance */
#define B4_SIDE_SPREAD_MG (80U)                 /* previous raw peak-to-peak allowance */
#define B4_SIDE_MAD_MG (B4_SIDE_SPREAD_MG / 2U) /* robust axis deviation */
/* v0.7: which axes the MAD guard checks. Side height is the projection of
 * gravity on the car's right axis, set by lateral and up; forward enters
 * only through second-order tilt terms and the magnitude (norm) check.
 * v0.6 flat control: forward MAD 52..112 mg in every moving interval while
 * lateral <=36 and up <=28 mg, so the forward guard rejected 30 % of ticks
 * whose height inputs were clean. 1 restores the v0.5/v0.6 rule. */
#ifndef B4_SIDE_MAD_GUARD_FWD
#define B4_SIDE_MAD_GUARD_FWD (0)
#endif
#define B4_SIDE_START_MM (5)  /* right wheel lift threshold  */
#define B4_SIDE_RETURN_MM (3) /* return-to-ground threshold  */
#define B4_SIDE_START_MS (60U)
#define B4_SIDE_RETURN_MS (200U)
#define B4_SIDE_MIN_TRAVEL_MM (10)
#define B4_SIDE_TIMEOUT_MS (8000U)
/* Buddy4 add-on v0.2: a tick rejected only for FIFO dispersion (MAD) or
 * longitudinal acceleration is skipped, not used as a height measurement.
 * v0.8: a tick rejected only for FIFO dispersion (MAD) is skipped, not a
 * verdict. Measured 29 Oct (v0.7 ramp run1 and practice pass): the right
 * wheel meeting the ramp edge gave single ticks with lateral MAD 44-48 mg,
 * which cleared level-ready exactly as the climb began, so the hump was
 * ignored. Gravity-norm, timing, odometry, turn/reverse and impact failures stay
 * fatal. A skip run longer than SKIP_RUN_MS disarms (before a candidate) or
 * rejects (during one); a candidate with more than SKIP_MAX_PCT of its time
 * skipped is rejected. SKIP_RUN_MS 0 restores the v0.7 behaviour. */
#ifndef B4_SIDE_SKIP_RUN_MS
#define B4_SIDE_SKIP_RUN_MS (100U)
#endif
#define B4_SIDE_SKIP_MAX_PCT (25U)
#define B4_HUMP_BIN_MM (4) /* profile resolution           */
#define B4_HUMP_BINS (64U) /* 256 mm of history (2^n)      */
#define B4_HUMP_START_MS                                                                           \
    (60U) /* nose up > B4_TILT_ON_DD this
                                         * long = hump start [sustained] */
#define B4_HUMP_END_MM                                                                             \
    (B4_WHEELBASE_MM + 30) /* level this far
                                         * after the descent = hump end */
#define B4_HUMP_END_MS                                                                             \
    (600U)                         /* same, by time, when there is
                                         * no odometry                   */
#define B4_HUMP_TIMEOUT_MS (8000U) /* profile timeout              */
#define B4_HUMP_MAX_LEN_MM                                                                         \
    (1000)                   /* profile longer than this = a
                                         * slope, not a hump (hump up to
                                         * ~700 mm + wheelbase + end)    */
#define B4_HUMP_MIN_MM (5)   /* smaller = noise              */
#define B4_HUMP_MAX_MM (120) /* larger = implausible for car */
#define B4_HUMP_CLOSE_MM (5) /* the height must come back to */
#define B4_HUMP_CLOSE_PCT                                                                          \
    (30) /* within max(5 mm, 30 % of the
                                         * peak) once the car is past it */

#if ((B4_WHEELBASE_MM % B4_HUMP_BIN_MM) != 0)
#error "B4_WHEELBASE_MM must be a multiple of B4_HUMP_BIN_MM"
#endif
#if ((B4_WHEELBASE_MM / B4_HUMP_BIN_MM) >= B4_HUMP_BINS)
#error "B4_HUMP_BINS too small for this wheelbase"
#endif

/* NFR-IMU-02 as clarified after Tribunal 1: the error may be up to
 * max(5 mm, 10 % of the measured reference). The two bounds meet at
 * 50 mm: below it the absolute 5 mm governs, above it the 10 %. */
#define B4_NFR02_ABS_MM (5)
#define B4_NFR02_PCT (10)

/* Bench only: with no motion estimate (no Buddy 2 yet), assume the car
 * moves at this speed while a run is active, so a hump pushed by hand at
 * a steady pace can be profiled. 0 = off (heights are then rejected as
 * "no odometry"). Never leave non-zero on the real car. */
#define B4_BENCH_ASSUME_MM_S (0)

/* ------------------------------------------------------------------------
 * Impact / Shock (NFR-IMU-03: notify within 50 ms)
 * --------------------------------------------------------------------- */
#define B4_SHOCK_MG                                                                                \
    (1200)                      /* TUNE (S4-C04): one FIFO sample
                                         * this far from the gravity
                                         * baseline = shock              */
#define B4_SHOCK_CLEAR_MG (250) /* back below this ...          */
#define B4_SHOCK_HOLD_MS (200U) /* ... for this long = cleared  */
#define B4_SHOCK_LIMIT_MS (50U) /* NFR-IMU-03                   */

/* ------------------------------------------------------------------------
 * Data health -> Fault / Recovery
 * --------------------------------------------------------------------- */
#define B4_I2C_ERR_MAX (3U) /* failed reads in a row (60 ms) */
#define B4_STALE_MAX                                                                               \
    (3U)                       /* periods in a row with no new
                                         * sample in the FIFO            */
#define B4_STUCK_PERIODS (50U) /* identical raw data for 1 s   */
#define B4_LATE_US                                                                                 \
    (30000U) /* period longer than this =
                                         * frame(s) missed (NFR-IMU-01) */

/* Team decision: is an IMU fault critical (Safety stops the car) or a
 * warning (the run goes on without hump data)? The IMU never moves the
 * car, so the default is warning. Shocks are always warnings: Safety
 * decides what to do with them. */
#define B4_FAULT_CRITICAL (0)

#endif /* B4_CONFIG_H */
