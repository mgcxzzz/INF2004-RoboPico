/**
 * @file    s3_config.h
 * @brief   Buddy 3 (Line and Barcode): pins, timing and every tuning value.
 *
 * Values marked TUNE are starting points to set on the real car (A3).
 * Units: mm, mm/s, ms.
 *
 * Course facts used here (Barcode_Sample.pdf and the course photo):
 *  - the line is 18 mm wide, black on a light surface;
 *  - barcodes are Code 39, "*X*" (start, one character, stop), about
 *    141 mm long, printed 20 mm beside the line;
 *  - narrow element 3.0 mm, wide element 9.0 mm (3 : 1). The half-size
 *    print in the same file has 1.5 mm narrow elements; both decode.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#ifndef S3_CONFIG_H
#define S3_CONFIG_H

/* ------------------------------------------------------------------------
 * Pins (TEAM.md section 2)
 * --------------------------------------------------------------------- */
#define S3_PIN_LINE_LEFT        (6U)    /* Grove 5; GP26 on that cable unused */
#define S3_PIN_LINE_RIGHT       (7U)    /* Grove 7; GP28 on that cable unused */
#define S3_PIN_BARCODE          (19U)   /* header, physical pin 25            */

/* TUNE (A3): D0 level when the sensor is over BLACK. Most TCRT5000-style
 * boards read HIGH over black (no reflection). Check it with the bench
 * printout (S3-M01) before anything else. */
#define S3_IR_BLACK_LEVEL       (1U)

/* Same level as Buddy 5's echo: the RP2040 has one GPIO interrupt and the
 * first attach sets its level (system/sys_hal.c). */
#define S3_BARCODE_IRQ_LEVEL    (1)

/* ------------------------------------------------------------------------
 * RTOS
 * --------------------------------------------------------------------- */
#define S3_TASK_PRIORITY        (16)    /* band 16-17 (TEAM.md section 4) */
#define S3_TASK_STACK_BYTES     (2048)
#define S3_PERIOD_MS            (10)    /* line cycle. The kernel tick is
                                         * 10 ms, so this is the fastest a
                                         * task delay allows; bar timing
                                         * does NOT depend on it (edge
                                         * interrupts on GP19).          */
#define S3_WATCH_MS             (100U)  /* Safety Supervisor health check */
#define S3_STATUS_PERIOD_MS     (200U)  /* telemetry at least this often  */

/* ------------------------------------------------------------------------
 * Line following (two digital front sensors)
 *
 * The sensors STRADDLE the 18 mm line (spacing wider than the line), so:
 *   neither black  centred (or lost, if it lasts)
 *   left black     the line is under the left sensor: steer left
 *   right black    steer right
 *   both black     a crossing line under both sensors: junction
 * A curve only ever puts ONE sensor on the line, because the line is
 * narrower than the sensor gap; a junction puts BOTH on it at once for
 * at least S3_JUNCTION_MM of travel. That is how the two are told apart
 * (Week 6 feedback: "state how a junction differs from a curve").
 * --------------------------------------------------------------------- */
#define S3_BASE_MM_S            (150)   /* TUNE: nominal speed (NFR-LINE-02) */
#define S3_TURN_OUTER_MM_S      (190)   /* TUNE: hard correction            */
#define S3_TURN_INNER_MM_S      (60)
#define S3_SOFT_OUTER_MM_S      (165)   /* TUNE: line between the sensors   */
#define S3_SOFT_INNER_MM_S      (125)   /*  again; lean back gently         */
#define S3_LOST_MM              (250)   /* TUNE: no contact this far = lost */
#define S3_JUNCTION_MM          (12)    /* both black this far = junction   */
#define S3_STUCK_MM             (400)   /* both black this far = sensor
                                         * fault (longer than any junction) */
#define S3_NOMINAL_STEP_MM      (2)     /* distance per cycle assumed when
                                         * Motion Control gives none        */

/* ------------------------------------------------------------------------
 * Barcode (Code 39 on the side-offset sensor, timed by edge interrupts)
 * --------------------------------------------------------------------- */
#define S3_BC_MAX_CHARS         (3U)    /* "*X*"                          */
#define S3_BC_MAX_ELEMENTS      (10U * S3_BC_MAX_CHARS - 1U)    /* 29     */
#define S3_BC_EDGE_RING         (64U)   /* edges buffered by the ISR (2^n) */
#define S3_BC_QUIET_MM          (25)    /* white this far after a bar ends
                                         * the barcode (widest space 9 mm) */
#define S3_BC_QUIET_MS          (400U)  /* same, by time, only when Motion
                                         * Control gives no distance       */
#define S3_BC_TIMEOUT_MS        (5000U) /* a read longer than this fails  */
#define S3_BC_REARM_MM          (40)    /* sensor white this far before the
                                         * next barcode can start (FR-LINE-05) */
#define S3_BC_MIN_RATIO_X10     (15)    /* narrowest wide >= 1.5 x widest
                                         * narrow (print is 3 : 1)         */
#define S3_BC_TURN_DELAY_MM     (0)     /* TUNE: travel after the barcode
                                         * before the event is posted, so
                                         * the coordinator turns at the
                                         * junction, not at the barcode    */
#define S3_AWAIT_MS             (300U)  /* coordinator kept line control
                                         * this long after an event: it
                                         * ignored it, so resume following */

/* Command table: Code 39 character -> navigation event (FR-LINE-04).
 * From the Project Write-up, "Barcode Navigation": A Turn Left, B Turn
 * Right, C Go Straight, D U-Turn (the write-up calls it an example: check
 * the barcodes on the final course). Any other character, such as the
 * "*Z*" in Barcode_Sample.pdf, is decoded and logged but sends NO command
 * (NFR-LINE-02). */
#define S3_BC_COMMANDS                                              \
    { 'A', NAV_BARCODE_LEFT },                                      \
    { 'B', NAV_BARCODE_RIGHT },                                     \
    { 'C', NAV_BARCODE_STRAIGHT },                                  \
    { 'D', NAV_BARCODE_UTURN }

/* Fault codes for safety_report_fault(SAFETY_SRC_LINE, code, ...): 8 bits. */
#define S3_FAULT_STUCK          (0x01U) /* front sensors black too long  */
#define S3_FAULT_INIT           (0x02U) /* pins or interrupt not set up  */
#define S3_WARN_EDGE_OVERFLOW   (0x03U) /* barcode edges lost (warning)  */

#endif /* S3_CONFIG_H */
