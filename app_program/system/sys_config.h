/**
 * @file    sys_config.h
 * @brief   Switches for the shared system code (app_program/system/).
 *
 * The three switches below fill in for subsystems that are not written
 * yet, so the car can be tried on the bench today. Each one defines
 * functions that a subsystem will later provide for real; when that
 * subsystem's code arrives the build fails with "multiple definition of
 * ...", and the fix is to set the matching switch here to 0.
 *
 * Group IS28 (shared). Coding standard: Barr-C:2018.
 */

#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H

/* 1 = simulate Motion Control (S2) and Line and Barcode (S3): a virtual
 *     car follows a line, so the real Buddy 5 sensor and servo, the
 *     Navigation Coordinator and the Safety Supervisor can run together.
 *     Set to 0 once S2 and S3 provide mc_*(), line_set_control(),
 *     b5_motion_get_estimate() and b5_line_detected(). */
#define SYS_SIM_CAR             (1)

/* 1 = print the Navigation Coordinator, Safety Supervisor and Buddy 5
 *     status on the console. Set to 0 once WiFi and MQTT (S1) provides
 *     the *_telemetry_publish() functions. */
#define SYS_CONSOLE_STATUS      (1)

/* 1 = the Robo Pico's two buttons act as the supervisor PC:
 *     GP20 = emergency stop, GP21 = reset and start a run.
 *     Pins are claimed in TEAM.md section 2. */
#define SYS_BENCH_BUTTONS       (1)

/* Simulation only: start a run by itself this long after power-up
 * (0 = wait for the GP21 button). */
#define SYS_SIM_AUTOSTART_MS    (2000)

/* Bench task priority: below the coordinator, above every subsystem. */
#define SYS_BENCH_PRIORITY      (11)
#define SYS_BENCH_STACK_BYTES   (2048)

#endif /* SYS_CONFIG_H */
