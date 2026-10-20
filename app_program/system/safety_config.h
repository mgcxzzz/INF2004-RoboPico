/**
 * @file    safety_config.h
 * @brief   Safety Supervisor build-time configuration.
 *
 * Group IS28 (team task). Coding standard: Barr-C:2018.
 */

#ifndef SAFETY_CONFIG_H
#define SAFETY_CONFIG_H

/* Highest application priority (Section 3). Lower number = higher;
 * 1-4 belong to the kernel (USB console 3, WiFi 4). */
#define SAFETY_TASK_PRIORITY    (6)
#define SAFETY_TASK_STACK_BYTES (2048)

/* How often the supervisor checks task health and stop timing. The kernel
 * tick is 10 ms, so this is the finest useful value. */
#define SAFETY_CHECK_MS         (20)

/* NFR-SYS-04: motor drive removed within this time of accepting a stop. */
#define SAFETY_STOP_LIMIT_MS    (100U)

/* Health monitoring of the Navigation Coordinator, which wakes at least
 * every 100 ms. Other tasks choose their own timeout in safety_watch(). */
#define SAFETY_NAV_TIMEOUT_MS   (500U)

#endif /* SAFETY_CONFIG_H */
