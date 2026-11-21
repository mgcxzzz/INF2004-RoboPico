/**
 * @file obstacle_service.h
 * @brief Buddy5 RTOS adapter (APP=rtos_buddy1_5 only).
 * @details Ownership:
 *  - core1 IO_IRQ_BANK0 handler (board_port.c): GP17 edge timestamps only;
 *  - core1 Buddy5 task, priority 20: servo slew, the TRIG/ECHO transaction,
 *    copied telemetry and the stopped USB report;
 *  - core1 motion task, priority 14: per-frame line observation and the
 *    mission decision step right after motion_update(), and Buddy5's signed
 *    wheel demand inside the existing single generation-checked PWM commit.
 * Buddy5 moves only through the shared coordinator (nav_api.h) with its lease.
 * @see app_program/obstacle_navigation/README.md
 */
#ifndef RTOS_OBSTACLE_SERVICE_H
#define RTOS_OBSTACLE_SERVICE_H

#include "../rtos_bringup/bringup.h"
#include "obstacle_policy.h"
#include "b5_straight.h"

/* Board port (board_obstacle.c). */
uint32_t board_obstacle_init (void);
bool board_obstacle_echo_high (void);
void board_obstacle_echo_ack (void);
void board_obstacle_servo_us (unsigned pulse_us);
uint32_t board_obstacle_trigger (void);

/* Kernel/IRQ boundary. */
void rb5_init (void);          /* usermain, before tasks */
uint32_t rb5_gpio_init (void); /* board_gpio_init, core1 */
void rb5_echo_isr (uint32_t edges, uint64_t now_us); /* core1 GPIO IRQ only */
void rb5_task (INT code, void * p_exinf); /* core1, priority 20     */
void rb5_service (uint64_t now_us);       /* one rb5_task pass (host tests) */

/* Motion task (core1) hooks, in control.c order. */
bool rb5_external (void);
/* p_level: freshly acquired front levels in an active run, or NULL when
 * stopped. p_line: this frame's Buddy3 straight-gate sample, or NULL when
 * Buddy3's line provider did not run (paused, Buddy5 owner, inactive). */
void rb5_frame (uint64_t now_us, const uint16_t * p_level,
                const b5_line_sample_t * p_line);
bool rb5_owned (const motion_output_t * p_demand);
bool rb5_take_handback (const motion_output_t * p_demand);
void rb5_power (const motion_output_t * p_demand, const float speed[2],
                float delta_time, uint64_t now, int32_t out[2]);
void rb5_power_committed (const int32_t applied[2]);
void rb5_power_idle (void);
bool rb5_begin (uint64_t now_us);
bool rb5_held (void); /* Motion task: stopped under Buddy5 mission hold. */
void rb5_guard_limit (int16_t target[2]);
bool rb5_owns_session (void); /* Motion task: Pause routing. */

/* Any task. */
bool rb5_start_allowed (void);
void rb5_cancel (void);
void rb5_report_request (void);

#endif

/* End of file. */
