/**
 * @file bringup.h
 * @brief Bringup.
 * @details Core1 owns normal motor commits. Safety paths may only remove
 * output; command/status access uses the existing kernel locks and copied
 * records.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_BRINGUP_H
#define RTOS_BRINGUP_H

/* Keep the kernel's legacy size_t typedef out of ISO C application headers. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include <tk/smp.h>
#include <tm/tmonitor.h>

#include "bringup_policy.h"
#include "encoder_fault.h"
#include "src/motion/motion_port.h"

typedef struct
{
    uint32_t kernel_ms;
    uint32_t timer_us;
    uint32_t control_count;
    uint32_t dt_us;
    uint32_t min_dt_us;
    uint32_t max_dt_us;
    uint32_t max_cost_us;
    uint32_t late_count;
    uint32_t clock_kernel_ms;
    uint32_t clock_timer_us;
    uint32_t clock_samples;
    int32_t ticks[2];
    uint32_t edges[2];
    uint32_t invalid[2];
    uint32_t encoder_bad[2];
    uint32_t encoder_changes[2];
    int16_t speed_x10[2];
    int16_t target[2];
    int16_t pwm[2];
    unsigned flags : 8;
    unsigned clock_good : 2;
    unsigned reason : 4;
    unsigned encoder_kind : 2;
    unsigned encoder_wheel : 2;
} bringup_status_t;

extern ID g_bringup_commands;
extern volatile uint32_t g_bringup_irq_stop;

uint64_t board_time_us (void);
uint32_t board_kernel_ms (void);
void board_motor_init (void);
void board_motor_write (unsigned wheel, unsigned pwm);
void board_motor_off (void);
bool board_gpio_init (void);
void board_encoder_drain (int32_t delta[2], uint32_t edges[2],
                          uint32_t invalid[2]);
bool board_button_pressed (unsigned pin);
bool board_clock_registers_ok (void);
void board_clock_report (void);
uintptr_t board_motion_lock (void * p_user);
void board_motion_unlock (void * p_user, uintptr_t key);

void bringup_control_task (INT code, void * p_exinf);
void bringup_safety_task (INT code, void * p_exinf);
void bringup_nav_task (INT code, void * p_exinf);
void bringup_console_task (INT code, void * p_exinf);
void bringup_status_task (INT code, void * p_exinf);
void bringup_stop (uint8_t reason, uint16_t latch);
void bringup_stop_from (uint8_t reason, uint16_t latch,
                        motion_owner_t requester);
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_IMU)
/* Buddy4 safety event. Reasons are ri_safety_reason(): 1 lift, 2 overturn,
 * 3 sensor not ready/stale. Lift/overturn always cancel and latch; an IMU
 * availability failure stops only active or pending motion. */
void bringup_imu_safety_trip (unsigned reason);
#endif
bool bringup_can_start (void);
bool bringup_clear_latch (void);
void bringup_zero_status (void);
void bringup_get_status (bringup_status_t * p_out);
void bringup_encoder_fault_packet (uint8_t seq,
                                   uint8_t out[BR_ENCODER_FAULT_PACKET_BYTES]);
void bringup_report (void);
void bringup_help (void);
ER bringup_create_objects (void);

extern UW tm_usb_state (void);
extern INT tm_usb_console_get (void);
extern volatile UW tm_usb_dropped;
extern volatile UW tm_usb_rx_dropped;

#endif

/* End of file. */
