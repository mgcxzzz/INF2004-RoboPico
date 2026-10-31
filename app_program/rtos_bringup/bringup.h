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
#include "src/motion/motion_port.h"

typedef struct {
    uint32_t kernel_ms, timer_us, control_count;
    uint32_t dt_us, min_dt_us, max_dt_us, max_cost_us, late_count;
    uint32_t clock_kernel_ms, clock_timer_us, clock_samples;
    int32_t ticks[2];
    uint32_t edges[2], invalid[2];
    int16_t speed_x10[2], target[2], pwm[2];
    unsigned flags : 7, clock_good : 2, reason : 4;
} BringupStatus;

extern ID bringup_commands;
extern volatile uint32_t bringup_irq_stop;

uint64_t board_time_us(void);
uint32_t board_kernel_ms(void);
void board_motor_init(void);
void board_motor_write(unsigned wheel, unsigned pwm);
void board_motor_off(void);
bool board_gpio_init(void);
void board_encoder_drain(int32_t delta[2], uint32_t edges[2], uint32_t invalid[2]);
bool board_button_pressed(unsigned pin);
bool board_clock_registers_ok(void);
void board_clock_report(void);
uintptr_t board_motion_lock(void *user);
void board_motion_unlock(void *user, uintptr_t key);

void bringup_control_task(INT code, void *exinf);
void bringup_safety_task(INT code, void *exinf);
void bringup_nav_task(INT code, void *exinf);
void bringup_console_task(INT code, void *exinf);
void bringup_status_task(INT code, void *exinf);
void bringup_stop(uint8_t reason, uint16_t latch);
void bringup_get_status(BringupStatus *out);
void bringup_report(void);
void bringup_help(void);
ER bringup_create_objects(void);

extern UW tm_usb_state(void);
extern INT tm_usb_console_get(void);
extern volatile UW tm_usb_dropped, tm_usb_rx_dropped;

#endif
