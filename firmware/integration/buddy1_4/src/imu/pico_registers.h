/* Buddy4-on-v2.3: the only definitions the validated RTOS driver
 * (drv_lsm303.c) took from micro T-Kernel / sys_hal, re-expressed for the
 * Pico SDK build. Values are the RP2040 register addresses and bits from
 * include/sys/sysdepend/cpu/rp2040/sysdef.h and tk/.../syslib.h of the RTOS
 * tree (identical to the SDK address map). No behaviour of its own. */
#ifndef V23B4_RP2040_SHIM_H
#define V23B4_RP2040_SHIM_H
#include <stdbool.h>
#include <stdint.h>
#include "hardware/timer.h" /* time_us_32() */

static inline void out_w(uint32_t a, uint32_t v) {
    *(volatile uint32_t *)(uintptr_t)a = v;
}
static inline uint32_t in_w(uint32_t a) {
    return *(volatile uint32_t *)(uintptr_t)a;
}
static inline void clr_w(uint32_t a, uint32_t v) {
    *(volatile uint32_t *)(uintptr_t)(a + 0x3000U) = v;
}
static inline void set_w(uint32_t a, uint32_t v) {
    *(volatile uint32_t *)(uintptr_t)(a + 0x2000U) = v;
}
static inline uint32_t sys_time_us(void) {
    return time_us_32();
}

#define V23B4_RESETS_BASE (0x4000C000U)
#define V23B4_IO_BANK0_BASE (0x40014000U)
#define V23B4_PADS_BANK0_BASE (0x4001C000U)
#define RESETS_RESET (V23B4_RESETS_BASE + 0x0U)
#define RESETS_RESET_DONE (V23B4_RESETS_BASE + 0x8U)
#define RESETS_RESET_I2C1 (0x00000010U)
#define GPIO_CTRL(n) (V23B4_IO_BANK0_BASE + 0x04U + ((n) * 8U))
#define GPIO_CTRL_FUNCSEL_I2C (3U)
#define GPIO(n) (V23B4_PADS_BANK0_BASE + 0x4U + ((n) * 4U))
#define GPIO_IE (1U << 6)
#define GPIO_DRIVE_4MA (1U << 4)
#define GPIO_PUE (1U << 3)
#define GPIO_SHEMITT (1U << 1)
#endif
