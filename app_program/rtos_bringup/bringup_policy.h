#ifndef RTOS_BRINGUP_POLICY_H
#define RTOS_BRINGUP_POLICY_H

#include <stdbool.h>
#include <stdint.h>

enum {
    BR_CONTROL_US = 10000U,
    BR_LATE_US = 50000U,
    BR_PULSE_US = 3000000U,
    BR_REST_US = 1000000U,
    BR_STALL_US = 750000U,
    BR_SPEED_MM_S = 45,
    BR_CLOCK_WINDOW_US = 1000000U,
    BR_CLOCK_GOOD_WINDOWS = 3
};
enum {
    BR_ACTIVE = 1U,
    BR_ESTOP = 2U,
    BR_CLOCK_OK = 4U,
    BR_READY = 8U,
    BR_STALL = 16U,
    BR_LATE = 32U,
    BR_ENCODER = 64U,
    BR_LATCHED = BR_ESTOP | BR_STALL | BR_LATE | BR_ENCODER
};
enum {
    BR_REASON_BOOT,
    BR_REASON_DONE,
    BR_REASON_SERIAL,
    BR_REASON_GP20,
    BR_REASON_GP21,
    BR_REASON_USB,
    BR_REASON_CLOCK,
    BR_REASON_LATE,
    BR_REASON_STALL,
    BR_REASON_ENCODER,
    BR_REASON_API
};

static inline bool br_clock_elapsed_ok(uint32_t timer_us, uint32_t kernel_ms) {
    if (kernel_ms < 900U || kernel_ms > 1200U)
        return false;
    uint32_t expected = kernel_ms * 1000U;
    uint32_t error = timer_us > expected ? timer_us - expected : expected - timer_us;
    return error <= 5000U + expected / 100U;
}

static inline bool br_can_start(uint16_t flags, uint32_t now, uint32_t stopped, unsigned usb,
                                bool buttons_released) {
    return (flags & (BR_READY | BR_CLOCK_OK | BR_ACTIVE | BR_LATCHED)) ==
               (BR_READY | BR_CLOCK_OK) &&
           now - stopped >= BR_REST_US && usb == 3U && buttons_released;
}

static inline bool br_command_fresh(uint32_t now, uint32_t received) {
    return now - received <= 500000U;
}

static inline bool br_outputs_allowed(uint16_t flags, bool irq_latched, unsigned usb,
                                      bool buttons_released) {
    return (flags & (BR_ACTIVE | BR_CLOCK_OK | BR_LATCHED)) == (BR_ACTIVE | BR_CLOCK_OK) &&
           !irq_latched && usb == 3U && buttons_released;
}

#endif
