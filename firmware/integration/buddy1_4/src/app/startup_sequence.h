#ifndef INTEGRATION_LOGIC_H
#define INTEGRATION_LOGIC_H
#include <stdint.h>
#include <stdbool.h>
enum { ARM_IDLE, ARM_HOLD_BOTH, ARM_RELEASE_BOTH, ARM_TAP, ARM_TAP_RELEASE, ARM_COUNTDOWN };
typedef struct {
    uint8_t state;
    uint32_t since;
} BenchArm;
/* Inputs must BOTH be debounced. Cancelling never consumes the only run. */
static inline bool bench_arm_step(BenchArm *a, uint32_t now, bool usb, bool busy, bool hold,
                                  bool stop) {
    if (usb || busy) {
        a->state = ARM_IDLE;
        return false;
    }
    switch (a->state) {
    case ARM_IDLE:
        if (hold && stop) {
            a->state = ARM_HOLD_BOTH;
            a->since = now;
        }
        break;
    case ARM_HOLD_BOTH:
        if (!hold || !stop)
            a->state = ARM_IDLE;
        else if (now - a->since >= 3000000u)
            a->state = ARM_RELEASE_BOTH;
        break;
    case ARM_RELEASE_BOTH:
        if (!hold && !stop) {
            a->state = ARM_TAP;
            a->since = now;
        }
        break;
    case ARM_TAP:
        if (stop)
            a->state = ARM_IDLE;
        else if (hold && now - a->since >= 200000u)
            a->state = ARM_TAP_RELEASE;
        break;
    case ARM_TAP_RELEASE:
        if (stop)
            a->state = ARM_IDLE;
        else if (!hold) {
            a->state = ARM_COUNTDOWN;
            a->since = now;
        }
        break;
    case ARM_COUNTDOWN:
        if (hold || stop)
            a->state = ARM_IDLE;
        else if (now - a->since >= 2000000u) {
            a->state = ARM_IDLE;
            return true;
        }
        break;
    default:
        a->state = ARM_IDLE;
        break;
    }
    return false;
}
/* Scale ALL S3 speeds by one half, preserving the intended steering ratio. */
static inline bool bench_targets(int16_t l, int16_t r, int16_t out[2]) {
    if (l < 0 || r < 0 || l > 200 || r > 200)
        return false;
    out[0] = l / 2;
    out[1] = r / 2;
    return true;
}
static inline bool bench_stale(uint32_t now, uint32_t last) {
    return now - last >= 50000u;
}
static inline uint32_t abs_ticks(int32_t n) {
    return n < 0 ? (uint32_t)(-(int64_t)n) : (uint32_t)n;
}
#endif
