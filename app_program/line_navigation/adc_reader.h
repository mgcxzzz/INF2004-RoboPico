#ifndef RTOS_LINE_ADC_READER_H
#define RTOS_LINE_ADC_READER_H
#include <stdbool.h>
#include <stdint.h>

/* RP2040 ADC CS definitions, matching the pinned Pico SDK register header. */
enum {
    RL_ADC_START = 4U,
    RL_ADC_READY = 1U << 8,
    RL_ADC_ERR = 1U << 9,
    RL_ADC_TIMEOUT_US = 100,
    RL_ADC_RETRY_US = 100000,
    RL_ADC_GOOD_FRAMES = 3
};
typedef enum {
    RL_ADC_NONE,
    RL_ADC_TIMEOUT,
    RL_ADC_CONVERSION_ERR,
    RL_ADC_RESET_TIMEOUT,
    RL_ADC_ARGUMENT,
    RL_ADC_INIT_TIMEOUT
} RlAdcReason;
typedef enum { RL_ADC_INIT, RL_ADC_DISCARD, RL_ADC_SAMPLE, RL_ADC_ARGUMENT_STAGE } RlAdcStage;
typedef struct {
    uint32_t cs, elapsed_us, timer_us, clk_ctrl, clk_div, adc_div, resets_done;
    unsigned reason : 3, channel : 3, stage : 2, sample : 6;
} RlAdcFailure;
typedef struct {
    uint32_t conversions, avoided_timeout, timeouts, errors, init_errors, bad_arguments;
    uint32_t read_ok[3], read_fail[3];
    RlAdcFailure first, last;
} RlAdcDiagnostics;
typedef struct {
    volatile uint32_t *cs, *result;
    uint64_t (*now)(void);
} RlAdcPort;

/* No interrupt masking and no longer timeout. A task can resume after a
 * conversion has completed: re-read READY before rejecting its deadline. */
static inline RlAdcReason rl_adc_conversion(const RlAdcPort *p, uint16_t *value, uint32_t *elapsed,
                                            uint32_t *observed_cs, uint32_t *avoided_timeout) {
    *p->cs |= RL_ADC_START;
    uint64_t start = p->now();
    uint32_t cs;
    for (;;) {
        cs = *p->cs;
        if (cs & RL_ADC_READY)
            break;
        uint64_t duration = p->now() - start;
        if (duration > RL_ADC_TIMEOUT_US) {
            cs = *p->cs; /* May have completed while this task was preempted. */
            *elapsed = duration > UINT32_MAX ? UINT32_MAX : (uint32_t)duration;
            if (!(cs & RL_ADC_READY)) {
                *observed_cs = cs;
                return RL_ADC_TIMEOUT;
            }
            ++*avoided_timeout;
            break;
        }
    }
    *observed_cs = cs;
    if (cs & RL_ADC_ERR) {
        uint64_t duration = p->now() - start;
        *elapsed = duration > UINT32_MAX ? UINT32_MAX : (uint32_t)duration;
        return RL_ADC_CONVERSION_ERR;
    }
    *value = (uint16_t)(*p->result & 4095U);
    return RL_ADC_NONE;
}

enum { RL_ADC_OFFLINE, RL_ADC_QUALIFYING, RL_ADC_HEALTHY };
typedef struct {
    uint64_t retry_us;
    unsigned state : 2, good : 2;
} RlAdcHealth;
static inline void rl_adc_failed(RlAdcHealth *h, uint64_t now) {
    *h = (RlAdcHealth){.state = RL_ADC_OFFLINE, .retry_us = now + RL_ADC_RETRY_US};
}
static inline bool rl_adc_can_retry(const RlAdcHealth *h, uint64_t now, bool active) {
    return !active && h->state == RL_ADC_OFFLINE && now >= h->retry_us;
}
static inline bool rl_adc_good_frame(RlAdcHealth *h) {
    if (h->state != RL_ADC_QUALIFYING)
        return false;
    if (h->good < RL_ADC_GOOD_FRAMES)
        ++h->good;
    if (h->good == RL_ADC_GOOD_FRAMES)
        h->state = RL_ADC_HEALTHY;
    return h->state == RL_ADC_HEALTHY;
}
#endif
