/**
 * @file adc_reader.h
 * @brief Adc reader.
 * @details ADC/control ownership covers sensor acquisition, planning and calibration.
 * Other tasks consume protected copies; flash and stored exports require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_LINE_ADC_READER_H
#define RTOS_LINE_ADC_READER_H
#include <stdbool.h>
#include <stdint.h>

/* RP2040 ADC CS definitions, matching the pinned Pico SDK register header. */
enum
{
    RL_ADC_START = 4U,
    RL_ADC_READY = 1U << 8,
    RL_ADC_ERR = 1U << 9,
    RL_ADC_TIMEOUT_US = 100,
    RL_ADC_RETRY_US = 100000,
    RL_ADC_GOOD_FRAMES = 3
};
typedef enum
{
    RL_ADC_NONE,
    RL_ADC_TIMEOUT,
    RL_ADC_CONVERSION_ERR,
    RL_ADC_RESET_TIMEOUT,
    RL_ADC_ARGUMENT,
    RL_ADC_INIT_TIMEOUT
} rl_adc_reason_t;
typedef enum
{
    RL_ADC_INIT,
    RL_ADC_DISCARD,
    RL_ADC_SAMPLE,
    RL_ADC_ARGUMENT_STAGE
} rl_adc_stage_t;
typedef struct
{
    uint32_t cs;
    uint32_t elapsed_us;
    uint32_t timer_us;
    uint32_t clk_ctrl;
    uint32_t clk_div;
    uint32_t adc_div;
    uint32_t resets_done;
    unsigned reason : 3;
    unsigned channel : 3;
    unsigned stage : 2;
    unsigned sample : 6;
} rl_adc_failure_t;
typedef struct
{
    uint32_t conversions;
    uint32_t avoided_timeout;
    uint32_t timeouts;
    uint32_t errors;
    uint32_t init_errors;
    uint32_t bad_arguments;
    uint32_t read_ok[3];
    uint32_t read_fail[3];
    rl_adc_failure_t first;
    rl_adc_failure_t last;
} rl_adc_diagnostics_t;
typedef struct
{
    volatile uint32_t * cs;
    volatile uint32_t * result;
    uint64_t (*now)(void);
} rl_adc_port_t;

/* No interrupt masking and no longer timeout. A task can resume after a
 * conversion has completed: re-read READY before rejecting its deadline. */
static inline rl_adc_reason_t rl_adc_conversion (const rl_adc_port_t * p_data,
                                                 uint16_t * p_value,
                                                 uint32_t * p_elapsed,
                                                 uint32_t * p_observed_cs,
                                                 uint32_t * p_avoided_timeout)
{
    *p_data->cs |= RL_ADC_START;
    uint64_t start = p_data->now();
    uint32_t cs_value;
    for (;;)
    {
        cs_value = *p_data->cs;
        if (cs_value & RL_ADC_READY)
        {
            break;
        }
        uint64_t duration = p_data->now() - start;
        if (duration > RL_ADC_TIMEOUT_US)
        {
            cs_value =
                *p_data
                     ->cs; /* May have completed while this task was preempted. */
            *p_elapsed =
                duration > UINT32_MAX ? UINT32_MAX : (uint32_t)duration;
            if (!(cs_value & RL_ADC_READY))
            {
                *p_observed_cs = cs_value;
                return RL_ADC_TIMEOUT;
            }
            ++*p_avoided_timeout;
            break;
        }
    }
    *p_observed_cs = cs_value;
    if (cs_value & RL_ADC_ERR)
    {
        uint64_t duration = p_data->now() - start;
        *p_elapsed = duration > UINT32_MAX ? UINT32_MAX : (uint32_t)duration;
        return RL_ADC_CONVERSION_ERR;
    }
    *p_value = (uint16_t)(*p_data->result & 4095U);
    return RL_ADC_NONE;
}

enum
{
    RL_ADC_OFFLINE,
    RL_ADC_QUALIFYING,
    RL_ADC_HEALTHY
};
typedef struct
{
    uint64_t retry_us;
    unsigned state : 2;
    unsigned good : 2;
} rl_adc_health_t;
static inline void rl_adc_failed (rl_adc_health_t * p_h_value, uint64_t now)
{
    *p_h_value = (rl_adc_health_t){.state = RL_ADC_OFFLINE,
                                   .retry_us = now + RL_ADC_RETRY_US};
}
static inline bool rl_adc_can_retry (const rl_adc_health_t * p_h_value,
                                     uint64_t now, bool active)
{
    return !active && p_h_value->state == RL_ADC_OFFLINE &&
           now >= p_h_value->retry_us;
}
static inline bool rl_adc_good_frame (rl_adc_health_t * p_h_value)
{
    if (p_h_value->state != RL_ADC_QUALIFYING)
    {
        return false;
    }
    if (p_h_value->good < RL_ADC_GOOD_FRAMES)
    {
        ++p_h_value->good;
    }
    if (p_h_value->good == RL_ADC_GOOD_FRAMES)
    {
        p_h_value->state = RL_ADC_HEALTHY;
    }
    return p_h_value->state == RL_ADC_HEALTHY;
}
#endif

/* End of file. */
