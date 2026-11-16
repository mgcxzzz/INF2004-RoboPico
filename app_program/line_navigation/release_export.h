/**
 * @file release_export.h
 * @brief Release export.
 * @details ADC/control ownership covers sensor acquisition, planning and calibration.
 * Other tasks consume protected copies; flash and stored exports require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_RELEASE_EXPORT_H
#define RTOS_RELEASE_EXPORT_H
#include <stdbool.h>
#include <stdint.h>
/* Owner-only transfer identity; only the deadline/busy view crosses the
 * existing line-copy lock. Full timestamps/sequence are modulo32 domains. */
typedef struct
{
    uint32_t until_us;
    uint32_t nav_sequence;
    unsigned total : 12;
    unsigned run : 16;
    unsigned dataset : 2;
    unsigned active : 1;
} rl_export_lease_t;
static inline bool rl_export_busy (const rl_export_lease_t * p_state,
                                   uint32_t now)
{
    return p_state->active && (int32_t)(p_state->until_us - now) > 0;
}
static inline void rl_export_open (rl_export_lease_t * p_state,
                                   unsigned dataset, unsigned total,
                                   uint16_t run, uint32_t nav_sequence,
                                   uint32_t now)
{
    *p_state = (rl_export_lease_t){.until_us = now + 5000000U,
                                   .nav_sequence = nav_sequence,
                                   .total = total,
                                   .run = run,
                                   .dataset = dataset,
                                   .active = 1};
}
static inline bool rl_export_same (const rl_export_lease_t * p_state,
                                   unsigned dataset, unsigned total,
                                   uint16_t run, uint32_t nav_sequence,
                                   uint32_t now)
{
    return rl_export_busy(p_state, now) && p_state->dataset == dataset &&
           p_state->total == total && p_state->run == run &&
           p_state->nav_sequence == nav_sequence;
}
/* Explicit little endian encoding; never send compiler-layout structs. */
static inline void rl_put16 (uint8_t * p_data, uint16_t value)
{
    p_data[0] = value;
    p_data[1] = value >> 8;
}
static inline void rl_put32 (uint8_t * p_data, uint32_t value)
{
    for (unsigned item_index = 0; item_index < 4; ++item_index)
    {
        p_data[item_index] = value >> (8 * item_index);
    }
}
#endif

/* End of file. */
