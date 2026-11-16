/**
 * @file encoder_quality.h
 * @brief Encoder quality.
 * @details Core1 owns normal motor commits. Safety paths may only remove
 * output; command/status access uses the existing kernel locks and copied
 * records.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_ENCODER_QUALITY_H
#define RTOS_ENCODER_QUALITY_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum
{
    BR_ENC_OK = 0,
    BR_ENC_BURST = 1,
    BR_ENC_DENSITY = 2,
    BR_ENC_DIRECTION = 3,
    BR_ENC_BURST_COUNT = 8,
    BR_ENC_BURST_US = 100000,
    BR_ENC_BUCKETS = 10,
    BR_ENC_BUCKET_US = 100000,
    BR_ENC_WINDOW_US = 1000000,
    BR_ENC_DENSITY_COUNT = 8,
    BR_ENC_DENSITY_DENOMINATOR = 20 /* >=5% invalid observed state changes. */
};
typedef struct
{
    uint16_t bad;
    uint16_t changes;
} br_encoder_bucket_t;
typedef struct
{
    uint32_t last_edges;
    uint32_t last_invalid;
    uint32_t bucket_since;
    uint32_t bad_at[BR_ENC_BURST_COUNT];
    uint32_t window_bad;
    uint32_t window_changes;
    br_encoder_bucket_t buckets[BR_ENC_BUCKETS];
    uint8_t slot : 4;
    uint8_t recent_count : 4;
    uint8_t recent_head : 3;
    uint8_t initialized : 1;
} br_encoder_quality_t;

static inline void br_encoder_reset (br_encoder_quality_t * p_other_data,
                                     uint32_t now, uint32_t edges,
                                     uint32_t invalid)
{
    *p_other_data = (br_encoder_quality_t){.last_edges = edges,
                                           .last_invalid = invalid,
                                           .bucket_since = now,
                                           .initialized = 1};
}
static inline uint16_t br_encoder_add16 (uint16_t value, uint32_t amount)
{
    return amount > (uint32_t)UINT16_MAX - value ? UINT16_MAX
                                                 : (uint16_t)(value + amount);
}
/* Owner-only, bounded work (<=10 buckets and8 event timestamps). No allocation,
 * GPIO, PWM or kernel call. Lifetime raw counters remain untouched. */
static inline unsigned br_encoder_quality (br_encoder_quality_t * p_other_data,
                                           uint32_t now, uint32_t edges,
                                           uint32_t invalid)
{
    if (!p_other_data->initialized)
    {
        br_encoder_reset(p_other_data, now, edges, invalid);
        return BR_ENC_OK;
    }
    uint32_t elapsed = now - p_other_data->bucket_since;
    if (elapsed >= BR_ENC_WINDOW_US)
    {
        memset(p_other_data->buckets, 0, sizeof(p_other_data->buckets));
        p_other_data->window_bad = p_other_data->window_changes = 0;
        p_other_data->slot = 0;
        p_other_data->bucket_since = now;
    }
    else
    {
        unsigned steps = elapsed / BR_ENC_BUCKET_US;
        for (unsigned count = 0; count < steps; ++count)
        {
            p_other_data->slot =
                (uint8_t)((p_other_data->slot + 1) % BR_ENC_BUCKETS);
            p_other_data->window_bad -=
                p_other_data->buckets[p_other_data->slot].bad;
            p_other_data->window_changes -=
                p_other_data->buckets[p_other_data->slot].changes;
            p_other_data->buckets[p_other_data->slot] =
                (br_encoder_bucket_t){0};
        }
        p_other_data->bucket_since += steps * BR_ENC_BUCKET_US;
    }
    uint32_t bad = invalid - p_other_data->last_invalid,
             edge_delta = edges - p_other_data->last_edges;
    if ((p_other_data->recent_count) &&
        (now - p_other_data->bad_at[(p_other_data->recent_head +
                                     BR_ENC_BURST_COUNT - 1) %
                                    BR_ENC_BURST_COUNT] >
         BR_ENC_BURST_US))
    {
        p_other_data->recent_count =
            0; /* Do not revive ancient errors at TIMER wrap. */
    }
    p_other_data->last_invalid = invalid;
    p_other_data->last_edges = edges;
    /* Decoder counts one A/B edge for a valid step and two for an invalid
     * two-bit step. Subtract the extra edge to get observed state changes. */
    uint32_t changes = edge_delta >= bad ? edge_delta - bad : bad;
    br_encoder_bucket_t * p_second = &p_other_data->buckets[p_other_data->slot];
    p_other_data->window_bad -= p_second->bad;
    p_other_data->window_changes -= p_second->changes;
    p_second->bad = br_encoder_add16(p_second->bad, bad);
    p_second->changes = br_encoder_add16(p_second->changes, changes);
    p_other_data->window_bad += p_second->bad;
    p_other_data->window_changes += p_second->changes;
    unsigned retained = bad > BR_ENC_BURST_COUNT ? BR_ENC_BURST_COUNT : bad;
    for (unsigned count_value = 0; count_value < retained; ++count_value)
    {
        p_other_data->bad_at[p_other_data->recent_head] = now;
        p_other_data->recent_head =
            (uint8_t)((p_other_data->recent_head + 1) % BR_ENC_BURST_COUNT);
        if (p_other_data->recent_count < BR_ENC_BURST_COUNT)
        {
            ++p_other_data->recent_count;
        }
    }
    if (BR_ENC_BURST_COUNT == p_other_data->recent_count &&
        (now - p_other_data->bad_at[p_other_data->recent_head] <=
         BR_ENC_BURST_US))
    {
        return BR_ENC_BURST;
    }
    /* Ten100ms bins cover the most recent900..1000ms. Sparse errors never
     * accumulate across an entire run or survive a long Pause. */
    if ((p_other_data->window_bad >= BR_ENC_DENSITY_COUNT) &&
        (p_other_data->window_bad * BR_ENC_DENSITY_DENOMINATOR >=
         p_other_data->window_changes))
    {
        return BR_ENC_DENSITY;
    }
    return BR_ENC_OK;
}
#endif

/* End of file. */
