#ifndef RTOS_ENCODER_QUALITY_H
#define RTOS_ENCODER_QUALITY_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

enum {
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
typedef struct {
    uint16_t bad, changes;
} BrEncoderBucket;
typedef struct {
    uint32_t last_edges, last_invalid, bucket_since;
    uint32_t bad_at[BR_ENC_BURST_COUNT];
    uint32_t window_bad, window_changes;
    BrEncoderBucket buckets[BR_ENC_BUCKETS];
    uint8_t slot : 4, recent_count : 4;
    uint8_t recent_head : 3, initialized : 1;
} BrEncoderQuality;

static inline void br_encoder_reset(BrEncoderQuality *q, uint32_t now, uint32_t edges,
                                    uint32_t invalid) {
    *q = (BrEncoderQuality){
        .last_edges = edges, .last_invalid = invalid, .bucket_since = now, .initialized = 1};
}
static inline uint16_t br_encoder_add16(uint16_t value, uint32_t amount) {
    return amount > (uint32_t)UINT16_MAX - value ? UINT16_MAX : (uint16_t)(value + amount);
}
/* Owner-only, bounded work (<=10 buckets and8 event timestamps). No allocation,
 * GPIO, PWM or kernel call. Lifetime raw counters remain untouched. */
static inline unsigned br_encoder_quality(BrEncoderQuality *q, uint32_t now, uint32_t edges,
                                          uint32_t invalid) {
    if (!q->initialized) {
        br_encoder_reset(q, now, edges, invalid);
        return BR_ENC_OK;
    }
    uint32_t elapsed = now - q->bucket_since;
    if (elapsed >= BR_ENC_WINDOW_US) {
        memset(q->buckets, 0, sizeof(q->buckets));
        q->window_bad = q->window_changes = 0;
        q->slot = 0;
        q->bucket_since = now;
    } else {
        unsigned steps = elapsed / BR_ENC_BUCKET_US;
        for (unsigned n = 0; n < steps; ++n) {
            q->slot = (uint8_t)((q->slot + 1) % BR_ENC_BUCKETS);
            q->window_bad -= q->buckets[q->slot].bad;
            q->window_changes -= q->buckets[q->slot].changes;
            q->buckets[q->slot] = (BrEncoderBucket){0};
        }
        q->bucket_since += steps * BR_ENC_BUCKET_US;
    }
    uint32_t bad = invalid - q->last_invalid, edge_delta = edges - q->last_edges;
    if (q->recent_count &&
        now - q->bad_at[(q->recent_head + BR_ENC_BURST_COUNT - 1) % BR_ENC_BURST_COUNT] >
            BR_ENC_BURST_US)
        q->recent_count = 0; /* Do not revive ancient errors at TIMER wrap. */
    q->last_invalid = invalid;
    q->last_edges = edges;
    /* Decoder counts one A/B edge for a valid step and two for an invalid
     * two-bit step. Subtract the extra edge to get observed state changes. */
    uint32_t changes = edge_delta >= bad ? edge_delta - bad : bad;
    BrEncoderBucket *b = &q->buckets[q->slot];
    q->window_bad -= b->bad;
    q->window_changes -= b->changes;
    b->bad = br_encoder_add16(b->bad, bad);
    b->changes = br_encoder_add16(b->changes, changes);
    q->window_bad += b->bad;
    q->window_changes += b->changes;
    unsigned retained = bad > BR_ENC_BURST_COUNT ? BR_ENC_BURST_COUNT : bad;
    for (unsigned n = 0; n < retained; ++n) {
        q->bad_at[q->recent_head] = now;
        q->recent_head = (uint8_t)((q->recent_head + 1) % BR_ENC_BURST_COUNT);
        if (q->recent_count < BR_ENC_BURST_COUNT)
            ++q->recent_count;
    }
    if (q->recent_count == BR_ENC_BURST_COUNT && now - q->bad_at[q->recent_head] <= BR_ENC_BURST_US)
        return BR_ENC_BURST;
    /* Ten100ms bins cover the most recent900..1000ms. Sparse errors never
     * accumulate across an entire run or survive a long Pause. */
    if (q->window_bad >= BR_ENC_DENSITY_COUNT &&
        q->window_bad * BR_ENC_DENSITY_DENOMINATOR >= q->window_changes)
        return BR_ENC_DENSITY;
    return BR_ENC_OK;
}
#endif
