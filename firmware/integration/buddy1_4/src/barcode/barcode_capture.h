/**
 * @file barcode_capture.h
 * @brief Barcode capture.
 * @details Pure barcode/route logic is called by the ADC owner and emits requests.
 * Native records are not wire packets; preserve both decode directions.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include "src/barcode/barcode_scan.h"
typedef struct
{
    uint64_t us;
    int64_t ticks[2];
    bool black;
} barcode_edge_t;
typedef struct
{
    barcode_edge_t edge[B_EDGE_QUEUE];
    volatile uint16_t head, tail;
    volatile uint32_t overflow;
    uint64_t candidate_us;
    int64_t candidate_ticks[2];
    volatile bool stable;
    bool candidate;
} barcode_capture_t;
static inline void barcode_capture_reset (barcode_capture_t * s, bool black,
                                          uint64_t now)
{
    s->head = s->tail = 0;
    s->overflow = 0;
    s->stable = s->candidate = black;
    s->candidate_us = now;
}
/* 1ms producer: only integer snapshots, no floating point or decoding in ISR.
   Require2ms stable D0; timestamp/counts retain the FIRST candidate sample. */
static inline void barcode_capture_tick (barcode_capture_t * s, uint64_t now,
                                         bool black, int64_t left,
                                         int64_t right)
{
    if (black != s->candidate)
    {
        s->candidate = black;
        s->candidate_us = now;
        s->candidate_ticks[0] = left;
        s->candidate_ticks[1] = right;
    }
    if (s->candidate == s->stable || now - s->candidate_us < 2000)
    {
        return;
    }
    s->stable = s->candidate;
    unsigned next = (s->head + 1) % B_EDGE_QUEUE;
    if (next == s->tail)
    {
        ++s->overflow;
        return;
    }
    s->edge[s->head] = (barcode_edge_t){
        .us = s->candidate_us,
        .ticks = {s->candidate_ticks[0], s->candidate_ticks[1]},
        .black = s->stable};
    s->head = (uint16_t)next;
}
/* Call only with IRQs masked; producer and consumer share the same core. */
static inline bool barcode_capture_pop (barcode_capture_t * s,
                                        barcode_edge_t * e)
{
    if (s->head == s->tail)
    {
        return false;
    }
    *e = s->edge[s->tail];
    s->tail = (s->tail + 1) % B_EDGE_QUEUE;
    return true;
}
#endif

/* End of file. */
