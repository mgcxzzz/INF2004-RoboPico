/**
 * @file circuit_record.h
 * @brief Circuit record.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef CIRCUIT_RECORD_H
#define CIRCUIT_RECORD_H
#include <stddef.h>
#include "src/line_following/circuit_logic.h"
#include "src/motion/log_store.h"
enum
{
    C_ROWS = 1200,
    C_SUMMARIES = 64,
    C_STORE_BYTES = 65536
};
#define C_MAGIC 0x31524343u
/* V5 uses unused phase/side bits for diagnostics without changing the record
   ABI. Older rows have zero high bits. Printed phase and side remain raw. */
static inline uint8_t circuit_trace_phase (unsigned phase,
                                           const circuit_tracker_t * p_sample,
                                           uint64_t now)
{
    if ((phase != C_FOLLOW) && (phase != J_ACQUIRE))
    {
        return (uint8_t)phase;
    }
    return (uint8_t)(phase | (circuit_memory(p_sample, now) << 4) |
                     (p_sample->core.white ? 64 : 0) |
                     (p_sample->reacquire_timing ? 128 : 0));
}
static inline uint8_t circuit_trace_side (unsigned raw, unsigned feedback)
{
    return (uint8_t)((raw & 1) | ((feedback & 3) << 1));
}
typedef struct
{
    uint64_t ms;
    uint32_t path10;
    int32_t wheel10[2];
    int16_t speed10[2];
    int16_t pwm[2];
    int16_t target[2];
    uint16_t level[2];
    uint16_t adc[2];
    uint32_t turns;
    uint8_t phase;
    uint8_t side;
    uint8_t decision;
    uint8_t active;
} circuit_trace_t;
typedef struct
{
    uint64_t ms;
    uint32_t path10;
    uint32_t number;
    uint32_t follow_ms;
    uint16_t centre_ms;
    uint16_t turn_ms;
    uint16_t align_ms;
    int16_t angle10;
} circuit_summary_t;
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t crc;
    uint32_t count;
    uint32_t summary_count;
    uint32_t overwritten;
    uint64_t duration_us;
    uint64_t path10;
    int64_t ticks[2];
    uint32_t invalid[2];
    uint32_t turns;
    uint32_t ignored;
    uint8_t reason;
    uint8_t key;
    uint8_t offline;
    uint8_t profile;
    uint8_t recorded_profile;
    uint8_t imported;
    line_calibration_t calibration;
    side_calibration_t side;
    circuit_trace_t rows[C_ROWS];
    circuit_summary_t summaries[C_SUMMARIES];
} circuit_saved_t;
_Static_assert(48 == sizeof(circuit_trace_t) && 32 == sizeof(circuit_summary_t),
               "Circuit trace ABI changed");
_Static_assert(sizeof(circuit_saved_t) <= C_STORE_BYTES,
               "Circuit saved log exceeds64KiB");
static inline bool circuit_record_valid (const circuit_saved_t * p_state)
{
    return C_MAGIC == p_state->magic && (p_state->version >= 1) &&
           (p_state->version <= 6) && (p_state->count <= C_ROWS) &&
           (p_state->summary_count <= C_SUMMARIES) && (p_state->profile >= 1) &&
           (p_state->profile <= 3) && (p_state->recorded_profile >= 1) &&
           (p_state->recorded_profile <= 3) &&
           (calibration_valid(&p_state->calibration)) &&
           (side_valid(&p_state->side)) &&
           (p_state->crc ==
            log_crc((const uint8_t *)p_state + offsetof(circuit_saved_t, count),
                    sizeof(*p_state) - offsetof(circuit_saved_t, count)));
}
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t crc;
    uint32_t count;
    uint32_t dropped;
    uint64_t duration;
    int32_t cutoff[2];
    uint32_t invalid[2];
    uint8_t reason;
    uint8_t key;
    uint8_t offline;
    uint8_t mode;
    line_calibration_t calibration;
    side_calibration_t side;
    uint8_t selected_mode;
    uint8_t firmware;
    junction_t junction;
    uint8_t rows[3600];
} circuit_legacy_t;
_Static_assert(3792 == sizeof(circuit_legacy_t) &&
                   192 == offsetof(circuit_legacy_t, rows),
               "V10 import ABI changed");
static inline bool circuit_import (const circuit_legacy_t * p_state,
                                   line_calibration_t * p_cal,
                                   side_calibration_t * p_side)
{
    if (((((((((((p_state->magic != 0x314e434au) || (p_state->version != 1)) ||
                (p_state->firmware > 10)) ||
               (p_state->count > 100)) ||
              (p_state->mode < 1)) ||
             (p_state->mode > 3)) ||
            (p_state->selected_mode < 1)) ||
           (p_state->selected_mode > 3)) ||
          (!calibration_valid(&p_state->calibration))) ||
         (!side_valid(&p_state->side))) ||
        (p_state->crc !=
         log_crc((const uint8_t *)p_state + offsetof(circuit_legacy_t, count),
                 sizeof(*p_state) - offsetof(circuit_legacy_t, count))))
    {
        return false;
    }
    *p_cal = p_state->calibration;
    *p_side = p_state->side;
    return true;
}
#endif

/* End of file. */
