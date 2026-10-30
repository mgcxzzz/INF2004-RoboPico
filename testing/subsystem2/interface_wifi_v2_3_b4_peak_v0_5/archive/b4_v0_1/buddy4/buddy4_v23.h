/* Buddy4 IMU/ramp add-on for the v2.3 SDK firmware: narrow interface used by
 * buddy4_harness.h (included in the v2.3 main translation unit). Plain types
 * only, so no Buddy4 macro reaches v2.3's translation unit. */
#ifndef BUDDY4_V23_H
#define BUDDY4_V23_H
#include <stdbool.h>
#include <stdint.h>

/* v2.3 odometry as Buddy4 consumes it (filled by the harness from v2.3's own
 * encoder drain each control tick; never written back). */
typedef struct {
    bool     have;            /* at least one control tick observed          */
    bool     clean;           /* latest 10 ms interval: no invalid transition
                                 and interval time within (0, 50 ms]          */
    uint32_t age_ms;          /* since that interval was drained             */
    double   centre_mm;       /* signed net centre distance since boot       */
    double   heading_dd;      /* (left-right)/110 mm track, 0.1 degree        */
    float    left_mm_s, right_mm_s;   /* v2.3 velocity[] (0.25 EMA)          */
    uint32_t invalid_left, invalid_right;  /* v2.3 cumulative counters       */
} b4v23_odo_t;

/* Mission edge requests, mapped from v2.3 run state by the harness. */
typedef enum { B4V23_NONE = 0, B4V23_START, B4V23_RESUME, B4V23_STOP, B4V23_RESET } b4v23_mission_t;

typedef bool (*b4v23_send_fn)(const uint8_t *frame, uint32_t len);

void     b4v23_init(b4v23_send_fn send, bool console);
/* Call each superloop pass; does work only when a 20 ms period is due. */
bool     b4v23_due(uint64_t now_us);
void     b4v23_tick(uint64_t now_us, b4v23_mission_t mission, const b4v23_odo_t *odo);
uint32_t b4v23_last_tick_us(void);      /* duration of the last tick (I2C + processing) */
uint32_t b4v23_max_tick_us(void);
#endif
