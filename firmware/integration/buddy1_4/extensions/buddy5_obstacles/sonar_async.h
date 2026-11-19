#ifndef B5_SONAR_ASYNC_H
#define B5_SONAR_ASYNC_H
#include "extensions/buddy5_obstacles/profile/obstacle_profile.h"
enum { B5E_IDLE, B5E_LOW, B5E_RISE, B5E_FALL, B5E_READY };
typedef struct {
    unsigned phase;
    uint64_t deadline, triggered, rose, next_ping;
    bool short_seen;
    b5_ping_t result;
    uint32_t latency_us; /* v0.6.3 diagnostic: trigger end -> accepted rise */
} B5Echo;
void b5_echo_init(B5Echo *e);
bool b5_echo_request(B5Echo *e, uint64_t now);
/* Caller serializes poll/edge/take. tick returns true exactly once per TRIG.
 * Trigger itself is the sole 12 us busy wait; no echo/servo/scan busy waits. */
bool b5_echo_tick(B5Echo *e, uint64_t now, bool echo_high);
void b5_echo_edge(B5Echo *e, uint64_t now, bool rise, bool fall);
bool b5_echo_take(B5Echo *e, b5_ping_t *out);
#endif
