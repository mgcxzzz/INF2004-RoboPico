#include "extensions/buddy5_obstacles/sonar_async.h"
#include "extensions/buddy5_obstacles/profile/b5_config.h"
#include <string.h>
static void finish(B5Echo *e, bool low, bool rise, bool fall, uint32_t width) {
    e->result = b5_classify_ping(low, rise, fall, e->short_seen, width);
    e->phase = B5E_READY;
}
void b5_echo_init(B5Echo *e) { memset(e, 0, sizeof(*e)); }
bool b5_echo_request(B5Echo *e, uint64_t now) {
    if (e->phase != B5E_IDLE || now < e->next_ping) return false;
    e->phase = B5E_LOW;
    e->short_seen = false;
    e->latency_us = 0;
    e->deadline = now + (uint64_t)B5_PRETRIG_LOW_WAIT_MS * 1000;
    /* Also rate-limit failed pre-trigger transactions. */
    e->next_ping = now + B5_PING_PERIOD_MS * 1000;
    return true;
}
bool b5_echo_tick(B5Echo *e, uint64_t now, bool high) {
    if (e->phase == B5E_LOW) {
        if (!high) {
            e->phase = B5E_RISE;
            e->triggered = now;
            e->deadline = now + 30000;
            e->next_ping = now + B5_PING_PERIOD_MS * 1000;
            return true;
        }
        if (now >= e->deadline) finish(e, false, false, false, 0);
    } else if (e->phase == B5E_RISE && now >= e->deadline)
        finish(e, true, false, false, 0);
    else if (e->phase == B5E_FALL && now >= e->deadline)
        finish(e, true, true, false, 0);
    return false;
}
void b5_echo_edge(B5Echo *e, uint64_t now, bool rise, bool fall) {
    if (e->phase != B5E_RISE && e->phase != B5E_FALL) return;
    if (rise && fall) { finish(e, true, false, false, 0); return; }
    /* A delayed IRQ after the transaction deadline cannot manufacture an echo. */
    if (now >= e->deadline) {
        finish(e, true, e->phase == B5E_FALL, false, 0);
        return;
    }
    if (e->phase == B5E_RISE && rise) {
        e->rose = now;
        e->latency_us = (uint32_t)(now - e->triggered);
        e->deadline = now + (uint64_t)B5_NORETURN_WAIT_MS * 1000;
        e->phase = B5E_FALL;
    } else if (e->phase == B5E_FALL && fall) {
        uint32_t width = (uint32_t)(now - e->rose);
        if (width < B5_ECHO_MIN_US) {
            e->short_seen = true;
            e->phase = B5E_RISE;
            e->deadline = e->triggered + 30000;
        } else finish(e, true, true, true, width);
    }
}
bool b5_echo_take(B5Echo *e, b5_ping_t *out) {
    if (e->phase != B5E_READY) return false;
    *out = e->result;
    e->phase = B5E_IDLE;
    return true;
}
