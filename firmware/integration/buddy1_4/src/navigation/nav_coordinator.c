#include "src/navigation/nav_coordinator.h"
#include "src/navigation/nav_codec.h"
#include "src/motion/motion_executor.h"
#include <math.h>
#include <string.h>
enum { EVENT_CAPACITY = 32, RESULT_CAPACITY = 16, AUDIT_CAPACITY = 120 };
static struct {
    MotionPort port;
    MotionStatus status;
    MotionFeedback feedback;
    MotionExecutor executor;
    MotionCommandStatus results[RESULT_CAPACITY];
    MotionEvent events[EVENT_CAPACITY], audit[AUDIT_CAPACITY];
    unsigned event_head, event_count, result_head, audit_head, audit_count, audit_overwritten;
    uint32_t next_id, sequence;
    unsigned reported_faults : 10;
    uint64_t pause_since, still_since, line_since, nav_since, target_log_since;
    bool still_timing, line_timing, dirty, resume, abandon;
    int16_t nav_target[2];
    char route;
    MotionRampAssist assist;
    uint64_t assist_deadline, assist_log_us;
    bool assist_eligible;
} g;
static uintptr_t lock(void) {
    return g.port.lock ? g.port.lock(g.port.user) : 0;
}
static void unlock(uintptr_t state) {
    if (g.port.unlock)
        g.port.unlock(g.port.user, state);
}
static uint64_t now(void) {
    return g.port.now_us ? g.port.now_us(g.port.user) : g.feedback.now_us;
}
static bool owner_valid(MotionOwner owner) {
    return owner >= MOTION_BUDDY1 && owner <= MOTION_BUDDY5;
}
static void emit(MotionEvent e) {
    e.sequence = ++g.sequence;
    g.events[g.event_head] = e;
    g.event_head = (g.event_head + 1) % EVENT_CAPACITY;
    if (g.event_count < EVENT_CAPACITY)
        ++g.event_count;
    g.audit[g.audit_head] = e;
    g.audit_head = (g.audit_head + 1) % AUDIT_CAPACITY;
    if (g.audit_count < AUDIT_CAPACITY)
        ++g.audit_count;
    else
        ++g.audit_overwritten;
}
static void event(unsigned type) {
    MotionEvent e = {.now_us = now(),
                     .owner = g.status.owner,
                     .requester = g.status.owner,
                     .id = g.status.command_id,
                     .type = type,
                     .state = g.status.command_state,
                     .fault = g.status.fault,
                     .target_mm_s = {g.status.target_mm_s[0], g.status.target_mm_s[1]},
                     .goal = g.executor.goal,
                     .speed = (unsigned)g.executor.speed,
                     .style = g.executor.style,
                     .kind = g.executor.turning ? 3 : 2,
                     .progress_mm = g.status.progress_mm,
                     .rotation_deg = g.status.rotation_deg};
    if (type == MOTION_EVENT_PAUSE || type == MOTION_EVENT_STOPPED) {
        e.kind = 0;
        e.goal = 0;
        e.speed = 0;
    }
    if (type == MOTION_EVENT_RESUME) {
        e.kind = 4;
        e.goal = 0;
        e.speed = 0;
    }
    if (type == MOTION_EVENT_ROUTE) {
        e.kind = 5;
        e.requester = MOTION_BUDDY3;
        e.goal = g.route;
        e.speed = 0;
    }
    if (type == MOTION_EVENT_TERRAIN) {
        e.kind = 6;
        e.requester = MOTION_BUDDY4;
        e.goal = g.status.terrain;
        e.speed = 0;
    }
    emit(e);
}
void nav_coordinator_reject(MotionOwner owner, unsigned kind, float goal, unsigned speed,
                            MotionResult result) {
    if (result == MOTION_OK)
        return;
    uintptr_t key = lock();
    MotionEvent e = {.now_us = now(),
                     .owner = g.status.owner,
                     .requester = owner_valid(owner) ? owner : 0,
                     .type = MOTION_EVENT_REJECT,
                     .kind = kind,
                     .goal = isfinite(goal) ? goal : 0,
                     .speed = speed <= 127 ? speed : 0,
                     .result = result,
                     .fault = g.status.fault,
                     .state = g.status.command_state};
    emit(e);
    unlock(key);
}
static bool assist_active(void) {
    return g.assist.boost_milli || g.assist.push_pwm_x100 || g.assist.hold_milli;
}
static void assist_event(unsigned reason) {
    MotionEvent e = {.now_us = now(),
                     .requester = MOTION_BUDDY4,
                     .owner = g.status.owner,
                     .type = MOTION_EVENT_TARGETS,
                     .kind = NAV_RAMP_ASSIST,
                     .goal = (float)reason,
                     .target_mm_s = {g.status.target_mm_s[0], g.status.target_mm_s[1]}};
    emit(e);
    g.assist_log_us = now();
}
static void clear_assist(unsigned reason) {
    if (assist_active()) {
        g.assist = (MotionRampAssist){0};
        g.assist_deadline = 0;
        assist_event(reason);
    }
}
static void off(void) {
    clear_assist(2);
    g.assist_eligible = false;
    ++g.status.generation;
    g.dirty = true;
    g.status.target_mm_s[0] = g.status.target_mm_s[1] = 0;
    if (g.port.outputs_off)
        g.port.outputs_off(g.port.user);
}
static void remember(MotionCommandState state) {
    if (!g.status.command_id)
        return;
    g.status.command_state = state;
    g.results[g.result_head] = (MotionCommandStatus){g.status.command_id, state, g.status.fault,
                                                     g.status.progress_mm, g.status.rotation_deg};
    g.result_head = (g.result_head + 1) % RESULT_CAPACITY;
}
static void cancel(void) {
    if (g.status.command_state == MOTION_CMD_RUNNING) {
        remember(MOTION_CMD_CANCELLED);
        event(MOTION_EVENT_CANCEL);
    }
    g.executor.active = false;
}
void nav_coordinator_init(const MotionPort *port) {
    memset(&g, 0, sizeof(g));
    if (port)
        g.port = *port;
    g.status.mode = MOTION_OFF;
}
void nav_coordinator_begin(uint64_t time) {
    uintptr_t key = lock();
    cancel();
    uint32_t generation = g.status.generation + 1, lease = g.status.lease + 1;
    g.status = (MotionStatus){.fault = g.reported_faults,
                              .session_active = true,
                              .mode = MOTION_AUTO,
                              .owner = MOTION_BUDDY3,
                              .generation = generation,
                              .lease = lease};
    g.feedback = (MotionFeedback){.now_us = time};
    g.assist = (MotionRampAssist){0};
    g.assist_deadline = g.assist_log_us = 0;
    g.assist_eligible = false;
    g.nav_since = time;
    g.nav_target[0] = g.nav_target[1] = 0;
    g.executor = (MotionExecutor){0};
    g.dirty = g.resume = g.abandon = g.still_timing = g.line_timing = false;
    g.route = 0;
    g.event_head = g.event_count = g.audit_head = g.audit_count = g.audit_overwritten = g.sequence =
        0;
    g.target_log_since = 0;
    unlock(key);
}
void nav_coordinator_end(uint8_t reason) {
    uintptr_t key = lock();
    cancel();
    g.status.session_active = false;
    g.status.mode = MOTION_OFF;
    g.status.stop_reason = reason <= 63 ? reason : 63;
    off();
    MotionEvent e = {.now_us = now(),
                     .owner = g.status.owner,
                     .requester = 0,
                     .type = MOTION_EVENT_SESSION_END,
                     .kind = 1,
                     .id = g.status.command_id,
                     .state = g.status.command_state,
                     .fault = g.status.fault,
                     .goal = reason};
    emit(e);
    unlock(key);
}
MotionResult nav_coordinator_pause(MotionOwner owner, uint32_t *lease) {
    if (!owner_valid(owner) || !lease)
        return MOTION_INVALID;
    uintptr_t key = lock();
    MotionResult result = MOTION_OK;
    if (!g.status.session_active)
        result = MOTION_INACTIVE;
    else if (g.status.mode != MOTION_AUTO && g.status.owner != owner && owner != MOTION_BUDDY5)
        result = MOTION_BUSY;
    else if (g.status.owner == owner &&
             (g.status.mode == MOTION_PAUSED || g.status.mode == MOTION_STOPPING))
        *lease = g.status.lease;
    else {
        cancel();
        g.resume = g.abandon = false;
        g.status.navigation_interrupted |= g.feedback.navigation_busy;
        g.status.owner = owner;
        ++g.status.lease;
        *lease = g.status.lease;
        g.status.mode = MOTION_STOPPING;
        g.status.fault = g.reported_faults;
        g.status.stopped = false;
        g.pause_since = now();
        g.still_timing = false;
        off();
        event(MOTION_EVENT_PAUSE);
    }
    unlock(key);
    return result;
}
MotionResult nav_coordinator_stop(MotionOwner owner) {
    if (!owner_valid(owner))
        return MOTION_INVALID;
    uintptr_t key = lock();
    if (!g.status.session_active) {
        unlock(key);
        return MOTION_INACTIVE;
    }
    if (g.status.mode == MOTION_AUTO) {
        g.status.owner = owner;
        ++g.status.lease;
    }
    cancel();
    g.resume = g.abandon = false;
    g.status.navigation_interrupted |= g.feedback.navigation_busy;
    g.status.mode = MOTION_STOPPING;
    g.status.stopped = false;
    g.pause_since = now();
    g.still_timing = false;
    off();
    event(MOTION_EVENT_PAUSE);
    g.audit[(g.audit_head + AUDIT_CAPACITY - 1) % AUDIT_CAPACITY].requester = owner;
    g.events[(g.event_head + EVENT_CAPACITY - 1) % EVENT_CAPACITY].requester = owner;
    g.audit[(g.audit_head + AUDIT_CAPACITY - 1) % AUDIT_CAPACITY].kind = 1;
    g.events[(g.event_head + EVENT_CAPACITY - 1) % EVENT_CAPACITY].kind = 1;
    unlock(key);
    return MOTION_OK;
}
static MotionResult access(MotionOwner owner, uint32_t lease) {
    if (!g.status.session_active)
        return MOTION_INACTIVE;
    if (owner != g.status.owner || lease != g.status.lease || !lease)
        return MOTION_NOT_OWNER;
    return MOTION_OK;
}
MotionResult nav_coordinator_move(MotionOwner owner, uint32_t lease, float goal, unsigned speed,
                                  bool turn, MotionTurnStyle style, uint32_t *id) {
    if (id)
        *id = 0;
    if (!owner_valid(owner) || !id || !isfinite(goal) || fabsf(goal) < 1 ||
        fabsf(goal) > (turn ? 180 : 1000) ||
        (speed && (speed < 20 || speed > (turn ? 90u : 60u))) ||
        (turn && style != MOTION_ROLLING && style != MOTION_IN_PLACE))
        return MOTION_INVALID;
    uintptr_t key = lock();
    MotionResult result = access(owner, lease);
    if (result == MOTION_OK && (g.status.mode != MOTION_PAUSED || !g.status.stopped))
        result = MOTION_NOT_STOPPED;
    if (result == MOTION_OK && now() - g.feedback.now_us > 100000)
        result = MOTION_NOT_STOPPED;
    if (result == MOTION_OK) {
        if (!speed)
            speed = 45;
        g.status.command_id = ++g.next_id;
        if (!g.status.command_id)
            g.status.command_id = ++g.next_id;
        *id = g.status.command_id;
        g.status.command_state = MOTION_CMD_RUNNING;
        g.status.progress_mm = g.status.rotation_deg = 0;
        g.status.fault = g.reported_faults;
        g.status.stopped = false;
        g.status.mode = turn ? MOTION_TURNING : MOTION_DRIVING;
        motion_executor_start(&g.executor, &g.feedback, turn, goal, speed, style);
        ++g.status.generation;
        g.dirty = true;
        event(MOTION_EVENT_START);
    }
    unlock(key);
    return result;
}
MotionResult nav_coordinator_resume(MotionOwner owner, uint32_t lease, bool abandon) {
    uintptr_t key = lock();
    MotionResult result = access(owner, lease);
    if (result == MOTION_OK && (g.status.mode != MOTION_PAUSED || !g.status.stopped))
        result = MOTION_NOT_STOPPED;
    if (result == MOTION_OK && g.status.navigation_interrupted && !abandon)
        result = MOTION_NEED_RECOVERY;
    if (result == MOTION_OK &&
        (!g.line_timing || now() - g.line_since < 150000 || now() - g.feedback.now_us > 100000))
        result = MOTION_NEED_LINE;
    if (result == MOTION_OK) {
        g.resume = true;
        g.abandon = abandon;
        g.status.owner = MOTION_BUDDY3;
        ++g.status.lease;
        off();
        event(MOTION_EVENT_RESUME);
        g.status.navigation_interrupted = false;
        g.audit[(g.audit_head + AUDIT_CAPACITY - 1) % AUDIT_CAPACITY].requester = owner;
        g.events[(g.event_head + EVENT_CAPACITY - 1) % EVENT_CAPACITY].requester = owner;
        g.nav_target[0] = g.nav_target[1] = 0;
        g.nav_since = now();
    }
    unlock(key);
    return result;
}
MotionResult nav_coordinator_navigation(MotionOwner owner, int16_t left, int16_t right) {
    if (owner != MOTION_BUDDY3 || left < -180 || right < -180 || left > 180 || right > 180)
        return MOTION_INVALID;
    uintptr_t key = lock();
    MotionResult r = !g.status.session_active       ? MOTION_INACTIVE
                     : g.status.mode != MOTION_AUTO ? MOTION_BUSY
                                                    : MOTION_OK;
    if (r == MOTION_OK) {
        g.nav_target[0] = left;
        g.nav_target[1] = right;
        g.nav_since = now();
        if (!g.target_log_since || now() - g.target_log_since >= 1000000) {
            MotionEvent e = {.now_us = now(),
                             .owner = MOTION_BUDDY3,
                             .requester = MOTION_BUDDY3,
                             .type = MOTION_EVENT_TARGETS,
                             .kind = 7,
                             .target_mm_s = {left, right}};
            emit(e);
            g.target_log_since = now();
        }
    }
    unlock(key);
    return r;
}
MotionResult nav_coordinator_route(MotionOwner owner, char code) {
#ifdef BUDDY_RUNTIME_MAPPING
    if (owner != MOTION_BUDDY3 || code < 'A' || code > 'Z')
        return MOTION_INVALID;
#else
    if (owner != MOTION_BUDDY3 || (code != 'A' && code != 'F' && code != 'Z'))
        return MOTION_INVALID;
#endif
    uintptr_t key = lock();
    MotionResult result = !g.status.session_active ? MOTION_INACTIVE
                          : g.status.mode != MOTION_AUTO || g.feedback.navigation_busy ? MOTION_BUSY
                                                                                       : MOTION_OK;
    if (result == MOTION_OK) {
        g.route = code;
        event(MOTION_EVENT_ROUTE);
    }
    unlock(key);
    return result;
}
MotionResult nav_coordinator_terrain(MotionOwner owner, MotionTerrain terrain) {
    if (owner != MOTION_BUDDY4 || (unsigned)terrain > (unsigned)MOTION_DESCENDING)
        return MOTION_INVALID;
    uintptr_t key = lock();
    g.status.terrain = terrain;
    event(MOTION_EVENT_TERRAIN);
    unlock(key);
    return MOTION_OK;
}
MotionStatus nav_coordinator_status(void) {
    uintptr_t key = lock();
    MotionStatus s = g.status;
    unlock(key);
    return s;
}
bool nav_coordinator_command(uint32_t id, MotionCommandStatus *status) {
    if (!id || !status)
        return false;
    uintptr_t key = lock();
    bool found = false;
    if (id == g.status.command_id) {
        *status = (MotionCommandStatus){id, g.status.command_state, g.status.fault,
                                        g.status.progress_mm, g.status.rotation_deg};
        found = true;
    } else
        for (unsigned n = 0; n < RESULT_CAPACITY; ++n)
            if (g.results[n].id == id) {
                *status = g.results[n];
                found = true;
                break;
            }
    unlock(key);
    return found;
}
MotionOutput nav_coordinator_update(const MotionFeedback *f) {
    uintptr_t key = lock();
    g.feedback = *f;
    MotionOutput output = {0};
    /* Requests use the live port clock and can be newer than this tick's
     * feedback snapshot. Compare request ages on that same live clock;
     * unsigned snapshot-request subtraction would underflow at startup. */
    uint64_t check_us = now();
    bool still = fabsf(f->speed_mm_s[0]) < 5 && fabsf(f->speed_mm_s[1]) < 5;
    if (!still)
        g.still_timing = false;
    else if (!g.still_timing) {
        g.still_timing = true;
        g.still_since = f->now_us;
    }
    g.status.stopped = g.still_timing && f->now_us - g.still_since >= 120000;
    bool line = f->line_level[0] >= 400 || f->line_level[1] >= 400;
    if (!line)
        g.line_timing = false;
    else if (!g.line_timing) {
        g.line_timing = true;
        g.line_since = f->now_us;
    }
    g.status.pending_route = f->pending_route;
    if (g.status.session_active) {
        if (g.resume)
            g.status.mode =
                MOTION_AUTO; /* adapter resets old navigation before the next optical tick */
        if (g.status.mode == MOTION_AUTO && check_us - g.nav_since > 100000) {
            g.status.mode = MOTION_FAULT;
            g.status.fault = g.reported_faults | NAV_FAULT_CONTROL;
            off();
            event(MOTION_EVENT_FAULT);
        } else if (g.status.mode == MOTION_STOPPING) {
            g.status.navigation_interrupted |= f->navigation_busy;
            if (g.status.stopped) {
                g.status.mode = MOTION_PAUSED;
                event(MOTION_EVENT_STOPPED);
            } else if (check_us - g.pause_since >= 2000000) {
                g.status.mode = MOTION_FAULT;
                g.status.fault = g.reported_faults | NAV_FAULT_SETTLE;
                off();
                event(MOTION_EVENT_FAULT);
            }
        } else if (g.status.mode == MOTION_DRIVING || g.status.mode == MOTION_TURNING ||
                   g.status.mode == MOTION_SETTLING) {
            MotionCommandState state = motion_executor_step(&g.executor, f);
            g.status.progress_mm = g.executor.progress_mm;
            g.status.rotation_deg = g.executor.rotation_deg;
            if (state != MOTION_CMD_RUNNING) {
                g.status.fault = g.reported_faults | g.executor.fault;
                remember(state);
                off();
                g.status.mode = state == MOTION_CMD_DONE ? MOTION_PAUSED : MOTION_FAULT;
                event(state == MOTION_CMD_DONE ? MOTION_EVENT_DONE : MOTION_EVENT_FAULT);
            } else
                g.status.mode = g.executor.settling  ? MOTION_SETTLING
                                : g.executor.turning ? MOTION_TURNING
                                                     : MOTION_DRIVING;
        }
    }
    output.external = !g.status.session_active || g.status.mode != MOTION_AUTO;
    if (g.status.session_active && g.status.mode == MOTION_AUTO) {
        output.target_mm_s[0] = g.nav_target[0];
        output.target_mm_s[1] = g.nav_target[1];
    } else if (g.status.mode == MOTION_DRIVING || g.status.mode == MOTION_TURNING) {
        output.target_mm_s[0] = g.executor.target[0];
        output.target_mm_s[1] = g.executor.target[1];
        output.turning = g.executor.turning;
    }
    memcpy(g.status.target_mm_s, output.target_mm_s, sizeof(g.status.target_mm_s));
    output.reset_controllers = g.dirty;
    g.dirty = false;
    output.resume_navigation = g.resume;
    output.abandon_navigation = g.abandon;
    g.resume = g.abandon = false;
    output.route = g.route;
    g.route = 0;
    output.generation = g.status.generation;
    unlock(key);
    return output;
}
bool nav_coordinator_commit(uint32_t epoch, void (*apply)(void *), void *user) {
    uintptr_t key = lock();
    bool ok =
        g.status.session_active && g.status.mode != MOTION_FAULT && epoch == g.status.generation;
    if (ok && apply)
        apply(user);
    unlock(key);
    return ok;
}
bool nav_coordinator_pop(MotionEvent *out) {
    if (!out)
        return false;
    uintptr_t key = lock();
    bool ok = g.event_count != 0;
    if (ok) {
        *out = g.events[(g.event_head + EVENT_CAPACITY - g.event_count) % EVENT_CAPACITY];
        --g.event_count;
    }
    unlock(key);
    return ok;
}

MotionResult nav_coordinator_fault(MotionOwner owner, unsigned bitmap, bool active) {
    if (!owner_valid(owner) || !bitmap || (bitmap & ~NAV_FAULT_MASK))
        return MOTION_INVALID;
    uintptr_t key = lock();
    if (active)
        g.reported_faults |= bitmap;
    else
        g.reported_faults &= ~bitmap;
    g.status.fault = g.reported_faults | g.executor.fault;
    MotionEvent e = {.now_us = now(),
                     .requester = owner,
                     .owner = g.status.owner,
                     .type = MOTION_EVENT_FAULT_REPORT,
                     .kind = 9,
                     .fault = g.status.fault,
                     .goal = (float)bitmap,
                     .state = active ? 1 : 0};
    emit(e);
    unlock(key);
    return MOTION_OK;
}
unsigned nav_coordinator_log_count(void) {
    uintptr_t key = lock();
    unsigned n = g.audit_count;
    unlock(key);
    return n;
}
unsigned nav_coordinator_log_overwritten(void) {
    uintptr_t key = lock();
    unsigned n = g.audit_overwritten;
    unlock(key);
    return n;
}
bool nav_coordinator_log_read(unsigned index, MotionEvent *e) {
    if (!e)
        return false;
    uintptr_t key = lock();
    bool ok = index < g.audit_count;
    if (ok)
        *e = g.audit[(g.audit_head + AUDIT_CAPACITY - g.audit_count + index) % AUDIT_CAPACITY];
    unlock(key);
    return ok;
}

void nav_coordinator_note(unsigned phase, char decoded) {
    uintptr_t key = lock();
    if (g.status.session_active && g.status.mode == MOTION_AUTO) {
        MotionEvent e = {.now_us = now(),
                         .owner = MOTION_BUDDY3,
                         .requester = MOTION_BUDDY3,
                         .type = MOTION_EVENT_NAV_STATE,
                         .kind = decoded ? 5 : 7,
                         .goal = decoded ? (float)decoded : (float)phase,
                         .target_mm_s = {g.nav_target[0], g.nav_target[1]}};
        emit(e);
    }
    unlock(key);
}

/* Every accepted assist is bounded, expires, and never changes the line owner. */
MotionResult nav_coordinator_ramp_assist(MotionOwner owner, const MotionRampAssist *a) {
    if (owner != MOTION_BUDDY4 || !a || a->boost_milli > 200 || a->push_pwm_x100 > 6000 ||
        a->hold_milli > 1000 || !a->ttl_ms || a->ttl_ms > 200)
        return MOTION_INVALID;
    uintptr_t key = lock();
    MotionResult r = MOTION_OK;
    bool requested = a->boost_milli || a->push_pwm_x100 || a->hold_milli;
    if (!requested)
        clear_assist(0);
    else if (!g.status.session_active)
        r = MOTION_INACTIVE;
    else if (g.status.mode != MOTION_AUTO || g.feedback.navigation_busy || !g.assist_eligible)
        r = MOTION_BUSY;
    else {
        bool was = assist_active();
        g.assist = *a;
        g.assist_deadline = now() + (uint64_t)a->ttl_ms * 1000;
        if (!was || now() - g.assist_log_us >= 100000)
            assist_event(1);
    }
    unlock(key);
    return r;
}
MotionRampAssist nav_coordinator_get_assist(void) {
    uintptr_t key = lock();
    if (assist_active() && (now() >= g.assist_deadline || !g.assist_eligible ||
                            g.status.mode != MOTION_AUTO || !g.status.session_active))
        clear_assist(3);
    MotionRampAssist a = g.assist;
    unlock(key);
    return a;
}
void nav_coordinator_assist_context(bool eligible) {
    uintptr_t key = lock();
    g.assist_eligible = eligible;
    if (!eligible)
        clear_assist(4);
    unlock(key);
}
MotionResult nav_coordinator_verified_barcode(MotionOwner owner, char code) {
    if (owner != MOTION_BUDDY3 || code < 'A' || code > 'Z')
        return MOTION_INVALID;
    uintptr_t key = lock();
    MotionResult r = !g.status.session_active                                     ? MOTION_INACTIVE
                     : g.status.mode != MOTION_AUTO || g.feedback.navigation_busy ? MOTION_BUSY
                                                                                  : MOTION_OK;
    if (r == MOTION_OK) {
        MotionEvent e = {.now_us = now(),
                         .owner = g.status.owner,
                         .requester = owner,
                         .type = MOTION_EVENT_ROUTE,
                         .kind = NAV_VERIFIED_BARCODE,
                         .goal = (float)code};
        emit(e);
    }
    unlock(key);
    return r;
}
