/**
 * @file nav_coordinator.c
 * @brief Nav coordinator.
 * @details The shared facade/coordinator arbitrates ownership and generations.
 * Acceptance is not completion; external callers never write motor registers.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#include "src/navigation/nav_coordinator.h"
#include "src/navigation/nav_codec.h"
#include "src/motion/motion_executor.h"
#include <math.h>
#include <string.h>
enum
{
    EVENT_CAPACITY = 32,
    RESULT_CAPACITY = 16,
    AUDIT_CAPACITY = 120
};
static struct
{
    motion_port_t port;
    motion_status_t status;
    motion_feedback_t feedback;
    motion_executor_t executor;
    motion_command_status_t results[RESULT_CAPACITY];
    motion_event_t events[EVENT_CAPACITY];
    motion_event_t audit[AUDIT_CAPACITY];
    unsigned event_head;
    unsigned event_count;
    unsigned result_head;
    unsigned audit_head;
    unsigned audit_count;
    unsigned audit_overwritten;
    uint32_t next_id;
    uint32_t sequence;
    unsigned reported_faults : 10;
    uint64_t pause_since;
    uint64_t still_since;
    uint64_t line_since;
    uint64_t nav_since;
    uint64_t target_log_since;
    bool still_timing;
    bool line_timing;
    bool dirty;
    bool resume;
    bool abandon;
    int16_t nav_target[2];
    char route;
    motion_ramp_assist_t assist;
    uint64_t assist_deadline;
    uint64_t assist_log_us;
    bool assist_eligible;
    unsigned speed_mode_requested : 2;
    unsigned speed_mode_result : 4;
    uint32_t speed_mode_revision;
#ifdef MOTION_STEERED_DRIVE
    uint64_t steer_us; /* last accepted heading setpoint */
#endif
} g_value;
static uintptr_t lock (void)
{
    return g_value.port.lock ? g_value.port.lock(g_value.port.user) : 0;
}
static void unlock (uintptr_t state)
{
    if (g_value.port.unlock)
    {
        g_value.port.unlock(g_value.port.user, state);
    }
}
static uint64_t now (void)
{
    return g_value.port.now_us ? g_value.port.now_us(g_value.port.user)
                               : g_value.feedback.now_us;
}
static bool owner_valid (motion_owner_t owner)
{
    return (owner >= MOTION_BUDDY1) && (owner <= MOTION_BUDDY5);
}
static void emit (motion_event_t event)
{
    event.sequence = ++g_value.sequence;
    g_value.events[g_value.event_head] = event;
    g_value.event_head = (g_value.event_head + 1) % EVENT_CAPACITY;
    if (g_value.event_count < EVENT_CAPACITY)
    {
        ++g_value.event_count;
    }
    g_value.audit[g_value.audit_head] = event;
    g_value.audit_head = (g_value.audit_head + 1) % AUDIT_CAPACITY;
    if (g_value.audit_count < AUDIT_CAPACITY)
    {
        ++g_value.audit_count;
    }
    else
    {
        ++g_value.audit_overwritten;
    }
}
static void event (unsigned type)
{
    motion_event_t event = {.now_us = now(),
                            .owner = g_value.status.owner,
                            .requester = g_value.status.owner,
                            .id = g_value.status.command_id,
                            .type = type,
                            .state = g_value.status.command_state,
                            .fault = g_value.status.fault,
                            .target_mm_s = {g_value.status.target_mm_s[0],
                                            g_value.status.target_mm_s[1]},
                            .goal = g_value.executor.goal,
                            .speed = (unsigned)g_value.executor.speed,
                            .style = g_value.executor.style,
                            .kind = g_value.executor.turning ? 3 : 2,
                            .progress_mm = g_value.status.progress_mm,
                            .rotation_deg = g_value.status.rotation_deg};
    if (MOTION_EVENT_PAUSE == type || MOTION_EVENT_STOPPED == type)
    {
        event.kind = 0;
        event.goal = 0;
        event.speed = 0;
    }
    if (MOTION_EVENT_RESUME == type)
    {
        event.kind = 4;
        event.goal = 0;
        event.speed = 0;
    }
    if (MOTION_EVENT_ROUTE == type)
    {
        event.kind = 5;
        event.requester = MOTION_BUDDY3;
        event.goal = g_value.route;
        event.speed = 0;
    }
    if (MOTION_EVENT_TERRAIN == type)
    {
        event.kind = 6;
        event.requester = MOTION_BUDDY4;
        event.goal = g_value.status.terrain;
        event.speed = 0;
    }
    emit(event);
}
void nav_coordinator_reject (motion_owner_t owner, unsigned kind, float goal,
                             unsigned speed, motion_result_t result)
{
    if (MOTION_OK == result)
    {
        return;
    }
    uintptr_t key = lock();
    motion_event_t event = {.now_us = now(),
                            .owner = g_value.status.owner,
                            .requester = owner_valid(owner) ? owner : 0,
                            .type = MOTION_EVENT_REJECT,
                            .kind = kind,
                            .goal = isfinite(goal) ? goal : 0,
                            .speed = speed <= 127 ? speed : 0,
                            .result = result,
                            .fault = g_value.status.fault,
                            .state = g_value.status.command_state};
    emit(event);
    unlock(key);
}
static bool assist_active (void)
{
    return ((g_value.assist.boost_milli) || (g_value.assist.push_pwm_x100)) ||
           (g_value.assist.hold_milli);
}
static void assist_event (unsigned reason)
{
    motion_event_t event = {.now_us = now(),
                            .requester = MOTION_BUDDY4,
                            .owner = g_value.status.owner,
                            .type = MOTION_EVENT_TARGETS,
                            .kind = NAV_RAMP_ASSIST,
                            .goal = (float)reason,
                            .target_mm_s = {g_value.status.target_mm_s[0],
                                            g_value.status.target_mm_s[1]}};
    emit(event);
    g_value.assist_log_us = now();
}
static void clear_assist (unsigned reason)
{
    if (assist_active())
    {
        g_value.assist = (motion_ramp_assist_t){0};
        g_value.assist_deadline = 0;
        assist_event(reason);
    }
}
static void off (void)
{
    clear_assist(2);
    g_value.assist_eligible = false;
    ++g_value.status.generation;
    g_value.dirty = true;
    g_value.status.target_mm_s[0] = g_value.status.target_mm_s[1] = 0;
    if (g_value.port.outputs_off)
    {
        g_value.port.outputs_off(g_value.port.user);
    }
}
static void remember (motion_command_state_t state)
{
    if (!g_value.status.command_id)
    {
        return;
    }
    g_value.status.command_state = state;
    g_value.results[g_value.result_head] = (motion_command_status_t){
        g_value.status.command_id, state, g_value.status.fault,
        g_value.status.progress_mm, g_value.status.rotation_deg};
    g_value.result_head = (g_value.result_head + 1) % RESULT_CAPACITY;
}
static void cancel (void)
{
    if (MOTION_CMD_RUNNING == g_value.status.command_state)
    {
        remember(MOTION_CMD_CANCELLED);
        event(MOTION_EVENT_CANCEL);
    }
    g_value.executor.active = false;
}
void nav_coordinator_init (const motion_port_t * p_port_value)
{
    memset(&g_value, 0, sizeof(g_value));
    if (p_port_value)
    {
        g_value.port = *p_port_value;
    }
    g_value.status.mode = MOTION_OFF;
    g_value.status.speed_mode = DRIVE_MODE_BASELINE45;
    g_value.speed_mode_requested = DRIVE_MODE_NONE;
    g_value.speed_mode_result = MOTION_OK;
}
void nav_coordinator_begin (uint64_t time)
{
    uintptr_t key = lock();
    cancel();
    uint32_t generation = g_value.status.generation + 1,
             lease = g_value.status.lease + 1;
    unsigned speed_mode = g_value.status.speed_mode;
    uint32_t speed_mode_revision = g_value.status.speed_mode_revision;
    g_value.status =
        (motion_status_t){.fault = g_value.reported_faults,
                          .session_active = true,
                          .mode = MOTION_AUTO,
                          .owner = MOTION_BUDDY3,
                          .generation = generation,
                          .lease = lease,
                          .speed_mode = speed_mode,
                          .speed_mode_revision = speed_mode_revision};
    g_value.feedback = (motion_feedback_t){.now_us = time};
    g_value.assist = (motion_ramp_assist_t){0};
    g_value.assist_deadline = g_value.assist_log_us = 0;
    g_value.assist_eligible = false;
    g_value.nav_since = time;
    g_value.nav_target[0] = g_value.nav_target[1] = 0;
    g_value.executor = (motion_executor_t){0};
    g_value.dirty = g_value.resume = g_value.abandon = g_value.still_timing =
        g_value.line_timing = false;
    g_value.route = 0;
    g_value.event_head = g_value.event_count = g_value.audit_head =
        g_value.audit_count = g_value.audit_overwritten = g_value.sequence = 0;
    g_value.target_log_since = 0;
    unlock(key);
}
void nav_coordinator_end (uint8_t reason)
{
    uintptr_t key = lock();
    cancel();
    g_value.status.session_active = false;
    g_value.status.mode = MOTION_OFF;
    g_value.status.stop_reason = reason <= 63 ? reason : 63;
    off();
    motion_event_t event = {.now_us = now(),
                            .owner = g_value.status.owner,
                            .requester = 0,
                            .type = MOTION_EVENT_SESSION_END,
                            .kind = 1,
                            .id = g_value.status.command_id,
                            .state = g_value.status.command_state,
                            .fault = g_value.status.fault,
                            .goal = reason};
    emit(event);
    unlock(key);
}
motion_result_t nav_coordinator_pause (motion_owner_t owner,
                                       uint32_t * p_lease_value)
{
    if ((!owner_valid(owner)) || (!p_lease_value))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    motion_result_t result = MOTION_OK;
    if (!g_value.status.session_active)
    {
        result = MOTION_INACTIVE;
    }
    else if (((g_value.status.mode != MOTION_AUTO) &&
              (g_value.status.owner != owner)) &&
             (owner != MOTION_BUDDY5))
    {
        result = MOTION_BUSY;
    }
    else if ((g_value.status.owner == owner) &&
             (MOTION_PAUSED == g_value.status.mode ||
              MOTION_STOPPING == g_value.status.mode))
    {
        *p_lease_value = g_value.status.lease;
    }
    else
    {
        cancel();
        g_value.resume = g_value.abandon = false;
        g_value.status.navigation_interrupted |=
            g_value.feedback.navigation_busy;
        g_value.status.owner = owner;
        ++g_value.status.lease;
        *p_lease_value = g_value.status.lease;
        g_value.status.mode = MOTION_STOPPING;
        g_value.status.fault = g_value.reported_faults;
        g_value.status.stopped = false;
        g_value.pause_since = now();
        g_value.still_timing = false;
        off();
        event(MOTION_EVENT_PAUSE);
    }
    unlock(key);
    return result;
}
motion_result_t nav_coordinator_stop (motion_owner_t owner)
{
    if (!owner_valid(owner))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    if (!g_value.status.session_active)
    {
        unlock(key);
        return MOTION_INACTIVE;
    }
    if (MOTION_AUTO == g_value.status.mode)
    {
        g_value.status.owner = owner;
        ++g_value.status.lease;
    }
    cancel();
    g_value.resume = g_value.abandon = false;
    g_value.status.navigation_interrupted |= g_value.feedback.navigation_busy;
    g_value.status.mode = MOTION_STOPPING;
    g_value.status.stopped = false;
    g_value.pause_since = now();
    g_value.still_timing = false;
    off();
    event(MOTION_EVENT_PAUSE);
    g_value.audit[(g_value.audit_head + AUDIT_CAPACITY - 1) % AUDIT_CAPACITY]
        .requester = owner;
    g_value.events[(g_value.event_head + EVENT_CAPACITY - 1) % EVENT_CAPACITY]
        .requester = owner;
    g_value.audit[(g_value.audit_head + AUDIT_CAPACITY - 1) % AUDIT_CAPACITY]
        .kind = 1;
    g_value.events[(g_value.event_head + EVENT_CAPACITY - 1) % EVENT_CAPACITY]
        .kind = 1;
    unlock(key);
    return MOTION_OK;
}
static motion_result_t access (motion_owner_t owner, uint32_t lease)
{
    if (!g_value.status.session_active)
    {
        return MOTION_INACTIVE;
    }
    if (((owner != g_value.status.owner) || (lease != g_value.status.lease)) ||
        (!lease))
    {
        return MOTION_NOT_OWNER;
    }
    return MOTION_OK;
}
motion_result_t nav_coordinator_move (motion_owner_t owner, uint32_t lease,
                                      float goal, unsigned speed, bool turn,
                                      motion_turn_style_t style,
                                      uint32_t * p_identifier_value)
{
    if (p_identifier_value)
    {
        *p_identifier_value = 0;
    }
    if (((((((!owner_valid(owner)) || (!p_identifier_value)) ||
            (!isfinite(goal))) ||
           (fabsf(goal) < 1)) ||
          (fabsf(goal) > (turn ? 180 : 1000))) ||
         (speed && ((speed < 20) || (speed > (turn ? 90u : 60u))))) ||
        ((turn && (style != MOTION_ROLLING)) && (style != MOTION_IN_PLACE)))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    motion_result_t result = access(owner, lease);
    if (MOTION_OK == result &&
        ((g_value.status.mode != MOTION_PAUSED) || (!g_value.status.stopped)))
    {
        result = MOTION_NOT_STOPPED;
    }
    if (MOTION_OK == result && (now() - g_value.feedback.now_us > 100000))
    {
        result = MOTION_NOT_STOPPED;
    }
    if (MOTION_OK == result)
    {
        if (!speed)
        {
            speed = 45;
        }
        g_value.status.command_id = ++g_value.next_id;
        if (!g_value.status.command_id)
        {
            g_value.status.command_id = ++g_value.next_id;
        }
        *p_identifier_value = g_value.status.command_id;
        g_value.status.command_state = MOTION_CMD_RUNNING;
        g_value.status.progress_mm = g_value.status.rotation_deg = 0;
        g_value.status.fault = g_value.reported_faults;
        g_value.status.stopped = false;
        g_value.status.mode = turn ? MOTION_TURNING : MOTION_DRIVING;
        motion_executor_start(&g_value.executor, &g_value.feedback, turn, goal,
                              speed, style);
        ++g_value.status.generation;
        g_value.dirty = true;
        event(MOTION_EVENT_START);
    }
    unlock(key);
    return result;
}
motion_result_t nav_coordinator_resume (motion_owner_t owner, uint32_t lease,
                                        bool abandon)
{
    uintptr_t key = lock();
    motion_result_t result = access(owner, lease);
    if (MOTION_OK == result &&
        ((g_value.status.mode != MOTION_PAUSED) || (!g_value.status.stopped)))
    {
        result = MOTION_NOT_STOPPED;
    }
    if (MOTION_OK == result && (g_value.status.navigation_interrupted) &&
        (!abandon))
    {
        result = MOTION_NEED_RECOVERY;
    }
    if (MOTION_OK == result &&
        (((!g_value.line_timing) || (now() - g_value.line_since < 150000)) ||
         (now() - g_value.feedback.now_us > 100000)))
    {
        result = MOTION_NEED_LINE;
    }
    if (MOTION_OK == result)
    {
        g_value.resume = true;
        g_value.abandon = abandon;
        g_value.status.owner = MOTION_BUDDY3;
        ++g_value.status.lease;
        off();
        event(MOTION_EVENT_RESUME);
        g_value.status.navigation_interrupted = false;
        g_value
            .audit[(g_value.audit_head + AUDIT_CAPACITY - 1) % AUDIT_CAPACITY]
            .requester = owner;
        g_value
            .events[(g_value.event_head + EVENT_CAPACITY - 1) % EVENT_CAPACITY]
            .requester = owner;
        g_value.nav_target[0] = g_value.nav_target[1] = 0;
        g_value.nav_since = now();
    }
    unlock(key);
    return result;
}
#ifdef MOTION_STEERED_DRIVE
/* Lease holder only, during its own straight drive (not settling, not a
 * turn): bounded heading setpoint relative to the drive's start heading.
 * Never starts, extends or resumes motion; goal, speed, timeout, settle and
 * the 12 deg heading fault are unchanged. */
motion_result_t nav_coordinator_steer (motion_owner_t owner, uint32_t lease,
                                       float degrees)
{
    if ((!isfinite(degrees)) ||
        (fabsf(degrees) * 10.0f > (float)NAV_STEER_MAX_DECIDEG + 0.5f))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    motion_result_t result = access(owner, lease);
    if (MOTION_OK == result &&
        ((MOTION_DRIVING != g_value.status.mode) ||
         (!g_value.executor.active) || (g_value.executor.turning) ||
         (g_value.executor.settling)))
    {
        result = MOTION_BUSY;
    }
    if (MOTION_OK == result)
    {
        g_value.executor.steer_mm =
            degrees * 110.0f * 3.14159265358979323846f / 180.0f;
        g_value.steer_us = now();
    }
    unlock(key);
    return result;
}
#endif
motion_result_t nav_coordinator_navigation (motion_owner_t owner, int16_t left,
                                            int16_t right)
{
    if (((((owner != MOTION_BUDDY3) || (left < -180)) || (right < -180)) ||
         (left > 180)) ||
        (right > 180))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    motion_result_t result = !g_value.status.session_active ? MOTION_INACTIVE
                             : g_value.status.mode != MOTION_AUTO ? MOTION_BUSY
                                                                  : MOTION_OK;
    if (MOTION_OK == result)
    {
        g_value.nav_target[0] = left;
        g_value.nav_target[1] = right;
        g_value.nav_since = now();
        if ((!g_value.target_log_since) ||
            (now() - g_value.target_log_since >= 1000000))
        {
            motion_event_t event = {.now_us = now(),
                                    .owner = MOTION_BUDDY3,
                                    .requester = MOTION_BUDDY3,
                                    .type = MOTION_EVENT_TARGETS,
                                    .kind = 7,
                                    .target_mm_s = {left, right}};
            emit(event);
            g_value.target_log_since = now();
        }
    }
    unlock(key);
    return result;
}
motion_result_t nav_coordinator_route (motion_owner_t owner, char code)
{
#ifdef BUDDY_RUNTIME_MAPPING
    if (((owner != MOTION_BUDDY3) || (code < 'A')) || (code > 'Z'))
    {
        return MOTION_INVALID;
    }
#else
    if (owner != MOTION_BUDDY3 || (code != 'A' && code != 'F' && code != 'Z'))
    {
        return MOTION_INVALID;
    }
#endif
    uintptr_t key = lock();
    motion_result_t result = !g_value.status.session_active ? MOTION_INACTIVE
                             : (g_value.status.mode != MOTION_AUTO) ||
                                     (g_value.feedback.navigation_busy)
                                 ? MOTION_BUSY
                                 : MOTION_OK;
    if (MOTION_OK == result)
    {
        g_value.route = code;
        event(MOTION_EVENT_ROUTE);
    }
    unlock(key);
    return result;
}
motion_result_t nav_coordinator_terrain (motion_owner_t owner,
                                         motion_terrain_t terrain)
{
    if ((owner != MOTION_BUDDY4) ||
        ((unsigned)terrain > (unsigned)MOTION_DESCENDING))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    g_value.status.terrain = terrain;
    event(MOTION_EVENT_TERRAIN);
    unlock(key);
    return MOTION_OK;
}
motion_status_t nav_coordinator_status (void)
{
    uintptr_t key = lock();
    motion_status_t state = g_value.status;
    unlock(key);
    return state;
}
motion_result_t nav_coordinator_speed_mode (motion_owner_t owner, unsigned mode)
{
    uintptr_t key = lock();
    g_value.speed_mode_requested =
        drive_mode_valid(mode) ? mode : DRIVE_MODE_NONE;
    motion_result_t result = MOTION_OK;
    if ((owner != MOTION_BUDDY1) || (!drive_mode_valid(mode)))
    {
        result = MOTION_INVALID;
    }
    else if (((g_value.status.session_active) ||
              (g_value.status.mode != MOTION_OFF)) ||
             (!g_value.status.stopped))
    {
        result = MOTION_NOT_STOPPED;
    }
    else
    {
        ++g_value.speed_mode_revision;
        g_value.status.speed_mode = mode;
        g_value.status.speed_mode_revision = g_value.speed_mode_revision;
    }
    g_value.speed_mode_result = result;
    unlock(key);
    return result;
}
motion_speed_mode_status_t nav_coordinator_speed_mode_status (void)
{
    uintptr_t key = lock();
    motion_speed_mode_status_t state = {
        .mode = g_value.status.speed_mode,
        .requested_mode = g_value.speed_mode_requested,
        .revision = g_value.speed_mode_revision,
        .last_result = g_value.speed_mode_result,
        .session_active = g_value.status.session_active,
        .stopped = g_value.status.stopped,
        .motion_mode = g_value.status.mode};
    unlock(key);
    return state;
}
bool nav_coordinator_command (uint32_t identifier_value,
                              motion_command_status_t * p_status_value)
{
    if ((!identifier_value) || (!p_status_value))
    {
        return false;
    }
    uintptr_t key = lock();
    bool found = false;
    if (identifier_value == g_value.status.command_id)
    {
        *p_status_value = (motion_command_status_t){
            identifier_value, g_value.status.command_state,
            g_value.status.fault, g_value.status.progress_mm,
            g_value.status.rotation_deg};
        found = true;
    }
    else
    {
        for (unsigned count = 0; count < RESULT_CAPACITY; ++count)
        {
            if (g_value.results[count].id == identifier_value)
            {
                *p_status_value = g_value.results[count];
                found = true;
                break;
            }
        }
    }
    unlock(key);
    return found;
}
motion_output_t nav_coordinator_update (const motion_feedback_t * p_flags)
{
    uintptr_t key = lock();
    g_value.feedback = *p_flags;
    motion_output_t output = {0};
    /* Requests use the live port clock and can be newer than this tick's
     * feedback snapshot. Compare request ages on that same live clock;
     * unsigned snapshot-request subtraction would underflow at startup. */
    uint64_t check_us = now();
    bool still = (fabsf(p_flags->speed_mm_s[0]) < 5) &&
                 (fabsf(p_flags->speed_mm_s[1]) < 5);
    if (!still)
    {
        g_value.still_timing = false;
    }
    else if (!g_value.still_timing)
    {
        g_value.still_timing = true;
        g_value.still_since = p_flags->now_us;
    }
    g_value.status.stopped = (g_value.still_timing) &&
                             (p_flags->now_us - g_value.still_since >= 120000);
    bool line =
        (p_flags->line_level[0] >= 400) || (p_flags->line_level[1] >= 400);
    if (!line)
    {
        g_value.line_timing = false;
    }
    else if (!g_value.line_timing)
    {
        g_value.line_timing = true;
        g_value.line_since = p_flags->now_us;
    }
    g_value.status.pending_route = p_flags->pending_route;
    if (g_value.status.session_active)
    {
        if (g_value.resume)
        {
            g_value.status.mode =
                MOTION_AUTO; /* adapter resets old navigation before the next
                                optical tick */
        }
        if (MOTION_AUTO == g_value.status.mode &&
            (check_us - g_value.nav_since > 100000))
        {
            g_value.status.mode = MOTION_FAULT;
            g_value.status.fault = g_value.reported_faults | NAV_FAULT_CONTROL;
            off();
            event(MOTION_EVENT_FAULT);
        }
        else if (MOTION_STOPPING == g_value.status.mode)
        {
            g_value.status.navigation_interrupted |= p_flags->navigation_busy;
            if (g_value.status.stopped)
            {
                g_value.status.mode = MOTION_PAUSED;
                event(MOTION_EVENT_STOPPED);
            }
            else if (check_us - g_value.pause_since >= 2000000)
            {
                g_value.status.mode = MOTION_FAULT;
                g_value.status.fault =
                    g_value.reported_faults | NAV_FAULT_SETTLE;
                off();
                event(MOTION_EVENT_FAULT);
            }
        }
        else if (MOTION_DRIVING == g_value.status.mode ||
                 MOTION_TURNING == g_value.status.mode ||
                 MOTION_SETTLING == g_value.status.mode)
        {
#ifdef MOTION_STEERED_DRIVE
            /* A setpoint not refreshed within 100 ms lapses to the original
             * straight hold, so a silent owner cannot keep steering. */
            if ((g_value.executor.steer_mm != 0.0f) &&
                ((p_flags->now_us < g_value.steer_us) ||
                 (p_flags->now_us - g_value.steer_us > 100000U)))
            {
                g_value.executor.steer_mm = 0.0f;
            }
#endif
            motion_command_state_t state =
                motion_executor_step(&g_value.executor, p_flags);
            g_value.status.progress_mm = g_value.executor.progress_mm;
            g_value.status.rotation_deg = g_value.executor.rotation_deg;
            if (state != MOTION_CMD_RUNNING)
            {
                g_value.status.fault =
                    g_value.reported_faults | g_value.executor.fault;
                remember(state);
                off();
                g_value.status.mode =
                    MOTION_CMD_DONE == state ? MOTION_PAUSED : MOTION_FAULT;
                event(MOTION_CMD_DONE == state ? MOTION_EVENT_DONE
                                               : MOTION_EVENT_FAULT);
            }
            else
            {
                g_value.status.mode =
                    g_value.executor.settling  ? MOTION_SETTLING
                    : g_value.executor.turning ? MOTION_TURNING
                                               : MOTION_DRIVING;
            }
        }
    }
    output.external = (!g_value.status.session_active) ||
                      (g_value.status.mode != MOTION_AUTO);
    if ((g_value.status.session_active) && MOTION_AUTO == g_value.status.mode)
    {
        output.target_mm_s[0] = g_value.nav_target[0];
        output.target_mm_s[1] = g_value.nav_target[1];
    }
    else if (MOTION_DRIVING == g_value.status.mode ||
             MOTION_TURNING == g_value.status.mode)
    {
        output.target_mm_s[0] = g_value.executor.target[0];
        output.target_mm_s[1] = g_value.executor.target[1];
        output.turning = g_value.executor.turning;
    }
    memcpy(g_value.status.target_mm_s, output.target_mm_s,
           sizeof(g_value.status.target_mm_s));
    output.reset_controllers = g_value.dirty;
    g_value.dirty = false;
    output.resume_navigation = g_value.resume;
    output.abandon_navigation = g_value.abandon;
    g_value.resume = g_value.abandon = false;
    output.route = g_value.route;
    g_value.route = 0;
    output.generation = g_value.status.generation;
    unlock(key);
    return output;
}
bool nav_coordinator_commit (uint32_t epoch, void (*p_apply_value)(void *),
                             void * p_user_value)
{
    uintptr_t key = lock();
    bool ok_value = ((g_value.status.session_active) &&
                     (g_value.status.mode != MOTION_FAULT)) &&
                    (epoch == g_value.status.generation);
    if (ok_value && p_apply_value)
    {
        p_apply_value(p_user_value);
    }
    unlock(key);
    return ok_value;
}
bool nav_coordinator_pop (motion_event_t * p_out)
{
    if (!p_out)
    {
        return false;
    }
    uintptr_t key = lock();
    bool ok_value = g_value.event_count != 0;
    if (ok_value)
    {
        *p_out = g_value.events[(g_value.event_head + EVENT_CAPACITY -
                                 g_value.event_count) %
                                EVENT_CAPACITY];
        --g_value.event_count;
    }
    unlock(key);
    return ok_value;
}

motion_result_t nav_coordinator_fault (motion_owner_t owner, unsigned bitmap,
                                       bool active)
{
    if (((!owner_valid(owner)) || (!bitmap)) || (bitmap & ~NAV_FAULT_MASK))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    if (active)
    {
        g_value.reported_faults |= bitmap;
    }
    else
    {
        g_value.reported_faults &= ~bitmap;
    }
    g_value.status.fault = g_value.reported_faults | g_value.executor.fault;
    motion_event_t event = {.now_us = now(),
                            .requester = owner,
                            .owner = g_value.status.owner,
                            .type = MOTION_EVENT_FAULT_REPORT,
                            .kind = 9,
                            .fault = g_value.status.fault,
                            .goal = (float)bitmap,
                            .state = active ? 1 : 0};
    emit(event);
    unlock(key);
    return MOTION_OK;
}
unsigned nav_coordinator_log_count (void)
{
    uintptr_t key = lock();
    unsigned count = g_value.audit_count;
    unlock(key);
    return count;
}
unsigned nav_coordinator_log_overwritten (void)
{
    uintptr_t key = lock();
    unsigned count = g_value.audit_overwritten;
    unlock(key);
    return count;
}
bool nav_coordinator_log_read (unsigned index, motion_event_t * p_event_value)
{
    if (!p_event_value)
    {
        return false;
    }
    uintptr_t key = lock();
    bool ok_value = index < g_value.audit_count;
    if (ok_value)
    {
        *p_event_value = g_value.audit[(g_value.audit_head + AUDIT_CAPACITY -
                                        g_value.audit_count + index) %
                                       AUDIT_CAPACITY];
    }
    unlock(key);
    return ok_value;
}

void nav_coordinator_note (unsigned phase, char decoded)
{
    uintptr_t key = lock();
    if ((g_value.status.session_active) && MOTION_AUTO == g_value.status.mode)
    {
        motion_event_t event = {
            .now_us = now(),
            .owner = MOTION_BUDDY3,
            .requester = MOTION_BUDDY3,
            .type = MOTION_EVENT_NAV_STATE,
            .kind = decoded ? 5 : 7,
            .goal = decoded ? (float)decoded : (float)phase,
            .target_mm_s = {g_value.nav_target[0], g_value.nav_target[1]}};
        emit(event);
    }
    unlock(key);
}

/* Every accepted assist is bounded, expires, and never changes the line owner.
 */
motion_result_t
nav_coordinator_ramp_assist (motion_owner_t owner,
                             const motion_ramp_assist_t * p_first)
{
    if (((((((owner != MOTION_BUDDY4) || (!p_first)) ||
            (p_first->boost_milli > 200)) ||
           (p_first->push_pwm_x100 > 6000)) ||
          (p_first->hold_milli > 1000)) ||
         (!p_first->ttl_ms)) ||
        (p_first->ttl_ms > 200))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    motion_result_t result = MOTION_OK;
    bool requested = ((p_first->boost_milli) || (p_first->push_pwm_x100)) ||
                     (p_first->hold_milli);
    if (!requested)
    {
        clear_assist(0);
    }
    else if (!g_value.status.session_active)
    {
        result = MOTION_INACTIVE;
    }
    else if (((g_value.status.mode != MOTION_AUTO) ||
              (g_value.feedback.navigation_busy)) ||
             (!g_value.assist_eligible))
    {
        result = MOTION_BUSY;
    }
    else
    {
        bool was = assist_active();
        g_value.assist = *p_first;
        g_value.assist_deadline = now() + (uint64_t)p_first->ttl_ms * 1000;
        if ((!was) || (now() - g_value.assist_log_us >= 100000))
        {
            assist_event(1);
        }
    }
    unlock(key);
    return result;
}
motion_ramp_assist_t nav_coordinator_get_assist (void)
{
    uintptr_t key = lock();
    if ((assist_active()) &&
        ((((now() >= g_value.assist_deadline) || (!g_value.assist_eligible)) ||
          (g_value.status.mode != MOTION_AUTO)) ||
         (!g_value.status.session_active)))
    {
        clear_assist(3);
    }
    motion_ramp_assist_t first = g_value.assist;
    unlock(key);
    return first;
}
void nav_coordinator_assist_context (bool eligible)
{
    uintptr_t key = lock();
    g_value.assist_eligible = eligible;
    if (!eligible)
    {
        clear_assist(4);
    }
    unlock(key);
}
motion_result_t nav_coordinator_verified_barcode (motion_owner_t owner,
                                                  char code)
{
    if (((owner != MOTION_BUDDY3) || (code < 'A')) || (code > 'Z'))
    {
        return MOTION_INVALID;
    }
    uintptr_t key = lock();
    motion_result_t result = !g_value.status.session_active ? MOTION_INACTIVE
                             : (g_value.status.mode != MOTION_AUTO) ||
                                     (g_value.feedback.navigation_busy)
                                 ? MOTION_BUSY
                                 : MOTION_OK;
    if (MOTION_OK == result)
    {
        motion_event_t event = {.now_us = now(),
                                .owner = g_value.status.owner,
                                .requester = owner,
                                .type = MOTION_EVENT_ROUTE,
                                .kind = NAV_VERIFIED_BARCODE,
                                .goal = (float)code};
        emit(event);
    }
    unlock(key);
    return result;
}

/* End of file. */
