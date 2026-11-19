#include "extensions/buddy5_obstacles/manual_bypass.h"
#include <string.h>
#include <math.h>

#define ARM_LIFETIME_US UINT64_C(30000000)
#define RUN_LIMIT_US UINT64_C(120000000)
/* Deliberately explicit provisional geometry for centred obstacles <=200 mm
 * wide/deep. Back off before rotating; do not equate the scan's 150 mm range
 * with corner clearance. Validate first on a clear floor, then on the car. */
#define BACKOFF_MM 150.0f
#define SIDE_SHIFT_MM 330.0f
#define STRAIGHT_PASS_MM 400.0f
#define DIAGONAL_MM (SIDE_SHIFT_MM * 1.4142135623730951f)

static void build_route(B5Manual *j, bool left, float lateral) {
    float sign = left ? 1.0f : -1.0f;
    B5ManualStep route[B5M_MAX_STEPS] = {
        {.goal = -BACKOFF_MM}, {.turn = 1, .goal = sign * 45.0f},
        {.goal = lateral * 1.4142135623730951f}, {.turn = 1, .goal = -sign * 45.0f},
        {.goal = STRAIGHT_PASS_MM}, {.turn = 1, .goal = -sign * 45.0f},
        {.goal = lateral * 1.4142135623730951f}, {.turn = 1, .goal = sign * 45.0f}};
    memcpy(j->plan, route, sizeof(route));
    j->count = B5M_MAX_STEPS;
}

static void note(B5Manual *j, unsigned event, unsigned detail, MotionResult result,
                 uint64_t now, const MotionCommandStatus *command) {
    if (j->event_count == B5M_EVENTS) {
        j->events_dropped = true;
        return;
    }
    B5ManualEvent *e = &j->events[j->event_count++];
    *e = (B5ManualEvent){.time_us = now, .command_id = j->command_id,
                        .event = event, .step = j->index, .detail = detail,
                        .result = result,
                        .goal = j->index < j->count ? j->plan[j->index].goal : 0};
    if (command) {
        e->progress_mm = command->progress_mm;
        e->rotation_deg = command->rotation_deg;
    }
}
static bool running(const B5Manual *j) {
    return j->state == B5M_TAKEOVER || j->state == B5M_MOVING || j->state == B5M_NEXT;
}
static void close_leg(B5Manual *j, unsigned outcome, unsigned reason) {
    if (j->index >= j->count || j->index >= B5M_MAX_STEPS) return;
    B5ManualLeg *l = &j->leg[j->index];
    if (l->outcome != B5LEG_PENDING && l->outcome != B5LEG_RUNNING) return;
    MotionCommandStatus c;
    if (l->outcome == B5LEG_RUNNING && j->command_id && nav_get_command(j->command_id, &c)) {
        l->measured_mm = c.progress_mm;
        l->measured_deg = c.rotation_deg;
    }
    l->outcome = (uint8_t)outcome;
    l->reason = (uint8_t)reason;
}
void b5_manual_mark_interrupted(B5Manual *j, unsigned reason) {
    close_leg(j, B5LEG_INTERRUPTED, reason);
}
const char *b5_manual_leg_outcome_name(unsigned o) {
    static const char *const names[] = {"PENDING", "RUNNING", "DONE", "SKIPPED_<1DEG",
        "FAILED", "INTERRUPTED"};
    return o < sizeof(names)/sizeof(names[0]) ? names[o] : "INVALID";
}
static void fail(B5Manual *j, unsigned why, MotionResult result, uint64_t now) {
    MotionStatus s = nav_get_status();
    /* A coordinator fault has already turned output off; preserve its mode. */
    if (s.session_active && s.mode != MOTION_FAULT)
        (void)nav_request_stop(MOTION_BUDDY5);
    close_leg(j, B5LEG_FAILED, why);
    j->state = B5M_FAILED;
    j->reason = why;
    note(j, B5M_ERROR, why, result, now, NULL);
}
void b5_manual_init(B5Manual *j) {
    memset(j, 0, sizeof(*j));
}
bool b5_manual_arm(B5Manual *j, unsigned kind, uint64_t now) {
    if (kind > B5M_BYPASS_RIGHT || nav_get_status().session_active || j->state == B5M_ARMED ||
        running(j))
        return false;
    b5_manual_init(j);
    j->state = B5M_ARMED;
    j->kind = kind;
    j->armed_us = now;
    if (kind <= B5M_RIGHT) {
        j->count = 1;
        j->plan[0] = (B5ManualStep){.turn = kind >= B5M_LEFT,
            .goal = kind == B5M_FORWARD ? 100.0f : kind == B5M_REVERSE ? -50.0f :
                    kind == B5M_LEFT ? 45.0f : -45.0f};
    } else {
        build_route(j, kind == B5M_BYPASS_LEFT, SIDE_SHIFT_MM);
    }
    note(j, B5M_ARM, 0, MOTION_OK, now, NULL);
    return true;
}
bool b5_manual_adopt(B5Manual *j, uint32_t lease, bool left, float lateral, uint64_t now) {
    MotionStatus s = nav_get_status();
    if (!isfinite(lateral) || lateral < SIDE_SHIFT_MM || lateral > 500 ||
        !s.session_active || s.fault || s.owner != MOTION_BUDDY5 || s.lease != lease ||
        s.mode != MOTION_PAUSED || !s.stopped || running(j))
        return false;
    b5_manual_init(j);
    j->kind = left ? B5M_BYPASS_LEFT : B5M_BYPASS_RIGHT;
    build_route(j, left, lateral);
    j->lease = lease;
    j->began_us = now;
    j->expected_generation = s.generation;
    j->state = B5M_TAKEOVER;
    note(j, B5M_CLAIM, 0, MOTION_OK, now, NULL);
    return true;
}
bool b5_manual_adopt_rect(B5Manual *j, uint32_t lease, bool left, float backoff,
                          float side, float forward, bool geometry_only, uint64_t now) {
    MotionStatus s = nav_get_status();
    if (!isfinite(backoff) || !isfinite(side) || !isfinite(forward) || backoff < 0 ||
        backoff > 150 || side < 1 || side > 1000 || forward < 1 || forward > 1000 ||
        !s.session_active || s.fault || s.owner != MOTION_BUDDY5 || s.lease != lease ||
        s.mode != MOTION_PAUSED || !s.stopped || running(j))
        return false;
    b5_manual_init(j);
    float sign = left ? 1.0f : -1.0f;
    unsigned n = 0;
    if (backoff >= 1)
        j->plan[n++] = (B5ManualStep){.goal = -backoff};
    j->plan[n++] = (B5ManualStep){.turn = 1, .absolute = 1, .goal = sign * 90.0f};
    j->plan[n++] = (B5ManualStep){.goal = side, .fast = 1};
    j->plan[n++] = (B5ManualStep){.turn = 1, .absolute = 1, .goal = 0.0f};
    j->plan[n++] = (B5ManualStep){.goal = forward, .fast = 1};
    j->plan[n++] = (B5ManualStep){.turn = 1, .absolute = 1, .goal = -sign * 90.0f};
    if (geometry_only) {
        j->plan[n++] = (B5ManualStep){.goal = side};
        j->plan[n++] = (B5ManualStep){.turn = 1, .absolute = 1, .goal = 0.0f};
    }
    j->count = n;
    j->kind = left ? B5M_BYPASS_LEFT : B5M_BYPASS_RIGHT;
    j->lease = lease;
    j->began_us = now;
    j->expected_generation = s.generation;
    j->state = B5M_TAKEOVER;
    note(j, B5M_CLAIM, 0, MOTION_OK, now, NULL);
    return true;
}
void b5_manual_cancel(B5Manual *j, uint64_t now) {
    if (j->state != B5M_ARMED && !running(j))
        return;
    MotionStatus s = nav_get_status();
    if (s.session_active && s.mode != MOTION_FAULT)
        (void)nav_request_stop(MOTION_BUDDY5);
    close_leg(j, B5LEG_FAILED, B5M_INTERRUPTED);
    j->state = B5M_CANCELLED;
    j->reason = B5M_INTERRUPTED;
    note(j, B5M_CANCEL, B5M_INTERRUPTED, MOTION_INACTIVE, now, NULL);
}
void b5_manual_poll(B5Manual *j, uint64_t now) {
    if (j->state == B5M_ARMED) {
        if (now < j->armed_us || now - j->armed_us >= ARM_LIFETIME_US) {
            j->state = B5M_CANCELLED;
            j->reason = B5M_EXPIRED;
            note(j, B5M_CANCEL, B5M_EXPIRED, MOTION_INACTIVE, now, NULL);
            return;
        }
        if (!nav_get_status().session_active)
            return;
        MotionResult r = nav_request_takeover(MOTION_BUDDY5, &j->lease);
        if (r != MOTION_OK) {
            fail(j, B5M_BAD_REQUEST, r, now);
            return;
        }
        j->began_us = now;
        j->expected_generation = nav_get_status().generation;
        j->state = B5M_TAKEOVER;
        note(j, B5M_CLAIM, 0, MOTION_OK, now, NULL);
        return;
    }
    if (!running(j))
        return;
    MotionStatus s = nav_get_status();
    if (!s.session_active) {
        b5_manual_cancel(j, now);
        return;
    }
    if (s.mode == MOTION_FAULT || s.fault) {
        fail(j, B5M_COORDINATOR_FAULT, MOTION_BUSY, now);
        return;
    }
    if (s.owner != MOTION_BUDDY5 || s.lease != j->lease) {
        fail(j, B5M_OWNER_LOST, MOTION_NOT_OWNER, now);
        return;
    }
    if (now < j->began_us || now - j->began_us >= RUN_LIMIT_US) {
        fail(j, B5M_TIMED_OUT, MOTION_BUSY, now);
        return;
    }
    if (j->state == B5M_MOVING) {
        MotionCommandStatus command;
        if (!nav_get_command(j->command_id, &command)) {
            fail(j, B5M_COMMAND_LOST, MOTION_INVALID, now);
            return;
        }
        if (command.state == MOTION_CMD_DONE) {
            /* Normal completion turns outputs off once. Any extra Stop, Pause
             * or takeover invalidation must not become the next route leg. */
            if (s.generation != j->expected_generation + 1U ||
                s.mode != MOTION_PAUSED || !s.stopped) {
                fail(j, B5M_INTERRUPTED, MOTION_BUSY, now);
                return;
            }
            note(j, B5M_FINISH, 0, MOTION_OK, now, &command);
            j->leg[j->index] = (B5ManualLeg){.issued = j->issued[j->index],
                .measured_mm = command.progress_mm, .measured_deg = command.rotation_deg,
                .outcome = B5LEG_DONE};
            j->heading_deg += command.rotation_deg;
            j->expected_generation = s.generation;
            ++j->index;
            if (j->index == j->count) {
                j->state = B5M_DONE;
                note(j, B5M_HOLD, 0, MOTION_OK, now, &command);
            } else
                j->state = B5M_NEXT;
        } else if (command.state != MOTION_CMD_RUNNING ||
                   s.generation != j->expected_generation) {
            fail(j, B5M_INTERRUPTED, MOTION_BUSY, now);
        }
        return;
    }
    if (s.generation != j->expected_generation) {
        fail(j, B5M_INTERRUPTED, MOTION_BUSY, now);
        return;
    }
    if (s.mode != MOTION_PAUSED || !s.stopped)
        return;
    B5ManualStep *step = &j->plan[j->index];
    float goal = step->goal;
    if (step->absolute) {
        goal = step->goal - j->heading_deg;
        j->issued[j->index] = goal;
        if (!isfinite(goal) || fabsf(goal) > 180) {
            fail(j, B5M_BAD_REQUEST, MOTION_INVALID, now);
            return;
        }
        if (fabsf(goal) < 1.0f) { /* below the API minimum: already on heading */
            note(j, B5M_FINISH, 0, MOTION_OK, now, NULL);
            j->leg[j->index] = (B5ManualLeg){.issued = goal, .outcome = B5LEG_SKIPPED};
            if (++j->index == j->count) {
                j->state = B5M_DONE;
                note(j, B5M_HOLD, 0, MOTION_OK, now, NULL);
            } else
                j->state = B5M_NEXT;
            return;
        }
    } else
        j->issued[j->index] = goal;
    MotionResult r = step->turn
        ? nav_turn_relative(MOTION_BUDDY5, j->lease, goal, B5M_SPEED,
                            MOTION_IN_PLACE, &j->command_id)
        : nav_drive_distance(MOTION_BUDDY5, j->lease, goal,
                             step->fast && goal > 0 ? B5M_STRAIGHT_SPEED : B5M_SPEED,
                             &j->command_id);
    if (r != MOTION_OK) {
        fail(j, B5M_BAD_REQUEST, r, now);
        return;
    }
    j->expected_generation = nav_get_status().generation;
    j->state = B5M_MOVING;
    j->leg[j->index] = (B5ManualLeg){.issued = goal, .outcome = B5LEG_RUNNING};
    note(j, B5M_START, 0, MOTION_OK, now, NULL);
}
MotionResult b5_manual_resume(B5Manual *j, uint64_t now) {
    if (j->state != B5M_DONE || j->kind < B5M_BYPASS_LEFT)
        return MOTION_INVALID;
    MotionStatus s = nav_get_status();
    if (s.owner != MOTION_BUDDY5 || s.lease != j->lease)
        return MOTION_NOT_OWNER;
    if (s.generation != j->expected_generation)
        return MOTION_BUSY;
    /* Do not abandon an interrupted junction implicitly. */
    MotionResult r = nav_resume_line(MOTION_BUDDY5, j->lease, false);
    note(j, B5M_RESUME, 0, r, now, NULL);
    if (r == MOTION_OK)
        j->state = B5M_RESUMED;
    return r;
}
bool b5_manual_pause(B5Manual *j, uint64_t now) {
    MotionStatus s = nav_get_status();
    if (!s.session_active || s.owner != MOTION_BUDDY5)
        return false;
    if (s.mode != MOTION_FAULT)
        (void)nav_request_stop(MOTION_BUDDY1);
    j->state = B5M_CANCELLED;
    j->reason = B5M_INTERRUPTED;
    note(j, B5M_CANCEL, B5M_INTERRUPTED, MOTION_OK, now, NULL);
    return true;
}
const char *b5_manual_state_name(unsigned s) {
    static const char *const names[] = {"IDLE", "ARMED", "TAKEOVER", "MOVING", "NEXT",
        "DONE_HOLD", "CANCELLED", "FAILED", "RESUMED"};
    return s < sizeof(names)/sizeof(names[0]) ? names[s] : "INVALID";
}
