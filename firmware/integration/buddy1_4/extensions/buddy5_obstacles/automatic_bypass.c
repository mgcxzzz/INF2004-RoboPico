#include "extensions/buddy5_obstacles/automatic_bypass.h"
#include "extensions/buddy5_obstacles/profile/b5_config.h"
#include <string.h>
#include <math.h>

/* rtos_buddy1_5 port of SDK automatic v0.6.1 automatic_bypass.c. Functions are
 * copied verbatim except where marked "rtos_buddy1_5"; the guard, stationary
 * scan, profiler side selection and geometry-only variant were removed. */
static void angle(B5Automatic *a, int dd, uint64_t now) {
    a->sensor.cancel();
    a->angle_dd = dd;
    a->sensor.angle(dd, now);
}
static int16_t tenth(float v) {
    float x = v * 10.0f;
    return (int16_t)(x > 32767 ? 32767 : x < -32768 ? -32768 : lroundf(x));
}
static float heading_now(const B5Automatic *a) {
    return a->state == B5A_BYPASS ? a->route.heading_deg : a->heading_deg;
}
/* Bounded event ring for the MQTT queue: O(1), no I/O here. The oldest entry
 * is overwritten (and counted) if the transport is down or slow. */
static void event(B5Automatic *a, unsigned code, float va, float vb, unsigned detail,
                  unsigned result, uint64_t now) {
    if (a->ev_count == B5_EVENT_RING) {
        a->ev_head = (uint8_t)((a->ev_head + 1) % B5_EVENT_RING);
        --a->ev_count;
        ++a->ev_dropped;
    }
    B5Event *e = &a->events[(a->ev_head + a->ev_count) % B5_EVENT_RING];
    ++a->ev_count;
    float h = heading_now(a) * 100.0f;
    *e = (B5Event){.ms = (uint32_t)(now / 1000), .a = tenth(va), .b = tenth(vb),
        .heading_cdeg = (int16_t)(h > 32767 ? 32767 : h < -32768 ? -32768 : lroundf(h)),
        .level = {a->line[0], a->line[1]}, .bg = {a->stripe.bg[0], a->stripe.bg[1]},
        .seq = ++a->ev_seq, .code = (uint8_t)code, .state = (uint8_t)a->state,
        .phase = (uint8_t)a->phase, .reason = (uint8_t)a->reason, .result = (uint8_t)result,
        .detail = (uint8_t)detail, .search = (uint8_t)a->searches,
        .rejects = (uint8_t)(a->stripe.rejects > 255 ? 255 : a->stripe.rejects)};
}
bool b5_auto_peek_event(const B5Automatic *a, B5Event *out) {
    if (!a->ev_count) return false;
    *out = a->events[a->ev_head];
    return true;
}
void b5_auto_drop_event(B5Automatic *a) {
    if (!a->ev_count) return;
    a->ev_head = (uint8_t)((a->ev_head + 1) % B5_EVENT_RING);
    --a->ev_count;
}
static void leg_events(B5Automatic *a, uint64_t now) {
    while (a->legs_reported < a->route.count && a->legs_reported < B5M_MAX_STEPS) {
        const B5ManualLeg *l = &a->route.leg[a->legs_reported];
        if (l->outcome == B5LEG_PENDING || l->outcome == B5LEG_RUNNING) break;
        bool turn = a->route.plan[a->legs_reported].turn;
        event(a, B5V_LEG_END, turn ? l->measured_deg : l->measured_mm, l->issued,
              a->legs_reported, l->outcome, now);
        ++a->legs_reported;
    }
}
static void stop(B5Automatic *a, unsigned why, uint64_t now) {
    if (nav_get_status().mode != MOTION_FAULT)
        (void)nav_request_stop(MOTION_BUDDY5);
    if (a->state == B5A_BYPASS) {
        b5_manual_mark_interrupted(&a->route, why);
        leg_events(a, now);
    }
    a->reason = why;
    a->state = why == B5A_NO_PATH || why == B5A_CLEARANCE ? B5A_BLOCKED : B5A_FAULT;
    a->since = now;
    a->sensor.cancel();
    event(a, why == B5A_CLEARANCE ? B5V_HOLD_CLEARANCE : B5V_STOPPED,
          why == B5A_CLEARANCE ? a->rect.first_turn_clear_mm : (float)a->stripe.last_p,
          why == B5A_CLEARANCE ? a->rect.reverse_needed_mm : (float)a->stripe.centre,
          a->stripe.last_reject, a->stripe_result, now);
    /* A state-machine failure stays stopped until an operator ends/restarts
     * the session. This does not clear another subsystem's fault. */
}
static bool take(B5Automatic *a, b5_ping_t *p, uint64_t now) {
    if (!a->sensor.take(p)) return false;
    ++a->pings;
    a->last_ping = now;
    if (p->cls == B5_PING_FAULT) { ++a->faults; ++a->fault_run; }
    else { a->fault_run = 0; a->last_good = now; }
    a->range_mm = p->cls == B5_PING_ECHO ? p->range_mm : B5_RANGE_FAR_MM;
    return true;
}
void b5_auto_init(B5Automatic *a, const B5SensorPort *port) {
    memset(a, 0, sizeof(*a));
    a->sensor = *port;
    a->range_mm = B5_RANGE_FAR_MM;
    a->depth_mm = (float)B5_PLAN_DEPTH_MM;
}
void b5_auto_observe(B5Automatic *a, uint64_t now, uint16_t left, uint16_t right) {
    a->observed = now;
    ++a->observations;
    a->line[0] = left; a->line[1] = right;
}
void b5_auto_cancel(B5Automatic *a, uint64_t now) {
    if (a->session && nav_get_status().session_active)
        (void)nav_request_stop(MOTION_BUDDY5);
    if (a->state == B5A_BYPASS) {
        b5_manual_mark_interrupted(&a->route, B5A_CANCELLED);
        leg_events(a, now);
    }
    if (a->session && a->state != B5A_IDLE)
        event(a, B5V_STOPPED, (float)a->stripe.last_p, 0, 0, a->stripe_result, now);
    a->session = false;
    a->state = B5A_IDLE;
    a->reason = B5A_CANCELLED;
    angle(a, 0, now);
}
static bool owns(const B5Automatic *a, MotionStatus s) {
    return s.session_active && s.owner == MOTION_BUDDY5 && s.lease == a->lease && !s.fault;
}
static bool progress_ok(B5Automatic *a, MotionStatus s, uint64_t now) {
    if ((s.mode != MOTION_DRIVING && s.mode != MOTION_TURNING) ||
        (!s.target_mm_s[0] && !s.target_mm_s[1])) {
        a->progress_id = 0;
        return true;
    }
    float progress = s.mode == MOTION_TURNING ? s.rotation_deg : s.progress_mm;
    if (a->progress_id != s.command_id || fabsf(progress - a->last_progress) >= 1.0f) {
        a->progress_id = s.command_id;
        a->last_progress = progress;
        a->progress_us = now;
    }
    if (now - a->progress_us >= 3000000) {
        stop(a, B5A_STALL, now);
        return false;
    }
    return true;
}
/* rtos_buddy1_5 replacement for v0.6.1 begin()+scan()+plan(): the side and
 * the ACTUAL stopped range come from the mission; the plan, its 80-220 mm
 * range gate, the clearance hold and the route adoption are the v0.6.1 code.
 * No safety scan is available, so only the forward echo enters the
 * first-turn clearance check (see README: unverified clearance). */
bool b5_auto_start(B5Automatic *a, uint32_t lease, unsigned side, unsigned range_mm,
                   float heading0_deg, float lateral0_mm, uint64_t now) {
    a->session = true;
    a->state = B5A_PLAN;
    a->reason = a->fault_run = a->searches = 0;
    a->since = a->last_ping = a->observed = now;
    a->lease = lease;
    a->search_id = 0;
    a->progress_id = 0;
    a->rect = (B5RectPlan){0};
    b5_stripe_init(&a->stripe, 0);
    a->stop_progress = a->advance_mm = a->last_rotation = 0;
    a->acq_onset = a->search_centre = 0;
    a->acq_samples = 0;
    a->pivot_return_mm = 0; a->fallback = false;
    a->armed_mask = 0; a->line_mask = 3;
    a->handback_sensor_mask = 0;
    a->phase = B5R_START;
    a->stripe_result = B5S_PENDING;
    a->seen_rejects = 0; a->seen_phase = 0; a->legs_reported = 0; a->last_handback = 0;
    a->evidence_timing = false;
    a->used_observation = 0;
    a->side = (uint8_t)side;
    a->start_range_mm = (uint16_t)(range_mm > 0xFFFFu ? 0xFFFFu : range_mm);
    a->heading0_deg = isfinite(heading0_deg) ? heading0_deg : 0.0f;
    a->lateral0_mm = isfinite(lateral0_mm) ? lateral0_mm : 0.0f;
    a->heading_deg = a->heading0_deg;
    b5_manual_init(&a->route);
    angle(a, 0, now);
    MotionStatus s = nav_get_status();
    if (!owns(a, s) || s.mode != MOTION_PAUSED || !s.stopped) {
        stop(a, B5A_LEASE, now); return false;
    }
    if ((side != B5_SIDE_LEFT && side != B5_SIDE_RIGHT) || !isfinite(heading0_deg) ||
        !isfinite(lateral0_mm) || fabsf(heading0_deg) > 15.0f ||
        range_mm < B5_PLAN_RANGE_MIN_MM || range_mm > B5_PLAN_RANGE_MAX_MM ||
        !b5_rect_plan_ex(NULL, range_mm, a->depth_mm, false, &a->rect)) {
        stop(a, B5A_NO_PATH, now); return false;
    }
    event(a, B5V_PLAN, a->rect.forward_mm, (float)range_mm, side,
          a->rect.turn_clearance_ok, now);
    if (!a->rect.turn_clearance_ok || a->rect.backoff_mm != 0) {
        stop(a, B5A_CLEARANCE, now); return false;
    }
    if (!b5_manual_adopt_rect(&a->route, a->lease, side == B5_SIDE_LEFT,
                              a->rect.backoff_mm, a->rect.side_mm, a->rect.forward_mm,
                              false, now)) {
        stop(a, B5A_LEASE, now); return false;
    }
    /* Absolute legs are referenced to the approach heading (heading0 = the
     * encoder rotation already accumulated since the direction probe began). */
    a->route.heading_deg = a->heading0_deg;
    a->generation = s.generation;
    a->state = B5A_BYPASS;
    a->since = now;
    a->last_ping = now;
    a->fault_run = 0;
    return true;
}
static void watch(B5Automatic *a, uint64_t now) {
    b5_ping_t p;
    if (take(a, &p, now) && p.cls == B5_PING_ECHO && p.range_mm <= 70) {
        stop(a, B5A_SENSOR, now); return;
    }
    if (a->fault_run >= 3 || now - a->last_ping > 300000) {
        stop(a, B5A_SENSOR, now); return;
    }
    (void)a->sensor.request(now);
}
/* ---- v0.5 return leg: (1) stripe detection, (2) positioning + heading
 * restoration, (3) stationary confirmation, (4) existing hand-back. ----
 * Only the generic coordinator API is used (drive/turn/stop/resume); no PWM,
 * second follower or calibration write. Heading is the summed ENCODER
 * rotation of every completed/cancelled Buddy5 command since adoption. */
static const float search_deg[B5R_SEARCH_STEPS] = {-4, 4, -8, 8, -12, 12}; /* v0.6 */
static bool fresh(B5Automatic *a) {
    if (a->used_observation == a->observed) return false;
    a->used_observation = a->observed;
    return true;
}
static void enter(B5Automatic *a, unsigned phase, uint64_t now) {
    a->phase = phase;
    a->phase_since = now;
    a->evidence_timing = false;
}
__attribute__((noinline)) static bool start_move(B5Automatic *a, bool turn, float goal,
                                                 unsigned speed, unsigned next, uint64_t now) {
    MotionResult r = turn ? nav_turn_relative(MOTION_BUDDY5, a->lease, goal, speed,
                                              MOTION_IN_PLACE, &a->search_id)
                          : nav_drive_distance(MOTION_BUDDY5, a->lease, goal, speed,
                                               &a->search_id);
    if (r != MOTION_OK) { stop(a, B5A_MOVE, now); return false; }
    a->generation = nav_get_status().generation;
    enter(a, next, now);
    return true;
}
/* 0 running, 1 completed normally (heading updated), -1 stopped/faulted. */
static int follow(B5Automatic *a, MotionStatus s, uint64_t now) {
    MotionCommandStatus cmd;
    if (!a->search_id || !nav_get_command(a->search_id, &cmd)) { stop(a, B5A_MOVE, now); return -1; }
    if (cmd.state == MOTION_CMD_DONE) {
        if (s.generation != a->generation + 1U) { stop(a, B5A_CANCELLED, now); return -1; }
        if (s.mode != MOTION_PAUSED || !s.stopped) return 0;
        a->heading_deg += cmd.rotation_deg;
        a->last_rotation = cmd.rotation_deg;
        a->generation = s.generation;
        return 1;
    }
    if (cmd.state != MOTION_CMD_RUNNING || s.generation != a->generation) {
        stop(a, B5A_CANCELLED, now); return -1;
    }
    return 0;
}
__attribute__((noinline)) static void halt(B5Automatic *a, unsigned next, uint64_t now) {
    a->sensor.cancel();
    (void)nav_request_stop(MOTION_BUDDY5);
    a->generation = nav_get_status().generation;
    enter(a, next, now);
}
/* After our own Stop: 0 waiting, 1 paused (heading/progress taken from the
 * coordinator's record of the cancelled command), -1 another stop/fault. */
__attribute__((noinline)) static int halted(B5Automatic *a, MotionStatus s, uint64_t now) {
    if (s.generation != a->generation) { stop(a, B5A_CANCELLED, now); return -1; }
    if (s.mode != MOTION_PAUSED || !s.stopped) return 0;
    MotionCommandStatus cmd;
    if (!nav_get_command(a->search_id, &cmd) ||
        (cmd.state != MOTION_CMD_CANCELLED && cmd.state != MOTION_CMD_DONE)) {
        stop(a, B5A_MOVE, now); return -1;
    }
    a->heading_deg += cmd.rotation_deg;
    a->last_rotation = cmd.rotation_deg;
    a->stop_progress = cmd.progress_mm;
    return 1;
}
/* v0.6.1: sensors at or above their line level (bit0 left, bit1 right). With
 * a valid crossing this is the stripe's own per-sensor rise level (v0.6
 * evidence); without one, the fixed detector/hand-back level. */
static unsigned level_mask(const B5Automatic *a) {
    unsigned m = 0;
    for (unsigned w = 0; w < 2; ++w) {
        uint16_t v = a->line[w] > 1000 ? 1000 : a->line[w];
        uint16_t t = a->stripe.result == B5S_FOUND ? b5_stripe_threshold(&a->stripe, w)
                                                   : (uint16_t)B5R_FALLBACK_LEVEL;
        if (v >= t) m |= 1u << w;
    }
    return m;
}
/* v0.6.1: estimated return distance at which the AXLE meets the original
 * path, from the encoder record of the completed bypass legs (chord heading of
 * each straight). 0 = no plausible estimate (fallback not allowed). */
__attribute__((noinline)) static float pivot_return(const B5Automatic *a) {
    const float rad = 3.14159265358979323846f / 180.0f;
    /* rtos_buddy1_5: start from the probes' encoder offsets (+LEFT) so the
     * estimate stays referenced to the approach line, not the adoption pose. */
    float h = a->heading0_deg, lateral = a->lateral0_mm;
    for (unsigned k = 0; k < a->route.count && k < B5M_MAX_STEPS; ++k) {
        const B5ManualLeg *l = &a->route.leg[k];
        if (l->outcome != B5LEG_DONE && l->outcome != B5LEG_SKIPPED) return 0;
        if (!a->route.plan[k].turn)
            lateral += l->measured_mm * sinf((h + .5f * l->measured_deg) * rad);
        h += l->measured_deg;
    }
    float sn = sinf(a->heading_deg * rad);
    if (!isfinite(sn) || fabsf(sn) < B5R_FALLBACK_MIN_SIN) return 0;
    float d = -lateral / sn;
    if (!isfinite(d) || fabsf(d - a->rect.side_mm) > B5R_FALLBACK_TOL_MM ||
        d > a->rect.return_max_mm || d < 1.0f)
        return 0;
    return d;
}
/* v0.6: start the slow sensor-guided sweep through the approach heading. */
__attribute__((noinline)) static void acquire(B5Automatic *a, float sign, uint64_t now) {
    float goal = sign * B5R_ACQ_PAST_DEG - a->heading_deg;
    if (!(sign * goal >= 1.0f) || fabsf(goal) > 180.0f) { stop(a, B5A_MOVE, now); return; }
    a->armed_mask = 0;
    a->sensor_clear_timing[0] = a->sensor_clear_timing[1] = false;
    a->acq_samples = 0;
    event(a, B5V_ACQUIRE, a->heading_deg, goal, a->fallback, 0, now);
    (void)start_move(a, true, goal, B5M_SLOW_SPEED, B5R_ACQUIRE, now);
}
__attribute__((noinline)) static void rejoin(B5Automatic *a, uint64_t now, MotionStatus s) {
    /* Bounded: never hands back because a distance or this timer ran out. */
    if (now - a->search_since > 30000000) { stop(a, B5A_NO_LINE, now); return; }
    bool obs = fresh(a);
    unsigned above = level_mask(a);
    if (a->handback_sensor_mask) above &= a->handback_sensor_mask;
    /* Confirmation/search evidence: only sensors proven clear during the
     * sweep (v0.6.1), so a sensor resting on a dark floor cannot confirm. */
    bool evidence = (above & a->line_mask) != 0;
    unsigned no_line = a->fallback ? B5A_FALLBACK_NO_LINE : B5A_NO_LINE;
    float sign = a->side == B5_SIDE_LEFT ? 1.0f : -1.0f; /* rtos_buddy1_5: injected side */
    int r;
    switch (a->phase) {
    case B5R_START:
        if (s.generation != a->generation) { stop(a, B5A_CANCELLED, now); return; }
        if (s.mode != MOTION_PAUSED || !s.stopped) return;
        b5_stripe_init(&a->stripe, a->rect.return_search_mm);
        a->stripe_result = B5S_PENDING;
        a->seen_rejects = 0; a->seen_phase = 0;
        a->last_ping = now;
        a->fault_run = 0;
        a->fallback = false;
        a->armed_mask = 0; a->line_mask = 3;
        /* v0.6.1: drive to the estimated axle alignment (bounded), or the v0.6
         * 300 mm cap when no plausible estimate exists. */
        a->pivot_return_mm = pivot_return(a);
        {
            float goal = a->pivot_return_mm > 0 ? a->pivot_return_mm : a->rect.return_max_mm;
            event(a, B5V_RETURN_START, goal, a->rect.return_search_mm, a->pivot_return_mm > 0, 0, now);
            (void)start_move(a, false, goal, B5M_SPEED, B5R_RETURN, now);
        }
        return;
    case B5R_RETURN:
        r = follow(a, s, now);
        if (r < 0) return;
        if (r > 0) {
            /* Goal reached without a valid crossing. Without an estimate this is
             * the v0.6 cap (NO_LINE). With one, the axle is at the estimated
             * original path: run the bounded sweep. NOT a line detection. */
            if (a->pivot_return_mm <= 0) { stop(a, B5A_NO_LINE, now); return; }
            a->fallback = true;
            a->sensor.cancel();
            event(a, B5V_FALLBACK, a->pivot_return_mm, s.progress_mm, a->stripe_result,
                  a->stripe.last_reject, now);
            enter(a, B5R_RESTORE, now);
            return;
        }
        watch(a, now);
        if (a->state != B5A_REJOIN || !obs) return;
        /* v0.6.1: after a detector NO_LINE the drive continues to the estimate */
        if (a->stripe_result != B5S_PENDING) return;
        /* (1) detection only: no motion decision until the stripe is complete */
        a->stripe_result = b5_stripe_step(&a->stripe, s.progress_mm, a->line[0], a->line[1]);
        if (a->stripe.rejects != a->seen_rejects) {
            a->seen_rejects = a->stripe.rejects;
            event(a, B5V_REJECT, s.progress_mm, 0, a->stripe.last_reject, 0, now);
        }
        if (a->stripe.phase == 1 && a->seen_phase == 0)
            event(a, B5V_CANDIDATE, a->stripe.first_rise, s.progress_mm, 0, 0, now);
        a->seen_phase = a->stripe.phase;
        if (a->stripe_result == B5S_FOUND) {
            event(a, B5V_STRIPE_FOUND, a->stripe.centre, s.progress_mm,
                  (unsigned)lroundf(a->stripe.width[0]), (unsigned)lroundf(a->stripe.width[1]), now);
            halt(a, B5R_RETURN_STOP, now);
        } else if (a->stripe_result != B5S_PENDING && a->pivot_return_mm <= 0)
            stop(a, B5A_NO_LINE, now);
        return;
    case B5R_RETURN_STOP:
        if (halted(a, s, now) <= 0) return;
        /* (2) Pivot over the stripe centre: sensors are LINE_AHEAD in front.
         * v0.6: stop_progress now includes the coast until the wheels were
         * still (coordinator keeps the cancelled record updated). */
        a->advance_mm = B5R_LINE_AHEAD_MM - (a->stop_progress - a->stripe.centre);
        event(a, B5V_POSITIONED, a->stop_progress, a->advance_mm, 0, 0, now);
        if (!isfinite(a->advance_mm) || a->advance_mm > B5R_ADVANCE_MAX_MM) {
            stop(a, B5A_MOVE, now); return;
        }
        if (a->advance_mm < 1.0f) { enter(a, B5R_RESTORE, now); return; }
        a->last_ping = now;
        a->fault_run = 0;
        (void)start_move(a, false, a->advance_mm, B5M_SPEED, B5R_ADVANCE, now);
        return;
    case B5R_ADVANCE:
        r = follow(a, s, now);
        if (r < 0) return;
        if (r == 0) { watch(a, now); return; }
        a->sensor.cancel();
        enter(a, B5R_RESTORE, now);
        return;
    case B5R_RESTORE: {
        /* v0.6: angle-controlled part of the final turn, ending ACQ_OPEN deg
         * short of the approach heading (0). Line evidence is ignored here. */
        float coarse = -sign * B5R_ACQ_OPEN_DEG - a->heading_deg;
        if (sign * coarse < 1.0f) { acquire(a, sign, now); return; }
        if (fabsf(coarse) > 180.0f) { stop(a, B5A_MOVE, now); return; }
        (void)start_move(a, true, coarse, B5M_SPEED, B5R_RESTORE_TURN, now);
        return;
    }
    case B5R_RESTORE_TURN:
        r = follow(a, s, now);
        if (r <= 0) return;
        event(a, B5V_RESTORED, a->heading_deg, 0, 0, 0, now);
        acquire(a, sign, now);
        return;
    case B5R_ACQUIRE: {
        r = follow(a, s, now);
        if (r < 0) return;
        if (r > 0) {
            /* Sweep reached +ACQ_PAST. Evidence still being timed at the end
             * goes to the stationary confirmation; otherwise stop. */
            if (!a->evidence_timing) { stop(a, no_line, now); return; }
            a->search_centre = a->acq_onset;
            a->line_mask = a->armed_mask;
            event(a, B5V_ACQUIRED, a->acq_onset, a->heading_deg, a->acq_samples, 1, now);
            enter(a, B5R_CONFIRM, now);
            return;
        }
        if (!obs) return;
        float live = a->heading_deg + s.rotation_deg; /* running command, encoder */
        /* v0.6.1: per-sensor arming. A sensor counts only after it has read
         * below its line level for ACQ_CLEAR_US during this sweep. */
        for (unsigned w = 0; w < 2; ++w) {
            if (above & (1u << w)) a->sensor_clear_timing[w] = false;
            else if (!a->sensor_clear_timing[w]) { a->sensor_clear_timing[w] = true; a->sensor_clear_since[w] = now; }
            else if (now - a->sensor_clear_since[w] >= B5R_ACQ_CLEAR_US) a->armed_mask |= (uint8_t)(1u << w);
        }
        if (!(above & a->armed_mask) || fabsf(live) > B5R_ACQ_OPEN_DEG) {
            a->evidence_timing = false;
            return;
        }
        if (!a->evidence_timing) {
            a->evidence_timing = true;
            a->evidence_since = now;
            a->acq_onset = live;
            a->acq_samples = 0;
        }
        if (++a->acq_samples >= B5R_ACQ_MIN_SAMPLES &&
            now - a->evidence_since >= B5R_ACQ_EVIDENCE_US)
            halt(a, B5R_ACQUIRE_STOP, now);
        return;
    }
    case B5R_ACQUIRE_STOP:
        if (halted(a, s, now) <= 0) return;
        a->search_centre = a->acq_onset;
        a->line_mask = a->armed_mask;
        event(a, B5V_ACQUIRED, a->acq_onset, a->heading_deg, a->acq_samples, 0, now);
        enter(a, B5R_CONFIRM, now);
        return;
    case B5R_SEARCH:
        r = follow(a, s, now);
        if (r < 0) return;
        if (r > 0) { enter(a, B5R_CONFIRM, now); return; }
        if (!obs) return;
        if (!evidence) a->evidence_timing = false;
        else if (!a->evidence_timing) { a->evidence_timing = true; a->evidence_since = now; }
        else if (now - a->evidence_since >= 30000) halt(a, B5R_SEARCH_STOP, now);
        return;
    case B5R_SEARCH_STOP:
        if (halted(a, s, now) > 0) enter(a, B5R_CONFIRM, now);
        return;
    case B5R_CONFIRM:
        /* (3) Stationary confirmation: the stripe crossed at the turning point
         * must also be seen 130 mm further along the approach direction, with
         * the same background contrast, continuously for 150 ms. */
        if (s.generation != a->generation) { stop(a, B5A_CANCELLED, now); return; }
        if (s.mode != MOTION_PAUSED || !s.stopped) return;
        if (obs) {
            if (!evidence) a->evidence_timing = false;
            else if (!a->evidence_timing) { a->evidence_timing = true; a->evidence_since = now; }
        }
        if (a->evidence_timing && now - a->evidence_since >= B5R_CONFIRM_US) {
            event(a, B5V_CONFIRMED, (float)((now - a->evidence_since) / 1000), a->heading_deg,
                  a->searches, 0, now);
            enter(a, B5R_RESUME, now); return;
        }
        if (now - a->phase_since < B5R_CONFIRM_WAIT_US) return;
        if (a->searches >= B5R_SEARCH_STEPS) { stop(a, no_line, now); return; }
        {
            /* v0.6: about the acquisition heading, inside the sweep window. */
            float target = a->search_centre + sign * search_deg[a->searches++];
            if (sign * target > B5R_ACQ_PAST_DEG) target = sign * B5R_ACQ_PAST_DEG;
            if (sign * target < -B5R_ACQ_OPEN_DEG) target = -sign * B5R_ACQ_OPEN_DEG;
            float goal = target - a->heading_deg;
            event(a, B5V_SEARCH_TURN, target, goal, a->searches, 0, now);
            if (fabsf(goal) < 1.0f) { enter(a, B5R_CONFIRM, now); return; }
            if (fabsf(goal) > 180.0f) { stop(a, B5A_MOVE, now); return; }
            (void)start_move(a, true, goal, B5M_SLOW_SPEED, B5R_SEARCH, now);
        }
        return;
    case B5R_RESUME: {
        /* (4) Hand-back through the existing coordinator rule only (which itself
         * requires >=400 for 150 ms and refuses an interrupted junction). */
        if (s.generation != a->generation) { stop(a, B5A_CANCELLED, now); return; }
        if (!evidence) { enter(a, B5R_CONFIRM, now); return; }
        MotionResult m = nav_resume_line(MOTION_BUDDY5, a->lease, false);
        a->last_handback = (uint8_t)m;
        if (m == MOTION_OK) {
            event(a, B5V_HANDBACK, a->heading_deg, 0, 0, m, now);
            /* rtos_buddy1_5: done; the mission owns re-arming (v0.6.1: GUARD). */
            a->state = B5A_DONE;
            a->session = false;
            a->since = a->last_ping = now;
            angle(a, 0, now);
            return;
        }
        if (m == MOTION_NEED_LINE && now - a->phase_since < 500000) return;
        event(a, B5V_HANDBACK, a->heading_deg, 0, 0, m, now);
        stop(a, m == MOTION_NEED_LINE ? B5A_NO_LINE : B5A_LEASE, now);
        return;
    }
    default:
        stop(a, B5A_MOVE, now);
    }
}
void b5_auto_tick(B5Automatic *a, uint64_t now) {
    if (!a->session) return;
    MotionStatus s = nav_get_status();
    if (!s.session_active) { b5_auto_cancel(a, now); return; }
    if (a->state != B5A_BYPASS && a->state != B5A_REJOIN) return;
    if (s.mode == MOTION_FAULT || s.fault || now < a->observed ||
        now - a->observed > 50000) { stop(a, B5A_STALE, now); return; }
    if (!owns(a, s)) { stop(a, B5A_LEASE, now); return; }
    if (!progress_ok(a, s, now)) return;
    if (a->state == B5A_BYPASS) {
        b5_manual_poll(&a->route, now);
        leg_events(a, now);
        if (a->route.state == B5M_FAILED || a->route.state == B5M_CANCELLED) {
            stop(a, B5A_MOVE, now); return;
        }
        if (a->route.state == B5M_DONE) {
            a->heading_deg = a->route.heading_deg;
            a->generation = nav_get_status().generation;
            a->sensor.cancel();
            a->state = B5A_REJOIN;
            a->since = a->search_since = now;
            a->searches = a->search_id = 0;
            a->search_centre = a->acq_onset = 0;
            enter(a, B5R_START, now);
            return;
        }
        bool forward = a->route.state == B5M_MOVING &&
                       !a->route.plan[a->route.index].turn &&
                       a->route.plan[a->route.index].goal > 0;
        if (forward) watch(a, now);
        else { a->sensor.cancel(); a->last_ping = now; a->fault_run = 0; }
        return;
    }
    rejoin(a, now, s);
}
const char *b5_auto_name(unsigned state) {
    static const char *const names[] = {"IDLE", "GUARD", "CONFIRM", "HALTING", "SCAN",
        "REFINE", "PLAN", "BYPASS", "REJOIN", "BLOCKED", "FAULT", "GEOMETRY_HOLD", "DONE"};
    return state < sizeof(names)/sizeof(names[0]) ? names[state] : "INVALID";
}
const char *b5_auto_reason_name(unsigned reason) {
    static const char *const names[] = {"OK", "SENSOR", "LEASE", "CANCELLED", "NO_PATH",
        "MOVE", "NO_LINE", "STALE", "NO_ENCODER_PROGRESS", "GEOMETRY_DONE",
        "INSUFFICIENT_TURN_CLEARANCE", "NO_LINE_AFTER_FALLBACK_SWEEP"};
    return reason < sizeof(names)/sizeof(names[0]) ? names[reason] : "INVALID";
}
const char *b5_auto_event_name(unsigned code) {
    static const char *const names[] = {"NONE", "TAKEOVER", "PLAN", "HOLD_CLEARANCE", "LEG_END",
        "RETURN_START", "CANDIDATE", "REJECT", "STRIPE_FOUND", "POSITIONED", "RESTORED",
        "SEARCH_TURN", "CONFIRMED", "HANDBACK", "STOPPED", "GEOMETRY_HOLD", "ACQUIRE",
        "ACQUIRED", "RETURN_FALLBACK"};
    return code < sizeof(names)/sizeof(names[0]) ? names[code] : "INVALID";
}
const char *b5_auto_phase_name(unsigned phase) {
    static const char *const names[] = {"START", "RETURN", "RETURN_STOP", "ADVANCE",
        "RESTORE", "RESTORE_TURN", "CONFIRM", "SEARCH", "SEARCH_STOP", "RESUME", "ACQUIRE",
        "ACQUIRE_STOP"};
    return phase < sizeof(names)/sizeof(names[0]) ? names[phase] : "INVALID";
}
