/**
 * @file b5_mission.c
 * @brief Buddy5 obstacle mission sequencing (see b5_mission.h).
 * @details Owner: the core1 motion task, one step per 10 ms frame right
 * after motion_update(). No blocking, no sensor or motor register access:
 * sonar/servo go through the B5SensorPort, motion through nav_api.h with the
 * one Buddy5 lease taken at the obstacle. The ported width/direction/bypass
 * modules are stepped unchanged; this file only sequences them and applies
 * the documented gates between them.
 * @see app_program/obstacle_navigation/README.md
 */
#include "b5_mission.h"
#include <math.h>
#include <string.h>

/* Alignment search about the trimmed heading (v0.6 rejoin search table). */
static const float g_align_deg[B5MI_ALIGN_STEPS] = {-4.0f, 4.0f,   -8.0f,
                                                    8.0f,  -12.0f, 12.0f};
/* Identity calibration: observed levels are already normalised 0..1000, so
 * line_level(level, 0, 1000) == level and analog_demand() sees exactly the
 * levels Buddy3's follower would. */
static const line_calibration_t g_level_identity = {
    .white = {0U, 0U}, .black = {1000U, 1000U}, .mask = 7U};

static bool still (motion_status_t status)
{
    return (MOTION_PAUSED == status.mode) && (status.stopped);
}

static bool owns (const b5_mission_t * p_mission, motion_status_t status)
{
    return (status.session_active) && (MOTION_BUDDY5 == status.owner) &&
           (status.lease == p_mission->lease) && (0U == status.fault) &&
           (status.mode != MOTION_FAULT);
}

static unsigned line_mask (const b5_mission_t * p_mission)
{
    unsigned mask = 0U;
    for (unsigned wheel = 0U; wheel < 2U; ++wheel)
    {
        if (p_mission->line[wheel] >= (uint16_t)B5MI_LINE_LEVEL)
        {
            mask |= 1U << wheel;
        }
    }
    return mask;
}

/* Fresh line evidence: a sensor that saw the line at the takeover sees it
 * again (or any sensor when none was above the level at the takeover). */
static bool line_evidence (const b5_mission_t * p_mission)
{
    unsigned mask = line_mask(p_mission);
    if (0U == p_mission->takeover_mask)
    {
        return mask != 0U;
    }
    return (mask & p_mission->takeover_mask) != 0U;
}

static bool fresh_observation (b5_mission_t * p_mission)
{
    if (p_mission->used_observation == p_mission->observed)
    {
        return false;
    }
    p_mission->used_observation = p_mission->observed;
    return true;
}

static void enter (b5_mission_t * p_mission, b5mi_stage_t stage, uint64_t now)
{
    p_mission->stage = stage;
    p_mission->stage_since = now;
    p_mission->sub_since = now;
    p_mission->sub = 0U;
    p_mission->evidence_timing = false;
    p_mission->confirm = 0U;
    p_mission->close_count = 0U;
    p_mission->ping_pending = false;
    p_mission->progress_id = 0U;
}

static void servo_front (b5_mission_t * p_mission, uint64_t now)
{
    p_mission->sensor.cancel();
    p_mission->sensor.angle(0, now);
    p_mission->ping_pending = false;
}

/* Stopped hold with the lease kept. The module that ended has already
 * stopped; a mission move is stopped here. Never clears another fault. */
static void hold (b5_mission_t * p_mission, b5mi_reason_t reason,
                  unsigned module_reason, uint64_t now)
{
    motion_status_t status = nav_get_status();
    /* v0.6.1 stop(): also stops line following when the guard fails. */
    if ((status.session_active) && (status.mode != MOTION_FAULT) &&
        (!still(status)))
    {
        (void)nav_request_stop(MOTION_BUDDY5);
    }
    p_mission->sensor.cancel();
    p_mission->reason = reason;
    p_mission->module_reason = (uint8_t)module_reason;
    enter(p_mission, B5MI_HOLD, now);
}

/* Discard unsent records of a PREVIOUS run so the frozen probe can start.
 * Counted and reported (E5), never silent. */
static void drain_width (b5_mission_t * p_mission)
{
    B5WPRec record;
    while (b5_wp_peek(&p_mission->width, &record))
    {
        b5_wp_drop(&p_mission->width);
        p_mission->stale_dropped++;
    }
    if (p_mission->width.summary_pending)
    {
        p_mission->width.summary_pending = false;
        p_mission->stale_dropped++;
    }
    if (p_mission->width.result_pending)
    {
        p_mission->width.result_pending = false;
        p_mission->stale_dropped++;
    }
    while (p_mission->width.retry_sent < p_mission->width.retry_count)
    {
        p_mission->width.retry_sent++;
        p_mission->stale_dropped++;
    }
    if (p_mission->width.limits_pending) /* long probe D5 */
    {
        p_mission->width.limits_pending = false;
        p_mission->stale_dropped++;
    }
}

static void drain_edge (b5_mission_t * p_mission)
{
    B5EPRec record;
    while (b5_ep_peek(&p_mission->edge, &record))
    {
        b5_ep_drop(&p_mission->edge);
        p_mission->stale_dropped++;
    }
    if (p_mission->edge.summary_pending)
    {
        p_mission->edge.summary_pending = false;
        p_mission->stale_dropped++;
    }
}

/* ---- Sonar -------------------------------------------------------------- */

/* One watch ping per call when the servo is settled. Returns true with a new
 * ping. Fault/timeout bookkeeping follows the v0.6.1 guard. */
static bool watch_ping (b5_mission_t * p_mission, uint64_t now,
                        b5_ping_t * p_ping)
{
    bool taken = false;
    if (!p_mission->sensor.settled(now))
    {
        return false;
    }
    if (p_mission->sensor.take(p_ping))
    {
        taken = true;
        p_mission->taken_requested = p_mission->ping_requested;
        p_mission->ping_pending = false;
        p_mission->last_ping = now;
        if (B5_PING_FAULT == p_ping->cls)
        {
            p_mission->fault_run++;
        }
        else
        {
            p_mission->fault_run = 0U;
        }
    }
    if (p_mission->sensor.request(now))
    {
        p_mission->ping_pending = true;
        p_mission->ping_requested = now;
    }
    return taken;
}

static unsigned watch_failure (const b5_mission_t * p_mission, uint64_t now)
{
    if (p_mission->fault_run >= 3U)
    {
        return B5MI_WATCH_FAULTS;
    }
    return ((now - p_mission->last_ping) > (uint64_t)300000U)
               ? B5MI_WATCH_TIMEOUT
               : 0U;
}

static void range_begin (b5_mission_t * p_mission, b5mi_stage_t after,
                         uint64_t now)
{
    p_mission->after_range = after;
    memset(&p_mission->range, 0, sizeof(p_mission->range));
    servo_front(p_mission, now);
    enter(p_mission, B5MI_RANGE, now);
}

static uint16_t median3 (const b5_ping_t ping[3])
{
    uint32_t value_a = ping[0].range_mm;
    uint32_t value_b = ping[1].range_mm;
    uint32_t value_c = ping[2].range_mm;
    uint32_t low = value_a < value_b ? value_a : value_b;
    uint32_t high = value_a < value_b ? value_b : value_a;
    uint32_t result = value_c < low ? low : (value_c > high ? high : value_c);
    return (uint16_t)result; /* 12-bit range field: fits uint16_t */
}

/* Stationary fresh triplet at servo 0: all clean echoes within 15 mm. */
static void range_tick (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now)
{
    b5mi_range_t * p_range = &p_mission->range;
    if (!still(status))
    {
        hold(p_mission, B5MI_R_MOVE, 0U, now);
        return;
    }
    if ((now - p_mission->stage_since) > (uint64_t)B5MI_HALT_LIMIT_US)
    {
        hold(p_mission, B5MI_R_SENSOR, 0U, now);
        return;
    }
    if ((p_range->npings < 3U) && (!p_range->ping_active))
    {
        if (p_mission->sensor.settled(now) && p_mission->sensor.request(now))
        {
            p_range->ping_active = true;
            p_range->ping_since = now;
        }
        return;
    }
    if (p_range->npings < 3U)
    {
        b5_ping_t ping;
        if (p_mission->sensor.take(&ping))
        {
            p_range->ping[p_range->npings++] = ping;
            p_range->ping_active = false;
        }
        else if ((now - p_range->ping_since) >= (uint64_t)B5MI_PING_TIMEOUT_US)
        {
            p_mission->sensor.cancel();
            p_range->ping[p_range->npings++] =
                (b5_ping_t){.cls = B5_PING_FAULT, .why = 7U};
            p_range->ping_active = false;
        }
        if (p_range->npings < 3U)
        {
            return;
        }
    }
    bool clean = true;
    uint32_t low = UINT32_MAX;
    uint32_t high = 0U;
    for (unsigned index = 0U; index < 3U; ++index)
    {
        if (p_range->ping[index].cls != B5_PING_ECHO)
        {
            clean = false;
        }
        low = p_range->ping[index].range_mm < low
                  ? p_range->ping[index].range_mm
                  : low;
        high = p_range->ping[index].range_mm > high
                   ? p_range->ping[index].range_mm
                   : high;
    }
    p_range->attempts++;
    if ((clean) && ((high - low) <= (uint32_t)B5MI_RANGE_SPREAD_MM))
    {
        p_range->range_mm = median3(p_range->ping);
        p_range->status = 1U;
        enter(p_mission, p_mission->after_range, now);
        return;
    }
    if (p_range->attempts >= (uint8_t)B5MI_RANGE_ATTEMPTS)
    {
        p_range->status = 2U;
        hold(p_mission, B5MI_R_RANGE, 0U, now);
        return;
    }
    p_range->npings = 0U;
    p_range->ping_active = false;
}

/* ---- Mission moves (alignment / approach) -------------------------------- */

static bool start_move (b5_mission_t * p_mission, bool turn, float goal,
                        unsigned speed, uint64_t now)
{
    motion_result_t result =
        turn ? nav_turn_relative(MOTION_BUDDY5, p_mission->lease, goal, speed,
                                 MOTION_IN_PLACE, &p_mission->command)
             : nav_drive_distance(MOTION_BUDDY5, p_mission->lease, goal, speed,
                                  &p_mission->command);
    if (result != MOTION_OK)
    {
        hold(p_mission, B5MI_R_MOVE, (unsigned)result, now);
        return false;
    }
    p_mission->generation = nav_get_status().generation;
    p_mission->halting_move = false;
    p_mission->progress_id = 0U;
    p_mission->sub_since = now;
    return true;
}

/* 0 running, 1 finished (completed or our own stop), -1 held. */
static int follow_move (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now, motion_command_status_t * p_command)
{
    if ((0U == p_mission->command) ||
        (!nav_get_command(p_mission->command, p_command)))
    {
        hold(p_mission, B5MI_R_MOVE, 0U, now);
        return -1;
    }
    if (MOTION_CMD_RUNNING == p_command->state)
    {
        if ((!p_mission->halting_move) &&
            (status.generation != p_mission->generation))
        {
            hold(p_mission, B5MI_R_CANCELLED, 0U, now);
            return -1;
        }
        return 0;
    }
    bool ours = (MOTION_CMD_DONE == p_command->state) ||
                ((p_mission->halting_move) &&
                 (MOTION_CMD_CANCELLED == p_command->state));
    if (!ours)
    {
        hold(p_mission, B5MI_R_MOVE, p_command->state, now);
        return -1;
    }
    if (!still(status))
    {
        return 0;
    }
    p_mission->generation = status.generation;
    p_mission->command = 0U;
    return 1;
}

static void halt_move (b5_mission_t * p_mission)
{
    if (!p_mission->halting_move)
    {
        (void)nav_request_stop(MOTION_BUDDY5);
        p_mission->halting_move = true;
    }
}

/* v0.6.1 progress rule applied to the mission's own moves. */
static bool progress_ok (b5_mission_t * p_mission, motion_status_t status,
                         uint64_t now)
{
    if (((status.mode != MOTION_DRIVING) && (status.mode != MOTION_TURNING)) ||
        ((0 == status.target_mm_s[0]) && (0 == status.target_mm_s[1])))
    {
        p_mission->progress_id = 0U;
        return true;
    }
    float progress = MOTION_TURNING == status.mode ? status.rotation_deg
                                                   : status.progress_mm;
    if ((p_mission->progress_id != status.command_id) ||
        (fabsf(progress - p_mission->last_progress) >= 1.0f))
    {
        p_mission->progress_id = status.command_id;
        p_mission->last_progress = progress;
        p_mission->progress_us = now;
    }
    if ((now - p_mission->progress_us) >= (uint64_t)B5MI_STALL_US)
    {
        hold(p_mission, B5MI_R_STALL, 0U, now);
        return false;
    }
    return true;
}

/* ---- Stages ---------------------------------------------------------------
 */

static void reset_results (b5_mission_t * p_mission)
{
    p_mission->width_d0 = 0U;
    p_mission->width_end = 0U;
    p_mission->width_attempted = false;
    p_mission->direction_side = B5_SIDE_NONE;
    p_mission->direction_basis = B5MI_DIR_NONE;
    p_mission->direction_decision = B5EP_DECISION_HOLD;
    p_mission->direction_end = 0U;
    p_mission->direction_attempted = false;
    p_mission->bypass_range_mm = 0U;
    p_mission->approach_legs = 0U;
    p_mission->approach_start_mm = 0U;
    p_mission->width_started = false;
    p_mission->width_policy = B5MI_WIDTH_NONE;
    p_mission->width_gate_mm = 0U;
    p_mission->steer_mm = 0.0f;
    p_mission->steer_decideg = 0;
    p_mission->steer_max_decideg = 0U;
    p_mission->steer_saturated = 0U;
    p_mission->align_steps = 0U;
    p_mission->align_heading_deg = 0.0f;
    p_mission->reason = B5MI_R_NONE;
    p_mission->module_reason = 0U;
}

static void guard_tick (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now)
{
    b5_ping_t ping;
    bool following =
        (MOTION_AUTO == status.mode) && (MOTION_BUDDY3 == status.owner);
    bool taken = watch_ping(p_mission, now, &ping);
    /* Straight-only takeover gate, evaluated while Buddy3 still owns the
     * motion. Losing it discards every qualifying echo counted so far. */
    uint8_t gate = b5_straight_check(&p_mission->straight, now);
    bool eligible = (B5S_STRAIGHT == gate);
    p_mission->gate_reason = gate;
    if (!eligible)
    {
        p_mission->confirm = 0U;
    }
    if (!following)
    {
        p_mission->confirm = 0U;
        p_mission->close_count = 0U;
        return;
    }
    if (taken)
    {
        bool echo = (B5_PING_ECHO == ping.cls);
        bool close = (echo) && (ping.range_mm <= (uint32_t)B5MI_CLOSE_HOLD_MM);
        bool in_range =
            (echo) && (ping.range_mm <= (uint32_t)B5MI_CONFIRM_RANGE_MM);
        if (p_mission->rearm)
        {
            bool clear = (B5_PING_FAULT != ping.cls) &&
                         ((B5_PING_NO_RETURN == ping.cls) || (!in_range));
            if (!clear)
            {
                p_mission->clear_timing = false;
            }
            else if (!p_mission->clear_timing)
            {
                p_mission->clear_timing = true;
                p_mission->clear_since = now;
            }
            else if ((now - p_mission->clear_since) >=
                     (uint64_t)B5MI_REARM_CLEAR_US)
            {
                p_mission->rearm = false;
            }
            if (close)
            {
                /* v0.6.1 rule: an obstacle inside the threshold while
                 * re-arming stops the robot rather than starting a probe. */
                hold(p_mission, B5MI_R_SENSOR, 0U, now);
                return;
            }
        }
        else
        {
            /* Only a ping REQUESTED after the gate opened qualifies, so an
             * echo from before (or across) a curve never carries over. */
            bool fresh = (eligible) && (p_mission->taken_requested >=
                                        p_mission->straight.open_us);
            p_mission->close_count = close ? p_mission->close_count + 1U : 0U;
            if ((!in_range) || (!fresh))
            {
                p_mission->confirm = 0U;
            }
            else if (p_mission->confirm < (unsigned)B5MI_TRIGGER_CONFIRM)
            {
                p_mission->confirm++;
            }
            if ((!eligible) &&
                (p_mission->close_count >= (unsigned)B5MI_TRIGGER_CONFIRM))
            {
                /* Close obstacle where the manoeuvre must not start: stop
                 * and hold; width, direction and bypass stay blocked. */
                hold(p_mission, B5MI_R_NOT_STRAIGHT, gate, now);
                return;
            }
            if ((p_mission->confirm >= (unsigned)B5MI_TRIGGER_CONFIRM) &&
                (ping.range_mm <= (uint32_t)B5MI_STRAIGHT_STOP_MM))
            {
                uint32_t lease = 0U;
                if (nav_request_takeover(MOTION_BUDDY5, &lease) != MOTION_OK)
                {
                    hold(p_mission, B5MI_R_LEASE, 0U, now);
                    return;
                }
                reset_results(p_mission);
                p_mission->lease = lease;
                p_mission->trigger_mm = (uint16_t)ping.range_mm;
                p_mission->run++;
                p_mission->sensor.cancel();
                enter(p_mission, B5MI_HALTING, now);
                return;
            }
        }
    }
    unsigned failure = watch_failure(p_mission, now);
    if (failure != 0U)
    {
        hold(p_mission, B5MI_R_SENSOR, failure, now);
    }
}

static void width_begin (b5_mission_t * p_mission, uint64_t now)
{
    drain_width(p_mission);
    if (!b5_wp_begin(&p_mission->width, now))
    {
        hold(p_mission, B5MI_R_WIDTH_BUSY, 0U, now);
        return;
    }
    if (p_mission->width.lease != p_mission->lease)
    {
        b5_wp_cancel(&p_mission->width, now);
        hold(p_mission, B5MI_R_LEASE, 0U, now);
        return;
    }
    p_mission->width_attempted = true;
    p_mission->width_started = true;
    p_mission->width_policy = B5MI_WIDTH_RUNNING;
    enter(p_mission, B5MI_WIDTH, now);
}

/* Width policy (review 2026-11-19): the frozen probe is started only when a
 * fresh CLEAN triplet (three echoes within 15 mm) lies inside its own REF
 * window. A clean range outside that window is a deliberate, labelled skip
 * with no motion. Once started, any non-COMPLETE end (REFERENCE included)
 * holds; its D3 150 mm assumption is still displayed. */
static void width_decide (b5_mission_t * p_mission, uint64_t now)
{
    uint16_t range = p_mission->range.range_mm;
    p_mission->width_gate_mm = range;
    if ((range >= (uint16_t)B5WP_D0_MIN_MM) &&
        (range <= (uint16_t)B5WP_D0_MAX_MM))
    {
        width_begin(p_mission, now);
        return;
    }
    p_mission->width_policy = B5MI_WIDTH_SKIPPED;
    p_mission->width_started = true;
    enter(p_mission, B5MI_APPROACH, now); /* nothing moved since the halt */
}

static void width_tick (b5_mission_t * p_mission, uint64_t now)
{
    b5_wp_tick(&p_mission->width, now);
    if (!b5_wp_ready(&p_mission->width))
    {
        return;
    }
    p_mission->width_end = (uint8_t)p_mission->width.end;
    p_mission->width_d0 = p_mission->width.d0;
    /* An attempted probe that did not complete holds, REFERENCE included:
     * END_REFERENCE alone is not evidence of a clean skip. The probe's D3
     * (150 mm PROJECT_RANGE_ASSUMPTION) remains the displayed number. */
    if (B5WP_END_COMPLETE != p_mission->width.end)
    {
        p_mission->width_policy = B5MI_WIDTH_FAILED;
        hold(p_mission, B5MI_R_WIDTH_ABORT, p_mission->width.end, now);
        return;
    }
    p_mission->width_policy = B5MI_WIDTH_MEASURED;
    p_mission->align_heading_deg = p_mission->width.heading_deg;
    servo_front(p_mission, now);
    enter(p_mission, B5MI_ALIGN, now);
}

/* sub 0 trim decision, 1 trim turn, 2 stationary evidence, 3 search turn. */
static void align_tick (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now)
{
    motion_command_status_t command;
    bool observation = fresh_observation(p_mission);
    if ((now - p_mission->stage_since) > (uint64_t)B5MI_ALIGN_LIMIT_US)
    {
        hold(p_mission, B5MI_R_TIMEOUT, 0U, now);
        return;
    }
    switch (p_mission->sub)
    {
    case 0U:
    {
        if (!still(status))
        {
            return;
        }
        float residual = p_mission->align_heading_deg;
        if (fabsf(residual) * 100.0f > (float)B5MI_ALIGN_TRIM_MAX_CDEG)
        {
            hold(p_mission, B5MI_R_MOVE, 0U, now);
            return;
        }
        if (fabsf(residual) < 1.0f)
        {
            p_mission->sub = 2U;
            p_mission->sub_since = now;
            return;
        }
        if (start_move(p_mission, true, -residual, B5MI_ALIGN_SPEED, now))
        {
            p_mission->sub = 1U;
        }
        return;
    }
    case 1U:
    case 3U:
    {
        int done = follow_move(p_mission, status, now, &command);
        if (done <= 0)
        {
            return;
        }
        p_mission->align_heading_deg += command.rotation_deg;
        p_mission->sub = 2U;
        p_mission->sub_since = now;
        p_mission->evidence_timing = false;
        return;
    }
    default:
        break;
    }
    if (!still(status))
    {
        hold(p_mission, B5MI_R_MOVE, 0U, now);
        return;
    }
    if (observation)
    {
        if (!line_evidence(p_mission))
        {
            p_mission->evidence_timing = false;
        }
        else if (!p_mission->evidence_timing)
        {
            p_mission->evidence_timing = true;
            p_mission->evidence_since = now;
        }
    }
    if ((p_mission->evidence_timing) &&
        ((now - p_mission->evidence_since) >= (uint64_t)B5MI_EVIDENCE_US))
    {
        range_begin(p_mission, B5MI_APPROACH, now);
        return;
    }
    if ((now - p_mission->sub_since) < (uint64_t)B5MI_EVIDENCE_WAIT_US)
    {
        return;
    }
    if (p_mission->align_steps >= (unsigned)B5MI_ALIGN_STEPS)
    {
        hold(p_mission, B5MI_R_NO_LINE_ALIGN, 0U, now);
        return;
    }
    float goal =
        g_align_deg[p_mission->align_steps++] - p_mission->align_heading_deg;
    if (fabsf(goal) < 1.0f)
    {
        p_mission->sub_since = now;
        return;
    }
    if (start_move(p_mission, true, goal, B5MI_ALIGN_SPEED, now))
    {
        p_mission->sub = 3U;
    }
}

static void direction_begin (b5_mission_t * p_mission, uint64_t now)
{
    drain_edge(p_mission);
    if (!b5_ep_begin(&p_mission->edge, now))
    {
        hold(p_mission, B5MI_R_DIRECTION_BUSY, 0U, now);
        return;
    }
    if (p_mission->edge.lease != p_mission->lease)
    {
        b5_ep_cancel(&p_mission->edge, now);
        hold(p_mission, B5MI_R_LEASE, 0U, now);
        return;
    }
    p_mission->direction_attempted = true;
    enter(p_mission, B5MI_DIRECTION, now);
}

/* Bounded line steering during the owned approach. Buddy3's own edge law
 * (analog_demand(): left sensor on the strip's right edge at level 500,
 * deadband 60, right-sensor guard) gives a wheel differential d at its 45
 * mm/s base. Scaled to the approach speed (same turn per distance) it moves
 * the drive's heading setpoint at the follower's yaw rate; the coordinator
 * keeps the executor's own heading hold, goal, timeout, settle and 12 deg
 * fault, and the setpoint is capped at +/-8 deg and lapses if not refreshed.
 * The follower's stop decisions (both-black/white timers) are not used: the
 * mission's 40 mm line-loss hold and sonar stop are unchanged. Returns false
 * only if the coordinator refused for a reason other than end-of-drive. */
static bool steer_step (b5_mission_t * p_mission, motion_status_t status,
                        uint64_t now)
{
    if ((MOTION_DRIVING != status.mode) || (now <= p_mission->steer_us))
    {
        p_mission->steer_us = now;
        return true;
    }
    float delta_time = (float)(now - p_mission->steer_us) / 1000000.0f;
    p_mission->steer_us = now;
    if (delta_time > 0.05f)
    {
        delta_time = 0.05f; /* never integrate across a stall/gap */
    }
    uint32_t travelled =
        status.progress_mm > 0.0f ? (uint32_t)status.progress_mm : 0U;
    analog_demand_t demand =
        analog_demand(&p_mission->steer_tracker, &g_level_identity,
                      p_mission->line[0], p_mission->line[1], now, travelled);
    float differential = 0.0f; /* + = left wheel faster = turn right */
    if ((AL_BOTH_BLACK != demand.decision) && (!demand.stop))
    {
        differential = 0.5f * (float)(demand.target[0] - demand.target[1]);
    }
    else if ((demand.stop) && (AL_LOST == demand.decision))
    {
        /* Keep seeking left at the follower's full differential while the
         * mission's own line-loss distance runs. */
        differential = -(float)LINE_DIFFERENTIAL_MM_S;
    }
    float scale = (float)B5MI_APPROACH_SPEED / (float)LINE_BASE_MM_S;
    p_mission->steer_mm += -2.0f * differential * scale * delta_time;
    float limit = (float)B5MI_STEER_MAX_DECIDEG / 10.0f * 110.0f *
                  3.14159265358979323846f / 180.0f;
    bool saturated = fabsf(p_mission->steer_mm) >= limit;
    if (p_mission->steer_mm > limit)
    {
        p_mission->steer_mm = limit;
    }
    else if (p_mission->steer_mm < -limit)
    {
        p_mission->steer_mm = -limit;
    }
    if ((saturated) && (p_mission->steer_saturated < 255U))
    {
        p_mission->steer_saturated++;
    }
    float degrees =
        p_mission->steer_mm * 180.0f / (110.0f * 3.14159265358979323846f);
    long decideg = lroundf(degrees * 10.0f);
    decideg = decideg > B5MI_STEER_MAX_DECIDEG    ? B5MI_STEER_MAX_DECIDEG
              : decideg < -B5MI_STEER_MAX_DECIDEG ? -B5MI_STEER_MAX_DECIDEG
                                                  : decideg;
    p_mission->steer_decideg = (int16_t)decideg; /* |decideg| <= 80 */
    unsigned magnitude = (unsigned)(decideg < 0 ? -decideg : decideg);
    if (magnitude > p_mission->steer_max_decideg)
    {
        p_mission->steer_max_decideg = (uint8_t)magnitude; /* <= 80 */
    }
    motion_result_t result = nav_steer_drive(MOTION_BUDDY5, p_mission->lease,
                                             (float)decideg / 10.0f);
    return (MOTION_OK == result) || (MOTION_BUSY == result);
}

/* Entered with a fresh stationary range. sub 0 decide, 1 driving, 2 halting
 * on range, 3 halting on lost line, 4 halting on sonar failure. */
static void approach_tick (b5_mission_t * p_mission, motion_status_t status,
                           uint64_t now)
{
    motion_command_status_t command;
    if ((now - p_mission->stage_since) > (uint64_t)B5MI_APPROACH_LIMIT_US)
    {
        hold(p_mission, B5MI_R_TIMEOUT, 0U, now);
        return;
    }
    if (0U == p_mission->sub)
    {
        uint16_t range = p_mission->range.range_mm;
        if (0U == p_mission->approach_start_mm)
        {
            p_mission->approach_start_mm = range;
        }
        if (range <= (uint16_t)B5MI_APPROACH_ACCEPT_MM)
        {
            direction_begin(p_mission, now);
            return;
        }
        if (p_mission->approach_legs >= (unsigned)B5MI_APPROACH_MAX_LEGS)
        {
            hold(p_mission, B5MI_R_APPROACH_RANGE, 0U, now);
            return;
        }
        float goal = (float)range - (float)B5MI_APPROACH_TARGET_MM;
        if (goal > (float)B5MI_APPROACH_MAX_LEG_MM)
        {
            hold(p_mission, B5MI_R_APPROACH_RANGE, 0U, now);
            return;
        }
        p_mission->approach_legs++;
        p_mission->approach_last_line_mm = 0.0f;
        p_mission->steer_tracker = (analog_tracker_t){0};
        p_mission->steer_mm = 0.0f;
        p_mission->steer_us = now;
        p_mission->fault_run = 0U;
        p_mission->last_ping = now;
        if (start_move(p_mission, false, goal, B5MI_APPROACH_SPEED, now))
        {
            p_mission->sub = 1U;
        }
        return;
    }
    if (!progress_ok(p_mission, status, now))
    {
        return;
    }
    int done = follow_move(p_mission, status, now, &command);
    if (done < 0)
    {
        return;
    }
    if (done > 0)
    {
        if (3U == p_mission->sub)
        {
            hold(p_mission, B5MI_R_NO_LINE_APPROACH, 0U, now);
        }
        else if (4U == p_mission->sub)
        {
            hold(p_mission, B5MI_R_SENSOR, 0U, now);
        }
        else
        {
            range_begin(p_mission, B5MI_APPROACH, now);
        }
        return;
    }
    if (p_mission->sub != 1U)
    {
        return; /* our own stop is settling */
    }
    if (fresh_observation(p_mission))
    {
        if (line_mask(p_mission) != 0U)
        {
            p_mission->approach_last_line_mm = status.progress_mm;
        }
        else if ((status.progress_mm - p_mission->approach_last_line_mm) >
                 (float)B5MI_APPROACH_NO_LINE_MM)
        {
            halt_move(p_mission);
            p_mission->sub = 3U;
            return;
        }
        if ((p_mission->steering) && (!steer_step(p_mission, status, now)))
        {
            halt_move(p_mission);
            hold(p_mission, B5MI_R_MOVE, 0U, now);
            return;
        }
    }
    b5_ping_t ping;
    if (watch_ping(p_mission, now, &ping) && (B5_PING_ECHO == ping.cls) &&
        (ping.range_mm <= (uint32_t)B5MI_APPROACH_STOP_MM))
    {
        halt_move(p_mission);
        p_mission->sub = 2U;
        return;
    }
    if (watch_failure(p_mission, now) != 0U)
    {
        halt_move(p_mission);
        p_mission->sub = 4U;
    }
}

/* Complete, healthy probe whose two sides are equal (OPEN/OPEN, CROSSED/
 * CROSSED or REMAINS/REMAINS): every gate of edge_probe.c decide() passed and
 * only the preference is absent. */
static bool direction_no_preference (const B5EdgeProbe * p_edge)
{
    const B5EPSummary * p_summary = &p_edge->summary;
    uint16_t flags = B5EP_FLAG_REF_OK | B5EP_FLAG_FINAL_REF_OK |
                     B5EP_FLAG_LEFT_BACK_OK | B5EP_FLAG_RIGHT_BACK_OK |
                     B5EP_FLAG_SERVO_IN_RANGE;
    int final_heading = p_summary->final_heading_cdeg;
    return (B5EP_END_COMPLETE == p_summary->end) &&
           (B5EP_RECORD_COUNT == p_summary->records) &&
           (0U == p_summary->lost) && (flags == (p_summary->flags & flags)) &&
           (final_heading <= 300) && (final_heading >= -300) &&
           (B5EP_SIDE_UNKNOWN != p_summary->state_l) &&
           (p_summary->state_l == p_summary->state_r);
}

static void direction_tick (b5_mission_t * p_mission, uint64_t now)
{
    b5_ep_tick(&p_mission->edge, now);
    if (!b5_ep_ready(&p_mission->edge))
    {
        return;
    }
    const B5EPSummary * p_summary = &p_mission->edge.summary;
    p_mission->direction_end = (uint8_t)p_mission->edge.end;
    p_mission->direction_decision = p_summary->decision;
    if (B5EP_END_COMPLETE != p_mission->edge.end)
    {
        p_mission->direction_basis = B5MI_DIR_HELD;
        hold(p_mission, B5MI_R_DIRECTION_ABORT, p_mission->edge.end, now);
        return;
    }
    if (B5EP_DECISION_LEFT == p_summary->decision)
    {
        p_mission->direction_side = B5_SIDE_LEFT;
        p_mission->direction_basis = B5MI_DIR_EVIDENCE;
    }
    else if (B5EP_DECISION_RIGHT == p_summary->decision)
    {
        p_mission->direction_side = B5_SIDE_RIGHT;
        p_mission->direction_basis = B5MI_DIR_EVIDENCE;
    }
    else if (direction_no_preference(&p_mission->edge))
    {
        p_mission->direction_side = (uint8_t)B5MI_DEFAULT_SIDE;
        p_mission->direction_basis = B5MI_DIR_DEFAULT;
    }
    else
    {
        p_mission->direction_basis = B5MI_DIR_HELD;
        hold(p_mission, B5MI_R_DIRECTION_UNKNOWN, 0U, now);
        return;
    }
    range_begin(p_mission, B5MI_BYPASS, now);
}

static void bypass_begin (b5_mission_t * p_mission, uint64_t now)
{
    uint16_t range = p_mission->range.range_mm;
    p_mission->bypass_range_mm = range;
    /* Edge-probe encoder pose (x right, + rotation left) -> route frame
     * (+LEFT). Model values, not physical pose feedback. */
    float heading0 = p_mission->edge.heading_deg;
    float lateral0 = -p_mission->edge.net_x_mm;
    if (!b5_auto_start(&p_mission->bypass, p_mission->lease,
                       p_mission->direction_side, range, heading0, lateral0,
                       now))
    {
        hold(p_mission,
             B5A_BLOCKED == p_mission->bypass.state ? B5MI_R_BYPASS_BLOCKED
                                                    : B5MI_R_BYPASS_FAULT,
             p_mission->bypass.reason, now);
        return;
    }
    p_mission->stage = B5MI_BYPASS; /* range_tick already entered BYPASS */
}

static void bypass_tick (b5_mission_t * p_mission, uint64_t now)
{
    b5_auto_tick(&p_mission->bypass, now);
    if (B5A_DONE == p_mission->bypass.state)
    {
        /* Handed back: line following owns the robot again. Re-arm only
         * after the front has been clear for 1 s (v0.6.1 rule). */
        p_mission->rearm = true;
        p_mission->clear_timing = false;
        p_mission->lease = 0U;
        p_mission->fault_run = 0U;
        p_mission->last_ping = now;
        servo_front(p_mission, now);
        enter(p_mission, B5MI_GUARD, now);
        return;
    }
    if ((B5A_BLOCKED == p_mission->bypass.state) ||
        (B5A_FAULT == p_mission->bypass.state))
    {
        hold(p_mission,
             B5A_BLOCKED == p_mission->bypass.state ? B5MI_R_BYPASS_BLOCKED
                                                    : B5MI_R_BYPASS_FAULT,
             p_mission->bypass.reason, now);
    }
}

/* ---- Public ---------------------------------------------------------------
 */

void b5_mission_init (b5_mission_t * p_mission, const B5SensorPort * p_port,
                      int (*p_applied)(void))
{
    memset(p_mission, 0, sizeof(*p_mission));
    p_mission->sensor = *p_port;
    b5_wp_init(&p_mission->width, p_port);
    p_mission->width.applied = p_applied;
    b5_ep_init(&p_mission->edge, p_port);
    p_mission->edge.applied = p_applied;
    b5_auto_init(&p_mission->bypass, p_port);
    p_mission->stage = B5MI_IDLE;
    p_mission->steering = true;
}

bool b5_mission_begin (b5_mission_t * p_mission, uint64_t now)
{
    if (b5_mission_active(p_mission))
    {
        return false;
    }
    if (!b5_wp_ready(&p_mission->width))
    {
        b5_wp_cancel(&p_mission->width, now);
    }
    if (!b5_ep_ready(&p_mission->edge))
    {
        b5_ep_cancel(&p_mission->edge, now);
    }
    p_mission->session = true;
    p_mission->rearm = false;
    p_mission->clear_timing = false;
    p_mission->lease = 0U;
    p_mission->fault_run = 0U;
    p_mission->last_ping = now;
    p_mission->observed = now;
    p_mission->used_observation = 0U;
    reset_results(p_mission);
    b5_straight_reset(&p_mission->straight, B5S_UNKNOWN);
    p_mission->gate_reason = B5S_UNKNOWN;
    p_mission->close_count = 0U;
    servo_front(p_mission, now);
    enter(p_mission, B5MI_GUARD, now);
    return true;
}

void b5_mission_line (b5_mission_t * p_mission,
                      const b5_line_sample_t * p_sample)
{
    b5_straight_step(&p_mission->straight, p_sample);
}

void b5_mission_observe (b5_mission_t * p_mission, uint64_t now, uint16_t left,
                         uint16_t right)
{
    p_mission->observed = now;
    p_mission->line[0] = left;
    p_mission->line[1] = right;
    b5_auto_observe(&p_mission->bypass, now, left, right);
}

bool b5_mission_active (const b5_mission_t * p_mission)
{
    switch (p_mission->stage)
    {
    case B5MI_HALTING:
    case B5MI_WIDTH:
    case B5MI_ALIGN:
    case B5MI_APPROACH:
    case B5MI_RANGE:
    case B5MI_DIRECTION:
    case B5MI_BYPASS:
    case B5MI_HOLD:
        return true;
    default:
        return false;
    }
}

void b5_mission_cancel (b5_mission_t * p_mission, uint64_t now)
{
    bool active = b5_mission_active(p_mission);
    if (!b5_wp_ready(&p_mission->width))
    {
        b5_wp_cancel(&p_mission->width, now);
    }
    if (!b5_ep_ready(&p_mission->edge))
    {
        b5_ep_cancel(&p_mission->edge, now);
    }
    if (p_mission->bypass.session)
    {
        b5_auto_cancel(&p_mission->bypass, now);
    }
    motion_status_t status = nav_get_status();
    if ((status.session_active) && (MOTION_BUDDY5 == status.owner) &&
        (status.mode != MOTION_FAULT))
    {
        (void)nav_request_stop(MOTION_BUDDY5);
    }
    servo_front(p_mission, now);
    p_mission->session = false;
    if (active || (B5MI_GUARD == p_mission->stage))
    {
        if (B5MI_HOLD != p_mission->stage)
        {
            p_mission->reason = B5MI_R_CANCELLED;
        }
        enter(p_mission, active ? B5MI_CANCELLED : B5MI_IDLE, now);
    }
}

void b5_mission_tick (b5_mission_t * p_mission, uint64_t now)
{
    if (!p_mission->session)
    {
        return;
    }
    motion_status_t status = nav_get_status();
    if (!status.session_active)
    {
        b5_mission_cancel(p_mission, now);
        return;
    }
    switch (p_mission->stage)
    {
    case B5MI_GUARD:
        guard_tick(p_mission, status, now);
        return;
    case B5MI_WIDTH:
        width_tick(p_mission, now);
        return;
    case B5MI_DIRECTION:
        direction_tick(p_mission, now);
        return;
    case B5MI_BYPASS:
        bypass_tick(p_mission, now);
        return;
    case B5MI_HALTING:
    case B5MI_ALIGN:
    case B5MI_APPROACH:
    case B5MI_RANGE:
        break;
    default:
        return; /* IDLE, HOLD, CANCELLED: stopped, nothing to step */
    }
    if (!owns(p_mission, status))
    {
        hold(p_mission, B5MI_R_LEASE, 0U, now);
        return;
    }
    if ((now < p_mission->observed) ||
        ((now - p_mission->observed) > (uint64_t)B5MI_OBSERVATION_MAX_US))
    {
        hold(p_mission, B5MI_R_STALE, 0U, now);
        return;
    }
    if (B5MI_HALTING == p_mission->stage)
    {
        if ((now - p_mission->stage_since) > (uint64_t)B5MI_HALT_LIMIT_US)
        {
            hold(p_mission, B5MI_R_TIMEOUT, 0U, now);
        }
        else if (still(status))
        {
            /* Decide the width policy from a fresh stationary triplet; a
             * faulty or inconsistent triplet holds (RANGE), never skips. */
            p_mission->takeover_mask = (uint8_t)line_mask(p_mission);
            range_begin(p_mission, B5MI_WIDTH, now);
        }
        return;
    }
    if (B5MI_ALIGN == p_mission->stage)
    {
        align_tick(p_mission, status, now);
        return;
    }
    if (B5MI_APPROACH == p_mission->stage)
    {
        approach_tick(p_mission, status, now);
        return;
    }
    range_tick(p_mission, status, now);
    if ((B5MI_WIDTH == p_mission->stage) && (!p_mission->width_started))
    {
        width_decide(p_mission, now);
    }
    else if ((B5MI_BYPASS == p_mission->stage) && (!p_mission->bypass.session))
    {
        bypass_begin(p_mission, now);
    }
}

const char * b5_mission_stage_name (unsigned stage)
{
    static const char * const names[B5MI_STAGE_COUNT] = {
        "IDLE",      "GUARD",  "HALTING", "WIDTH",     "ALIGN", "APPROACH",
        "DIRECTION", "BYPASS", "HOLD",    "CANCELLED", "RANGE"};
    return stage < (unsigned)B5MI_STAGE_COUNT ? names[stage] : "INVALID";
}

const char * b5_mission_reason_name (unsigned reason)
{
    static const char * const names[B5MI_R_COUNT] = {"NONE",
                                                     "CANCELLED",
                                                     "SENSOR",
                                                     "LEASE",
                                                     "MOVE",
                                                     "TIMEOUT",
                                                     "STALE",
                                                     "STALL",
                                                     "WIDTH_ABORT",
                                                     "WIDTH_BUSY",
                                                     "NO_LINE_ALIGN",
                                                     "NO_LINE_APPROACH",
                                                     "APPROACH_RANGE",
                                                     "DIRECTION_ABORT",
                                                     "DIRECTION_UNKNOWN",
                                                     "DIRECTION_BUSY",
                                                     "RANGE",
                                                     "BYPASS_BLOCKED",
                                                     "BYPASS_FAULT",
                                                     "NOT_STRAIGHT"};
    return reason < (unsigned)B5MI_R_COUNT ? names[reason] : "INVALID";
}

/* End of file. */
