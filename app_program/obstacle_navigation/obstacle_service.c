/**
 * @file obstacle_service.c
 * @brief Buddy5 RTOS adapter: sonar/servo ownership, mission stepping hooks
 * and bounded copied telemetry.
 * @details Who touches what (all on core1, so DI/EI is a complete critical
 * section for state shared between the IRQ, the motion task and the Buddy5
 * task):
 *  - g_mission belongs to the motion task, which steps it once per 10 ms
 *    frame directly after motion_update(). The Buddy5 task only copies its
 *    telemetry queues/flags under DI and drops an entry after the bridge
 *    accepted the copy (identity-checked, so a concurrent new run is safe).
 *  - g_echo, g_servo and the last-ping record are shared by the GP17 IRQ, the
 *    motion task and the Buddy5 task; every access is under DI.
 *  - Cancellation is the one cross-core request (Stop may come from core0);
 *    it uses a spin lock and is consumed by the motion task.
 * Telemetry is copied into the existing bounded network TX queue with
 * TMO_POL; this file never calls lwIP/CYW43 and never waits.
 * @see app_program/obstacle_navigation/README.md
 */
#include "obstacle_service.h"
#include "b5_mission.h"
#include "b5_mission_pack.h"
#include "../network_bridge/network_bridge.h"
#include "extensions/buddy5_obstacles/sonar_async.h"
#include "extensions/buddy5_obstacles/servo_slew.h"
#include "extensions/buddy5_obstacles/signed_continuous.h"
#include "extensions/buddy5_obstacles/width_probe_pack.h"
#include "extensions/buddy5_obstacles/b5_event_pack.h"

enum
{
    RB5_PING_HISTORY = 8,
    RB5_STAGE_MIN_US = 100000 /* stage-change status no faster than this */
};

typedef struct
{
    uint32_t width_us;
    uint32_t rise_us;
    uint32_t gap_us;
    uint16_t range_mm;
    uint8_t cls;
    uint8_t why;
} rb5_ping_record_t;

typedef struct
{
    uint32_t triggers;
    uint32_t edges;
    uint32_t pulse_min;
    uint32_t pulse_max;
    uint32_t rise_min;
    uint32_t rise_max;
    uint32_t gap_min;
    uint32_t gap_max;
    uint32_t classes[3];
    uint32_t frames;
    uint32_t refused;
    uint32_t status_frames;
    uint64_t last_trigger_us;
    rb5_ping_record_t last[RB5_PING_HISTORY];
    unsigned head;
    uint32_t pending_rise_us;
    uint32_t pending_gap_us;
} rb5_diagnostics_t;

typedef struct
{
    uint64_t at_us;
    uint16_t range_mm;
    uint8_t cls;
    uint8_t why;
    int16_t applied_dd;
    bool valid;
} rb5_last_ping_t;

static b5_mission_t g_mission;
static T_SPLOCK g_request_lock;
static uint32_t g_cancel_requests;
static uint32_t g_report_requested;
static uint32_t g_cancel_seen;
static bool g_handback;
static B5Echo g_echo;
static B5ServoSlew g_servo;
static rb5_last_ping_t g_last_ping;
static rb5_diagnostics_t g_diag;
static uint32_t g_hardware_ready;
static uint32_t g_ready_flag;
static uint32_t g_ready_ms;
/* Idle start gate (motion task only). */
static unsigned g_idle_fault_run;
static uint64_t g_idle_last_good;
/* Signed Buddy5 wheel controller (motion task only). */
static B5SignedContinuous g_drive[2];
static signed_wheel_t g_legacy[2];
static int32_t g_previous_output[2];
/* Telemetry bookkeeping (Buddy5 task only). */
static uint16_t g_width_run_seen;
static uint8_t g_width_admitted;
static uint8_t g_stage_sent = 0xffU;
static uint64_t g_status_due;
static uint64_t g_stage_sent_us;
static uint64_t g_frame_due;

static UINT enter (void)
{
    UINT key;
    DI(key);
    return key;
}

static void leave (UINT key)
{
    EI(key);
}

/* ---- B5SensorPort: called by the motion task (mission/probes/bypass). */

static void port_angle (int angle_dd, uint64_t now)
{
    UINT key = enter();
    b5_servo_slew_request(&g_servo, angle_dd, now); /* never restarts cadence */
    leave(key);
}

static bool port_settled (uint64_t now)
{
    UINT key = enter();
    bool settled = b5_servo_slew_settled(&g_servo, now);
    leave(key);
    return settled;
}

static bool port_request (uint64_t now)
{
    UINT key = enter();
    bool accepted = false;
    if ((B5E_IDLE == g_echo.phase) && (now >= g_echo.next_ping))
    {
        board_obstacle_echo_ack(); /* SDK: acknowledge stale edges first */
        accepted = b5_echo_request(&g_echo, now);
    }
    leave(key);
    return accepted;
}

static bool port_take (b5_ping_t * p_out)
{
    UINT key = enter();
    bool taken = b5_echo_take(&g_echo, p_out);
    if (taken)
    {
        unsigned cls = p_out->cls < 3U ? p_out->cls : 2U;
        g_diag.classes[cls]++;
        g_diag.last[g_diag.head] =
            (rb5_ping_record_t){.width_us = p_out->width_us,
                                .rise_us = g_diag.pending_rise_us,
                                .gap_us = g_diag.pending_gap_us,
                                .range_mm = (uint16_t)p_out->range_mm,
                                .cls = (uint8_t)cls,
                                .why = (uint8_t)p_out->why};
        g_diag.head = (g_diag.head + 1U) % (unsigned)RB5_PING_HISTORY;
        g_diag.pending_rise_us = 0U;
        g_last_ping = (rb5_last_ping_t){.at_us = board_time_us(),
                                        .range_mm = (uint16_t)p_out->range_mm,
                                        .cls = (uint8_t)cls,
                                        .why = (uint8_t)p_out->why,
                                        .applied_dd = (int16_t)g_servo.applied,
                                        .valid = true};
    }
    leave(key);
    return taken;
}

static void port_cancel (void)
{
    UINT key = enter();
    g_echo.phase = B5E_IDLE; /* keeps next_ping: 60 ms spacing survives */
    board_obstacle_echo_ack();
    leave(key);
}

/* Applied PWM command (0.1 deg), never a measured horn angle. */
static int port_applied (void)
{
    UINT key = enter();
    int applied = g_servo.applied;
    leave(key);
    return applied;
}

void rb5_init (void)
{
    b5_echo_init(&g_echo);
    b5_servo_slew_init(&g_servo, RB5_SERVO_LIMIT_DD, B5_SERVO_STEP_MS,
                       B5_SERVO_SPEED_DD_S, B5_SERVO_SETTLE_MS, 0U);
    B5SensorPort port = {port_angle, port_settled, port_request,
                         port_take,  port_cancel,  NULL};
    b5_mission_init(&g_mission, &port, port_applied);
}

uint32_t rb5_gpio_init (void)
{
    uint32_t mask = board_obstacle_init();
    UINT key = enter();
    b5_servo_slew_init(&g_servo, RB5_SERVO_LIMIT_DD, B5_SERVO_STEP_MS,
                       B5_SERVO_SPEED_DD_S, B5_SERVO_SETTLE_MS,
                       board_time_us());
    leave(key);
    __atomic_store_n(&g_hardware_ready, 1U, __ATOMIC_RELEASE);
    return mask;
}

/* GPIO IRQ on core1: timestamp only; the echo state machine is O(1). */
void rb5_echo_isr (uint32_t edges, uint64_t now)
{
    bool rise = (edges & RB5_ECHO_RISE) != 0U;
    bool fall = (edges & RB5_ECHO_FALL) != 0U;
    g_diag.edges++;
    if ((rise) && (B5E_RISE == g_echo.phase) && (now >= g_echo.triggered))
    {
        g_diag.pending_rise_us = (uint32_t)(now - g_echo.triggered);
        rb5_track(g_diag.pending_rise_us, &g_diag.rise_min, &g_diag.rise_max);
    }
    b5_echo_edge(&g_echo, now, rise, fall);
}

/* The Buddy5 task normally advances sonar and servo every ~1 ms.
 * During line-following watch, the motion frame also advances pending pings.
 * Both tasks are on core1. Disable dispatch through the trigger pulse so
 * only one task advances the transaction, while still allowing ECHO IRQs. */
static void hardware_tick (bool watch_assist)
{
    (void)tk_dis_dsp();
    UINT key = enter();
    if (watch_assist && (B5E_IDLE == g_echo.phase || B5E_READY == g_echo.phase))
    {
        leave(key);
        (void)tk_ena_dsp();
        return;
    }
    uint64_t now = board_time_us();
    unsigned pulse = 0U;
    if ((!watch_assist) && b5_servo_slew_tick(&g_servo, now))
    {
        pulse = rb5_servo_pulse_us(g_servo.applied);
    }
    bool trigger = b5_echo_tick(&g_echo, now, board_obstacle_echo_high());
    if (trigger)
    {
        g_diag.pending_gap_us = g_diag.last_trigger_us
                                    ? (uint32_t)(now - g_diag.last_trigger_us)
                                    : 0U;
        if (g_diag.last_trigger_us)
        {
            rb5_track(g_diag.pending_gap_us, &g_diag.gap_min, &g_diag.gap_max);
        }
        g_diag.last_trigger_us = now;
        g_diag.triggers++;
    }
    leave(key);
    if (pulse)
    {
        board_obstacle_servo_us(pulse);
    }
    uint32_t width = trigger ? board_obstacle_trigger() : 0U;
    (void)tk_ena_dsp();
    if (trigger)
    {
        key = enter();
        rb5_track(width, &g_diag.pulse_min, &g_diag.pulse_max);
        leave(key);
    }
}

static uint32_t requests (bool take_report, bool * p_report)
{
    UINT key;
    (void)ISpinLock(&g_request_lock, &key);
    uint32_t value = g_cancel_requests;
    if (take_report)
    {
        *p_report = g_report_requested != 0U;
        g_report_requested = 0U;
    }
    (void)ISpinUnlock(&g_request_lock, key);
    return value;
}

/* Motion task only. */
static void consume_cancel (uint64_t now)
{
    uint32_t requested = requests(false, NULL);
    if (requested != g_cancel_seen)
    {
        g_cancel_seen = requested;
        b5_mission_cancel(&g_mission, now);
    }
}

/* Idle ranging (no session): keeps a fresh centred start-gate result. Cannot
 * command motion. v0.6.1 b5_auto_ready() rule. */
static void idle_ranging (uint64_t now)
{
    if (!port_settled(now))
    {
        return;
    }
    b5_ping_t ping;
    if (port_take(&ping))
    {
        if (B5_PING_FAULT == ping.cls)
        {
            g_idle_fault_run++;
        }
        else
        {
            g_idle_fault_run = 0U;
            g_idle_last_good = now;
        }
    }
    (void)port_request(now);
}

static bool ready_now (uint64_t now)
{
    return (g_idle_last_good != 0U) && (now >= g_idle_last_good) &&
           ((now - g_idle_last_good) <= (uint64_t)RB5_READY_PING_US) &&
           (0U == g_idle_fault_run) && (port_settled(now)) &&
           (!b5_mission_active(&g_mission));
}

static void publish_ready (uint64_t now)
{
    bool ready = (!g_mission.session) && (ready_now(now));
    __atomic_store_n(&g_ready_ms, board_kernel_ms(), __ATOMIC_RELEASE);
    __atomic_store_n(&g_ready_flag, ready ? 1U : 0U, __ATOMIC_RELEASE);
}

/* ---- Motion-task hooks ------------------------------------------------- */

bool rb5_external (void)
{
    motion_status_t status = motion_get_status();
    if ((!status.session_active) || (MOTION_AUTO == status.mode))
    {
        g_handback = false;
    }
    return rb5_external_frame(&status, g_handback);
}

void rb5_frame (uint64_t now, const uint16_t * p_level,
                const b5_line_sample_t * p_line)
{
    if (!__atomic_load_n(&g_hardware_ready, __ATOMIC_ACQUIRE))
    {
        return;
    }
    consume_cancel(now);
    /* No DI here: the Buddy5 task (lower priority, same core) cannot run
     * inside this step, and its own copy/drop sections are DI-protected, so
     * it never observes a half-updated queue. Keeping the step outside DI
     * leaves the encoder and ECHO interrupts unaffected. */
    motion_status_t before = motion_get_status();
    if (g_mission.session)
    {
        if ((NULL == p_level) || (!before.session_active))
        {
            b5_mission_cancel(&g_mission, now);
        }
        else
        {
            b5_mission_observe(&g_mission, now, p_level[0], p_level[1]);
            b5_line_sample_t none = {.now_us = now, .valid = false};
            b5_mission_line(&g_mission, (NULL != p_line) ? p_line : &none);
            if (B5MI_GUARD == g_mission.stage)
            {
                hardware_tick(true);
            }
            b5_mission_tick(&g_mission, now);
        }
    }
    else
    {
        idle_ranging(now);
    }
    motion_status_t after = motion_get_status();
    if ((MOTION_BUDDY5 == before.owner) && (after.session_active) &&
        (MOTION_BUDDY3 == after.owner) && (after.mode != MOTION_AUTO))
    {
        g_handback = true; /* nav_resume_line() accepted; AUTO next update */
    }
    publish_ready(now);
}

bool rb5_held (void)
{
    return B5MI_HOLD == g_mission.stage;
}

bool rb5_owned (const motion_output_t * p_demand)
{
    motion_status_t status = motion_get_status();
    return rb5_owned_demand(&status, p_demand->external, g_handback);
}

bool rb5_take_handback (const motion_output_t * p_demand)
{
    bool resumed = (g_handback) && (p_demand->resume_navigation);
    if (resumed)
    {
        g_handback = false;
    }
    return resumed;
}

void rb5_power (const motion_output_t * p_demand, const float speed[2],
                float delta_time, uint64_t now, int32_t out[2])
{
    for (unsigned wheel = 0U; wheel < 2U; ++wheel)
    {
        if (p_demand->reset_controllers)
        {
            b5_signed_reset(&g_drive[wheel], &g_legacy[wheel]);
        }
        out[wheel] = b5_signed_step(&g_drive[wheel], &g_legacy[wheel], wheel,
                                    p_demand->target_mm_s[wheel], speed[wheel],
                                    delta_time, now, g_previous_output[wheel]);
    }
}

void rb5_power_committed (const int32_t applied[2])
{
    g_previous_output[0] = applied[0];
    g_previous_output[1] = applied[1];
}

void rb5_power_idle (void)
{
    for (unsigned wheel = 0U; wheel < 2U; ++wheel)
    {
        b5_signed_reset(&g_drive[wheel], &g_legacy[wheel]);
        g_previous_output[wheel] = 0;
    }
}

bool rb5_begin (uint64_t now)
{
    if (!__atomic_load_n(&g_hardware_ready, __ATOMIC_ACQUIRE))
    {
        return false;
    }
    g_cancel_seen = requests(false, NULL);
    if (!ready_now(now))
    {
        return false;
    }
    g_handback = false;
    rb5_power_idle();
    return b5_mission_begin(&g_mission, now);
}

/* ---- Any task ---------------------------------------------------------- */

bool rb5_start_allowed (void)
{
    uint32_t ready = __atomic_load_n(&g_ready_flag, __ATOMIC_ACQUIRE);
    uint32_t stamp = __atomic_load_n(&g_ready_ms, __ATOMIC_ACQUIRE);
    return (ready != 0U) &&
           ((board_kernel_ms() - stamp) <= (uint32_t)RB5_READY_MAX_MS);
}

void rb5_cancel (void)
{
    UINT key;
    (void)ISpinLock(&g_request_lock, &key);
    g_cancel_requests++;
    (void)ISpinUnlock(&g_request_lock, key);
}

/* Control (motion) task only: network Pause while Buddy5 holds the lease. */
bool rb5_owns_session (void)
{
    motion_status_t status = motion_get_status();
    return rb5_pause_cancels(&status);
}

void rb5_report_request (void)
{
    UINT key;
    (void)ISpinLock(&g_request_lock, &key);
    g_report_requested = 1U;
    (void)ISpinUnlock(&g_request_lock, key);
}

/* ---- Buddy5 task: telemetry ------------------------------------------- */

/* The MQTT consumer assigns the source-15 header sequence in queue order. */
static bool push (const uint8_t * p_frame, uint8_t length)
{
    if (rn_tx_push(RN_TLM, p_frame, length, board_kernel_ms()))
    {
        g_diag.frames++;
        return true;
    }
    g_diag.refused++;
    return false;
}

static uint8_t saturate_u8 (unsigned value)
{
    return value > 255U ? 255U : (uint8_t)value;
}

static bool send_width (void)
{
    B5WidthProbe * p_width = &g_mission.width;
    B5WPRec record;
    uint8_t frame[B5_WP_RES_FRAME];
    UINT key = enter();
    if (p_width->run != g_width_run_seen)
    {
        g_width_run_seen = p_width->run;
        g_width_admitted = 0U;
    }
    bool have_record = b5_wp_peek(p_width, &record);
    uint16_t d0 = p_width->d0;
    uint16_t lost = p_width->lost;
    leave(key);
    if (have_record)
    {
        uint8_t record_frame[B5_WP_REC_FRAME];
        b5_wp_rec_pack(&record, d0, lost, 0U, record_frame);
        if (push(record_frame, sizeof(record_frame)))
        {
            B5WPRec head;
            key = enter();
            if ((b5_wp_peek(p_width, &head)) && (head.run == record.run) &&
                (head.index == record.index))
            {
                b5_wp_drop(p_width);
                g_width_admitted = saturate_u8(g_width_admitted + 1U);
            }
            leave(key);
        }
        return true;
    }
    key = enter();
    bool summary = p_width->summary_pending;
    B5WPSummary summary_copy = p_width->summary;
    bool retry = p_width->retry_sent < p_width->retry_count;
    unsigned retry_index = p_width->retry_sent;
    B5WPRetry retry_copy =
        retry ? p_width->retries[retry_index] : (B5WPRetry){0};
    bool result = p_width->result_pending;
    B5WEResult result_copy = p_width->result;
    uint32_t result_ms = p_width->result_ms;
    leave(key);
    if (summary)
    {
        uint8_t summary_frame[B5_WP_SUM_FRAME];
        b5_wp_sum_pack(&summary_copy, 0U, summary_frame);
        if (push(summary_frame, sizeof(summary_frame)))
        {
            key = enter();
            if (p_width->summary.run == summary_copy.run)
            {
                p_width->summary_pending = false;
            }
            leave(key);
        }
        return true;
    }
    /* Long probe: D5 per-side limits after D2 (SDK radio order). Packed under
     * the lock from the finished probe; 39 bytes. */
    uint8_t limits_frame[B5_WP_LIM_FRAME];
    key = enter();
    bool limits = (!p_width->summary_pending) && (p_width->limits_pending);
    uint16_t limits_run = p_width->summary.run;
    if (limits)
    {
        b5_wp_limits_pack(p_width, 0U, limits_frame);
    }
    leave(key);
    if (limits)
    {
        if (push(limits_frame, sizeof(limits_frame)))
        {
            key = enter();
            if (p_width->summary.run == limits_run)
            {
                p_width->limits_pending = false;
            }
            leave(key);
        }
        return true;
    }
    if (retry)
    {
        uint8_t retry_frame[B5_WP_RETRY_FRAME];
        b5_wp_retry_pack(&retry_copy, 0U, retry_frame);
        if (push(retry_frame, sizeof(retry_frame)))
        {
            key = enter();
            if (p_width->retry_sent == retry_index)
            {
                p_width->retry_sent++;
            }
            leave(key);
        }
        return true;
    }
    if (result)
    {
        b5_wp_result_pack(&result_copy, result_ms, summary_copy.run,
                          summary_copy.end, summary_copy.d0,
                          summary_copy.records, summary_copy.lost,
                          summary_copy.width_result, 0U, frame);
        if (push(frame, sizeof(frame)))
        {
            key = enter();
            if (p_width->summary.run == summary_copy.run)
            {
                p_width->result_pending = false;
            }
            leave(key);
        }
        return true;
    }
    return false;
}

static bool send_event (void)
{
    B5Event event;
    UINT key = enter();
    bool queued = b5_auto_peek_event(&g_mission.bypass, &event);
    uint16_t overwritten = g_mission.bypass.ev_dropped;
    leave(key);
    if (!queued)
    {
        return false;
    }
    uint8_t frame[B5_EVENT_FRAME];
    (void)b5_event_pack(&event, false, overwritten, 0U, frame);
    if (push(frame, sizeof(frame)))
    {
        B5Event head;
        key = enter();
        if ((b5_auto_peek_event(&g_mission.bypass, &head)) &&
            (head.seq == event.seq))
        {
            b5_auto_drop_event(&g_mission.bypass);
        }
        leave(key);
    }
    return true;
}

static uint16_t clamp_u16 (uint64_t value, uint16_t limit)
{
    return value > (uint64_t)limit ? limit : (uint16_t)value;
}

static void snapshot (uint64_t now, b5_mission_status_t * p_out)
{
    motion_status_t coordinator = motion_get_status();
    UINT key = enter();
    const b5_mission_t * p_mission = &g_mission;
    const B5WidthProbe * p_width = &p_mission->width;
    bool width_result = (p_mission->width_attempted) && (b5_wp_ready(p_width));
    bool width_skipped = B5MI_WIDTH_SKIPPED == p_mission->width_policy;
    bool backlog = (p_width->rec_count != 0U) || (p_width->summary_pending) ||
                   (p_width->result_pending) || (p_width->limits_pending) ||
                   (p_width->retry_sent < p_width->retry_count) ||
                   (p_mission->bypass.ev_count != 0U);
    uint16_t flags = 0U;
    flags |= p_mission->session ? (uint16_t)B5MP_F_ARMED : 0U;
    flags |= p_mission->rearm ? (uint16_t)B5MP_F_REARM : 0U;
    flags |= ((g_last_ping.valid) &&
              ((now - g_last_ping.at_us) <= (uint64_t)RB5_READY_PING_US))
                 ? (uint16_t)B5MP_F_PING_FRESH
                 : 0U;
    flags |= p_mission->width_attempted ? (uint16_t)B5MP_F_WIDTH_ATTEMPTED : 0U;
    flags |= width_result ? (uint16_t)B5MP_F_WIDTH_RESULT : 0U;
    flags |= p_mission->direction_attempted
                 ? (uint16_t)B5MP_F_DIRECTION_ATTEMPTED
                 : 0U;
    flags |= (B5MI_DIR_DEFAULT == p_mission->direction_basis)
                 ? (uint16_t)B5MP_F_DEFAULT_SIDE
                 : 0U;
    flags |= (B5MI_BYPASS == p_mission->stage) ? (uint16_t)B5MP_F_BYPASS : 0U;
    flags |= backlog ? (uint16_t)B5MP_F_BACKLOG : 0U;
    flags |= b5_servo_slew_settled(&g_servo, now)
                 ? (uint16_t)B5MP_F_SERVO_SETTLED
                 : 0U;
    flags |= (B5MI_HOLD == p_mission->stage) ? (uint16_t)B5MP_F_HOLD : 0U;
    flags |= __atomic_load_n(&g_ready_flag, __ATOMIC_ACQUIRE)
                 ? (uint16_t)B5MP_F_READY
                 : 0U;
    *p_out = (b5_mission_status_t){
        .ms = (uint32_t)(now / 1000U),
        .run = p_mission->run,
        .stage = (uint8_t)p_mission->stage,
        .reason = (uint8_t)p_mission->reason,
        .module_reason = p_mission->module_reason,
        .coordinator = (uint8_t)((coordinator.owner & 7U) |
                                 ((coordinator.mode & 7U) << 3) |
                                 (coordinator.session_active ? 0x40U : 0U) |
                                 (coordinator.stopped ? 0x80U : 0U)),
        .flags = flags,
        .ping_mm = ((g_last_ping.valid) && (B5_PING_ECHO == g_last_ping.cls))
                       ? g_last_ping.range_mm
                       : (uint16_t)B5MP_NONE,
        .ping_age_ms =
            g_last_ping.valid
                ? clamp_u16((now - g_last_ping.at_us) / 1000U, 0xfffeU)
                : (uint16_t)B5MP_NONE,
        .ping_class =
            (uint8_t)((g_last_ping.cls & 3U) | ((g_last_ping.why & 15U) << 2)),
        .servo_requested_dd = (int16_t)g_servo.target,
        .servo_applied_dd = (int16_t)g_servo.applied,
        .decision = p_mission->direction_decision,
        .basis = p_mission->direction_basis,
        .states = p_mission->direction_states,
        .direction_end = p_mission->direction_end,
        .side = p_mission->direction_side,
        .width_end = p_mission->width_end,
        /* A deliberate skip still shows a number: the labelled project
         * assumption (policy byte 19 = skipped), never a measurement. */
        .width_basis = width_result ? p_width->result.basis
                       : width_skipped
                           ? (uint8_t)B5WE_BASIS_PROJECT_RANGE_ASSUMPTION
                           : 0U,
        .width_status = width_result    ? p_width->result.status
                        : width_skipped ? (uint8_t)B5WE_STATUS_INVALID
                                        : 0U,
        .width_reason = width_result ? p_width->result.reason : 0U,
        .width_policy = p_mission->width_policy,
        .width_retries = saturate_u8(p_width->retry_count),
        .width_display_mm100 = width_result    ? p_width->result.display_mm100
                               : width_skipped ? (uint16_t)B5WE_MID_MM100
                                               : (uint16_t)B5MP_NONE,
        .width_raw_mm100 =
            width_result ? p_width->result.raw_mm100 : (uint16_t)B5MP_NONE,
        .width_corrected_mm100 = width_result ? p_width->result.corrected_mm100
                                              : (uint16_t)B5MP_NONE,
        .width_d0_mm = p_mission->width_d0,
        .trigger_mm = p_mission->trigger_mm,
        .bypass_range_mm = p_mission->bypass_range_mm,
        .bypass_state = (uint8_t)p_mission->bypass.state,
        .rejoin_phase = (uint8_t)p_mission->bypass.phase,
        .width_produced = saturate_u8(p_width->rec_index),
        .width_admitted = g_width_admitted,
        .approach = (uint8_t)((p_mission->approach_legs & 15U) |
                              ((p_mission->align_steps & 15U) << 4)),
        .discarded = saturate_u8(p_mission->stale_dropped),
        .steer_decideg = (int8_t)p_mission->steer_decideg, /* |v| <= 80 */
        .steer_max_decideg = p_mission->steer_max_decideg,
        .steer_saturated = p_mission->steer_saturated,
        .width_gate_mm = p_mission->width_gate_mm,
        /* E5 v3: live long-probe pose; side only for side phases */
        .width_state = (uint8_t)((p_width->state & 15U) |
                                 (((B5WP_PH_REF == p_width->sample_phase) ||
                                   (B5WP_PH_FINAL == p_width->sample_phase) ||
                                   (B5WP_PH_MID == p_width->sample_phase))
                                      ? 0U
                                      : ((p_width->cur ? 2U : 1U) << 4))),
        .width_sample =
            (uint8_t)((p_width->sample_phase & 7U) |
                      ((p_width->sample_step > 31U ? 31U : p_width->sample_step)
                       << 3)),
        .gate_reason = p_mission->gate_reason,
        .gate_echoes = (uint8_t)((B5S_STRAIGHT == p_mission->gate_reason)
                                     ? p_mission->confirm
                                     : p_mission->close_count)};
    leave(key);
}

static void send_status (uint64_t now)
{
    b5_mission_status_t status;
    uint8_t frame[B5MP_FRAME];
    snapshot(now, &status);
    b5mp_pack(&status, 0U, frame);
    if (push(frame, sizeof(frame)))
    {
        g_diag.status_frames++;
        g_stage_sent = status.stage;
        g_stage_sent_us = now;
    }
}

static void report (void)
{
    b5_mission_status_t status;
    rb5_diagnostics_t diag;
    uint64_t now = board_time_us();
    snapshot(now, &status);
    UINT key = enter();
    diag = g_diag;
    leave(key);
    tm_printf((UB *)"B5_GATE reason=%u (1=STRAIGHT open) echoes=%u "
                    "straight_travel_mm=%d heading_cdeg=%d\n",
              status.gate_reason, status.gate_echoes,
              (int)g_mission.straight.travel_mm,
              (int)g_mission.straight.heading_cdeg);
    tm_printf((UB *)"B5_MISSION version=rtos_buddy1_5-v0.3.2 "
                    "stage=%s reason=%s "
                    "module=%u run=%u side=%u basis=%u decision=%u "
                    "states=0x%02x width_end=%u d0=%u trigger=%u start=%u\n",
              b5_mission_stage_name(status.stage),
              b5_mission_reason_name(status.reason), status.module_reason,
              status.run, status.side, status.basis, status.decision,
              status.states, status.width_end, status.width_d0_mm,
              status.trigger_mm, status.bypass_range_mm);
    tm_printf((UB *)"B5_WIDTH long_probe=v0.2 rule=%u raw_mm100=%u "
                    "corrected_before_clamp_mm100=%u display_mm100=%u "
                    "(-10mm, 100-200mm) basis=%u status=%u reason=%u "
                    "retries=%u produced=%u admitted=%u limits=%u/%u "
                    "out_mm10=%d/%d (DISPLAY ONLY; never used for motion)\n",
              (unsigned)B5WE_RULE_ID, status.width_raw_mm100,
              status.width_corrected_mm100, status.width_display_mm100,
              status.width_basis, status.width_status, status.width_reason,
              status.width_retries, status.width_produced,
              status.width_admitted, g_mission.width.side[0].limit,
              g_mission.width.side[1].limit, g_mission.width.summary.out_l_mm10,
              g_mission.width.summary.out_r_mm10);
    tm_printf((UB *)"B5_SONAR triggers=%u edges=%u echo=%u no_return=%u "
                    "fault=%u trig_pulse_us=%u..%u trig_to_rise_us=%u..%u "
                    "trig_gap_us=%u..%u frames=%u refused=%u status=%u "
                    "servo_req=%d servo_applied=%d\n",
              diag.triggers, diag.edges, diag.classes[0], diag.classes[1],
              diag.classes[2], diag.pulse_min, diag.pulse_max, diag.rise_min,
              diag.rise_max, diag.gap_min, diag.gap_max, diag.frames,
              diag.refused, diag.status_frames, status.servo_requested_dd,
              status.servo_applied_dd);
    for (unsigned count = 0U; count < (unsigned)RB5_PING_HISTORY; ++count)
    {
        unsigned index = (diag.head + count) % (unsigned)RB5_PING_HISTORY;
        if ((diag.last[index].gap_us) || (diag.last[index].width_us) ||
            (diag.last[index].cls))
        {
            tm_printf((UB *)"B5_PING class=%u why=%u width_us=%u range_mm=%u "
                            "trig_to_rise_us=%u gap_us=%u\n",
                      diag.last[index].cls, diag.last[index].why,
                      diag.last[index].width_us, diag.last[index].range_mm,
                      diag.last[index].rise_us, diag.last[index].gap_us);
        }
    }
    tm_printf((UB *)"B5_END\n");
}

/* One Buddy5 task pass (~1 ms): hardware, copied telemetry, report. */
void rb5_service (uint64_t now)
{
    hardware_tick(false);
    uint32_t free_bytes = rn_tx_free();
    if ((now >= g_frame_due) && (free_bytes >= (uint32_t)RB5_RECORD_MIN_FREE))
    {
        bool sent = send_width();
        if (!sent)
        {
            sent = send_event();
        }
        if (sent)
        {
            g_frame_due = now + (uint64_t)RB5_FRAME_SPACING_US;
        }
    }
    uint8_t stage =
        (uint8_t)__atomic_load_n(&g_mission.stage, __ATOMIC_ACQUIRE);
    bool stage_changed =
        (stage != g_stage_sent) &&
        ((now - g_stage_sent_us) >= (uint64_t)RB5_STAGE_MIN_US);
    if (((now >= g_status_due) || (stage_changed)) &&
        (free_bytes >= (uint32_t)RB5_STATUS_MIN_FREE))
    {
        g_status_due = now + (uint64_t)RB5_STATUS_PERIOD_US;
        send_status(now);
    }
    bool report_now = false;
    (void)requests(true, &report_now);
    if (report_now)
    {
        report();
    }
}

void rb5_task (INT code, void * p_exinf)
{
    (void)code;
    (void)p_exinf;
    while (!__atomic_load_n(&g_hardware_ready, __ATOMIC_ACQUIRE))
    {
        (void)tk_dly_tsk(1);
    }
    for (;;)
    {
        rb5_service(board_time_us());
        (void)tk_dly_tsk(1);
    }
}

/* End of file. */
