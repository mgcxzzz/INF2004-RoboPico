/* Host test: Buddy5 RTOS mission (app_program/obstacle_navigation/b5_mission.c)
 * with the REAL shared coordinator/executor (motion_api.c, nav_coordinator.c,
 * nav_codec.c, motion_executor.c) and the ported Buddy5 modules, driven in
 * control.c order (line provider -> motion_update -> mission step) against a
 * simple kinematic plant (wheel speed = target), a geometric HC-SR04 cone
 * model of one rectangular box and an 19 mm line on x = 0.
 * LOGIC EVIDENCE ONLY: no timing, sonar physics or hardware validation. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "b5_mission.h"
#include "src/motion/motion_port.h"
#include "extensions/buddy5_obstacles/servo_slew.h"
#include "extensions/buddy5_obstacles/width_probe_pack.h"
#include "extensions/buddy5_obstacles/edge_probe_pack.h"
#include "extensions/buddy5_obstacles/b5_event_pack.h"
#include "b5_mission_pack.h"
#include "src/line_following/circuit_logic.h"
#include "src/line_following/symmetric_logic.h"

#define PI_F 3.14159265358979323846f
#define FRAME_US 10000u

typedef struct {
    float cx, face_y, width, depth; /* box: x right, y forward (mm) */
    bool present;
} box_t;

static uint64_t g_now;
static double g_wheel[2];
static float g_speed[2];
static float g_x, g_y, g_heading; /* axle midpoint; heading rad, +LEFT */
static box_t g_box;
static B5ServoSlew g_servo;
static b5_mission_t g_mission;
/* sonar engine */
static bool g_busy, g_ready;
static uint64_t g_ready_at, g_next_ping;
static b5_ping_t g_result;
static unsigned g_fault_pings; /* inject: next N pings are faults */
static unsigned g_noise_pings; /* inject: next N echoes alternate +40 mm */
static bool g_line_present = true;
static unsigned g_contacts;
static float g_min_clear = 1e9f;

static uint64_t now_fn(void *p) { (void)p; return g_now; }
static uint64_t g_ping_latency_us = 3000, g_req_at; /* request -> result */

/* ---- geometry ---- */
static void fwd(float h, float *fx, float *fy) { *fx = -sinf(h); *fy = cosf(h); }
static bool in_box(float x, float y) {
    return g_box.present && x >= g_box.cx - g_box.width / 2 && x <= g_box.cx + g_box.width / 2 &&
           y >= g_box.face_y && y <= g_box.face_y + g_box.depth;
}
static float box_gap(float x, float y) { /* distance from point to box (0 inside) */
    float dx = fmaxf(fmaxf(g_box.cx - g_box.width / 2 - x, 0), x - (g_box.cx + g_box.width / 2));
    float dy = fmaxf(fmaxf(g_box.face_y - y, 0), y - (g_box.face_y + g_box.depth));
    return sqrtf(dx * dx + dy * dy);
}
static void check_contact(void) {
    if (!g_box.present) return;
    float fx, fy; fwd(g_heading, &fx, &fy);
    float rx = fy, ry = -fx; /* right unit: rotate forward -90 */
    for (int i = -60; i <= 130; i += 5)
        for (int j = -1; j <= 1; j += 2) {
            float px = g_x + fx * i + rx * 67.5f * j, py = g_y + fy * i + ry * 67.5f * j;
            if (in_box(px, py)) ++g_contacts;
            float gap = box_gap(px, py);
            if (gap < g_min_clear) g_min_clear = gap;
        }
    for (int j = -67; j <= 67; j += 5)
        for (int e = 0; e < 2; ++e) {
            float a = e ? 130.0f : -60.0f;
            float px = g_x + fx * a + rx * j, py = g_y + fy * a + ry * j;
            if (in_box(px, py)) ++g_contacts;
            float gap = box_gap(px, py);
            if (gap < g_min_clear) g_min_clear = gap;
        }
}
/* Cone model: nearest box-outline point within +-7.5 deg of the beam. */
static int (*g_ping_hook_fwd)(float path);
static b5_ping_t measure(void) {
    b5_ping_t p = {0};
    if (g_ping_hook_fwd) {
        int r = g_ping_hook_fwd((float)((g_wheel[0] + g_wheel[1]) / 2));
        if (r >= 0) {
            if (r > 0) { p.cls = B5_PING_ECHO; p.range_mm = (uint32_t)r; p.width_us = (uint32_t)(r * 2000 / 343); }
            else { p.cls = B5_PING_NO_RETURN; p.range_mm = B5_RANGE_FAR_MM; p.width_us = 30000; }
            return p;
        }
    }
    float fx, fy; fwd(g_heading, &fx, &fy);
    float sx = g_x + 62 * fx, sy = g_y + 62 * fy;
    float beam = g_heading - (float)g_servo.applied / 10.0f * PI_F / 180.0f;
    float bx, by; fwd(beam, &bx, &by);
    float best = 1e9f;
    if (g_box.present) {
        float x0 = g_box.cx - g_box.width / 2, x1 = g_box.cx + g_box.width / 2;
        float y0 = g_box.face_y, y1 = g_box.face_y + g_box.depth;
        for (int edge = 0; edge < 4; ++edge) {
            float ax = edge == 1 ? x1 : x0, ay = edge == 2 ? y1 : y0;
            float ex = edge == 0 || edge == 2 ? x1 : (edge == 1 ? x1 : x0);
            float ey = edge == 0 ? y0 : edge == 2 ? y1 : y1;
            float len = hypotf(ex - ax, ey - ay);
            for (float t = 0; t <= len; t += 1.0f) {
                float px = ax + (ex - ax) * t / len - sx, py = ay + (ey - ay) * t / len - sy;
                float d = hypotf(px, py);
                if (d < 1) continue;
                float c = (px * bx + py * by) / d;
                if (c >= cosf(7.5f * PI_F / 180.0f) && d < best) best = d;
            }
        }
    }
    if (g_noise_pings && best < 3900) { /* clean but inconsistent echoes */
        best += (g_noise_pings % 2) ? 40.0f : 0.0f;
        --g_noise_pings;
    }
    if (g_fault_pings) {
        --g_fault_pings;
        p.cls = B5_PING_FAULT; p.why = B5_PING_WHY_NO_RISE; p.range_mm = B5_RANGE_FAR_MM;
        return p;
    }
    if (best < 3900) {
        p.cls = B5_PING_ECHO; p.range_mm = (uint32_t)lroundf(best);
        p.width_us = (uint32_t)(best * 2000 / 343);
    } else {
        p.cls = B5_PING_NO_RETURN; p.range_mm = B5_RANGE_FAR_MM; p.width_us = 30000;
    }
    return p;
}
/* Analog reflectance: a 10 mm spot; level = 40 + 920 x black fraction of
 * the spot. Sensors at -8 (left) / +8 (right), 130 mm ahead of the axle. The
 * 19 mm strip spans x = -27..-8 so that Buddy3's edge law (LEFT sensor on the
 * strip's RIGHT edge at level 500) has its equilibrium on the path x = 0. */
static float sensor_x(float lateral) {
    float fx, fy; fwd(g_heading, &fx, &fy);
    float rx = fy;
    return g_x + 130 * fx + lateral * rx;
}
static uint16_t sim_level(float lateral) {
    if (!g_line_present) return 40;
    float px = sensor_x(lateral);
    float lo = fmaxf(px - 5.0f, -27.0f), hi = fminf(px + 5.0f, -8.0f);
    float frac = hi > lo ? (hi - lo) / 10.0f : 0.0f;
    return (uint16_t)lroundf(40 + 920 * frac);
}
/* Buddy3 stand-in = the same analog_demand() edge law Buddy3 uses. */
static analog_tracker_t g_b3;
static const line_calibration_t g_ident = {.white = {0, 0}, .black = {1000, 1000}, .mask = 7};

/* ---- port ---- */
static void p_angle(int dd, uint64_t now) { b5_servo_slew_request(&g_servo, dd, now); }
static bool p_settled(uint64_t now) { return b5_servo_slew_settled(&g_servo, now); }
static bool p_request(uint64_t now) {
    if (g_busy || g_ready || now < g_next_ping) return false;
    g_busy = true; g_next_ping = now + 60000; g_ready_at = now + g_ping_latency_us; g_req_at = now;
    g_result = measure();
    return true;
}
static bool g_took;            /* a ping was taken in this frame */
static bool p_take(b5_ping_t *out) {
    if (!g_ready) return false;
    *out = g_result; g_ready = false; g_took = true; return true;
}
static void p_cancel(void) { g_busy = g_ready = false; }
static int p_applied(void) { return g_servo.applied; }

/* ---- telemetry dump (real packers; optional dropped D1 index) ---- */
static FILE *g_frames;
static uint8_t g_seq;
static int g_drop_d1 = -1;
static void hexline(const uint8_t *b, unsigned n) {
    if (!g_frames) return;
    fputs("TLM ", g_frames);
    for (unsigned i = 0; i < n; ++i) fprintf(g_frames, "%02x", b[i]);
    fputc('\n', g_frames);
}
static void drain(void) {
    B5WPRec r; B5EPRec e; B5Event ev;
    while (b5_wp_peek(&g_mission.width, &r)) {
        uint8_t f[B5_WP_REC_FRAME];
        b5_wp_rec_pack(&r, g_mission.width.d0, g_mission.width.lost, g_seq++, f);
        if ((int)r.index != g_drop_d1) hexline(f, sizeof f);
        b5_wp_drop(&g_mission.width);
    }
    if (g_mission.width.summary_pending) {
        uint8_t f[B5_WP_SUM_FRAME]; b5_wp_sum_pack(&g_mission.width.summary, g_seq++, f); hexline(f, sizeof f);
        g_mission.width.summary_pending = false;
    }
    if (g_mission.width.limits_pending) {          /* long probe D5 after D2 */
        uint8_t f[B5_WP_LIM_FRAME]; b5_wp_limits_pack(&g_mission.width, g_seq++, f); hexline(f, sizeof f);
        g_mission.width.limits_pending = false;
    }
    while (g_mission.width.retry_sent < g_mission.width.retry_count) {
        uint8_t f[B5_WP_RETRY_FRAME];
        b5_wp_retry_pack(&g_mission.width.retries[g_mission.width.retry_sent++], g_seq++, f); hexline(f, sizeof f);
    }
    if (g_mission.width.result_pending) {
        const B5WPSummary *m = &g_mission.width.summary;
        uint8_t f[B5_WP_RES_FRAME];
        b5_wp_result_pack(&g_mission.width.result, g_mission.width.result_ms, m->run, m->end, m->d0,
                          m->records, m->lost, m->width_result, g_seq++, f);
        hexline(f, sizeof f);
        g_mission.width.result_pending = false;
    }
    while (b5_ep_peek(&g_mission.edge, &e)) {
        uint8_t f[B5_EP_REC_FRAME];
        b5_ep_rec_pack(&e, g_mission.edge.run, g_mission.edge.d0, g_mission.edge.lost, g_seq++, f);
        hexline(f, sizeof f); b5_ep_drop(&g_mission.edge);
    }
    if (g_mission.edge.summary_pending) {
        uint8_t f[B5_EP_SUM_FRAME]; b5_ep_sum_pack(&g_mission.edge.summary, g_seq++, f); hexline(f, sizeof f);
        g_mission.edge.summary_pending = false;
    }
    while (b5_auto_peek_event(&g_mission.bypass, &ev)) {
        uint8_t f[B5_EVENT_FRAME];
        (void)b5_event_pack(&ev, false, g_mission.bypass.ev_dropped, g_seq++, f); hexline(f, sizeof f);
        b5_auto_drop_event(&g_mission.bypass);
    }
}
static void dump_status(void) {
    const b5_mission_t *m = &g_mission;
    bool wr = m->width_attempted && b5_wp_ready(&m->width);
    bool skip = m->width_policy == B5MI_WIDTH_SKIPPED; /* obstacle_service.c snapshot rule */
    b5_mission_status_t st = {.ms = (uint32_t)(g_now / 1000), .run = m->run, .stage = (uint8_t)m->stage,
        .reason = (uint8_t)m->reason, .module_reason = m->module_reason,
        .coordinator = (uint8_t)(nav_get_status().owner | (nav_get_status().mode << 3) |
                                 (nav_get_status().session_active ? 0x40 : 0)),
        .flags = (uint16_t)((m->session ? 1 : 0) | (wr ? 16 : 0) | (m->width_attempted ? 8 : 0) |
                            (m->direction_attempted ? 32 : 0) |
                            (m->direction_basis == B5MI_DIR_DEFAULT ? 64 : 0) | (m->stage == B5MI_HOLD ? 1024 : 0)),
        .ping_mm = B5MP_NONE, .ping_age_ms = B5MP_NONE, .decision = m->direction_decision,
        .basis = m->direction_basis, .side = m->direction_side, .width_end = m->width_end,
        .width_basis = wr ? m->width.result.basis : skip ? B5WE_BASIS_PROJECT_RANGE_ASSUMPTION : 0,
        .width_status = wr ? m->width.result.status : skip ? B5WE_STATUS_INVALID : 0,
        .width_policy = m->width_policy, .steer_decideg = (int8_t)m->steer_decideg,
        .steer_max_decideg = m->steer_max_decideg, .steer_saturated = m->steer_saturated,
        .width_gate_mm = m->width_gate_mm,
        .width_state = (uint8_t)((m->width.state & 15u) |
                                 ((m->width.sample_phase == B5WP_PH_REF || m->width.sample_phase == B5WP_PH_FINAL ||
                                   m->width.sample_phase == B5WP_PH_MID) ? 0u : ((m->width.cur ? 2u : 1u) << 4))),
        .width_sample = (uint8_t)((m->width.sample_phase & 7u) | ((m->width.sample_step > 31u ? 31u : m->width.sample_step) << 3)),
        .gate_reason = m->gate_reason,
        .gate_echoes = (uint8_t)(m->gate_reason == B5S_STRAIGHT ? m->confirm : m->close_count),
        .width_reason = wr ? m->width.result.reason : 0,
        .width_display_mm100 = wr ? m->width.result.display_mm100 : skip ? B5WE_MID_MM100 : B5MP_NONE,
        .width_raw_mm100 = wr ? m->width.result.raw_mm100 : B5MP_NONE,
        .width_corrected_mm100 = wr ? m->width.result.corrected_mm100 : B5MP_NONE,
        .width_d0_mm = m->width_d0, .trigger_mm = m->trigger_mm, .bypass_range_mm = m->bypass_range_mm,
        .bypass_state = (uint8_t)m->bypass.state, .rejoin_phase = (uint8_t)m->bypass.phase,
        .width_produced = (uint8_t)m->width.rec_index, .direction_produced = (uint8_t)m->edge.rec_index,
        .direction_end = m->direction_end,
        .states = (uint8_t)(m->edge.summary.state_l | (m->edge.summary.state_r << 4))};
    uint8_t f[B5MP_FRAME];
    b5mp_pack(&st, g_seq++, f);
    hexline(f, sizeof f);
}

/* ---- straight takeover gate inputs (scenario hooks) ---- */
typedef void (*line_hook_t)(b5_line_sample_t *s, float path);
typedef int (*ping_hook_t)(float path); /* >0 echo mm, 0 no return, <0 geometric */
static line_hook_t g_line_hook;
static ping_hook_t g_ping_hook;
static bool g_no_line_samples, g_appear_when_open;
typedef struct { uint64_t req_us, open_us; bool near; uint8_t gate; unsigned confirm, close; unsigned stage; } take_log_t;
static take_log_t g_takes[4096];
static unsigned g_ntakes;
static bool g_drop_seen;               /* gate lost between two takes */
static unsigned g_drop_before, g_drop_after;

/* ---- simulation ---- */
static unsigned g_stage_mask;
static void frame(void) {
    g_now += FRAME_US;
    b5_servo_slew_tick(&g_servo, g_now);
    if (g_busy && g_now >= g_ready_at) { g_busy = false; g_ready = true; }
    motion_feedback_t f = {.now_us = g_now};
    f.wheel_mm[0] = g_wheel[0]; f.wheel_mm[1] = g_wheel[1];
    f.speed_mm_s[0] = g_speed[0]; f.speed_mm_s[1] = g_speed[1];
    f.line_level[0] = sim_level(-8); f.line_level[1] = sim_level(8);
    motion_status_t s = nav_get_status();
    bool follow = s.session_active && s.mode == MOTION_AUTO && s.owner == MOTION_BUDDY3;
    /* Buddy3 frame for the takeover gate (control.c: copied right after the
     * provider ran). The sim line is straight; hooks inject bends etc. */
    float path = (float)((g_wheel[0] + g_wheel[1]) / 2);
    b5_line_sample_t ls = {.now_us = g_now, .wheel_mm = {(float)g_wheel[0], (float)g_wheel[1]},
                           .phase = C_FOLLOW, .decision = 25, .valid = follow, .steering = true};
    if (follow) {
        analog_demand_t d = analog_demand(&g_b3, &g_ident, f.line_level[0], f.line_level[1], g_now,
                                          (uint32_t)fabs((g_wheel[0] + g_wheel[1]) / 2));
        int16_t l = d.stop ? 45 : d.target[0], r = d.stop ? 45 : d.target[1];
        ls.decision = 25; /* continuous steering output code (small corrections) */
        (void)nav_submit_navigation(MOTION_BUDDY3, l, r);
    }
    if (g_line_hook && follow) g_line_hook(&ls, path);
    motion_output_t o = motion_update(&f);
    b5_mission_observe(&g_mission, g_now, f.line_level[0], f.line_level[1]);
    if (!(g_no_line_samples && path >= 100)) /* stale: frames stop after 100 mm */
        b5_mission_line(&g_mission, follow ? &ls : &(b5_line_sample_t){.now_us = g_now});
    if (g_appear_when_open && g_mission.straight.open && !g_box.present) g_box.present = true;
    g_took = false;
    unsigned c_before = g_mission.confirm; uint8_t gate_before = g_mission.gate_reason;
    b5_mission_tick(&g_mission, g_now);
    if (gate_before == B5S_STRAIGHT && g_mission.gate_reason != B5S_STRAIGHT && !g_took && !g_drop_seen) {
        g_drop_seen = true; g_drop_before = c_before; g_drop_after = g_mission.confirm;
    }
    if (g_took && g_ntakes < sizeof g_takes / sizeof g_takes[0])
        g_takes[g_ntakes++] = (take_log_t){g_mission.taken_requested, g_mission.straight.open_us,
                                           g_result.cls == B5_PING_ECHO && g_result.range_mm <= 300,
                                           g_mission.gate_reason, g_mission.confirm, g_mission.close_count,
                                           g_mission.stage};
    g_stage_mask |= 1u << g_mission.stage;
    drain();
    if ((g_now / FRAME_US) % 50 == 0) dump_status();
    /* plant */
    float d[2];
    for (int w = 0; w < 2; ++w) {
        g_speed[w] = o.target_mm_s[w];
        d[w] = g_speed[w] * (FRAME_US / 1e6f);
        g_wheel[w] += d[w];
    }
    float dh = (d[1] - d[0]) / 110.0f, ds = (d[0] + d[1]) / 2;
    float fx, fy; fwd(g_heading + dh / 2, &fx, &fy);
    g_x += ds * fx; g_y += ds * fy; g_heading += dh;
    check_contact();
}

typedef struct {
    const char *name;
    float cx, width, face_y;
    unsigned fault_at_stage, fault_count;
    bool fault_after_ref; /* width stage: inject only after the REF sample */
    bool fault_in_width_gate; /* faults in the fresh range deciding the width policy */
    bool noise_in_width_gate; /* clean but inconsistent echoes in that range */
    float disturb_deg, disturb_mm; /* physical heading (+LEFT)/lateral (+RIGHT)
                                      error injected at the first approach leg */
    bool steering_off;     /* comparison only: mission steering disabled */
    unsigned hold_frames;  /* extra frames to observe a HOLD (default 100) */
    bool lose_line_in_approach, cancel;
    unsigned cancel_stage;
    line_hook_t line_hook;   /* Buddy3 state injected into the gate frames */
    ping_hook_t ping_hook;   /* scripted guard pings (>=0) or geometric (<0) */
    bool no_line_samples;    /* Buddy3 frames stop arriving: stale */
    bool appear_when_open;   /* box placed only once the gate first opens */
    bool stop_at_takeover;   /* end the run once HALTING is entered */
    unsigned ping_latency_us; /* request -> result (default 3 ms) */
} scenario_t;

typedef struct {
    unsigned final_stage, reason, side, basis, width_basis, handbacks;
    uint16_t display_mm100, d0, bypass_range;
    float end_x, end_y, end_heading_deg;
    float dir_edge_err_mm, dir_heading_deg; /* at direction-probe start */
    bool reached_direction;
} outcome_t;

static outcome_t run(const scenario_t *sc) {
    if (g_frames) fprintf(g_frames, "SCENARIO %s\n", sc->name);
    memset(&g_mission, 0, sizeof(g_mission));
    g_now = 1000000; g_wheel[0] = g_wheel[1] = 0; g_speed[0] = g_speed[1] = 0;
    g_x = 0; g_y = 0; g_heading = 0; g_busy = g_ready = false; g_next_ping = 0;
    g_fault_pings = 0; g_noise_pings = 0; g_line_present = true; g_b3 = (analog_tracker_t){0}; g_contacts = 0; g_min_clear = 1e9f;
    g_stage_mask = 0;
    g_box = (box_t){.cx = sc->cx, .face_y = sc->face_y, .width = sc->width, .depth = 150,
                    .present = sc->width > 0 && !sc->appear_when_open};
    g_line_hook = sc->line_hook; g_ping_hook = sc->ping_hook; g_ping_hook_fwd = sc->ping_hook;
    g_no_line_samples = sc->no_line_samples; g_appear_when_open = sc->appear_when_open && sc->width > 0;
    g_ntakes = 0; g_drop_seen = false;
    g_ping_latency_us = sc->ping_latency_us ? sc->ping_latency_us : 3000;
    b5_servo_slew_init(&g_servo, 460, B5_SERVO_STEP_MS, B5_SERVO_SPEED_DD_S, B5_SERVO_SETTLE_MS, g_now);
    motion_port_t port = {.now_us = now_fn};
    motion_init(&port);
    B5SensorPort sp = {p_angle, p_settled, p_request, p_take, p_cancel, NULL};
    b5_mission_init(&g_mission, &sp, p_applied);
    if (sc->steering_off) g_mission.steering = false; /* comparison only */
    motion_session_begin(g_now);
    assert(b5_mission_begin(&g_mission, g_now));
    outcome_t out = {0};
    bool faulted = false, lost = false, disturbed = false;
    unsigned prev_stage = g_mission.stage;
    for (unsigned k = 0; k < 120000; ++k) { /* <= 1200 s simulated */
        frame();
        unsigned st = g_mission.stage;
        if (sc->fault_count && !faulted && st == sc->fault_at_stage &&
            (!sc->fault_after_ref || g_mission.width.state == B5WP_LADDER)) {
            g_fault_pings = sc->fault_count; faulted = true;
        }
        if ((sc->fault_in_width_gate || sc->noise_in_width_gate) && !faulted && st == B5MI_RANGE &&
            g_mission.after_range == B5MI_WIDTH) {
            if (sc->fault_in_width_gate) g_fault_pings = 6; else g_noise_pings = 6;
            faulted = true;
        }
        if ((sc->disturb_deg != 0 || sc->disturb_mm != 0) && !disturbed && st == B5MI_APPROACH &&
            g_mission.sub == 1) { /* a push just after the leg starts */
            g_heading += sc->disturb_deg * PI_F / 180; g_x += sc->disturb_mm; disturbed = true;
        }
        if (st == B5MI_DIRECTION && !out.reached_direction) {
            out.reached_direction = true;
            out.dir_edge_err_mm = sensor_x(-8) + 8; out.dir_heading_deg = g_heading * 180 / PI_F;
        }
        if (sc->lose_line_in_approach && !lost && st == B5MI_APPROACH && g_mission.sub == 1) {
            g_line_present = false; lost = true;
        }
        if (sc->cancel && st == sc->cancel_stage) {
            /* Operator Stop: control.c bringup_stop_from() order. */
            b5_mission_cancel(&g_mission, g_now);
            (void)motion_stop(MOTION_BUDDY1);
            motion_session_end(1);
            for (int i = 0; i < 50; ++i) frame();
            motion_status_t s = nav_get_status();
            assert(!s.session_active);
            assert(g_speed[0] == 0 && g_speed[1] == 0);
            break;
        }
        if (sc->stop_at_takeover && st == B5MI_HALTING) break;
        if (prev_stage == B5MI_BYPASS && st == B5MI_GUARD) {
            ++out.handbacks;
            for (int i = 0; i < 300; ++i) frame(); /* line following resumes */
            break;
        }
        if (st == B5MI_HOLD) {
            unsigned extra = sc->hold_frames ? sc->hold_frames : 100;
            for (unsigned i = 0; i < extra; ++i) {
                frame();
                assert(g_mission.stage == B5MI_HOLD); /* never released */
            }
            assert(fabsf(g_speed[0]) < 1e-3f && fabsf(g_speed[1]) < 1e-3f); /* held stopped */
            break;
        }
        prev_stage = st;
    }
    out.final_stage = g_mission.stage; out.reason = g_mission.reason;
    out.side = g_mission.direction_side; out.basis = g_mission.direction_basis;
    out.width_basis = g_mission.width.result.basis;
    out.display_mm100 = g_mission.width.result.display_mm100;
    out.d0 = g_mission.width_d0; out.bypass_range = g_mission.bypass_range_mm;
    out.end_x = g_x; out.end_y = g_y; out.end_heading_deg = g_heading * 180 / PI_F;
    printf("  width policy=%u gate=%u attempted=%u | steer last=%d max=%u sat=%u | dir edge_err=%.1f hdg=%.2f\n",
           g_mission.width_policy, g_mission.width_gate_mm, g_mission.width_attempted,
           g_mission.steer_decideg, g_mission.steer_max_decideg, g_mission.steer_saturated,
           out.dir_edge_err_mm, out.dir_heading_deg);
    printf("  edge end=%u decision=%u states=%u/%u records=%u flags=0x%x | width end=%u\n",
           g_mission.edge.summary.end, g_mission.edge.summary.decision,
           g_mission.edge.summary.state_l, g_mission.edge.summary.state_r,
           g_mission.edge.summary.records, g_mission.edge.summary.flags, g_mission.width.end);
    printf("%-34s stage=%-9s reason=%-17s side=%u basis=%u width=%u.%02u(b%u) d0=%u "
           "start=%u contacts=%u min_clear=%.0f end=(%.0f,%.0f,%.1fdeg) stages=0x%x\n",
           sc->name, b5_mission_stage_name(out.final_stage), b5_mission_reason_name(out.reason),
           out.side, out.basis, out.display_mm100 / 100, out.display_mm100 % 100, out.width_basis,
           out.d0, out.bypass_range, g_contacts, g_min_clear, out.end_x, out.end_y,
           out.end_heading_deg, g_stage_mask);
    return out;
}

/* ---- straight-only takeover gate scenarios ---- */
static void hook_bend(b5_line_sample_t *s, float path) { (void)path; s->bend_candidate = s->bend_confirmed = true; }
static void hook_bend_exit(b5_line_sample_t *s, float path) { (void)path; s->bend_candidate = s->bend_exiting = true; }
static void hook_junction(b5_line_sample_t *s, float path) { (void)path; s->junction_mark = true; }
static void hook_recovery(b5_line_sample_t *s, float path) { (void)path; s->searching = true; s->decision = SYM_RECOVER_LEFT; }
static void hook_invalid(b5_line_sample_t *s, float path) { (void)path; s->valid = false; }
/* Carry-over: a bend until 300 mm with two close echoes inside it, then a
 * straight; the gate opens after 200 mm of straight travel. Close echoes
 * start one or two pings BEFORE the gate opens (requested while closed). */
static void hook_carry(b5_line_sample_t *s, float path) {
    if (path < 300) { s->bend_candidate = s->bend_confirmed = true; }
    /* a close echo IN FLIGHT across the opening (requested while closed) */
    if (g_mission.straight.open && g_busy && g_req_at < g_mission.straight.open_us) {
        g_result.cls = B5_PING_ECHO; g_result.range_mm = 280;
    }
}
static unsigned g_script_near;
static int ping_carry(float path) {
    if (path >= 250 && path < 300 && g_script_near < 2) { ++g_script_near; return 280; }
    if (g_mission.straight.open) return 280;
    /* the gate opens at ~207 mm of straight; 1-2 pings requested in the last
     * 4 mm before it are close (fewer than three: no NOT_STRAIGHT hold) */
    if (g_mission.gate_reason == B5S_SETTLING && g_mission.straight.travel_mm >= 203.0f) return 280;
    return 0;
}
/* Re-qualify: straight; two qualifying close echoes after the gate opens,
 * then (before any far echo is taken) a bend entry for 60 mm with far pings,
 * then the straight again: three fresh echoes after the gate re-opens. */
static unsigned g_req_phase1, g_req_state;
static float g_req_bend_end;
static void hook_requalify(b5_line_sample_t *s, float path) {
    if (g_req_state == 0 && g_mission.confirm >= 2) { g_req_state = 1; g_req_bend_end = path + 60; }
    if (g_req_state == 1) { if (path < g_req_bend_end) s->bend_pending = true; else g_req_state = 2; }
}
static int ping_requalify(float path) {
    if (g_req_state == 0) return (g_mission.straight.open && path >= 420 && g_req_phase1 < 2) ? (++g_req_phase1, 280) : 0;
    if (g_req_state == 1) return 0;
    return g_mission.straight.open ? 280 : 0;
}
static void gate_scenarios(unsigned *p_failures) {
#define GCHECK(c) do { if (!(c)) { printf("  FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++*p_failures; } } while (0)
    const unsigned probing = (1u << B5MI_WIDTH) | (1u << B5MI_ALIGN) | (1u << B5MI_RANGE) | (1u << B5MI_APPROACH) |
                             (1u << B5MI_DIRECTION) | (1u << B5MI_BYPASS);
    /* 1. Close obstacle while the gate is closed: stop and HOLD NOT_STRAIGHT
     * with the gate reason; no probe, no bypass; Buddy5 keeps the stopped
     * robot until an operator Stop. */
    struct { const char *name; line_hook_t hook; bool no_samples; uint8_t reason; } blocked[] = {
        {"blocked: confirmed bend", hook_bend, false, B5S_BEND},
        {"blocked: bend exit", hook_bend_exit, false, B5S_BEND_EXIT},
        {"blocked: junction mark", hook_junction, false, B5S_JUNCTION},
        {"blocked: recovery search", hook_recovery, false, B5S_LINE_LOST},
        {"blocked: Buddy3 state unavailable", hook_invalid, false, B5S_UNAVAILABLE},
        {"blocked: stale Buddy3 frames", NULL, true, B5S_STALE}};
    for (unsigned i = 0; i < sizeof blocked / sizeof blocked[0]; ++i) {
        scenario_t sb = {.name = blocked[i].name, .cx = 60, .width = 185, .face_y = 900,
                         .line_hook = blocked[i].hook, .no_line_samples = blocked[i].no_samples};
        outcome_t o = run(&sb);
        motion_status_t st = nav_get_status();
        GCHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_NOT_STRAIGHT);
        GCHECK(g_mission.module_reason == blocked[i].reason);
        GCHECK((g_stage_mask & probing) == 0 && g_mission.width.rec_index == 0 && g_mission.edge.rec_index == 0);
        GCHECK(!g_mission.width_attempted && !g_mission.direction_attempted && !g_mission.bypass.session);
        GCHECK(st.owner == MOTION_BUDDY5 && st.mode != MOTION_AUTO && g_contacts == 0);
        GCHECK(o.handbacks == 0 && 900 - g_y > 300 && 900 - g_y < 400); /* stopped near the trigger (sonar 62 mm ahead of the axle) */
        /* operator Stop still ends it (control.c order) */
        b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
        for (int k = 0; k < 50; ++k) frame();
        GCHECK(!nav_get_status().session_active && g_speed[0] == 0 && g_speed[1] == 0);
        printf("  %-34s -> HOLD %s module(gate)=%u, axle %.0f mm from the face\n", blocked[i].name,
               b5_mission_reason_name(o.reason), g_mission.module_reason, 900 - g_y);
    }
    /* 2. Old close echoes never carry over into a new straight. */
    g_script_near = 0;
    scenario_t sc = {.name = "carry-over: echoes in bend, then straight", .line_hook = hook_carry,
                     .ping_hook = ping_carry, .stop_at_takeover = true, .ping_latency_us = 55000};
    outcome_t o = run(&sc);
    GCHECK(g_mission.stage == B5MI_HALTING);
    unsigned near_closed = 0, pre_open_after = 0, fresh = 0, at_takeover = 0;
    for (unsigned k = 0; k < g_ntakes; ++k) {
        const take_log_t *t = &g_takes[k];
        if (t->near && t->gate != B5S_STRAIGHT) { ++near_closed; GCHECK(t->confirm == 0 && t->close <= 2); }
        if (t->near && t->gate == B5S_STRAIGHT && t->req_us < t->open_us) { ++pre_open_after; GCHECK(t->confirm == 0); }
        if (t->near && t->gate == B5S_STRAIGHT && t->req_us >= t->open_us) {
            ++fresh; GCHECK(t->stage == B5MI_HALTING ? fresh == 3 : t->confirm == fresh); /* HALTING resets it */
        }
        if (t->stage == B5MI_HALTING) at_takeover = k;
    }
    GCHECK(near_closed >= 2 && pre_open_after >= 1 && fresh == 3 && at_takeover == g_ntakes - 1);
    printf("  carry-over: %u close echoes while closed (bend/settling) and %u in-flight echo(es) requested before "
           "opening never counted; takeover on the 3rd fresh echo (%u)\n", near_closed, pre_open_after, fresh);
    (void)o;
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
    /* 3. Losing the gate resets the count: 2 qualifying echoes, bend entry,
     * then three fresh echoes after the gate re-opens. */
    g_req_phase1 = 0; g_req_state = 0;
    scenario_t sr = {.name = "re-qualify: 2 echoes, bend entry, 3 fresh", .line_hook = hook_requalify,
                     .ping_hook = ping_requalify, .stop_at_takeover = true};
    o = run(&sr);
    GCHECK(g_mission.stage == B5MI_HALTING);
    unsigned max_before = 0, after = 0; uint64_t first_open = 0;
    for (unsigned k = 0; k < g_ntakes; ++k) {
        const take_log_t *t = &g_takes[k];
        if (!first_open && t->gate == B5S_STRAIGHT) first_open = t->open_us;
        if (t->gate == B5S_STRAIGHT && t->open_us == first_open && t->confirm > max_before) max_before = t->confirm;
        if (t->gate == B5S_STRAIGHT && t->open_us != first_open && t->near) {
            ++after; GCHECK(t->stage == B5MI_HALTING ? after == 3 : t->confirm == after);
        }
        if (t->gate == B5S_BEND_ENTRY) GCHECK(t->confirm == 0);
    }
    GCHECK(max_before == 2 && after == 3);
    GCHECK(g_drop_seen && g_drop_before == 2 && g_drop_after == 0); /* reset by the gate, not by a far echo */
    printf("  re-qualify: %u qualifying echoes discarded at bend entry; takeover after %u fresh echoes on the "
           "re-opened straight\n", max_before, after);
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
#undef GCHECK
}

int main(int argc, char **argv) {
    unsigned failures = 0;
    if (argc > 1) g_frames = fopen(argv[1], "w");
#define CHECK(c) do { if (!(c)) { printf("  FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++failures; } } while (0)
    /* Box face 900 mm ahead of the axle on the line. */
    scenario_t sr = {.name = "185 offset RIGHT 60 -> LEFT", .cx = 60, .width = 185, .face_y = 900};
    outcome_t o = run(&sr);
    CHECK(o.handbacks == 1); CHECK(o.side == B5_SIDE_LEFT); CHECK(o.basis == B5MI_DIR_EVIDENCE);
    CHECK(g_contacts == 0); CHECK(o.d0 >= 270 && o.d0 <= 330);
    CHECK(o.bypass_range >= 150 && o.bypass_range <= 220);
    CHECK(fabsf(o.end_x) < 15); CHECK(o.end_y > 900 + 150);
    CHECK(g_mission.width_policy == B5MI_WIDTH_MEASURED);
    CHECK(g_mission.steer_max_decideg <= B5MI_STEER_MAX_DECIDEG);
    /* Long probe (SDK width_long_probe v0.2): the whole chain width -> align ->
     * approach -> direction -> bypass ran in one mission; every outward pass is
     * bounded by clearance and the 220 mm ceiling and reports a stop reason;
     * D2, D5 and D3 were all drained; width stays display-only. */
    {
        const unsigned need = (1u << B5MI_WIDTH) | (1u << B5MI_ALIGN) | (1u << B5MI_RANGE) |
                              (1u << B5MI_APPROACH) | (1u << B5MI_DIRECTION) | (1u << B5MI_BYPASS);
        CHECK((g_stage_mask & need) == need);
        CHECK(g_mission.width.end == B5WP_END_COMPLETE);
        CHECK(!g_mission.width.summary_pending && !g_mission.width.limits_pending &&
              !g_mission.width.result_pending);
        CHECK(g_mission.width.summary.records <= B5WP_MAX_RECORDS);
        for (unsigned k = 0; k < 2u; ++k) {
            const B5WPSide *sd = &g_mission.width.side[k];
            CHECK(sd->limit == B5WP_LIM_FAR || sd->limit == B5WP_LIM_CEILING ||
                  sd->limit == B5WP_LIM_CLEARANCE || sd->limit == B5WP_LIM_STEPS);
            CHECK(sd->out_mm <= B5WP_BUDGET_MM + 0.05f && sd->steps <= B5WP_MAX_STEPS);
            CHECK(sd->min_turn_gap >= B5WP_FRONT_MARGIN_MM - 0.15f);
        }
        CHECK(g_mission.width.summary.min_front_gap_mm10 >= (int16_t)(B5WP_FRONT_MARGIN_MM * 10.0f) - 2);
    }
    scenario_t sl = {.name = "185 offset LEFT 60 -> RIGHT", .cx = -60, .width = 185, .face_y = 900};
    o = run(&sl);
    CHECK(o.handbacks == 1); CHECK(o.side == B5_SIDE_RIGHT); CHECK(o.basis == B5MI_DIR_EVIDENCE);
    CHECK(g_contacts == 0); CHECK(fabsf(o.end_x) < 15);
    scenario_t s100 = {.name = "100 offset RIGHT 35 -> LEFT", .cx = 35, .width = 100, .face_y = 900};
    o = run(&s100);
    CHECK(o.handbacks == 1); CHECK(o.side == B5_SIDE_LEFT); CHECK(g_contacts == 0);
    scenario_t sc = {.name = "185 centred -> default (labelled)", .cx = 0, .width = 185, .face_y = 900};
    g_drop_d1 = 5; /* telemetry fixture: one D1 never reaches the dashboard */
    o = run(&sc);
    g_drop_d1 = -1;
    CHECK(o.basis == B5MI_DIR_DEFAULT || o.basis == B5MI_DIR_HELD);
    if (o.basis == B5MI_DIR_DEFAULT) { CHECK(o.side == B5_SIDE_RIGHT); CHECK(o.handbacks == 1); }
    else CHECK(o.final_stage == B5MI_HOLD && o.handbacks == 0);
    CHECK(g_contacts == 0);
    scenario_t sf = {.name = "sonar faults in direction probe", .cx = 35, .width = 185, .face_y = 900,
                     .fault_at_stage = B5MI_DIRECTION, .fault_count = 6};
    o = run(&sf);
    CHECK(o.final_stage == B5MI_HOLD); CHECK(o.reason == B5MI_R_DIRECTION_ABORT);
    CHECK(o.basis == B5MI_DIR_HELD); CHECK(o.side == B5_SIDE_NONE); CHECK(o.handbacks == 0);
    scenario_t sw = {.name = "sonar faults after width REF", .cx = 35, .width = 185, .face_y = 900,
                     .fault_at_stage = B5MI_WIDTH, .fault_count = 12, .fault_after_ref = true};
    o = run(&sw);
    CHECK(o.final_stage == B5MI_HOLD); CHECK(o.reason == B5MI_R_WIDTH_ABORT);
    CHECK(g_mission.width.end == B5WP_END_SENSOR); CHECK(o.handbacks == 0);
    /* Review 2026-11-19 item 1: an ATTEMPTED probe whose REF is rejected holds
     * (no END_REFERENCE continuation) even though the sonar is clean again
     * afterwards; the D3 150 mm assumption stays the displayed number. */
    scenario_t sr0 = {.name = "REF rejected, then clean sonar: HOLD", .cx = 35, .width = 185, .face_y = 900,
                      .fault_at_stage = B5MI_WIDTH, .fault_count = 1, .hold_frames = 3000};
    o = run(&sr0);
    CHECK(g_mission.width.end == B5WP_END_REFERENCE); CHECK(g_fault_pings == 0);
    CHECK(o.width_basis == B5WE_BASIS_PROJECT_RANGE_ASSUMPTION && o.display_mm100 == 15000);
    CHECK(o.final_stage == B5MI_HOLD); CHECK(o.reason == B5MI_R_WIDTH_ABORT);
    CHECK(g_mission.module_reason == B5WP_END_REFERENCE);
    CHECK(g_mission.width_attempted && g_mission.width_policy == B5MI_WIDTH_FAILED);
    CHECK(o.handbacks == 0 && !g_mission.direction_attempted);
    CHECK(g_mission.width_gate_mm >= 270 && g_mission.width_gate_mm <= 330);
    /* A deliberate clean out-of-window stop is a labelled SKIP: no probe
     * attempt, no motion for it, mission continues. Distinct from FAILED. */
    /* v0.2.12 straight gate: the gate needs 200 mm of straight travel first,
     * so the 250 mm box is placed in front once the gate has opened (a
     * sudden obstacle), as a fixed 250 mm start would now hold NOT_STRAIGHT. */
    scenario_t snear = {.name = "start 250 mm: clean skip", .cx = 60, .width = 185, .face_y = 250 + 230,
                        .appear_when_open = true};
    o = run(&snear);
    CHECK(g_mission.width_policy == B5MI_WIDTH_SKIPPED); CHECK(!g_mission.width_attempted);
    CHECK(g_mission.width_gate_mm < 270 || g_mission.width_gate_mm > 330);
    CHECK(g_mission.width.rec_index == 0); /* frozen probe never ran */
    CHECK(o.handbacks == 1); CHECK(g_contacts == 0);
    /* A faulty or inconsistent policy range is never a skip: HOLD RANGE. */
    scenario_t sgf = {.name = "faults in width-policy range", .cx = 35, .width = 185, .face_y = 900,
                      .fault_in_width_gate = true};
    o = run(&sgf);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_RANGE);
    CHECK(g_mission.width_policy == B5MI_WIDTH_NONE && !g_mission.width_attempted);
    scenario_t sgn = {.name = "noisy width-policy range", .cx = 35, .width = 185, .face_y = 900,
                      .noise_in_width_gate = true};
    o = run(&sgn);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_RANGE);
    CHECK(g_mission.width_policy == B5MI_WIDTH_NONE && !g_mission.width_attempted);
    /* Review item 2: bounded line steering during the owned approach. A push
     * after the leg starts (odometry cannot see it) is corrected by the line;
     * the same push with steering disabled is not. */
    const float pushes[][2] = {{3.0f, -2.0f}, {-1.0f, 0.0f}, {1.5f, 1.0f}};
    for (unsigned i = 0; i < 3; ++i) {
        char n_on[64], n_off[64];
        snprintf(n_on, sizeof n_on, "approach push %.1fdeg %.0fmm steered", pushes[i][0], pushes[i][1]);
        snprintf(n_off, sizeof n_off, "approach push %.1fdeg %.0fmm straight", pushes[i][0], pushes[i][1]);
        scenario_t s_on = {.name = n_on, .cx = 60, .width = 185, .face_y = 900,
                           .disturb_deg = pushes[i][0], .disturb_mm = pushes[i][1]};
        scenario_t s_off = s_on; s_off.name = n_off; s_off.steering_off = true;
        outcome_t on = run(&s_on);
        unsigned sat = g_mission.steer_saturated, peak = g_mission.steer_max_decideg;
        CHECK(on.reached_direction); CHECK(on.handbacks == 1); CHECK(g_contacts == 0);
        CHECK(on.bypass_range >= 195 && on.bypass_range <= 215); /* sonar stop unchanged */
        CHECK(peak > 0 && peak <= B5MI_STEER_MAX_DECIDEG); (void)sat;
        outcome_t off = run(&s_off);
        CHECK(g_mission.steer_max_decideg == 0);
        float e_on = fabsf(on.dir_edge_err_mm), e_off = fabsf(off.dir_edge_err_mm);
        float h_on = fabsf(on.dir_heading_deg), h_off = fabsf(off.dir_heading_deg);
        printf("  push %u: edge err %.1f (steered) vs %.1f (straight)%s; heading %.2f vs %.2f\n", i, e_on,
               e_off, off.reached_direction ? "" : " [straight: no direction]", h_on, h_off);
        CHECK(e_on < 3.5f); CHECK(h_on < fabsf(pushes[i][0]));
        if (off.reached_direction) CHECK(e_on < e_off && h_on < h_off);
        else CHECK(off.final_stage == B5MI_HOLD && off.reason == B5MI_R_NO_LINE_APPROACH);
    }
    scenario_t sg = {.name = "sonar faults in guard", .cx = 35, .width = 185, .face_y = 900,
                     .fault_at_stage = B5MI_GUARD, .fault_count = 5};
    o = run(&sg);
    CHECK(o.final_stage == B5MI_HOLD); CHECK(o.reason == B5MI_R_SENSOR);
    scenario_t sn = {.name = "line lost during approach", .cx = 35, .width = 185, .face_y = 900,
                     .lose_line_in_approach = true};
    o = run(&sn);
    CHECK(o.final_stage == B5MI_HOLD); CHECK(o.reason == B5MI_R_NO_LINE_APPROACH);
    CHECK(o.handbacks == 0); CHECK(g_mission.steer_max_decideg <= B5MI_STEER_MAX_DECIDEG);
    CHECK(o.final_stage != B5MI_HOLD || g_mission.module_reason == 0); /* not an executor fault */
    const unsigned stages[] = {B5MI_GUARD, B5MI_HALTING, B5MI_WIDTH, B5MI_ALIGN, B5MI_RANGE,
                               B5MI_APPROACH, B5MI_DIRECTION, B5MI_BYPASS};
    for (unsigned i = 0; i < sizeof(stages) / sizeof(stages[0]); ++i) {
        char name[64];
        snprintf(name, sizeof(name), "Stop during %s", b5_mission_stage_name(stages[i]));
        scenario_t ss = {.name = name, .cx = 35, .width = 185, .face_y = 900, .cancel = true,
                         .cancel_stage = stages[i]};
        o = run(&ss);
        CHECK(o.final_stage == B5MI_CANCELLED || o.final_stage == B5MI_IDLE);
        CHECK(o.handbacks == 0);
        CHECK(b5_wp_ready(&g_mission.width) && b5_ep_ready(&g_mission.edge));
        CHECK(!g_mission.bypass.session);
    }
    gate_scenarios(&failures);
    if (g_frames) fclose(g_frames);
    printf("%s: %u failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
