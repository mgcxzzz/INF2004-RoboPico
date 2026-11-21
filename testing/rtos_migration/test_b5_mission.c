/* Host test: Buddy5 RTOS mission (app_program/obstacle_navigation/b5_mission.c)
 * with the REAL shared coordinator/executor (motion_api.c, nav_coordinator.c,
 * nav_codec.c, motion_executor.c) and the ported Buddy5 modules, driven in
 * control.c order (line provider -> motion_update -> mission step) against a
 * simple kinematic plant (wheel speed = target), a geometric HC-SR04 cone
 * model of one rectangular box and a tape strip centred on x = 0.
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
#include "extensions/buddy5_obstacles/b5_event_pack.h"
#include "b5_mission_pack.h"
#include "width_retry_policy.h"
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
static const uint16_t *g_ref_script;
static const uint16_t *g_stopped_approach_echoes;
static unsigned g_stopped_script_batches;
static unsigned g_first_width_gate_mm;
static unsigned g_ref_count;
static unsigned g_ref_index;
static unsigned g_pose_script_phase;
static unsigned g_pose_script_triplets;
static unsigned g_pose_script_pings;
static double g_script_pose[2];
static bool g_line_present = true;
static bool g_early_right_mark;
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
    const B5WidthProbe *probe = &g_mission.width;
    if (g_stopped_approach_echoes && g_mission.stage == B5MI_RANGE &&
        g_mission.after_range == B5MI_APPROACH && g_mission.approach_legs > 0U &&
        (g_stopped_script_batches == 0 || g_mission.range.attempts < g_stopped_script_batches)) {
        unsigned range = g_stopped_approach_echoes[g_mission.range.npings];
        return (b5_ping_t){.cls = B5_PING_ECHO, .range_mm = range,
                           .width_us = range * 2000U / 343U};
    }
    if (g_first_width_gate_mm && g_mission.stage == B5MI_RANGE &&
        g_mission.after_range == B5MI_WIDTH && !g_mission.width_rechecked) {
        return (b5_ping_t){.cls = B5_PING_ECHO, .range_mm = g_first_width_gate_mm,
                           .width_us = g_first_width_gate_mm * 2000U / 343U};
    }
    if (g_pose_script_triplets != 0 && g_mission.stage == B5MI_WIDTH &&
        probe->sample_phase == g_pose_script_phase && probe->sample_step == 1 && probe->cur == 0) {
        if (g_pose_script_pings == 0) {
            g_script_pose[0] = g_wheel[0];
            g_script_pose[1] = g_wheel[1];
        }
        assert(g_wheel[0] == g_script_pose[0] && g_wheel[1] == g_script_pose[1]);
        if (g_pose_script_pings < 3 * g_pose_script_triplets) {
            int expected = b5_wp_expected_mm(probe->d0, (int16_t)lroundf(probe->heading_deg * 100),
                                            (int16_t)lroundf(probe->net_y_mm * 10));
            int range = (probe->npings == 2) ? expected - B5WP_NEAR_MM - 8 : expected;
            ++g_pose_script_pings;
            return (b5_ping_t){.cls = B5_PING_ECHO, .range_mm = (uint32_t)range,
                               .width_us = (uint32_t)(range * 2000 / 343)};
        }
    }
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
    if (g_ref_index < g_ref_count && g_mission.stage == B5MI_WIDTH &&
        g_mission.width.state == B5WP_REF) {
        best = g_ref_script[g_ref_index++];
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
/* Two 10 mm sensor spots at x = -8 and +8. The strip spans both spots on
 * centre, as in the v0.3.3 run with two saturated sensors at takeover. */
static float sensor_x(float lateral) {
    float fx, fy; fwd(g_heading, &fx, &fy);
    float rx = fy;
    return g_x + 130 * fx + lateral * rx;
}
static uint16_t sim_level(float lateral) {
    if (g_early_right_mark && g_mission.bypass.state == B5A_REJOIN &&
        g_mission.bypass.phase == B5R_RETURN && lateral > 0.0f &&
        nav_get_status().progress_mm < 16.0f) return 1000;
    if (!g_line_present) return 40;
    float px = sensor_x(lateral);
    float lo = fmaxf(px - 5.0f, -13.0f), hi = fminf(px + 5.0f, 13.0f);
    float frac = hi > lo ? (hi - lo) / 10.0f : 0.0f;
    return (uint16_t)lroundf(40 + 920 * frac);
}
/* The model uses Buddy3's symmetric reflectance law. */
static symmetric_tracker_t g_b3;
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
    B5WPRec r; B5Event ev;
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
        .width_produced = (uint8_t)m->width.rec_index,
        .direction_end = m->direction_end,
        .states = m->direction_states};
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
        analog_demand_t d = symmetric_demand(&g_b3, &g_ident, f.line_level[0], f.line_level[1],
                                            g_now, (uint32_t)fabs(path));
        int16_t target[2] = {d.stop ? 45 : d.target[0], d.stop ? 45 : d.target[1]};
        b5_mission_guard_limit(&g_mission, target);
        ls.decision = 25; /* continuous steering output code (small corrections) */
        (void)nav_submit_navigation(MOTION_BUDDY3, target[0], target[1]);
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
    bool contradict_left_width_edge;
    bool fault_after_ref; /* width stage: inject only after the REF sample */
    bool fault_in_width_gate; /* faults in the fresh range deciding the width policy */
    bool noise_in_width_gate; /* clean but inconsistent echoes in that range */
    const uint16_t *ref_script;
    const uint16_t *stopped_approach_echoes;
    unsigned stopped_script_batches;
    unsigned first_width_gate_mm;
    float late_approach_loss_mm;
    bool stop_after_guard_recheck;
    unsigned ref_count;
    unsigned pose_script_phase, pose_script_triplets;
    float width_return_drift_mm; /* modelled and simulated lateral return drift */
    float width_return_yaw_deg;
    float width_return_slip_mm; /* physical-only shift after the scan */
    float bypass_return_slip_mm; /* lateral error visible to the IR, not encoders */
    bool no_rejoin_tape;
    bool early_right_mark;
    float rejoin_yaw_error_deg; /* wheel odometry does not see this pose change */
    bool lose_tape_after_crossing;
    unsigned takeover_sensor_override;
    float disturb_deg, disturb_mm; /* physical heading (+LEFT)/lateral (+RIGHT)
                                      error injected at the first approach leg */
    bool steering_off;     /* comparison only: mission steering disabled */
    unsigned hold_frames;  /* extra frames to observe a HOLD (default 100) */
    bool lose_line_in_approach, cancel, cancel_recenter_drive;
    unsigned startup_line_losses; /* mask tape while the specified legs move */
    unsigned cancel_stage;
    line_hook_t line_hook;   /* Buddy3 state injected into the gate frames */
    ping_hook_t ping_hook;   /* scripted guard pings (>=0) or geometric (<0) */
    bool no_line_samples;    /* Buddy3 frames stop arriving: stale */
    bool appear_when_open;   /* box placed only once the gate first opens */
    bool start_rearming;     /* start the guard in its state after hand-back */
    bool stop_at_takeover;   /* end the run once HALTING is entered */
    bool stop_at_width;      /* end when the normal width stage is reached */
    unsigned ping_latency_us; /* request -> result (default 3 ms) */
} scenario_t;

typedef struct {
    unsigned final_stage, reason, side, basis, width_basis, handbacks;
    uint16_t display_mm100, d0, bypass_range;
    float end_x, end_y, end_heading_deg;
    float bypass_edge_err_mm, bypass_heading_deg; /* at bypass start */
    unsigned approach_line_mask;
    unsigned handback_line_mask;
    bool reached_bypass;
} outcome_t;

static outcome_t run(const scenario_t *sc) {
    if (g_frames) fprintf(g_frames, "SCENARIO %s\n", sc->name);
    memset(&g_mission, 0, sizeof(g_mission));
    g_now = 1000000; g_wheel[0] = g_wheel[1] = 0; g_speed[0] = g_speed[1] = 0;
    g_x = 0; g_y = 0; g_heading = 0; g_busy = g_ready = false; g_next_ping = 0;
    g_fault_pings = 0; g_noise_pings = 0; g_line_present = true; g_b3 = (symmetric_tracker_t){0}; g_contacts = 0; g_min_clear = 1e9f;
    g_early_right_mark = sc->early_right_mark;
    g_ref_script = sc->ref_script; g_ref_count = sc->ref_count; g_ref_index = 0;
    g_stopped_approach_echoes = sc->stopped_approach_echoes;
    g_stopped_script_batches = sc->stopped_script_batches;
    g_first_width_gate_mm = sc->first_width_gate_mm;
    g_pose_script_phase = sc->pose_script_phase;
    g_pose_script_triplets = sc->pose_script_triplets; g_pose_script_pings = 0;
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
    g_mission.rearm = sc->start_rearming;
    outcome_t out = {0};
    bool faulted = false, lost = false, disturbed = false;
    bool rejoin_started = false;
    bool rejoin_turn_checked = false;
    unsigned startup_losses = 0;
    unsigned prev_stage = g_mission.stage;
    for (unsigned k = 0; k < 120000; ++k) { /* <= 1200 s simulated */
        if (sc->contradict_left_width_edge && g_mission.stage == B5MI_WIDTH &&
            g_mission.width.sample_phase == B5WP_PH_MID) {
            g_mission.width.side[0].contradictory = true;
        }
        frame();
        unsigned st = g_mission.stage;
        if (sc->takeover_sensor_override != 0 && prev_stage == B5MI_HALTING &&
            st == B5MI_RANGE) {
            g_mission.takeover_mask = (uint8_t)sc->takeover_sensor_override;
        }
        if (!rejoin_started && g_mission.bypass.state == B5A_REJOIN) {
            rejoin_started = true;
            g_x += sc->bypass_return_slip_mm;
            if (sc->no_rejoin_tape) g_line_present = false;
        }
        if (!rejoin_turn_checked && st == B5MI_BYPASS &&
            g_mission.bypass.state == B5A_REJOIN &&
            g_mission.bypass.phase == B5R_CONFIRM) {
            rejoin_turn_checked = true;
            g_heading += sc->rejoin_yaw_error_deg * PI_F / 180.0f;
            if (sc->lose_tape_after_crossing) g_line_present = false;
        }
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
        if (st == B5MI_ALIGN && prev_stage != B5MI_ALIGN && !g_mission.approach_retried) {
            g_x += sc->width_return_drift_mm + sc->width_return_slip_mm;
            g_heading += sc->width_return_yaw_deg * PI_F / 180;
            g_mission.route_lateral_mm -= sc->width_return_drift_mm;
            g_mission.align_heading_deg += sc->width_return_yaw_deg;
        }
        if (st == B5MI_APPROACH && g_mission.sub == 1 && out.approach_line_mask == 0) {
            out.approach_line_mask = symmetric_line_mask(g_mission.line[0], g_mission.line[1]);
        }
        if (st == B5MI_BYPASS && !out.reached_bypass) {
            out.reached_bypass = true;
            out.bypass_edge_err_mm = sensor_x(-8) + 8; out.bypass_heading_deg = g_heading * 180 / PI_F;
        }
        if (sc->lose_line_in_approach && !lost && st == B5MI_APPROACH && g_mission.sub == 1) {
            g_line_present = false; lost = true;
        }
        if (sc->late_approach_loss_mm > 0.0f && !lost &&
            st == B5MI_APPROACH && g_mission.sub == 1 &&
            nav_get_status().progress_mm >= sc->late_approach_loss_mm) {
            g_line_present = false; lost = true;
        }
        if (st == B5MI_APPROACH && g_mission.sub == 1 &&
            startup_losses < sc->startup_line_losses &&
            startup_losses < g_mission.approach_legs &&
            nav_get_status().progress_mm >= 5.0f) {
            g_line_present = false;
            ++startup_losses;
        }
        if (st == B5MI_ALIGN && g_mission.approach_retried) {
            assert(nav_get_status().mode != MOTION_DRIVING);
            if (startup_losses != 0 || sc->late_approach_loss_mm > 0.0f) g_line_present = true;
        }
        if ((sc->cancel && st == sc->cancel_stage) ||
            (sc->cancel_recenter_drive && st == B5MI_ALIGN &&
             nav_get_status().mode == MOTION_DRIVING)) {
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
        if (sc->stop_at_width && st == B5MI_WIDTH) break;
        if (sc->stop_after_guard_recheck && prev_stage == B5MI_RANGE && st == B5MI_GUARD) break;
        if (prev_stage == B5MI_BYPASS && st == B5MI_GUARD) {
            ++out.handbacks;
            out.handback_line_mask = symmetric_line_mask(g_mission.line[0], g_mission.line[1]);
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
    printf("  width policy=%u gate=%u attempted=%u | steer last=%d max=%u sat=%u | bypass edge_err=%.1f hdg=%.2f\n",
           g_mission.width_policy, g_mission.width_gate_mm, g_mission.width_attempted,
           g_mission.steer_decideg, g_mission.steer_max_decideg, g_mission.steer_saturated,
           out.bypass_edge_err_mm, out.bypass_heading_deg);
    printf("  direction=%u states=0x%x | width end=%u | recenter=%.1f mm\n",
           g_mission.direction_decision, g_mission.direction_states,
           g_mission.width.end, g_mission.recenter_mm);
    printf("%-34s stage=%-9s reason=%-17s side=%u basis=%u width=%u.%02u(b%u) d0=%u "
           "start=%u contacts=%u min_clear=%.0f end=(%.0f,%.0f,%.1fdeg) stages=0x%x\n",
           sc->name, b5_mission_stage_name(out.final_stage), b5_mission_reason_name(out.reason),
           out.side, out.basis, out.display_mm100 / 100, out.display_mm100 % 100, out.width_basis,
           out.d0, out.bypass_range, g_contacts, g_min_clear, out.end_x, out.end_y,
           out.end_heading_deg, g_stage_mask);
    return out;
}

/* ---- straight-only takeover gate scenarios ---- */
static void hook_curve(b5_line_sample_t *s, float path) {
    /* Present encoder travel for a 250 mm-radius bend while the sonar plant
     * sees the close reflector. The curve has ordinary tracking state. */
    float differential = fmaxf(0, path - 400.0f) * (110.0f / 250.0f) / 2;
    s->wheel_mm[0] -= differential;
    s->wheel_mm[1] += differential;
}
static int ping_straight_lead(float path) {
    if (path < 300.0f) return B5MI_CONFIRM_RANGE_MM + 1;
    return path < 320.0f ? B5MI_CONFIRM_RANGE_MM : B5MI_STRAIGHT_STOP_MM;
}
static int ping_curve_lead(float path) {
    if (path < 440.0f) return B5MI_CONFIRM_RANGE_MM + 1;
    return path < 530.0f ? B5MI_CONFIRM_RANGE_MM : B5MI_CLOSE_HOLD_MM;
}
static int ping_rearm_lead(float path) {
    return path < 330.0f ? B5MI_CLOSE_HOLD_MM + 1 : B5MI_STRAIGHT_STOP_MM;
}
static void hook_junction(b5_line_sample_t *s, float path) { (void)path; s->junction_mark = true; }
static void hook_recovery(b5_line_sample_t *s, float path) { (void)path; s->searching = true; s->decision = SYM_RECOVER_LEFT; }
static void hook_invalid(b5_line_sample_t *s, float path) { (void)path; s->valid = false; }
/* Carry-over: a junction until 300 mm with two close echoes inside it, then a
 * straight; the gate opens after its straight-travel window. Close echoes
 * start one or two pings BEFORE the gate opens (requested while closed). */
static void hook_carry(b5_line_sample_t *s, float path) {
    if (path < 300) { s->junction_mark = true; }
    /* a close echo IN FLIGHT across the opening (requested while closed) */
    if (g_mission.straight.open && g_busy && g_req_at < g_mission.straight.open_us) {
        g_result.cls = B5_PING_ECHO; g_result.range_mm = 280;
    }
}
static unsigned g_script_near;
static int ping_carry(float path) {
    if (path >= 250 && path < 300 && g_script_near < 2) { ++g_script_near; return 280; }
    if (g_mission.straight.open) return 280;
    /* Inject a close echo near the end of the window, still in flight at
     * the transition to open. */
    if (g_mission.gate_reason == B5S_SETTLING &&
        g_mission.straight.travel_mm >= (float)B5S_WINDOW_MM + 2.0f) return 280;
    return 0;
}
/* Re-qualify: straight; two qualifying close echoes after the gate opens,
 * then (before any far echo is taken) a junction for 60 mm with far pings,
 * then the straight again: three fresh echoes after the gate re-opens. */
static unsigned g_req_phase1, g_req_state;
static float g_req_bend_end;
static void hook_requalify(b5_line_sample_t *s, float path) {
    if (g_req_state == 0 && g_mission.confirm >= 2) { g_req_state = 1; g_req_bend_end = path + 60; }
    if (g_req_state == 1) { if (path < g_req_bend_end) s->junction_mark = true; else g_req_state = 2; }
}
static int ping_requalify(float path) {
    if (g_req_state == 0) return (g_mission.straight.open && path >= 420 && g_req_phase1 < 2) ? (++g_req_phase1, 280) : 0;
    if (g_req_state == 1) return 0;
    return g_mission.straight.open ? 280 : 0;
}
static void hook_route_turn(b5_line_sample_t *s, float path) {
    hook_requalify(s, path);
    s->route_turn = s->junction_mark;
    s->junction_mark = false;
}
static int ping_route_turn(float path) {
    return g_req_state == 1 ? 263 : ping_requalify(path);
}
static int ping_transient_while_settling(float path) {
    return (path >= 32.0f && g_mission.run == 0U) ? 292 : 0;
}
static int ping_persistent_while_settling(float path) {
    static const int stopped[] = {275, 276, 278};
    if (g_mission.stage == B5MI_RANGE && g_mission.guard_rechecking) {
        return stopped[g_mission.range.npings];
    }
    return path >= 32.0f ? 281 : 0;
}
static int ping_mixed_while_settling(float path) {
    static const int mixed[] = {276, 470, 0};
    if (g_mission.stage == B5MI_RANGE && g_mission.guard_rechecking && g_mission.range.attempts == 0) {
        return mixed[g_mission.range.npings];
    }
    return ping_persistent_while_settling(path);
}
static int ping_borderline_while_settling(float path) {
    static const int stopped[] = {272, 265, 271};
    if (g_mission.stage == B5MI_RANGE && g_mission.guard_rechecking) {
        return stopped[g_mission.range.npings];
    }
    if (g_mission.stage == B5MI_GUARD && g_mission.run == 0U) {
        return path >= 32.0f ? 273 : 0;
    }
    return -1; /* geometric echoes once the real probe starts */
}
static int ping_too_close_while_settling(float path) {
    return path >= 32.0f ? 250 : 0;
}
static int ping_without_tape_while_settling(float path) {
    if (g_mission.stage == B5MI_RANGE && g_mission.guard_rechecking) g_line_present = false;
    return ping_persistent_while_settling(path);
}
static int ping_near_straight_window(float path) {
    return path >= 113.0f ? 292 : 0;
}
static void gate_scenarios(unsigned *p_failures) {
#define GCHECK(c) do { if (!(c)) { printf("  FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++*p_failures; } } while (0)
    const unsigned probing = (1u << B5MI_WIDTH) | (1u << B5MI_ALIGN) | (1u << B5MI_RANGE) | (1u << B5MI_APPROACH) |
                             (1u << B5MI_DIRECTION) | (1u << B5MI_BYPASS);
    /* 1. Close obstacle while the gate is closed: stop and HOLD NOT_STRAIGHT
     * with the gate reason; no probe, no bypass; Buddy5 keeps the stopped
     * robot until an operator Stop. */
    struct { const char *name; line_hook_t hook; bool no_samples; uint8_t reason; } blocked[] = {
        {"blocked: close echo on encoder curve", hook_curve, false, B5S_CURVE},
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
        GCHECK((g_stage_mask & probing) == 0 && g_mission.width.rec_index == 0);
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
    scenario_t straight_lead = {.name = "early echoes confirm before first stop echo",
        .ping_hook = ping_straight_lead, .stop_at_takeover = true};
    outcome_t o = run(&straight_lead);
    GCHECK(o.final_stage == B5MI_HALTING && g_y >= 319.0f && g_y < 325.0f);
    GCHECK(g_mission.trigger_mm == B5MI_STRAIGHT_STOP_MM);
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
    scenario_t settling = {.name = "settling grace opens existing straight gate",
        .ping_hook = ping_near_straight_window, .stop_at_takeover = true};
    o = run(&settling);
    GCHECK(o.final_stage == B5MI_HALTING && !g_mission.guard_rechecking);
    settling = (scenario_t){.name = "settling false echo clears at stopped recheck",
        .ping_hook = ping_transient_while_settling, .stop_after_guard_recheck = true};
    o = run(&settling);
    GCHECK(o.final_stage == B5MI_GUARD && g_mission.run == 1U);
    GCHECK((g_stage_mask & (1u << B5MI_RANGE)) != 0U && !g_mission.width_attempted);
    GCHECK(nav_get_status().owner == MOTION_BUDDY3);
    settling = (scenario_t){.name = "settling recheck admits captured 275/276/278 mm",
        .ping_hook = ping_persistent_while_settling, .stop_at_width = true};
    o = run(&settling);
    GCHECK(o.final_stage == B5MI_WIDTH && g_mission.width_started && g_mission.range.attempts == 1U);
    GCHECK(g_mission.trigger_mm == 281 && g_mission.range.range_mm == 276);
    GCHECK(g_mission.gate_reason == B5S_SETTLING && !g_mission.guard_rechecking);
    GCHECK((g_stage_mask & (1u << B5MI_HOLD)) == 0);
    settling.name = "settling recheck retries a mixed batch";
    settling.ping_hook = ping_mixed_while_settling;
    o = run(&settling);
    GCHECK(o.final_stage == B5MI_WIDTH && g_mission.width_started && g_mission.range.attempts == 2U);
    GCHECK(g_mission.range.range_mm == 276 && (g_stage_mask & (1u << B5MI_HOLD)) == 0);
    static const uint16_t borderline_ref[] = {272, 265, 271};
    scenario_t borderline = {.name = "captured borderline batch: probe and bypass",
        .cx = 0, .width = 150, .face_y = 373,
        .ping_hook = ping_borderline_while_settling,
        .ref_script = borderline_ref, .ref_count = 3};
    o = run(&borderline);
    GCHECK(g_mission.width.allow_borderline_ref && g_mission.width.ref_ok && o.d0 == 271);
    GCHECK(g_ref_index == 3 && g_mission.width.retry_count == 0);
    GCHECK(o.handbacks == 1 && g_contacts == 0 && (g_stage_mask & (1u << B5MI_HOLD)) == 0);
    settling.name = "settling recheck below the width reference window";
    settling.ping_hook = ping_too_close_while_settling;
    o = run(&settling);
    GCHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_NOT_STRAIGHT);
    GCHECK(g_mission.range.attempts == 2U && (g_stage_mask & (probing & ~(1u << B5MI_RANGE))) == 0);
    settling.name = "settling recheck loses tape after stopping";
    settling.ping_hook = ping_without_tape_while_settling;
    o = run(&settling);
    GCHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_NOT_STRAIGHT);
    GCHECK(g_mission.range.attempts == 2U && (g_stage_mask & (probing & ~(1u << B5MI_RANGE))) == 0);
    scenario_t short_straight = {.name = "obstacle inside the old 200 mm lead-in",
        .cx = 0, .width = 150, .face_y = 550, .stop_at_takeover = true};
    o = run(&short_straight);
    GCHECK(o.final_stage == B5MI_HALTING && g_y >= B5S_WINDOW_MM && g_y < 200.0f);
    GCHECK(g_mission.trigger_mm <= B5MI_STRAIGHT_STOP_MM && g_contacts == 0);
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
    scenario_t curve_lead = {.name = "early echo on curve, HOLD at 300 mm",
        .line_hook = hook_curve, .ping_hook = ping_curve_lead};
    o = run(&curve_lead);
    GCHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_NOT_STRAIGHT);
    GCHECK(g_mission.module_reason == B5S_CURVE && g_y >= 529.0f);
    GCHECK(g_mission.run == 0 && (g_stage_mask & probing) == 0);
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
    scenario_t rearm_lead = {.name = "rearming requires the full confirmation zone to clear",
        .ping_hook = ping_rearm_lead, .start_rearming = true};
    o = run(&rearm_lead);
    GCHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_SENSOR);
    GCHECK(g_y >= 329.0f && g_mission.run == 0 && (g_stage_mask & probing) == 0);
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
    /* 2. Old close echoes never carry over into a new straight. */
    g_script_near = 0;
    scenario_t sc = {.name = "carry-over: echoes in junction, then straight", .line_hook = hook_carry,
                     .ping_hook = ping_carry, .stop_at_takeover = true, .ping_latency_us = 55000};
    o = run(&sc);
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
    printf("  carry-over: %u close echoes while closed (junction/settling) and %u in-flight echo(es) requested before "
           "opening never counted; takeover on the 3rd fresh echo (%u)\n", near_closed, pre_open_after, fresh);
    (void)o;
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);
    /* 3. Losing the gate resets the count: 2 qualifying echoes, junction entry,
     * then three fresh echoes after the gate re-opens. */
    g_req_phase1 = 0; g_req_state = 0;
    scenario_t sr = {.name = "re-qualify: 2 echoes, junction entry, 3 fresh", .line_hook = hook_requalify,
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
        if (t->gate == B5S_JUNCTION) GCHECK(t->confirm == 0);
    }
    GCHECK(max_before == 2 && after == 3);
    GCHECK(g_drop_seen && g_drop_before == 2 && g_drop_after == 0); /* reset by the gate, not by a far echo */
    printf("  re-qualify: %u qualifying echoes discarded at junction entry; takeover after %u fresh echoes on the "
           "re-opened straight\n", max_before, after);
    b5_mission_cancel(&g_mission, g_now); (void)motion_stop(MOTION_BUDDY1); motion_session_end(1);

    /* A close reflector during a turn would cause HOLD under the ordinary
     * closed-gate rule. Discard the old echoes and qualify the real obstacle
     * with new pings after the turn, even if a ping was already in flight. */
    g_req_phase1 = 0; g_req_state = 0;
    scenario_t turn = {.name = "protect route turn, then confirm next obstacle",
        .line_hook = hook_route_turn, .ping_hook = ping_route_turn,
        .stop_at_takeover = true, .ping_latency_us = 55000};
    o = run(&turn);
    GCHECK(g_req_state == 2 && g_drop_seen && g_drop_before == 2 && g_drop_after == 0);
    GCHECK(o.final_stage == B5MI_HALTING && (g_stage_mask & (1u << B5MI_HOLD)) == 0);
    after = 0;
    first_open = 0;
    for (unsigned k = 0; k < g_ntakes; ++k) {
        const take_log_t *t = &g_takes[k];
        GCHECK(t->gate != B5S_TURN);
        if (!first_open && t->gate == B5S_STRAIGHT) first_open = t->open_us;
        if (t->gate == B5S_STRAIGHT && t->open_us != first_open && t->near) {
            if (t->req_us < t->open_us) {
                GCHECK(t->confirm == 0);
            } else {
                ++after;
                GCHECK(t->stage == B5MI_HALTING ? after == B5MI_TRIGGER_CONFIRM : t->confirm == after);
            }
        }
    }
    GCHECK(after == B5MI_TRIGGER_CONFIRM);
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
    CHECK(g_mission.takeover_mask == 3 && o.approach_line_mask == 3);
    /* Long probe (SDK width_long_probe v0.2): the whole chain width -> align ->
     * approach -> bypass ran in one mission; every outward pass is
     * bounded by clearance and the 220 mm ceiling and reports a stop reason;
     * D2, D5 and D3 were all drained; width stays display-only. */
    {
        const unsigned need = (1u << B5MI_WIDTH) | (1u << B5MI_ALIGN) | (1u << B5MI_RANGE) |
                              (1u << B5MI_APPROACH) | (1u << B5MI_BYPASS);
        CHECK((g_stage_mask & need) == need);
        CHECK(!(g_stage_mask & (1u << B5MI_DIRECTION)));
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
    scenario_t straight_return = sl;
    straight_return.name = "straight return: tape beyond the old 220 mm window";
    straight_return.bypass_return_slip_mm = 105.0f;
    straight_return.early_right_mark = true;
    o = run(&straight_return);
    CHECK(o.handbacks == 1 && g_contacts == 0 && fabsf(o.end_x) < 15.0f);
    CHECK(g_mission.bypass.stripe_result == B5S_FOUND);
    CHECK(g_mission.bypass.stripe.first_rise > 200.0f);
    CHECK(g_mission.bypass.stripe.last_p > g_mission.bypass.rect.return_search_mm);
    straight_return.name = "straight return: one early dark sensor, no tape";
    straight_return.bypass_return_slip_mm = 0.0f;
    straight_return.no_rejoin_tape = true;
    o = run(&straight_return);
    CHECK(o.final_stage == B5MI_HOLD && o.handbacks == 0 && g_contacts == 0);
    CHECK(g_mission.bypass.reason == B5A_NO_LINE);
    CHECK(g_mission.bypass.stripe.last_p >= B5R_RETURN_MAX_MM - 2.0f);
    CHECK(nav_get_status().progress_mm < B5R_RETURN_MAX_MM + 5.0f);
    scenario_t sweep = sr;
    sweep.name = "wide sweep right after left-only takeover";
    sweep.rejoin_yaw_error_deg = 27.0f;
    sweep.takeover_sensor_override = 1U;
    o = run(&sweep);
    CHECK(o.handbacks == 1 && g_contacts == 0 && fabsf(o.end_x) < 15.0f);
    CHECK(g_mission.bypass.heading_deg < -12.0f && g_mission.bypass.searches == 1U);
    CHECK(o.handback_line_mask == 2U);
    sweep = sl;
    sweep.name = "wide sweep left after right-only takeover";
    sweep.rejoin_yaw_error_deg = -27.0f;
    sweep.takeover_sensor_override = 2U;
    o = run(&sweep);
    CHECK(o.handbacks == 1 && g_contacts == 0 && fabsf(o.end_x) < 15.0f);
    CHECK(g_mission.bypass.heading_deg > 12.0f && g_mission.bypass.searches == 1U);
    CHECK(o.handback_line_mask == 1U);
    sweep.name = "wide sweep reverses after an empty first side";
    sweep.rejoin_yaw_error_deg = 27.0f;
    sweep.takeover_sensor_override = 1U;
    o = run(&sweep);
    CHECK(o.handbacks == 1 && g_contacts == 0 && o.handback_line_mask == 2U);
    CHECK(g_mission.bypass.heading_deg < -12.0f && g_mission.bypass.searches == 2U);
    sweep.name = "wide sweep exhausts both sides with tape missing";
    sweep.lose_tape_after_crossing = true;
    o = run(&sweep);
    CHECK(o.final_stage == B5MI_HOLD && o.handbacks == 0 && g_contacts == 0);
    CHECK(g_mission.bypass.stripe_result == B5S_FOUND);
    CHECK(g_mission.bypass.reason == B5A_NO_LINE &&
          g_mission.bypass.searches == B5R_SEARCH_STEPS);
    scenario_t s100 = {.name = "100 offset RIGHT 35 -> LEFT", .cx = 35, .width = 100, .face_y = 900};
    o = run(&s100);
    CHECK(o.handbacks == 1); CHECK(o.side == B5_SIDE_LEFT); CHECK(g_contacts == 0);
    /* The capture showed both spots dark at takeover, then a saturated
     * right spot alone after the scan. Exercise that and its mirror. */
    scenario_t one_spot = sr;
    one_spot.name = "right tape only after scan";
    one_spot.width_return_slip_mm = -10.0f;
    o = run(&one_spot);
    CHECK(g_mission.takeover_mask == 3 && o.approach_line_mask == 2);
    CHECK(o.handbacks == 1 && g_contacts == 0);
    one_spot.name = "left tape only after scan";
    one_spot.width_return_slip_mm = 10.0f;
    o = run(&one_spot);
    CHECK(g_mission.takeover_mask == 3 && o.approach_line_mask == 1);
    CHECK(o.handbacks == 1 && g_contacts == 0);
    scenario_t return_drift = sr;
    return_drift.name = "width return 14.5 mm off line, recenter";
    return_drift.width_return_drift_mm = -14.5f;
    o = run(&return_drift);
    CHECK(g_mission.recenter_mm > 18.0f && g_mission.recenter_mm < 24.0f);
    CHECK(o.reached_bypass && o.handbacks == 1 && g_contacts == 0);
    static const uint16_t spread_echoes[] = {185, 207, 219}; /* mean 204 */
    static const uint16_t close_echoes[] = {95, 205, 312};   /* mean 204 */
    static const uint16_t conflict_echoes[] = {190, 241, 250};
    scenario_t average_range = sr;
    average_range.name = "spread forward echoes: use mean for bypass";
    average_range.stopped_approach_echoes = spread_echoes;
    o = run(&average_range);
    CHECK(o.reached_bypass && o.bypass_range == 204U && o.handbacks == 1 && g_contacts == 0);
    CHECK(g_mission.range.nearest_mm == 185U);
    CHECK(g_mission.range.farthest_mm == 219U);
    B5RectPlan mean_plan;
    CHECK(b5_rect_plan_ex(NULL, 204U, B5_PLAN_DEPTH_MM, false, &mean_plan));
    CHECK(g_mission.bypass.rect.forward_mm == mean_plan.forward_mm + 15.0f);
    average_range.name = "close forward echo blocks turn despite usable mean";
    average_range.stopped_approach_echoes = close_echoes;
    o = run(&average_range);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_BYPASS_BLOCKED);
    CHECK(o.bypass_range == 204U && g_mission.module_reason == B5A_CLEARANCE);
    CHECK(!o.reached_bypass && o.handbacks == 0 && g_contacts == 0);
    average_range.name = "near echo conflicts with an averaged approach";
    average_range.stopped_approach_echoes = conflict_echoes;
    o = run(&average_range);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_RANGE);
    CHECK(g_mission.range.attempts == B5MI_APPROACH_RANGE_ATTEMPTS);
    CHECK(g_mission.approach_legs == 1U && !o.reached_bypass && g_contacts == 0);
    average_range.name = "third stopped approach batch recovers";
    average_range.stopped_script_batches = 2U;
    o = run(&average_range);
    CHECK(o.handbacks == 1 && g_contacts == 0);
    CHECK(g_mission.range.attempts == B5MI_APPROACH_RANGE_ATTEMPTS);
    scenario_t width_recheck = sr;
    width_recheck.name = "slightly out-of-window width gate rechecks in place";
    width_recheck.first_width_gate_mm = 264U;
    o = run(&width_recheck);
    CHECK(o.handbacks == 1 && g_mission.width_rechecked && g_contacts == 0);
    scenario_t lost_late = sr;
    lost_late.name = "late approach line loss recovers once";
    lost_late.late_approach_loss_mm = 20.0f;
    o = run(&lost_late);
    CHECK(o.handbacks == 1 && g_mission.approach_retried && g_contacts == 0);
    CHECK(fabsf(o.bypass_edge_err_mm) < 6.0f);
    return_drift.name = "Stop during alignment recenter drive";
    return_drift.cancel_recenter_drive = true;
    o = run(&return_drift);
    CHECK(o.final_stage == B5MI_CANCELLED && !o.reached_bypass);
    CHECK(!(g_stage_mask & (1u << B5MI_APPROACH)));
    return_drift.cancel_recenter_drive = false;
    return_drift.name = "too much width return drift: hold before approach";
    return_drift.width_return_drift_mm = -26.0f;
    o = run(&return_drift);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_NO_LINE_ALIGN);
    CHECK(!(g_stage_mask & (1u << B5MI_APPROACH)) && !o.reached_bypass);
    return_drift.name = "width return yaw outside safe window: hold";
    return_drift.width_return_drift_mm = 0.0f;
    return_drift.width_return_yaw_deg = 13.0f;
    o = run(&return_drift);
    CHECK(o.final_stage == B5MI_HOLD && !o.reached_bypass);
    CHECK(!(g_stage_mask & (1u << B5MI_APPROACH)));
    scenario_t sc = {.name = "185 centred -> default (labelled)", .cx = 0, .width = 185, .face_y = 900};
    g_drop_d1 = 5; /* telemetry fixture: one D1 never reaches the dashboard */
    o = run(&sc);
    g_drop_d1 = -1;
    CHECK(o.basis == B5MI_DIR_DEFAULT && o.side == B5_SIDE_RIGHT);
    CHECK(o.handbacks == 1 && g_contacts == 0);
    scenario_t watch_faults = {.name = "watch: three genuine sonar faults",
                               .fault_at_stage = B5MI_GUARD, .fault_count = 3};
    o = run(&watch_faults);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_SENSOR);
    CHECK(g_mission.module_reason == B5MI_WATCH_FAULTS);
    CHECK(g_mission.run == 0 && !g_mission.width_attempted);

    scenario_t watch_timeout = {.name = "watch: pending sonar result goes stale",
                                .ping_latency_us = 400000};
    o = run(&watch_timeout);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_SENSOR);
    CHECK(g_mission.module_reason == B5MI_WATCH_TIMEOUT);
    CHECK(g_mission.run == 0 && !g_mission.width_attempted);

    scenario_t uncertain_side = sr;
    uncertain_side.name = "completed width with contradictory edge: default right";
    uncertain_side.contradict_left_width_edge = true;
    o = run(&uncertain_side);
    CHECK(g_mission.width.end == B5WP_END_COMPLETE);
    CHECK(g_mission.width.summary.state_l == B5WP_ST_CONTRADICTORY);
    CHECK(o.basis == B5MI_DIR_DEFAULT && o.side == B5_SIDE_RIGHT);
    CHECK(o.reached_bypass && o.handbacks == 1 && g_contacts == 0);
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
    /* Real stopped capture: first triplet is 289/265/290 mm. Retry the pose
     * once and apply the original REF test to the replacement triplet. */
    static const uint16_t ref_recovered[] = {289, 265, 290, 292, 293, 294};
    static const uint16_t ref_rejected[] = {289, 265, 290, 289, 265, 290};
    static const uint16_t ref_too_near[] = {289, 240, 290};
    scenario_t ref = {.name = "REF capture outlier, clean replacement",
        .cx = 60, .width = 185, .face_y = 900,
        .ref_script = ref_recovered, .ref_count = 6};
    o = run(&ref);
    CHECK(g_ref_index == 6 && g_mission.width.retries[0].value[1] == 265);
    CHECK(o.d0 == 293 && g_mission.width.end == B5WP_END_COMPLETE);
    CHECK(o.handbacks == 1 && g_contacts == 0);
    ref.name = "REF repeat outlier holds";
    ref.ref_script = ref_rejected;
    o = run(&ref);
    CHECK(g_ref_index == 6 && g_mission.width.retry_count == 1);
    CHECK(o.final_stage == B5MI_HOLD && g_mission.width.end == B5WP_END_REFERENCE);
    CHECK(!g_mission.direction_attempted && !g_mission.bypass.session);
    ref.name = "REF too-near echo holds without retry";
    ref.ref_script = ref_too_near;
    ref.ref_count = 3;
    o = run(&ref);
    CHECK(g_ref_index == 3 && g_mission.width.retry_count == 0);
    CHECK(o.final_stage == B5MI_HOLD && g_mission.width.end == B5WP_END_REFERENCE);
    /* Replay both captured outward triplets against the normal acceptance
     * rule and the new retry rule. The original triplets remain UNKNOWN. */
    b5_ping_t run3[3] = {
        {.cls = B5_PING_FAULT, .why = B5_PING_WHY_NO_RISE},
        {.cls = B5_PING_ECHO, .range_mm = 239},
        {.cls = B5_PING_ECHO, .range_mm = 279}};
    b5_ping_t run4[3] = {
        {.cls = B5_PING_ECHO, .range_mm = 264},
        {.cls = B5_PING_ECHO, .range_mm = 268},
        {.cls = B5_PING_ECHO, .range_mm = 236}};
    CHECK(b5_wp_classify(run3, 268) == B5WP_UNKNOWN && width_pose_retry_plausible(run3, 268, 120));
    CHECK(b5_wp_classify(run4, 266) == B5WP_UNKNOWN && width_pose_retry_plausible(run4, 266, 120));
    run3[0].why = B5_PING_WHY_SHORT;
    CHECK(!width_pose_retry_plausible(run3, 268, 120));
    CHECK(!width_pose_retry_plausible(run4, 266, 79));
    const b5_ping_t first_angle[3] = {
        {.cls = B5_PING_ECHO, .range_mm = 267},
        {.cls = B5_PING_ECHO, .range_mm = 240},
        {.cls = B5_PING_ECHO, .range_mm = 265}};
    const b5_ping_t second_angle[3] = {
        {.cls = B5_PING_ECHO, .range_mm = 287},
        {.cls = B5_PING_ECHO, .range_mm = 256},
        {.cls = B5_PING_ECHO, .range_mm = 285}};
    CHECK(b5_wp_classify(first_angle, 267) == B5WP_UNKNOWN &&
          width_pose_retry_plausible(first_angle, 267, 120));
    CHECK(b5_wp_classify(second_angle, 286) == B5WP_UNKNOWN &&
          width_pose_retry_plausible(second_angle, 286, 120));
    /* v0.3.5 hardware runs 1, 6 and 7. Use the logged reference distance
     * and forward pose for the conservative in-place corner clearance. */
    const struct {
        uint16_t expected_mm;
        uint16_t reference_mm;
        float forward_mm;
        uint16_t echo_mm[3];
    } captures[] = {
        {289, 287, 16.5f, {293, 254, 259}},
        {289, 288, 0.7f, {287, 287, 258}},
        {293, 306, 31.9f, {260, 278, 293}},
    };
    for (unsigned i = 0; i < sizeof captures / sizeof captures[0]; ++i) {
        b5_ping_t pings[3] = {{0}};
        for (unsigned k = 0; k < 3; ++k) {
            pings[k] = (b5_ping_t){.cls = B5_PING_ECHO, .range_mm = captures[i].echo_mm[k]};
        }
        uint16_t expected = captures[i].expected_mm;
        float gap = b5_wp_turn_front_gap(captures[i].reference_mm, captures[i].forward_mm);
        CHECK(b5_wp_classify(pings, expected) == B5WP_UNKNOWN);
        CHECK(width_pose_retry_plausible(pings, expected, gap));
        CHECK(!width_pose_retry_plausible(pings, expected, B5WP_FRONT_MARGIN_MM));
        for (unsigned k = 0; k < 3; ++k) pings[k].range_mm = expected - B5WP_NEAR_MM - 8;
        CHECK(!width_pose_retry_plausible(pings, expected, gap));
    }

    scenario_t outward = {.name = "OUT marginal echo, clean replacement",
        .cx = 60, .width = 185, .face_y = 900, .pose_script_phase = B5WP_PH_OUT, .pose_script_triplets = 1};
    o = run(&outward);
    CHECK(g_pose_script_pings == 3 && g_mission.width.retry_count == 1);
    CHECK(g_mission.width.retries[0].phase == B5WP_PH_OUT);
    CHECK(g_mission.width.end == B5WP_END_COMPLETE && o.handbacks == 1 && g_contacts == 0);
    outward.name = "OUT repeated marginal echo holds";
    outward.pose_script_triplets = 2;
    o = run(&outward);
    CHECK(g_pose_script_pings == 6 && g_mission.width.retry_count == 1);
    CHECK(o.final_stage == B5MI_HOLD && g_mission.width.end == B5WP_END_SENSOR);
    CHECK(!g_mission.direction_attempted && !g_mission.bypass.session);
    const B5WidthProbe *failed_probe = &g_mission.width;
    const B5WPRec *terminal = &failed_probe->terminal_record;
    CHECK(failed_probe->terminal_record_valid && failed_probe->rec_count == 0);
    CHECK(terminal->run == failed_probe->run && terminal->index + 1 == failed_probe->rec_index);
    CHECK(terminal->phase == B5WP_PH_OUT && terminal->evidence == B5WP_UNKNOWN);
    CHECK(terminal->value[2] == terminal->expected_mm - B5WP_NEAR_MM - 8);
    outward.name = "outward ladder marginal echo, clean replacement";
    outward.pose_script_phase = B5WP_PH_LAD_OUT;
    outward.pose_script_triplets = 1;
    o = run(&outward);
    CHECK(g_pose_script_pings == 3 && g_mission.width.retry_count == 1);
    CHECK(g_mission.width.retries[0].phase == B5WP_PH_LAD_OUT);
    CHECK(o.handbacks == 1 && !g_mission.width.terminal_record_valid && g_contacts == 0);
    outward.name = "return marginal echo, clean replacement";
    outward.pose_script_phase = B5WP_PH_RET;
    o = run(&outward);
    CHECK(g_pose_script_pings == 3 && g_mission.width.retry_count == 1);
    CHECK(g_mission.width.retries[0].phase == B5WP_PH_RET);
    CHECK(o.handbacks == 1 && !g_mission.width.terminal_record_valid && g_contacts == 0);
    /* A deliberate clean out-of-window stop is a labelled SKIP: no probe
     * attempt, no direction evidence, and a hold. Distinct from FAILED. */
    /* The straight gate needs qualifying straight travel first,
     * so the 250 mm box is placed in front once the gate has opened (a
     * sudden obstacle), as a fixed 250 mm start would now hold NOT_STRAIGHT. */
    scenario_t snear = {.name = "start 250 mm: clean skip", .cx = 60, .width = 185,
                        .face_y = 250 + B5S_WINDOW_MM + B5WP_SHAFT_MM,
                        .appear_when_open = true};
    o = run(&snear);
    CHECK(g_mission.width_policy == B5MI_WIDTH_SKIPPED); CHECK(!g_mission.width_attempted);
    CHECK(g_mission.width_gate_mm < 270 || g_mission.width_gate_mm > 330);
    CHECK(g_mission.width.rec_index == 0); /* probe never ran */
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_DIRECTION_UNKNOWN);
    CHECK(!o.reached_bypass && o.handbacks == 0 && g_contacts == 0);
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
        CHECK(on.reached_bypass); CHECK(on.handbacks == 1); CHECK(g_contacts == 0);
        CHECK(on.bypass_range >= 195 && on.bypass_range <= 215); /* sonar stop unchanged */
        CHECK(peak > 0 && peak <= B5MI_STEER_MAX_DECIDEG); (void)sat;
        outcome_t off = run(&s_off);
        CHECK(g_mission.steer_max_decideg == 0);
        float e_on = fabsf(on.bypass_edge_err_mm), e_off = fabsf(off.bypass_edge_err_mm);
        float h_on = fabsf(on.bypass_heading_deg), h_off = fabsf(off.bypass_heading_deg);
        printf("  push %u: edge err %.1f (steered) vs %.1f (straight)%s; heading %.2f vs %.2f\n", i, e_on,
               e_off, off.reached_bypass ? "" : " [straight: no bypass]", h_on, h_off);
        CHECK(e_on < 3.5f); CHECK(h_on < fabsf(pushes[i][0]));
        if (off.reached_bypass) CHECK(e_on < e_off && h_on < h_off);
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
    CHECK(g_mission.approach_retried && g_mission.approach_legs == 1);
    scenario_t startup = sr;
    startup.name = "startup line loss at 5 mm, recover once";
    startup.startup_line_losses = 1;
    o = run(&startup);
    CHECK(g_mission.approach_retried && g_mission.approach_legs == 2);
    CHECK(o.handbacks == 1 && o.bypass_range <= B5MI_APPROACH_ACCEPT_MM && g_contacts == 0);
    startup.name = "startup line loss twice, hold";
    startup.startup_line_losses = 2;
    o = run(&startup);
    CHECK(g_mission.approach_retried && g_mission.approach_legs == 2);
    CHECK(o.final_stage == B5MI_HOLD && o.reason == B5MI_R_NO_LINE_APPROACH);
    CHECK(!o.reached_bypass && g_contacts == 0);
    const unsigned stages[] = {B5MI_GUARD, B5MI_HALTING, B5MI_WIDTH, B5MI_ALIGN, B5MI_RANGE,
                               B5MI_APPROACH, B5MI_BYPASS};
    for (unsigned i = 0; i < sizeof(stages) / sizeof(stages[0]); ++i) {
        char name[64];
        snprintf(name, sizeof(name), "Stop during %s", b5_mission_stage_name(stages[i]));
        scenario_t ss = {.name = name, .cx = 35, .width = 185, .face_y = 900, .cancel = true,
                         .cancel_stage = stages[i]};
        o = run(&ss);
        CHECK(o.final_stage == B5MI_CANCELLED || o.final_stage == B5MI_IDLE);
        CHECK(o.handbacks == 0);
        CHECK(b5_wp_ready(&g_mission.width));
        CHECK(!g_mission.bypass.session);
    }
    gate_scenarios(&failures);
    if (g_frames) fclose(g_frames);
    printf("%s: %u failure(s)\n", failures ? "FAIL" : "PASS", failures);
    return failures ? 1 : 0;
}
