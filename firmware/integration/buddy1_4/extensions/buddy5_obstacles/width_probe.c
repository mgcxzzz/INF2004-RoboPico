#include "extensions/buddy5_obstacles/width_probe.h"
#ifdef APP_RTOS_BUDDY5
#include "app_program/obstacle_navigation/width_ref_retry.h"
#endif

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Shared with the direction edge probe v0.1.1 (values copied, logic re-used):
 * gate expected-25..expected+60 mm in v0.2.1 (v0.2 and the direction probe:
 * expected-15), three pings per stopped pose,
 * 30/20/30 mm/s turn/trim/drive speeds, 400/100 ms dwell, 400 ms ping timeout.
 * v0.2: REF-plane expected range, stopped turn ladder, symmetric return with
 * MID, explicit TRANSITION (see width_probe.h and CHANGES.md). */
#define WP_PI              3.14159265358979323846f
#define WP_DEG             (WP_PI / 180.0f)
#define WP_GATE_NEAR_MM    B5WP_NEAR_MM   /* v0.2.1: 25 (v0.2: 15) */
#define WP_GATE_FAR_MM     B5WP_FAR_MM
#define WP_TURN_SPEED      30u
#define WP_TRIM_SPEED      20u
#define WP_DRIVE_SPEED     30u
#define WP_FIRST_DWELL_US  UINT64_C(250000)
#define WP_DWELL_US        UINT64_C(50000)
#define WP_PING_TIMEOUT_US UINT64_C(400000)
#define WP_RUN_LIMIT_US    UINT64_C(180000000)
#define WP_MIN_TURN_DEG    1.0f   /* smaller heading changes: sample in place (trim may follow) */

#ifdef B5WP_HOST_TEST_HOOKS
/* HOST TESTS ONLY (never defined in the firmware build): model-visible heading error on
 * ladder/PRE/return turns, and a step scale to force the maximum-length sequence. */
float b5wp_test_turn_bias_deg = 0.0f;
float b5wp_test_step_scale = 1.0f;
#endif

static bool still(MotionStatus s) { return s.mode == MOTION_PAUSED && s.stopped; }

static bool lease_ok(const B5WidthProbe *t, MotionStatus s) {
    return s.session_active && s.owner == MOTION_BUDDY5 && s.lease == t->lease &&
           !s.fault && s.mode != MOTION_FAULT;
}
static int16_t sat_i16(long v) {
    return (int16_t)(v > INT16_MAX ? INT16_MAX : v < INT16_MIN ? INT16_MIN : v);
}
static uint16_t sat_u16(long v) {
    return (uint16_t)(v > UINT16_MAX ? UINT16_MAX : v < 0 ? 0 : v);
}
static int16_t mm10(float mm) { return sat_i16(lroundf(mm * 10.0f)); }
static int16_t cdeg(float deg) { return sat_i16(lroundf(deg * 100.0f)); }
static float side_sign(const B5WidthProbe *t) { return t->cur ? -1.0f : 1.0f; }

static int track_servo(float heading_deg) {
    long dd = lroundf(heading_deg * 10.0f);
    if (dd > B5WP_SERVO_LIMIT_DD) return B5WP_SERVO_LIMIT_DD;
    if (dd < -B5WP_SERVO_LIMIT_DD) return -B5WP_SERVO_LIMIT_DD;
    return (int)dd;
}
static bool sensor_available(const B5WidthProbe *t) {
    return t->sensor.angle && t->sensor.settled && t->sensor.request && t->sensor.take &&
           t->sensor.cancel;
}
static int applied_dd(const B5WidthProbe *t, int fallback) {
    return t->applied ? t->applied() : fallback;
}
static void set_servo(B5WidthProbe *t, int dd, uint64_t now) {
    if (dd > B5WP_SERVO_LIMIT_DD) dd = B5WP_SERVO_LIMIT_DD;
    if (dd < -B5WP_SERVO_LIMIT_DD) dd = -B5WP_SERVO_LIMIT_DD;
    if (dd != t->servo_dd) {
        t->sensor.angle(dd, now);
        t->servo_dd = dd;
    }
    int actual = applied_dd(t, dd);
    if (actual < -B5WP_SERVO_LIMIT_DD || actual > B5WP_SERVO_LIMIT_DD)
        t->servo_in_range = false;
}
static void stop_if_owned(B5WidthProbe *t) {
    MotionStatus s = nav_get_status();
    if (s.session_active && s.owner == MOTION_BUDDY5 && s.lease == t->lease &&
        !still(s) && s.mode != MOTION_FAULT)
        (void)nav_request_stop(MOTION_BUDDY5);
}

/* ---- pure model helpers ------------------------------------------------ */
float b5_wp_axis_x(float net_x_mm, float heading_deg, float applied_deg, float range_mm) {
    float shaft_x = net_x_mm - B5WP_SHAFT_MM * sinf(heading_deg * WP_DEG);
    float beam = (heading_deg - applied_deg) * WP_DEG;      /* + = left */
    return shaft_x - range_mm * tanf(beam);
}
/* Modelled face plane: y = shaft-at-REF (62) + d0 - FACE_MARGIN (25). The margin
 * propagates the widened near allowance: an echo accepted up to 25 mm short of
 * the model is treated as if the face could be 25 mm nearer. The sonar face ~3 mm
 * ahead of the shaft is still ignored (shortens the gap: conservative). Front
 * corner: pivot_y + FRONT*cos(h) + HALF_W*|sin(h)|. A model, never a physical
 * clearance guarantee. */
float b5_wp_front_gap(float d0_mm, float pivot_y_mm, float heading_deg) {
    float h = heading_deg * WP_DEG;
    return (B5WP_SHAFT_MM + d0_mm - B5WP_FACE_MARGIN_MM) -
           (pivot_y_mm + B5WP_CAR_FRONT_MM * cosf(h) + B5WP_CAR_HALF_W_MM * fabsf(sinf(h)));
}
/* Any heading during an in-place turn: worst case is the corner radius. */
float b5_wp_turn_front_gap(float d0_mm, float pivot_y_mm) {
    return (B5WP_SHAFT_MM + d0_mm - B5WP_FACE_MARGIN_MM) -
           (pivot_y_mm + sqrtf(B5WP_CAR_FRONT_MM * B5WP_CAR_FRONT_MM +
                               B5WP_CAR_HALF_W_MM * B5WP_CAR_HALF_W_MM));
}
/* Next outward command from the MEASURED cumulative outward travel. The
 * overshoot allowance is reserved so an accepted segment cannot carry the
 * cumulative measured travel past the budget. */
/* width_long_probe: outward travel (along the current leg) still allowed by the
 * clearance model from the integrated pose. After the travel, BOTH the worst in-place
 * turn corner radius (so a trim, turn or the retrace start at the new stop stays
 * inside the model) and the heading-dependent front corner must keep the 50 mm
 * front margin to the modelled face plane (shaft + d0 - 25). The forward projection
 * uses cos(|heading| - 5 deg) (heading reserve, never above 1). The 5 mm overshoot
 * reserve is applied by the caller. Model only, never a physical guarantee. */
float b5_wp_clear_travel(float d0_mm, float net_y_mm, float heading_deg) {
    float plane = B5WP_SHAFT_MM + d0_mm - B5WP_FACE_MARGIN_MM - B5WP_FRONT_MARGIN_MM;
    float h = heading_deg * WP_DEG;
    float turn_lim = plane - sqrtf(B5WP_CAR_FRONT_MM * B5WP_CAR_FRONT_MM +
                                   B5WP_CAR_HALF_W_MM * B5WP_CAR_HALF_W_MM);
    float front_lim = plane - (B5WP_CAR_FRONT_MM * cosf(h) + B5WP_CAR_HALF_W_MM * fabsf(sinf(h)));
    float lim = turn_lim < front_lim ? turn_lim : front_lim;
    float reserve = fabsf(heading_deg) - B5WP_TURN_RESERVE_DEG;
    float c = cosf((reserve > 0.0f ? reserve : 0.0f) * WP_DEG);
    return (lim - net_y_mm) / c;
}
float b5_wp_next_step(float out_mm, unsigned steps) {
    if (steps >= B5WP_MAX_STEPS) return 0.0f;
    float cmd = B5WP_BUDGET_MM - out_mm - B5WP_OVERSHOOT_MM;
    if (cmd > B5WP_STEP_MM) cmd = B5WP_STEP_MM;
    return cmd >= B5WP_MIN_STEP_MM ? cmd : 0.0f;
}
/* Segment acceptance: same sign, at least half the command, at most the
 * command plus the overshoot allowance (encoder progress at standstill). */
bool b5_wp_segment_ok(float command_mm, float measured_mm) {
    float c = fabsf(command_mm), m = fabsf(measured_mm);
    if (c < 1.0f) return fabsf(measured_mm) <= B5WP_OVERSHOOT_MM;
    if ((command_mm > 0.0f) != (measured_mm > 0.0f)) return false;
    return m >= 0.5f * c && m <= c + B5WP_OVERSHOOT_MM;
}
/* Fixed frontal REF plane carried by the continuous encoder pose. Uses the
 * quantized wire values so the decoder reproduces it exactly. */
uint16_t b5_wp_expected_mm(uint16_t d0, int16_t heading_cdeg, int16_t net_y_mm10) {
    float h = ((float)heading_cdeg / 100.0f) * WP_DEG;
    float y = (float)net_y_mm10 / 10.0f;
    return sat_u16(lroundf((float)d0 + B5WP_SHAFT_MM * (1.0f - cosf(h)) - y));
}
/* Per ping: clean echo inside the gate = IN; clean echo beyond it or clean
 * no-return = FAR; a clean echo below the gate (too near) or any fault,
 * timeout or invalid ping = BAD. v0.2.1: near = 25 mm (v0.2: 15). Triplet: any BAD -> UNKNOWN (the run stops);
 * all IN -> IN; all FAR -> FAR; otherwise only clean IN and clean FAR are
 * mixed -> TRANSITION. Unchanged gate; no resampling. */
uint8_t b5_wp_classify(const b5_ping_t p[3], uint16_t expected) {
    unsigned upper = (unsigned)expected + WP_GATE_FAR_MM;
    long lower = (long)expected - WP_GATE_NEAR_MM;
    unsigned in = 0, far = 0;
    for (unsigned i = 0; i < 3; ++i) {
        bool ok = p[i].why == B5_PING_WHY_OK;
        if (ok && p[i].cls == B5_PING_ECHO) {
            if ((long)p[i].range_mm < lower) return B5WP_UNKNOWN;          /* too near */
            if (p[i].range_mm <= upper) ++in; else ++far;
        } else if (ok && p[i].cls == B5_PING_NO_RETURN) {
            ++far;
        } else {
            return B5WP_UNKNOWN;                                         /* fault/invalid */
        }
    }
    return in == 3u ? B5WP_IN : far == 3u ? B5WP_FAR : B5WP_TRANSITION;
}

/* ---- evidence ------------------------------------------------------------ */
static uint16_t median_three(const b5_ping_t p[3]) {
    uint16_t v[3] = {(uint16_t)p[0].range_mm, (uint16_t)p[1].range_mm, (uint16_t)p[2].range_mm};
    for (unsigned i = 1; i < 3; ++i)
        for (unsigned j = i; j && v[j - 1] > v[j]; --j) {
            uint16_t tmp = v[j]; v[j] = v[j - 1]; v[j - 1] = tmp;
        }
    return v[1];
}
static bool valid_reference(const b5_ping_t p[3], uint16_t *median) {
    uint16_t lo = UINT16_MAX, hi = 0;
    for (unsigned i = 0; i < 3; ++i) {
        if (p[i].cls != B5_PING_ECHO || p[i].why != B5_PING_WHY_OK ||
            p[i].range_mm < B5WP_D0_MIN_MM || p[i].range_mm > B5WP_D0_MAX_MM)
            return false;
        if (p[i].range_mm < lo) lo = (uint16_t)p[i].range_mm;
        if (p[i].range_mm > hi) hi = (uint16_t)p[i].range_mm;
    }
    if ((unsigned)(hi - lo) > 15u) return false;
    *median = median_three(p);
    return true;
}
/* Nominal heading of the pose being sampled (trim and alignment target). */
static float sample_target(const B5WidthProbe *t) {
    switch (t->sample_phase) {
    case B5WP_PH_REF: return t->heading_deg;
    case B5WP_PH_FINAL: case B5WP_PH_MID: return 0.0f;
    case B5WP_PH_LAD_OUT: case B5WP_PH_LAD_RET:
        return side_sign(t) * B5WP_RUNG_DEG * (float)t->sample_step;
    default: return side_sign(t) * B5WP_PRE_DEG;
    }
}
static void note_footprint(B5WidthProbe *t, float gap) {
    if (gap < t->min_front_gap) t->min_front_gap = gap;
}
static bool side_phase(uint8_t phase) {
    return phase == B5WP_PH_PRE || phase == B5WP_PH_OUT || phase == B5WP_PH_RET ||
           phase == B5WP_PH_LAD_OUT || phase == B5WP_PH_LAD_RET;
}

/* Build, classify and queue one stopped-sample record. Returns the verdict. */
static uint8_t push_record(B5WidthProbe *t, uint64_t now) {
    B5WPRec r = {0};
    B5WPSide *sd = &t->side[t->cur];
    r.ms = t->sample_ms ? t->sample_ms : (uint32_t)(now / 1000u);
    r.run = t->run;
    r.index = t->rec_index;
    r.phase = t->sample_phase;
    r.side = side_phase(r.phase) ? (uint8_t)(t->cur ? B5WP_SIDE_R : B5WP_SIDE_L) : B5WP_SIDE_NONE;
    r.step = t->sample_step;
    r.n = 3;
    r.heading_cdeg = t->sample_heading_cdeg;
    r.servo_dd = t->sample_servo_dd;
    r.applied_dd = t->sample_applied_dd;
    r.path_mm10 = r.side ? mm10(sd->path_mm) : 0;
    r.seg_cmd_mm10 = mm10(t->seg_cmd_mm);
    r.seg_meas_mm10 = mm10(t->seg_meas_mm);
    r.net_x_mm10 = mm10(t->net_x_mm);
    r.net_y_mm10 = mm10(t->net_y_mm);
    r.correction_cdeg = t->correction_total_cdeg;
    /* Plane provenance: every record refers to the REF record (index 0, d0). */
    r.anchor_mm = t->d0;
    r.anchor_index = 0;
    if (r.phase == B5WP_PH_REF) {
        r.expected_mm = t->d0;
        r.evidence = t->ref_ok ? B5WP_IN : B5WP_UNKNOWN;
    } else {
        r.expected_mm = b5_wp_expected_mm(t->d0, r.heading_cdeg, r.net_y_mm10);
        r.evidence = b5_wp_classify(t->ping, r.expected_mm);
    }
    /* Axis position from quantized wire values, so the decoder can reproduce it. */
    float axis = b5_wp_axis_x((float)r.net_x_mm10 / 10.0f, (float)r.heading_cdeg / 100.0f,
                              (float)r.applied_dd / 10.0f, (float)r.expected_mm);
    r.axis_x_mm10 = mm10(axis);
    t->last_axis_x = (float)r.axis_x_mm10 / 10.0f;
    float gap = b5_wp_front_gap((float)t->d0, t->net_y_mm, t->heading_deg);
    r.front_gap_mm10 = mm10(gap);
    if (r.phase != B5WP_PH_REF) note_footprint(t, gap);

    for (unsigned i = 0; i < 3; ++i) {
        r.cls[i] = (uint8_t)((t->ping[i].cls & 3u) | ((t->ping[i].why & 7u) << 4));
        r.value[i] = t->ping[i].cls == B5_PING_ECHO ? (uint16_t)t->ping[i].range_mm :
            (uint16_t)(t->ping[i].width_us > UINT16_MAX ? UINT16_MAX : t->ping[i].width_us);
    }
    if (abs(r.servo_dd) > B5WP_SERVO_LIMIT_DD || abs(r.applied_dd) > B5WP_SERVO_LIMIT_DD)
        t->servo_in_range = false;
    if (t->est_in.n < B5WE_MAX_POINTS) {          /* display estimator input (no feedback) */
        bool centre = r.phase == B5WP_PH_REF || r.phase == B5WP_PH_MID || r.phase == B5WP_PH_FINAL;
        /* Actual adjacent centre observation, never an assumed edge at zero.
         * The estimator may use it only after this side's boundary corroborates. */
        uint8_t est_side = r.side == B5WP_SIDE_L ? B5WE_PSIDE_L :
                           r.side == B5WP_SIDE_R ? B5WE_PSIDE_R : B5WE_PSIDE_NONE;
        if (r.phase == B5WP_PH_REF) est_side = B5WE_PSIDE_L;
        else if (r.phase == B5WP_PH_MID) est_side = B5WE_PSIDE_R;
        t->est_in.pt[t->est_in.n++] = b5_we_point(
            est_side, centre, r.cls, r.value, r.expected_mm, r.axis_x_mm10);
    }
#if B5WP_RUN_ARCHIVE
    if (t->arch_n < B5WP_MAX_RECORDS) t->arch[t->arch_n++] = r;   /* v0.1.1 run log */
    else if (t->arch_lost != UINT16_MAX) ++t->arch_lost;
#endif
    if (t->rec_count >= B5WP_QUEUE_CAP) {
        if (t->lost != UINT16_MAX) ++t->lost;
    } else {
        uint16_t tail = (uint16_t)((t->rec_head + t->rec_count) % B5WP_QUEUE_CAP);
        t->rec[tail] = r;
        ++t->rec_count;
    }
    if (t->rec_index != UINT16_MAX) ++t->rec_index;
    t->seg_cmd_mm = t->seg_meas_mm = 0.0f;
    return r.evidence;
}

/* Union of the outgoing and return brackets (conservative). Left edges are
 * negative x: IN bound = the less negative IN, FAR bound = the FAR sample.
 * Only corroborated BOUNDARY sides on a COMPLETE run with valid REF, MID and
 * FINAL produce a number. */
static void compute_width(B5WidthProbe *t) {
    const B5WPSide *l = &t->side[0], *r = &t->side[1];
    t->width_result = B5WP_WIDTH_INCOMPLETE;
    t->width_lo = t->width_hi = 0.0f;
    if (t->end != B5WP_END_COMPLETE || t->lost || t->rec_index > B5WP_MAX_RECORDS ||
        !t->ref_ok || !t->mid_ok || !t->final_ref_ok || !t->servo_in_range ||
        l->state != B5WP_ST_BOUNDARY || r->state != B5WP_ST_BOUNDARY ||
        !l->return_ok || !r->return_ok || !l->ret_in_ok || !r->ret_in_ok)
        return;
    float l_in = fmaxf(l->in_x, l->ret_in_x), l_far = l->far_x;
    float r_in = fminf(r->in_x, r->ret_in_x), r_far = r->far_x;
    if (!(l_far < l_in && r_far > r_in && r_in > l_in)) return; /* inconsistent geometry */
    t->width_lo = r_in - l_in;
    t->width_hi = r_far - l_far;
    t->width_result = B5WP_WIDTH_MEASURED;
}

static void finish(B5WidthProbe *t, unsigned end, uint64_t now) {
    if (!b5_wp_ready(t)) stop_if_owned(t);
    /* A side whose outward pass had not ended when the run ended was aborted. */
    if (end != B5WP_END_COMPLETE && t->state != B5WP_CLAIM && t->d0 &&
        t->side[t->cur].limit == B5WP_LIM_NONE)
        t->side[t->cur].limit = B5WP_LIM_ABORT;
    t->limits_pending = true;
    if (sensor_available(t)) t->sensor.cancel();
    t->ping_active = false;
    t->command_active = false;
    t->end = end;
    t->state = end == B5WP_END_COMPLETE ? B5WP_DONE : B5WP_HOLD;
    compute_width(t);
    const B5WPSide *l = &t->side[0], *r = &t->side[1];
    bool measured = t->width_result == B5WP_WIDTH_MEASURED;
    uint16_t flags = 0;
    if (t->ref_ok) flags |= B5WP_FLAG_REF_OK;
    if (t->final_ref_ok) flags |= B5WP_FLAG_FINAL_REF_OK;
    if (l->return_ok) flags |= B5WP_FLAG_L_RETURN_OK;
    if (r->return_ok) flags |= B5WP_FLAG_R_RETURN_OK;
    if (t->servo_in_range) flags |= B5WP_FLAG_SERVO_IN_RANGE;
    if (l->cap_budget) flags |= B5WP_FLAG_L_CAP_BUDGET;
    if (r->cap_budget) flags |= B5WP_FLAG_R_CAP_BUDGET;
    if (l->cap_clear) flags |= B5WP_FLAG_L_CAP_CLEAR;
    if (r->cap_clear) flags |= B5WP_FLAG_R_CAP_CLEAR;
    if (l->cap_steps) flags |= B5WP_FLAG_L_CAP_STEPS;
    if (r->cap_steps) flags |= B5WP_FLAG_R_CAP_STEPS;
    if (l->far_stop >= 0 && l->far_stop <= (int)B5WP_STOP_PRE) flags |= B5WP_FLAG_L_FAR_NO_STEP;
    if (r->far_stop >= 0 && r->far_stop <= (int)B5WP_STOP_PRE) flags |= B5WP_FLAG_R_FAR_NO_STEP;
    if (l->transition) flags |= B5WP_FLAG_L_TRANSITION;
    if (r->transition) flags |= B5WP_FLAG_R_TRANSITION;
    if (t->mid_ok) flags |= B5WP_FLAG_MID_OK;
    t->summary = (B5WPSummary){
        .ms = (uint32_t)(now / 1000u), .run = t->run, .end = (uint8_t)end,
        .width_result = t->width_result, .state_l = l->state, .state_r = r->state,
        .steps_l = (uint8_t)l->steps, .steps_r = (uint8_t)r->steps,
        .records = t->rec_index, .lost = t->lost, .d0 = t->d0,
        .out_l_mm10 = mm10(l->out_mm), .out_r_mm10 = mm10(r->out_mm),
        .res_l_mm10 = mm10(l->path_mm), .res_r_mm10 = mm10(r->path_mm),
        .turn_l_cdeg = cdeg(l->turn_deg), .turn_r_cdeg = cdeg(r->turn_deg),
        .final_heading_cdeg = cdeg(t->heading_deg),
        .net_x_mm10 = mm10(t->net_x_mm), .net_y_mm10 = mm10(t->net_y_mm), .flags = flags,
        .correction_count = (uint16_t)t->correction_count,
        .correction_total_cdeg = t->correction_total_cdeg,
        .l_in_mm10 = measured ? mm10(fmaxf(l->in_x, l->ret_in_x)) : 0,
        .l_far_mm10 = measured ? mm10(l->far_x) : 0,
        .r_in_mm10 = measured ? mm10(fminf(r->in_x, r->ret_in_x)) : 0,
        .r_far_mm10 = measured ? mm10(r->far_x) : 0,
        .width_lo_mm10 = measured ? sat_u16(lroundf(t->width_lo * 10.0f)) : 0,
        .width_hi_mm10 = measured ? sat_u16(lroundf(t->width_hi * 10.0f)) : 0,
        .min_front_gap_mm10 = mm10(t->min_front_gap)};
    t->summary_pending = true;

    /* Display estimate for every attempted run. Computed AFTER the end code, state and
     * summary are fixed; nothing reads it back for motion or decisions. */
    B5WEInput *in = &t->est_in;
    in->end = (uint8_t)end;
    in->records = t->rec_index;
    in->consistent = !t->lost && t->rec_index <= B5WP_MAX_RECORDS && t->servo_in_range;
    in->mid_in = t->mid_ok;
    in->final_in = t->final_ref_ok;
    for (unsigned k = 0; k < 2u; ++k) {
        const B5WPSide *sd = &t->side[k];
        in->side[k].not_reached = sd->state == B5WP_ST_NOT_REACHED;
        in->side[k].last_out_in = sd->stop[sd->last].visited && sd->stop[sd->last].out_cls == B5WP_IN;
        in->side[k].return_ok = sd->return_ok;
        in->side[k].cap_budget = sd->cap_budget;
        in->side[k].boundary_ok = sd->state == B5WP_ST_BOUNDARY &&
            sd->return_ok && sd->ret_in_ok && !sd->contradictory;
    }
    b5_we_estimate(in, &t->result);
    t->result_ms = (uint32_t)(now / 1000u);
    t->result_pending = true;
}

static void begin_pose(B5WidthProbe *t, unsigned state, uint8_t phase, uint8_t step,
                       uint64_t now) {
    t->state = state;
    t->sub = 0;
    t->sample_phase = phase;
    t->sample_step = step;
    t->npings = 0;
    t->ping_active = false;
    t->pose_timing = false;
    t->pose_trimmed = false;
    t->pose_retried = false;
    t->sample_since = now;
    t->sample_ms = 0;
    set_servo(t, track_servo(t->heading_deg), now);
}
static void begin_move_common(B5WidthProbe *t, unsigned state, uint64_t now) {
    t->state = state;
    t->command_active = true;
    t->move_start_heading = t->heading_deg;
    t->since = now;
}
static void begin_turn(B5WidthProbe *t, unsigned state, float degrees, uint64_t now) {
    /* Inspected footprint check before any turn: worst-case corner radius. */
    if (b5_wp_turn_front_gap((float)t->d0, t->net_y_mm) < B5WP_FRONT_MARGIN_MM) {
        finish(t, B5WP_END_CLEARANCE, now);
        return;
    }
    if (nav_turn_relative(MOTION_BUDDY5, t->lease, degrees, WP_TURN_SPEED, MOTION_IN_PLACE,
                          &t->command) != MOTION_OK) {
        finish(t, B5WP_END_MOVE, now);
        return;
    }
    begin_move_common(t, state, now);
}
/* Turn in place to an absolute model heading, then sample the given pose.
 * A change below the API's useful size samples in place (the bounded trim
 * still applies). Accumulated odometry is never reset. */
static void turn_to(B5WidthProbe *t, unsigned turn_state, float target_deg,
                    unsigned pose_state, uint8_t phase, uint8_t step, uint64_t now) {
    t->after_state = pose_state;
    t->after_phase = phase;
    t->after_step = step;
    float delta = target_deg - t->heading_deg;
#ifdef B5WP_HOST_TEST_HOOKS
    if (fabsf(delta) >= WP_MIN_TURN_DEG) delta += b5wp_test_turn_bias_deg;
#endif
    if (fabsf(delta) < WP_MIN_TURN_DEG) {
        begin_pose(t, pose_state, phase, step, now);
        return;
    }
    begin_turn(t, turn_state, delta, now);
}
static void begin_drive(B5WidthProbe *t, unsigned state, float mm, uint64_t now) {
    t->seg_cmd_mm = mm;
    t->seg_meas_mm = 0.0f;
    if (nav_drive_distance(MOTION_BUDDY5, t->lease, mm, WP_DRIVE_SPEED, &t->command) !=
        MOTION_OK) {
        finish(t, B5WP_END_MOVE, now);
        return;
    }
    begin_move_common(t, state, now);
}
static bool begin_trim(B5WidthProbe *t, uint64_t now) {
    float delta = sample_target(t) - t->heading_deg;
    if (fabsf(delta) <= 2.0f) {
        t->pose_trimmed = true;
        return false;
    }
    if (fabsf(delta) > 12.0f || t->correction_count >= B5WP_MAX_TRIMS) {
        finish(t, B5WP_END_ALIGNMENT, now);
        return true;
    }
    if (b5_wp_turn_front_gap((float)t->d0, t->net_y_mm) < B5WP_FRONT_MARGIN_MM) {
        finish(t, B5WP_END_CLEARANCE, now);
        return true;
    }
    if (nav_turn_relative(MOTION_BUDDY5, t->lease, delta, WP_TRIM_SPEED, MOTION_IN_PLACE,
                          &t->command) != MOTION_OK) {
        finish(t, B5WP_END_MOVE, now);
        return true;
    }
    t->trim_return_state = t->state;
    t->trim_target_deg = sample_target(t);
    begin_move_common(t, B5WP_TRIM, now);
    return true;
}
/* Continuous encoder pose: every completed move (turn, trim or drive) adds its
 * reported centre progress along the measured midpoint heading. An ideal
 * in-place turn has zero centre progress. */
static void integrate(B5WidthProbe *t, const MotionCommandStatus *c) {
    float mid = (t->move_start_heading + c->rotation_deg * 0.5f) * WP_DEG;
    t->net_x_mm -= c->progress_mm * sinf(mid);  /* x right, y forward, + rotation = left */
    t->net_y_mm += c->progress_mm * cosf(mid);
    t->heading_deg = t->move_start_heading + c->rotation_deg;
}

static void begin_return(B5WidthProbe *t, uint64_t now);

/* ---- side sequence --------------------------------------------------------
 * Stops per side: 0 frontal (REF for L, MID for R), 1 = +-15, 2 = +-30,
 * 3 = PRE +-45, 4.. = forward steps. Outward: 1, 2, 3, steps, until the first
 * all-FAR or a cap. Return: only stops actually visited, in reverse order,
 * ending at heading 0 (MID after L, FINAL after R). */
/* width_long_probe: the next outward command is the smallest of the nominal step, the
 * ceiling remainder and the clearance remainder (both less the overshoot reserve). Below
 * the minimum step the pass ends on whichever of ceiling or clearance is the tighter
 * (a tie counts as the ceiling); the step bound is checked first. The original
 * end-of-segment front-corner inspection is kept as a second guard. */
static void plan_step(B5WidthProbe *t, uint64_t now) {
    B5WPSide *sd = &t->side[t->cur];
    if (sd->steps >= B5WP_MAX_STEPS) {
        sd->cap_steps = true;
        sd->limit = B5WP_LIM_STEPS;
        begin_return(t, now);
        return;
    }
    float clear = b5_wp_clear_travel((float)t->d0, t->net_y_mm, t->heading_deg);
    sd->clear_total_mm = sd->out_mm + clear;
    float rem_ceiling = B5WP_BUDGET_MM - sd->out_mm - B5WP_OVERSHOOT_MM;
    float rem_clear = clear - B5WP_OVERSHOOT_MM;
    float cmd = B5WP_STEP_MM;
    if (rem_ceiling < cmd) cmd = rem_ceiling;
    if (rem_clear < cmd) cmd = rem_clear;
#ifdef B5WP_HOST_TEST_HOOKS
    if (cmd >= B5WP_MIN_STEP_MM && b5wp_test_step_scale < 1.0f) {
        cmd *= b5wp_test_step_scale;
        if (cmd < B5WP_MIN_STEP_MM) cmd = B5WP_MIN_STEP_MM;
    }
#endif
    if (cmd < B5WP_MIN_STEP_MM) {
        if (rem_ceiling <= rem_clear) {
            sd->cap_budget = true;          /* travel ceiling: completion-eligible as before */
            sd->limit = B5WP_LIM_CEILING;
        } else {
            sd->cap_clear = true;
            sd->limit = B5WP_LIM_CLEARANCE;
        }
        begin_return(t, now);
        return;
    }
    /* Inspected footprint at the end of the worst accepted segment (unchanged guard). */
    float reach = cmd + B5WP_OVERSHOOT_MM;
    float y_end = t->net_y_mm + reach * cosf(t->heading_deg * WP_DEG);
    if (b5_wp_front_gap((float)t->d0, y_end, t->heading_deg) < B5WP_FRONT_MARGIN_MM) {
        sd->cap_clear = true;
        sd->limit = B5WP_LIM_CLEARANCE;
        begin_return(t, now);
        return;
    }
    begin_drive(t, B5WP_STEP, cmd, now);
}
static void outward_to(B5WidthProbe *t, unsigned k, uint64_t now) {
    float s = side_sign(t);
    if (k < B5WP_STOP_PRE)
        turn_to(t, B5WP_TURN, s * B5WP_RUNG_DEG * (float)k, B5WP_LADDER, B5WP_PH_LAD_OUT,
                (uint8_t)k, now);
    else if (k == B5WP_STOP_PRE)
        turn_to(t, B5WP_TURN, s * B5WP_PRE_DEG, B5WP_PRE, B5WP_PH_PRE, 0, now);
    else
        plan_step(t, now);
}
static void start_side(B5WidthProbe *t, unsigned cur, uint8_t frontal, float axis,
                       uint64_t now) {
    t->cur = cur;
    B5WPSide *sd = &t->side[cur];
    sd->stop[0] = (B5WPStop){.out_cls = frontal, .visited = true, .out_axis = axis};
    sd->last = 0;
    if (frontal == B5WP_IN) sd->last_in = 0;
    else if (frontal == B5WP_TRANSITION) sd->transition = true;
    outward_to(t, 1, now);
}
/* Retrace along the PRE leg to forward stop ret_k (path coordinate recorded
 * outward) and sample there. */
static void retrace_to_stop(B5WidthProbe *t, uint64_t now) {
    B5WPSide *sd = &t->side[t->cur];
    float cmd = sd->stop[sd->ret_k].path_mm - sd->path_mm;
    uint8_t step = (uint8_t)(sd->ret_k - B5WP_STOP_PRE);
    if (cmd > -1.0f) {              /* below the API minimum: sample in place */
        t->seg_cmd_mm = cmd;
        t->seg_meas_mm = 0.0f;
        begin_pose(t, B5WP_RET, B5WP_PH_RET, step, now);
        return;
    }
    begin_drive(t, B5WP_RETRACE, cmd, now);
}
static void revisit_next(B5WidthProbe *t, uint64_t now) {
    B5WPSide *sd = &t->side[t->cur];
    if (sd->ret_k == 0u) { finish(t, B5WP_END_MOVE, now); return; }   /* unreachable */
    unsigned j = --sd->ret_k;
    float s = side_sign(t);
    if (j >= B5WP_STOP_PRE) retrace_to_stop(t, now);
    else if (j > 0u)
        turn_to(t, B5WP_TURN, s * B5WP_RUNG_DEG * (float)j, B5WP_LADDER, B5WP_PH_LAD_RET,
                (uint8_t)j, now);
    else if (t->cur == 0u)          /* MID: target heading 0; REF and odometry kept */
        turn_to(t, B5WP_TURN, 0.0f, B5WP_MID, B5WP_PH_MID, 0, now);
    else
        turn_to(t, B5WP_TURN_BACK, 0.0f, B5WP_FINAL, B5WP_PH_FINAL, 0, now);
}
static void begin_return(B5WidthProbe *t, uint64_t now) {
    B5WPSide *sd = &t->side[t->cur];
    sd->return_ok = sd->steps == 0u;   /* forward leg checked again at the PRE revisit */
    sd->ret_k = sd->last;
    revisit_next(t, now);
}
static void conclude_side(B5WidthProbe *t, uint8_t frontal, float axis, uint64_t now) {
    B5WPSide *sd = &t->side[t->cur];
    sd->state = sd->contradictory ? B5WP_ST_CONTRADICTORY :
                sd->boundary ? (sd->ret_in_ok ? B5WP_ST_BOUNDARY : B5WP_ST_UNCORROBORATED) :
                sd->far_stop >= 0 ? B5WP_ST_NO_INNER_IN : B5WP_ST_NOT_REACHED;
    if (t->cur == 0u) start_side(t, 1u, frontal, axis, now);
    else finish(t, B5WP_END_COMPLETE, now);
}
static void outward_sample(B5WidthProbe *t, unsigned k, uint8_t v, float axis, uint64_t now) {
    B5WPSide *sd = &t->side[t->cur];
    B5WPStop *st = &sd->stop[k];
    st->visited = true;
    st->out_cls = v;
    st->out_axis = axis;
    st->path_mm = sd->path_mm;
    sd->last = k;
    if (k <= B5WP_STOP_PRE) sd->turn_deg = t->heading_deg;
    float turn_gap = b5_wp_turn_front_gap((float)t->d0, t->net_y_mm);
    if (turn_gap < sd->min_turn_gap) sd->min_turn_gap = turn_gap;
    if (v == B5WP_FAR) {            /* first all-FAR: stop outward here */
        sd->limit = B5WP_LIM_FAR;
        sd->far_stop = (int)k;
        sd->far_x = axis;
        if (sd->last_in >= 0) {
            sd->boundary = true;
            sd->in_stop = sd->last_in;
            sd->in_x = sd->stop[sd->last_in].out_axis;
        }
        begin_return(t, now);
        return;
    }
    if (v == B5WP_IN) sd->last_in = (int)k;
    else sd->transition = true;    /* TRANSITION: neither IN nor FAR; keep going */
    outward_to(t, k + 1u, now);
}
static void return_sample(B5WidthProbe *t, uint8_t v, float axis, uint64_t now) {
    B5WPSide *sd = &t->side[t->cur];
    unsigned j = sd->ret_k;
    B5WPStop *st = &sd->stop[j];
    st->revisited = true;
    st->ret_cls = v;
    st->ret_axis = axis;
    if (st->out_cls == B5WP_IN && v == B5WP_FAR) sd->contradictory = true;
    if ((int)j == sd->in_stop) {    /* only the IN-bound stop can corroborate */
        sd->ret_in_ok = v == B5WP_IN;
        sd->ret_in_x = axis;
    }
    if (j == B5WP_STOP_PRE && sd->steps > 0u) {
        sd->return_ok = fabsf(sd->path_mm) <= B5WP_RETURN_RESIDUAL_MM;
        if (!sd->return_ok) {       /* same rule as the direction probe: hold */
            finish(t, B5WP_END_TRAVEL, now);
            return;
        }
    }
    if (j == 0u) {
        if (t->cur == 0u) t->mid_ok = v == B5WP_IN;
        else t->final_ref_ok = v == B5WP_IN;
        conclude_side(t, v, axis, now);
        return;
    }
    revisit_next(t, now);
}

static bool finish_move(B5WidthProbe *t, MotionStatus s, uint64_t now) {
    MotionCommandStatus c;
    if (!nav_get_command(t->command, &c)) {
        finish(t, B5WP_END_MOVE, now);
        return true;
    }
    bool trimming = t->state == B5WP_TRIM;
    bool turning = trimming || t->state == B5WP_TURN || t->state == B5WP_TURN_BACK;
    if (turning && (c.state == MOTION_CMD_RUNNING || !still(s)))
        set_servo(t, track_servo(t->move_start_heading + c.rotation_deg), now);
    if (c.state == MOTION_CMD_RUNNING) return true;
    if (c.state != MOTION_CMD_DONE) {
        finish(t, B5WP_END_MOVE, now);
        return true;
    }
    if (!still(s)) return true;     /* progress keeps updating until standstill */
    t->command_active = false;
    integrate(t, &c);
    B5WPSide *sd = &t->side[t->cur];

    switch (t->state) {
    case B5WP_TRIM:
        if (fabsf(c.rotation_deg) > 15.0f || fabsf(t->heading_deg - t->trim_target_deg) > 3.0f) {
            finish(t, B5WP_END_ALIGNMENT, now);
            return true;
        }
        ++t->correction_count;
        t->correction_total_cdeg = cdeg((float)t->correction_total_cdeg / 100.0f + c.rotation_deg);
        t->state = t->trim_return_state;
        t->pose_trimmed = true;
        t->pose_timing = false;
        t->sub = 0;
        t->sample_since = now;
        set_servo(t, track_servo(t->heading_deg), now);
        return true;
    case B5WP_TURN_BACK:
        t->turn_back_deg = c.rotation_deg;
        begin_pose(t, t->after_state, t->after_phase, t->after_step, now);
        return true;
    case B5WP_TURN:
        begin_pose(t, t->after_state, t->after_phase, t->after_step, now);
        return true;
    case B5WP_STEP:
        t->seg_meas_mm = c.progress_mm;
        if (!b5_wp_segment_ok(t->seg_cmd_mm, c.progress_mm)) {
            finish(t, B5WP_END_TRAVEL, now);   /* overshoot/undershoot: hold, no retrace */
            return true;
        }
        sd->path_mm += c.progress_mm;
        sd->out_mm += c.progress_mm;
        ++sd->steps;
        begin_pose(t, B5WP_OUT, B5WP_PH_OUT, (uint8_t)sd->steps, now);
        return true;
    case B5WP_RETRACE:
        t->seg_meas_mm = c.progress_mm;
        if (!b5_wp_segment_ok(t->seg_cmd_mm, c.progress_mm)) {
            finish(t, B5WP_END_TRAVEL, now);
            return true;
        }
        sd->path_mm += c.progress_mm;
        begin_pose(t, B5WP_RET, B5WP_PH_RET, (uint8_t)(sd->ret_k - B5WP_STOP_PRE), now);
        return true;
    default:
        finish(t, B5WP_END_MOVE, now);
        return true;
    }
}

static bool collect_triplet(B5WidthProbe *t, uint64_t now) {
    if (t->npings >= 3u) return true;
    if (!t->ping_active) {
        if (!t->sensor.request(now)) return false;
        t->ping_active = true;
        t->ping_since = now;
        if (t->npings == 0u) {
            t->sample_ms = (uint32_t)(now / 1000u);
            t->sample_heading_cdeg = cdeg(t->heading_deg);
            t->sample_servo_dd = (int16_t)t->servo_dd;
            t->sample_applied_dd = (int16_t)applied_dd(t, t->servo_dd);
        }
        return false;
    }
    b5_ping_t p;
    if (t->sensor.take(&p)) {
        t->ping[t->npings++] = p;
        t->ping_active = false;
        t->ping_since = 0;
    } else if (now - t->ping_since >= WP_PING_TIMEOUT_US) {
        t->sensor.cancel();
        t->ping[t->npings++] = (b5_ping_t){.cls = B5_PING_FAULT, .why = 7u};
        t->ping_active = false;
        t->ping_since = 0;
    }
    return t->npings >= 3u;
}

/* A retry preserves the stopped pose and the original acceptance gates.
 * Retain rejected triplets separately so they cannot become edge evidence. */
static bool retry_sample(B5WidthProbe *t, uint16_t expected, uint64_t now) {
    if (t->pose_retried || t->retry_count >= B5WP_RETRIES_PER_RUN) return false;
    bool reference = t->sample_phase == B5WP_PH_REF;
#ifdef APP_RTOS_BUDDY5
    bool retry_outlier = reference && width_ref_retry_outlier(t->ping);
#endif
    B5WPSide *sd = &t->side[t->cur];
    bool returning = t->sample_phase == B5WP_PH_RET || t->sample_phase == B5WP_PH_LAD_RET;
    bool boundary_transition = returning && sd->boundary && sd->in_stop >= 0 &&
        sd->ret_k == (unsigned)sd->in_stop &&
        b5_wp_classify(t->ping, expected) == B5WP_TRANSITION;
    bool short_fault = false, clean_in_window = true;
    if (!boundary_transition) {
        for (unsigned i = 0; i < 3u; ++i) {
            b5_ping_t p = t->ping[i];
            if (p.cls == B5_PING_FAULT && p.why == B5_PING_WHY_SHORT) {
                short_fault = true;
                clean_in_window = false;
            } else if (p.cls != B5_PING_ECHO || p.why != B5_PING_WHY_OK) {
                return false; /* faults and arbitrary mixed observations are not edge evidence */
            } else if (reference) {
#ifdef APP_RTOS_BUDDY5
                if ((!retry_outlier) &&
                    ((p.range_mm < B5WP_D0_MIN_MM) ||
                     (p.range_mm > B5WP_D0_MAX_MM)))
                {
                    return false;
                }
#else
                if (p.range_mm < B5WP_D0_MIN_MM || p.range_mm > B5WP_D0_MAX_MM) return false;
#endif
            } else if ((long)p.range_mm < (long)expected - WP_GATE_NEAR_MM) {
                return false;
            }
        }
        if (!short_fault && !(reference && clean_in_window)) return false;
    }

    B5WPRetry *r = &t->retries[t->retry_count++];
    *r = (B5WPRetry){.ms = t->sample_ms, .run = t->run,
        .expected_mm = expected, .heading_cdeg = t->sample_heading_cdeg,
        .applied_dd = t->sample_applied_dd, .phase = t->sample_phase,
        .side = reference ? B5WP_SIDE_NONE : (t->cur ? B5WP_SIDE_R : B5WP_SIDE_L),
        .step = t->sample_step, .attempt = 1u};
    for (unsigned i = 0; i < 3u; ++i) {
        b5_ping_t p = t->ping[i];
        r->cls[i] = (uint8_t)((p.cls & 3u) | ((p.why & 7u) << 4));
        r->value[i] = p.cls == B5_PING_ECHO ? (uint16_t)p.range_mm :
            (uint16_t)(p.width_us > UINT16_MAX ? UINT16_MAX : p.width_us);
    }
    t->sensor.cancel();
    t->npings = 0;
    t->ping_active = false;
    t->ping_since = 0;
    t->sample_ms = 0;
    t->pose_timing = false;
    t->sample_since = now;
    t->sub = 0;
    t->pose_retried = true;
    return true;
}

static void sample_done(B5WidthProbe *t, uint64_t now) {
    uint8_t phase = t->sample_phase;
    uint16_t expected = b5_wp_expected_mm(t->d0, t->sample_heading_cdeg, mm10(t->net_y_mm));
    uint16_t reference = 0;
    uint8_t candidate = phase == B5WP_PH_REF ? B5WP_UNKNOWN : b5_wp_classify(t->ping, expected);
    bool rejected = phase == B5WP_PH_REF ? !valid_reference(t->ping, &reference) :
                    candidate == B5WP_UNKNOWN;
    if ((rejected || candidate == B5WP_TRANSITION) &&
        retry_sample(t, phase == B5WP_PH_REF ? 0u : expected, now)) return;
    if (phase == B5WP_PH_REF) {
        uint16_t d0 = 0;
        t->ref_ok = valid_reference(t->ping, &d0);
        t->d0 = t->ref_ok ? d0 : 0u;
    }
    uint8_t v = push_record(t, now);
    float axis = t->last_axis_x;      /* identical to the record's axis_x */
    if (phase != B5WP_PH_REF && v == B5WP_UNKNOWN) {
        finish(t, B5WP_END_SENSOR, now);   /* strict: fault/timeout/invalid/too-near */
        return;
    }
    switch (phase) {
    case B5WP_PH_REF:
        if (!t->ref_ok) finish(t, B5WP_END_REFERENCE, now);
        else start_side(t, 0u, B5WP_IN, axis, now);
        return;
    case B5WP_PH_LAD_OUT:
        outward_sample(t, t->sample_step, v, axis, now);
        return;
    case B5WP_PH_PRE:
        outward_sample(t, B5WP_STOP_PRE, v, axis, now);
        return;
    case B5WP_PH_OUT:
        outward_sample(t, B5WP_STOP_PRE + t->sample_step, v, axis, now);
        return;
    case B5WP_PH_RET: case B5WP_PH_LAD_RET: case B5WP_PH_MID: case B5WP_PH_FINAL:
        return_sample(t, v, axis, now);
        return;
    default:
        finish(t, B5WP_END_MOVE, now);
        return;
    }
}

/* Stopped + settled + dwell + aligned before any ping (same checks as v0.1.1). */
static void sample_tick(B5WidthProbe *t, MotionStatus s, uint64_t now) {
    if (t->sub == 0u) {
        if (still(s) && t->sensor.settled(now)) {
            if (!t->pose_trimmed && t->sample_phase != B5WP_PH_REF && begin_trim(t, now))
                return;
            if (!t->pose_timing) {
                t->pose_timing = true;
                t->sample_since = now;
            }
            if (now - t->sample_since >= WP_DWELL_US) t->sub = 1u;
        } else {
            t->pose_timing = false;
        }
        return;
    }
    if (!still(s) || !t->sensor.settled(now)) {
        finish(t, B5WP_END_MOVE, now);
        return;
    }
    int h = cdeg(t->heading_deg);
    if (abs(h - 10 * t->servo_dd) > 300 || abs(h - 10 * applied_dd(t, t->servo_dd)) > 300 ||
        abs(h - cdeg(sample_target(t))) > 300) {
        finish(t, B5WP_END_ALIGNMENT, now);
        return;
    }
    if (collect_triplet(t, now)) sample_done(t, now);
}

void b5_wp_init(B5WidthProbe *t, const B5SensorPort *port) {
    memset(t, 0, sizeof(*t));
    if (port) t->sensor = *port;
    t->state = B5WP_IDLE;
    t->end = B5WP_END_NONE;
    t->servo_in_range = true;
}
bool b5_wp_ready(const B5WidthProbe *t) {
    return t->state == B5WP_IDLE || t->state == B5WP_DONE || t->state == B5WP_HOLD;
}
bool b5_wp_begin(B5WidthProbe *t, uint64_t now) {
    if (!b5_wp_ready(t) || t->rec_count || t->summary_pending || t->result_pending ||
        t->retry_sent < t->retry_count || t->limits_pending ||
        !sensor_available(t))
        return false;
    uint32_t lease = 0;
    if (nav_request_takeover(MOTION_BUDDY5, &lease) != MOTION_OK) return false;
    B5SensorPort port = t->sensor;
    int (*applied)(void) = t->applied;
    uint16_t run = t->run;
    memset(t, 0, sizeof(*t));
    t->sensor = port;
    t->applied = applied;
    t->run = (uint16_t)(run + 1u);
    t->lease = lease;
    t->state = B5WP_CLAIM;
    t->end = B5WP_END_NONE;
    t->began = t->since = now;
    t->servo_in_range = true;
    t->min_front_gap = 1.0e6f;
    for (unsigned i = 0; i < 2u; ++i) {
        t->side[i].in_stop = t->side[i].far_stop = t->side[i].last_in = -1;
        t->side[i].min_turn_gap = 1.0e6f;
        t->side[i].clear_total_mm = -1.0f;   /* no step planned */
    }
    t->sensor.cancel();
    t->sensor.angle(0, now);
    t->servo_dd = 0;
    return true;
}
void b5_wp_cancel(B5WidthProbe *t, uint64_t now) {
    if (b5_wp_ready(t)) return;
    finish(t, B5WP_END_CANCELLED, now);
}
void b5_wp_reset(B5WidthProbe *t) {
    if (b5_wp_ready(t)) t->state = B5WP_IDLE;
}
bool b5_wp_peek(const B5WidthProbe *t, B5WPRec *out) {
    if (!t->rec_count || !out) return false;
    *out = t->rec[t->rec_head];
    return true;
}
void b5_wp_drop(B5WidthProbe *t) {
    if (!t->rec_count) return;
    t->rec_head = (uint16_t)((t->rec_head + 1u) % B5WP_QUEUE_CAP);
    --t->rec_count;
}
void b5_wp_tick(B5WidthProbe *t, uint64_t now) {
    if (b5_wp_ready(t)) return;
    MotionStatus s = nav_get_status();
    if (!s.session_active) { finish(t, B5WP_END_CANCELLED, now); return; }
    if (s.fault || s.mode == MOTION_FAULT) { finish(t, B5WP_END_MOVE, now); return; }
    if (!lease_ok(t, s)) { finish(t, B5WP_END_LEASE, now); return; }
    if (now - t->began > WP_RUN_LIMIT_US) { finish(t, B5WP_END_TIMEOUT, now); return; }
    if (t->command_active) {
        (void)finish_move(t, s, now);
        return;
    }
    if (t->state == B5WP_CLAIM) {
        if (still(s) && t->sensor.settled(now) && now - t->began >= WP_FIRST_DWELL_US) {
            t->state = B5WP_REF;
            t->sub = 1u;
            t->sample_phase = B5WP_PH_REF;
            t->sample_step = 0;
            t->npings = 0;
            t->ping_active = false;
            t->sample_ms = 0;
        }
        return;
    }
    switch (t->state) {
    case B5WP_REF: case B5WP_PRE: case B5WP_OUT: case B5WP_RET: case B5WP_FINAL:
    case B5WP_LADDER: case B5WP_MID:
        sample_tick(t, s, now);
        return;
    default:
        finish(t, B5WP_END_MOVE, now);
        return;
    }
}
const char *b5_wp_state_name(unsigned state) {
    static const char *const names[] = {"IDLE", "CLAIM", "REF", "TURN", "PRE", "STEP", "OUT",
        "RETRACE", "RET", "TURN_BACK", "FINAL", "DONE", "HOLD", "TRIM", "LADDER", "MID"};
    return state < sizeof(names) / sizeof(names[0]) ? names[state] : "UNKNOWN";
}
const char *b5_wp_end_name(unsigned end) {
    static const char *const names[] = {"NONE", "COMPLETE", "CANCELLED", "MOVE", "SENSOR",
        "LEASE", "TIMEOUT", "REFERENCE", "ALIGNMENT", "TRAVEL", "CLEARANCE"};
    return end < sizeof(names) / sizeof(names[0]) ? names[end] : "UNKNOWN";
}
