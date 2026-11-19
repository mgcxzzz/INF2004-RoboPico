#include "extensions/buddy5_obstacles/edge_probe.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define B5EP_PI                 3.14159265358979323846f
#define B5EP_DEG                (B5EP_PI / 180.0f)
#define B5EP_SHAFT_MM           62.0f
#ifndef B5EP_GATE_NEAR_MM
#define B5EP_GATE_NEAR_MM       25    /* rtos_buddy1_5 = edge_scan v0.1.1 rule (edge_probe v0.1.1: 15); see B5EP_DIR_WIRE */
#endif
#define B5EP_GATE_FAR_MM        60
#define B5EP_TURN_SPEED         30u
#define B5EP_TRIM_SPEED         20u
#define B5EP_DRIVE_SPEED        30u
#define B5EP_FIRST_DWELL_US     UINT64_C(400000)
#define B5EP_DWELL_US           UINT64_C(100000)
#define B5EP_PING_TIMEOUT_US    UINT64_C(400000)
#define B5EP_RUN_LIMIT_US       UINT64_C(120000000)
#define B5EP_MAX_TRIMS           7u

static bool still(MotionStatus s) { return s.mode == MOTION_PAUSED && s.stopped; }

static bool lease_ok(const B5EdgeProbe *t, MotionStatus s) {
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

static int16_t heading_cdeg(float deg) { return sat_i16(lroundf(deg * 100.0f)); }

static int track_servo(float heading_deg) {
    long dd = lroundf(heading_deg * 10.0f);
    if (dd > B5EP_SERVO_LIMIT_DD) return B5EP_SERVO_LIMIT_DD;
    if (dd < -B5EP_SERVO_LIMIT_DD) return -B5EP_SERVO_LIMIT_DD;
    return (int)dd;
}

static bool sensor_available(const B5EdgeProbe *t) {
    return t->sensor.angle && t->sensor.settled && t->sensor.request && t->sensor.take &&
           t->sensor.cancel;
}

static int applied_dd(const B5EdgeProbe *t, int fallback) {
    return t->applied ? t->applied() : fallback;
}

static void set_servo(B5EdgeProbe *t, int dd, uint64_t now) {
    if (dd > B5EP_SERVO_LIMIT_DD) dd = B5EP_SERVO_LIMIT_DD;
    if (dd < -B5EP_SERVO_LIMIT_DD) dd = -B5EP_SERVO_LIMIT_DD;
    if (dd != t->servo_dd) {
        t->sensor.angle(dd, now);
        t->servo_dd = dd;
    }
    t->last_track_dd = dd;
    if (dd < -B5EP_SERVO_LIMIT_DD || dd > B5EP_SERVO_LIMIT_DD) t->servo_in_range = false;
    int actual = applied_dd(t, dd);
    if (actual < -B5EP_SERVO_LIMIT_DD || actual > B5EP_SERVO_LIMIT_DD)
        t->servo_in_range = false;
}

static void stop_if_owned(B5EdgeProbe *t) {
    MotionStatus s = nav_get_status();
    if (s.session_active && s.owner == MOTION_BUDDY5 && s.lease == t->lease &&
        !still(s) && s.mode != MOTION_FAULT)
        (void)nav_request_stop(MOTION_BUDDY5);
}

static uint8_t side_state(uint8_t pre, uint8_t post, uint8_t back) {
    if (pre == B5EP_FAR && post == B5EP_FAR && back == B5EP_FAR)
        return B5EP_SIDE_OPEN;
    if (pre == B5EP_IN && post == B5EP_FAR && back == B5EP_IN)
        return B5EP_SIDE_CROSSED;
    if (pre == B5EP_IN && post == B5EP_IN && back == B5EP_IN)
        return B5EP_SIDE_REMAINS;
    return B5EP_SIDE_UNKNOWN;
}

static uint8_t decide(const B5EdgeProbe *t, uint8_t left, uint8_t right,
                      int16_t final_heading) {
    if (t->end != B5EP_END_COMPLETE || t->rec_index != B5EP_RECORD_COUNT || t->lost ||
        !t->ref_ok || !t->final_ref_ok || !t->servo_in_range || abs(final_heading) > 300 ||
        !t->left_back_ok || !t->right_back_ok || left == B5EP_SIDE_UNKNOWN ||
        right == B5EP_SIDE_UNKNOWN)
        return B5EP_DECISION_HOLD;

    if (left == B5EP_SIDE_OPEN &&
        (right == B5EP_SIDE_CROSSED || right == B5EP_SIDE_REMAINS))
        return B5EP_DECISION_LEFT;
    if (right == B5EP_SIDE_OPEN &&
        (left == B5EP_SIDE_CROSSED || left == B5EP_SIDE_REMAINS))
        return B5EP_DECISION_RIGHT;
    if (left == B5EP_SIDE_CROSSED && right == B5EP_SIDE_REMAINS)
        return B5EP_DECISION_LEFT;
    if (right == B5EP_SIDE_CROSSED && left == B5EP_SIDE_REMAINS)
        return B5EP_DECISION_RIGHT;
    return B5EP_DECISION_HOLD;
}

static void finish(B5EdgeProbe *t, unsigned end, uint64_t now) {
    if (!b5_ep_ready(t)) stop_if_owned(t);
    if (sensor_available(t)) t->sensor.cancel();
    t->ping_active = false;
    t->command_active = false;
    t->end = end;
    t->state = end == B5EP_END_COMPLETE ? B5EP_DONE : B5EP_HOLD;

    uint8_t left = side_state(t->verdict[B5EP_PH_PRE_L], t->verdict[B5EP_PH_POST_L],
                              t->verdict[B5EP_PH_BACK_L]);
    uint8_t right = side_state(t->verdict[B5EP_PH_PRE_R], t->verdict[B5EP_PH_POST_R],
                               t->verdict[B5EP_PH_BACK_R]);
    int16_t final_heading = heading_cdeg(t->heading_deg);
    uint16_t flags = 0;
    if (t->ref_ok) flags |= B5EP_FLAG_REF_OK;
    if (t->final_ref_ok) flags |= B5EP_FLAG_FINAL_REF_OK;
    if (t->left_back_ok) flags |= B5EP_FLAG_LEFT_BACK_OK;
    if (t->right_back_ok) flags |= B5EP_FLAG_RIGHT_BACK_OK;
    if (t->servo_in_range) flags |= B5EP_FLAG_SERVO_IN_RANGE;

    t->summary = (B5EPSummary){
        .ms = (uint32_t)(now / 1000u), .run = t->run, .end = (uint8_t)end,
        .decision = B5EP_DECISION_HOLD, .state_l = left, .state_r = right,
        .records = t->rec_index, .lost = t->lost, .d0 = t->d0,
        .advance_l_mm10 = mm10(t->advance_l_mm), .reverse_l_mm10 = mm10(t->reverse_l_mm),
        .advance_r_mm10 = mm10(t->advance_r_mm), .reverse_r_mm10 = mm10(t->reverse_r_mm),
        .turn_l_cdeg = heading_cdeg(t->turn_l_deg),
        .turn_r_cdeg = heading_cdeg(t->turn_r_deg),
        .turn_back_cdeg = heading_cdeg(t->turn_back_deg),
        .final_heading_cdeg = final_heading,
        .correction_count = (uint16_t)t->correction_count,
        .correction_total_cdeg = t->correction_total_cdeg,
        .net_x_mm10 = mm10(t->net_x_mm), .net_y_mm10 = mm10(t->net_y_mm), .flags = flags
    };
    t->summary.decision = decide(t, left, right, final_heading);
    t->summary_pending = true;
}

static uint16_t expected_u16(float mm) {
    return sat_u16(lroundf(mm));
}

static uint16_t median_three(const b5_ping_t p[3]) {
    uint16_t v[3] = {(uint16_t)p[0].range_mm, (uint16_t)p[1].range_mm,
                     (uint16_t)p[2].range_mm};
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
            p[i].range_mm < B5EP_D0_MIN_MM ||
            p[i].range_mm > B5EP_D0_MAX_MM)
            return false;
        if (p[i].range_mm < lo) lo = (uint16_t)p[i].range_mm;
        if (p[i].range_mm > hi) hi = (uint16_t)p[i].range_mm;
    }
    if ((unsigned)(hi - lo) > 15u) return false;
    *median = median_three(p);
    return true;
}

static uint8_t classify_triplet(const b5_ping_t p[3], uint16_t expected) {
    unsigned upper = (unsigned)expected + B5EP_GATE_FAR_MM;
    int lower = (int)expected - B5EP_GATE_NEAR_MM;
    bool all_in = true, all_far = true;
    for (unsigned i = 0; i < 3; ++i) {
        bool in = p[i].cls == B5_PING_ECHO && p[i].why == B5_PING_WHY_OK &&
                  (int)p[i].range_mm >= lower &&
                  (unsigned)p[i].range_mm <= upper;
        bool far = (p[i].cls == B5_PING_NO_RETURN && p[i].why == B5_PING_WHY_OK) ||
                   (p[i].cls == B5_PING_ECHO && p[i].why == B5_PING_WHY_OK &&
                    (unsigned)p[i].range_mm > upper);
        all_in &= in;
        all_far &= far;
    }
    return all_in ? B5EP_IN : all_far ? B5EP_FAR : B5EP_UNKNOWN;
}

static unsigned side_for_phase(unsigned phase) {
    return phase == B5EP_PH_PRE_R || phase == B5EP_PH_POST_R || phase == B5EP_PH_BACK_R;
}

static uint16_t expected_for_record(const B5EdgeProbe *t, uint8_t phase,
                                    int16_t heading, int16_t travel, int16_t net_y) {
    float h = ((float)heading / 100.0f) * B5EP_DEG;
    float y = (float)net_y / 10.0f;
    float distance = (float)travel / 10.0f;
    if (phase == B5EP_PH_REF) return t->d0;
    if (phase == B5EP_PH_FINAL)
        return expected_u16((float)t->d0 + B5EP_SHAFT_MM * (1.0f - cosf(h)) - y);
    if (phase == B5EP_PH_PRE_L || phase == B5EP_PH_PRE_R)
        return expected_u16((float)t->d0 + B5EP_SHAFT_MM * (1.0f - cosf(h)) - y);

    unsigned side = side_for_phase(phase);
    uint8_t pre_phase = side ? B5EP_PH_PRE_R : B5EP_PH_PRE_L;
    float base = t->verdict[pre_phase] == B5EP_IN ?
        (float)t->pre_median_mm[side] : (float)t->pre_expected_mm[side];
    if (phase == B5EP_PH_POST_L || phase == B5EP_PH_BACK_L ||
        phase == B5EP_PH_POST_R || phase == B5EP_PH_BACK_R) {
        float pre_h = ((float)t->pre_heading_cdeg[side] / 100.0f) * B5EP_DEG;
        float pre_y = (float)t->pre_net_y_mm10[side] / 10.0f;
        return expected_u16(base + B5EP_SHAFT_MM * (cosf(pre_h) - cosf(h)) - (y - pre_y));
    }
    return expected_u16(base - distance * cosf(h));
}

static void push_record(B5EdgeProbe *t, uint64_t now) {
    B5EPRec r = {0};
    r.ms = t->sample_ms ? t->sample_ms : (uint32_t)(now / 1000u);
    r.run = t->run;
    r.index = t->rec_index;
    r.phase = t->sample_phase;
    r.n = 3;
    r.heading_cdeg = t->sample_heading_cdeg;
    r.servo_dd = t->sample_servo_dd;
    r.applied_dd = t->sample_applied_dd;
    r.travel_mm10 = mm10(t->travel_mm);
    r.net_x_mm10 = mm10(t->net_x_mm);
    r.net_y_mm10 = mm10(t->net_y_mm);
    r.correction_cdeg = t->correction_total_cdeg;

    if (r.phase == B5EP_PH_REF) {
        r.anchor_mm = t->d0;
    } else if (r.phase == B5EP_PH_FINAL) {
        r.anchor_mm = t->d0;
    } else if (r.phase == B5EP_PH_PRE_L || r.phase == B5EP_PH_PRE_R) {
        unsigned side = side_for_phase(r.phase);
        r.expected_mm = expected_for_record(t, r.phase, r.heading_cdeg, r.travel_mm10,
                                           r.net_y_mm10);
        t->pre_expected_mm[side] = (int16_t)r.expected_mm;
        t->pre_heading_cdeg[side] = r.heading_cdeg;
        t->pre_net_y_mm10[side] = r.net_y_mm10;
        r.evidence = classify_triplet(t->ping, r.expected_mm);
        t->verdict[r.phase] = r.evidence;
        if (r.evidence == B5EP_IN) {
            t->pre_median_mm[side] = median_three(t->ping);
            r.anchor_mm = t->pre_median_mm[side];
        }
    } else {
        r.expected_mm = expected_for_record(t, r.phase, r.heading_cdeg, r.travel_mm10,
                                           r.net_y_mm10);
        unsigned side = side_for_phase(r.phase);
        r.anchor_mm = t->verdict[side ? B5EP_PH_PRE_R : B5EP_PH_PRE_L] == B5EP_IN ?
            t->pre_median_mm[side] : 0u;
        r.evidence = classify_triplet(t->ping, r.expected_mm);
        t->verdict[r.phase] = r.evidence;
    }

    if (r.phase == B5EP_PH_REF || r.phase == B5EP_PH_FINAL) {
        r.expected_mm = expected_for_record(t, r.phase, r.heading_cdeg, r.travel_mm10,
                                           r.net_y_mm10);
        r.evidence = r.phase == B5EP_PH_FINAL ? classify_triplet(t->ping, r.expected_mm) :
                     t->ref_ok ? B5EP_IN : B5EP_UNKNOWN;
        if (r.phase == B5EP_PH_FINAL) t->verdict[r.phase] = r.evidence;
    }

    for (unsigned i = 0; i < 3; ++i) {
        r.cls[i] = (uint8_t)((t->ping[i].cls & 3u) | ((t->ping[i].why & 7u) << 4));
        r.value[i] = t->ping[i].cls == B5_PING_ECHO ?
            (uint16_t)t->ping[i].range_mm :
            (uint16_t)(t->ping[i].width_us > UINT16_MAX ? UINT16_MAX : t->ping[i].width_us);
    }

    if (abs(r.servo_dd) > B5EP_SERVO_LIMIT_DD || abs(r.applied_dd) > B5EP_SERVO_LIMIT_DD)
        t->servo_in_range = false;
    if (t->rec_count >= B5EP_QUEUE_CAP) {
        if (t->lost != UINT16_MAX) ++t->lost;
    } else {
        uint16_t tail = (uint16_t)((t->rec_head + t->rec_count) % B5EP_QUEUE_CAP);
        t->rec[tail] = r;
        ++t->rec_count;
    }
    if (t->rec_index != UINT16_MAX) ++t->rec_index;
}

static void begin_pose(B5EdgeProbe *t, unsigned state, uint8_t phase,
                       int servo, uint64_t now, float travel) {
    t->state = state;
    t->sub = 0;
    t->sample_phase = phase;
    t->travel_mm = travel;
    t->npings = 0;
    t->ping_active = false;
    t->pose_timing = false;
    t->pose_trimmed = false;
    t->sample_since = now;
    t->sample_ms = 0;
    set_servo(t, servo, now);
}

static void begin_turn(B5EdgeProbe *t, unsigned state, float degrees, uint64_t now) {
    if (nav_turn_relative(MOTION_BUDDY5, t->lease, degrees, B5EP_TURN_SPEED,
                          MOTION_IN_PLACE, &t->command) != MOTION_OK) {
        finish(t, B5EP_END_MOVE, now);
        return;
    }
    t->state = state;
    t->command_active = true;
    t->move_committed = false;
    t->move_start_heading = t->heading_deg;
    t->since = now;
}

static float sample_target(const B5EdgeProbe *t) {
    switch (t->sample_phase) {
    case B5EP_PH_PRE_L: case B5EP_PH_POST_L: case B5EP_PH_BACK_L: return 45.0f;
    case B5EP_PH_PRE_R: case B5EP_PH_POST_R: case B5EP_PH_BACK_R: return -45.0f;
    case B5EP_PH_FINAL: return 0.0f;
    default: return t->heading_deg;
    }
}

static bool begin_trim(B5EdgeProbe *t, uint64_t now) {
    float delta = sample_target(t) - t->heading_deg;
    if (fabsf(delta) <= 2.0f) {
        t->pose_trimmed = true;
        return false;
    }
    if (fabsf(delta) > 12.0f || t->correction_count >= B5EP_MAX_TRIMS) {
        finish(t, B5EP_END_ALIGNMENT, now);
        return true;
    }
    if (nav_turn_relative(MOTION_BUDDY5, t->lease, delta, B5EP_TRIM_SPEED,
                          MOTION_IN_PLACE, &t->command) != MOTION_OK) {
        finish(t, B5EP_END_MOVE, now);
        return true;
    }
    t->trim_return_state = t->state;
    t->trim_target_deg = sample_target(t);
    t->state = B5EP_TRIM;
    t->command_active = true;
    t->move_committed = false;
    t->move_start_heading = t->heading_deg;
    t->since = now;
    return true;
}

static void begin_drive(B5EdgeProbe *t, unsigned state, float mm, uint64_t now) {
    if (nav_drive_distance(MOTION_BUDDY5, t->lease, mm, B5EP_DRIVE_SPEED,
                           &t->command) != MOTION_OK) {
        finish(t, B5EP_END_MOVE, now);
        return;
    }
    t->state = state;
    t->command_active = true;
    t->move_committed = false;
    t->move_start_heading = t->heading_deg;
    t->since = now;
}

static void integrate_drive(B5EdgeProbe *t, const MotionCommandStatus *c) {
    float mid = (t->move_start_heading + c->rotation_deg * 0.5f) * B5EP_DEG;
    /* x points right, y points forward. Positive body rotation is left. */
    t->net_x_mm -= c->progress_mm * sinf(mid);
    t->net_y_mm += c->progress_mm * cosf(mid);
    t->heading_deg = t->move_start_heading + c->rotation_deg;
}

static bool finish_move(B5EdgeProbe *t, MotionStatus s, uint64_t now) {
    MotionCommandStatus c;
    if (!t->command_active) return false;
    if (!nav_get_command(t->command, &c)) {
        finish(t, B5EP_END_MOVE, now);
        return true;
    }

    bool trimming = t->state == B5EP_TRIM;
    bool turning = trimming || t->state == B5EP_TURN_L || t->state == B5EP_TURN_R ||
                   t->state == B5EP_TURN_BACK;
    if (turning && (c.state == MOTION_CMD_RUNNING || !still(s))) {
        float live = t->move_start_heading + c.rotation_deg;
        set_servo(t, track_servo(live), now);
    }
    if (c.state == MOTION_CMD_RUNNING) return true;
    if (c.state == MOTION_CMD_FAILED || c.state == MOTION_CMD_CANCELLED ||
        c.state != MOTION_CMD_DONE) {
        finish(t, B5EP_END_MOVE, now);
        return true;
    }
    if (!still(s)) return true;

    if (turning) {
        t->heading_deg = t->move_start_heading + c.rotation_deg;
        if (trimming) {
            if (fabsf(c.rotation_deg) > 15.0f ||
                fabsf(t->heading_deg - t->trim_target_deg) > 3.0f) {
                finish(t, B5EP_END_ALIGNMENT, now);
                return true;
            }
            /* An ideal in-place turn has zero centre travel. Account encoder
             * imbalance as a small translation using the measured midpoint. */
            float mid = (t->move_start_heading + c.rotation_deg * 0.5f) * B5EP_DEG;
            t->net_x_mm -= c.progress_mm * sinf(mid);
            t->net_y_mm += c.progress_mm * cosf(mid);
            ++t->correction_count;
            t->correction_total_cdeg = heading_cdeg(
                (float)t->correction_total_cdeg / 100.0f + c.rotation_deg);
            t->state = t->trim_return_state;
            t->command_active = false;
            t->move_committed = true;
            t->pose_trimmed = true;
            t->pose_timing = false;
            t->sub = 0;
            t->sample_since = now;
            set_servo(t, track_servo(t->heading_deg), now);
            return true;
        }
        if (t->state == B5EP_TURN_L) t->turn_l_deg = c.rotation_deg;
        else if (t->state == B5EP_TURN_R) t->turn_r_deg = c.rotation_deg;
        else t->turn_back_deg = c.rotation_deg;
    } else {
        integrate_drive(t, &c);
        if (t->state == B5EP_DRIVE_L) {
            t->advance_l_mm = c.progress_mm;
            if (c.progress_mm < 35.0f || c.progress_mm > 55.0f) {
                finish(t, B5EP_END_MOVE, now);
                return true;
            }
            t->travel_mm = c.progress_mm;
        } else if (t->state == B5EP_DRIVE_R) {
            t->advance_r_mm = c.progress_mm;
            if (c.progress_mm < 35.0f || c.progress_mm > 55.0f) {
                finish(t, B5EP_END_MOVE, now);
                return true;
            }
            t->travel_mm = c.progress_mm;
        } else if (t->state == B5EP_RETRACE_L) {
            t->reverse_l_mm = c.progress_mm;
            t->travel_mm = t->advance_l_mm + t->reverse_l_mm;
            t->left_back_ok = c.progress_mm < 0.0f && fabsf(t->travel_mm) <= 2.0f;
        } else {
            t->reverse_r_mm = c.progress_mm;
            t->travel_mm = t->advance_r_mm + t->reverse_r_mm;
            t->right_back_ok = c.progress_mm < 0.0f && fabsf(t->travel_mm) <= 2.0f;
        }
    }
    t->command_active = false;
    t->move_committed = true;

    switch (t->state) {
    case B5EP_TURN_L:
        begin_pose(t, B5EP_PRE_L, B5EP_PH_PRE_L, track_servo(t->heading_deg), now, 0.0f);
        break;
    case B5EP_TURN_R:
        begin_pose(t, B5EP_PRE_R, B5EP_PH_PRE_R, track_servo(t->heading_deg), now, 0.0f);
        break;
    case B5EP_TURN_BACK:
        begin_pose(t, B5EP_FINAL, B5EP_PH_FINAL, track_servo(t->heading_deg), now, 0.0f);
        break;
    case B5EP_DRIVE_L:
        begin_pose(t, B5EP_POST_L, B5EP_PH_POST_L, track_servo(t->heading_deg), now,
                   t->advance_l_mm);
        break;
    case B5EP_DRIVE_R:
        begin_pose(t, B5EP_POST_R, B5EP_PH_POST_R, track_servo(t->heading_deg), now,
                   t->advance_r_mm);
        break;
    case B5EP_RETRACE_L: {
        begin_pose(t, B5EP_BACK_L, B5EP_PH_BACK_L, track_servo(t->heading_deg), now,
                   t->travel_mm);
        break;
    }
    case B5EP_RETRACE_R:
        begin_pose(t, B5EP_BACK_R, B5EP_PH_BACK_R, track_servo(t->heading_deg), now,
                   t->travel_mm);
        break;
    default:
        finish(t, B5EP_END_MOVE, now);
        break;
    }
    return true;
}

static bool collect_triplet(B5EdgeProbe *t, uint64_t now) {
    if (t->npings >= 3u) return true;
    if (!t->ping_active) {
        if (!t->sensor.request(now)) return false;
        t->ping_active = true;
        t->ping_since = now;
        if (t->npings == 0u) {
            t->sample_ms = (uint32_t)(now / 1000u);
            t->sample_heading_cdeg = heading_cdeg(t->heading_deg);
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
    } else if (now - t->ping_since >= B5EP_PING_TIMEOUT_US) {
        t->sensor.cancel();
        t->ping[t->npings++] = (b5_ping_t){.cls = B5_PING_FAULT, .why = 7u};
        t->ping_active = false;
        t->ping_since = 0;
    }
    return t->npings >= 3u;
}

static void start_left_retrace(B5EdgeProbe *t, uint64_t now) {
    float mm = fminf(t->advance_l_mm, (float)B5EP_PROBE_MM);
    begin_drive(t, B5EP_RETRACE_L, -mm, now);
}

static void start_right_retrace(B5EdgeProbe *t, uint64_t now) {
    float mm = fminf(t->advance_r_mm, (float)B5EP_PROBE_MM);
    begin_drive(t, B5EP_RETRACE_R, -mm, now);
}

static void sample_done(B5EdgeProbe *t, uint64_t now) {
    uint8_t phase = t->sample_phase;
    if (phase == B5EP_PH_REF) {
        uint16_t d0 = 0;
        t->ref_ok = valid_reference(t->ping, &d0);
        t->d0 = t->ref_ok ? d0 : 0u;
        t->verdict[B5EP_PH_REF] = t->ref_ok ? B5EP_IN : B5EP_UNKNOWN;
    }
    push_record(t, now);
    if (phase != B5EP_PH_REF && phase != B5EP_PH_FINAL &&
        t->verdict[phase] == B5EP_UNKNOWN) {
        finish(t, B5EP_END_SENSOR, now);
        return;
    }

    switch (t->state) {
    case B5EP_REF: {
        if (!t->ref_ok) finish(t, B5EP_END_REFERENCE, now);
        else begin_turn(t, B5EP_TURN_L, 46.0f, now);
        return;
    }
    case B5EP_PRE_L:
        begin_drive(t, B5EP_DRIVE_L, (float)B5EP_PROBE_MM, now);
        return;
    case B5EP_POST_L:
        start_left_retrace(t, now);
        return;
    case B5EP_BACK_L:
        if (!t->left_back_ok ||
            t->verdict[B5EP_PH_BACK_L] != t->verdict[B5EP_PH_PRE_L]) {
            t->left_back_ok = false;
            finish(t, B5EP_END_SENSOR, now);
        }
        else begin_turn(t, B5EP_TURN_R, -46.0f - t->heading_deg, now);
        return;
    case B5EP_PRE_R:
        begin_drive(t, B5EP_DRIVE_R, (float)B5EP_PROBE_MM, now);
        return;
    case B5EP_POST_R:
        start_right_retrace(t, now);
        return;
    case B5EP_BACK_R:
        if (!t->right_back_ok ||
            t->verdict[B5EP_PH_BACK_R] != t->verdict[B5EP_PH_PRE_R]) {
            t->right_back_ok = false;
            finish(t, B5EP_END_SENSOR, now);
        }
        else begin_turn(t, B5EP_TURN_BACK, -t->heading_deg, now);
        return;
    case B5EP_FINAL:
        t->final_ref_ok = t->verdict[B5EP_PH_FINAL] == B5EP_IN;
        if (t->verdict[B5EP_PH_FINAL] == B5EP_UNKNOWN) finish(t, B5EP_END_SENSOR, now);
        else finish(t, B5EP_END_COMPLETE, now);
        return;
    default:
        finish(t, B5EP_END_MOVE, now);
        return;
    }
}

static void sample_tick(B5EdgeProbe *t, MotionStatus s, uint64_t now) {
    if (t->sub == 0u) {
        if (still(s) && t->sensor.settled(now)) {
            if (!t->pose_trimmed && begin_trim(t, now)) return;
            if (!t->pose_timing) {
                t->pose_timing = true;
                t->sample_since = now;
            }
            if (now - t->sample_since >= B5EP_DWELL_US) t->sub = 1u;
        } else {
            t->pose_timing = false;
        }
        return;
    }
    if (!still(s) || !t->sensor.settled(now)) {
        finish(t, B5EP_END_MOVE, now);
        return;
    }
    int requested_error = heading_cdeg(t->heading_deg) - 10 * t->servo_dd;
    int applied_error = heading_cdeg(t->heading_deg) - 10 * applied_dd(t, t->servo_dd);
    if (abs(requested_error) > 300 || abs(applied_error) > 300 ||
        abs(heading_cdeg(t->heading_deg) - heading_cdeg(sample_target(t))) > 300) {
        finish(t, B5EP_END_ALIGNMENT, now);
        return;
    }
    if (collect_triplet(t, now)) sample_done(t, now);
}

void b5_ep_init(B5EdgeProbe *t, const B5SensorPort *port) {
    memset(t, 0, sizeof(*t));
    if (port) t->sensor = *port;
    t->state = B5EP_IDLE;
    t->end = B5EP_END_NONE;
    t->servo_in_range = true;
}

bool b5_ep_ready(const B5EdgeProbe *t) {
    return t->state == B5EP_IDLE || t->state == B5EP_DONE || t->state == B5EP_HOLD;
}

bool b5_ep_begin(B5EdgeProbe *t, uint64_t now) {
    if (!b5_ep_ready(t) || t->rec_count || t->summary_pending || !sensor_available(t))
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
    t->state = B5EP_CLAIM;
    t->end = B5EP_END_NONE;
    t->began = t->since = now;
    t->servo_in_range = true;
    t->sensor.cancel();
    t->sensor.angle(0, now);
    t->servo_dd = 0;
    t->last_track_dd = 0;
    return true;
}

void b5_ep_cancel(B5EdgeProbe *t, uint64_t now) {
    if (b5_ep_ready(t)) return;
    finish(t, B5EP_END_CANCELLED, now);
}

void b5_ep_reset(B5EdgeProbe *t) {
    if (b5_ep_ready(t)) t->state = B5EP_IDLE;
}

bool b5_ep_peek(const B5EdgeProbe *t, B5EPRec *out) {
    if (!t->rec_count || !out) return false;
    *out = t->rec[t->rec_head];
    return true;
}

void b5_ep_drop(B5EdgeProbe *t) {
    if (!t->rec_count) return;
    t->rec_head = (uint16_t)((t->rec_head + 1u) % B5EP_QUEUE_CAP);
    --t->rec_count;
}

void b5_ep_tick(B5EdgeProbe *t, uint64_t now) {
    if (b5_ep_ready(t)) return;
    MotionStatus s = nav_get_status();
    if (!s.session_active) { finish(t, B5EP_END_CANCELLED, now); return; }
    if (s.fault || s.mode == MOTION_FAULT) { finish(t, B5EP_END_MOVE, now); return; }
    if (!lease_ok(t, s)) { finish(t, B5EP_END_LEASE, now); return; }
    if (now - t->began > B5EP_RUN_LIMIT_US) { finish(t, B5EP_END_TIMEOUT, now); return; }

    if (t->command_active) {
        (void)finish_move(t, s, now);
        return;
    }
    if (t->state == B5EP_CLAIM) {
        if (still(s) && t->sensor.settled(now) && now - t->began >= B5EP_FIRST_DWELL_US) {
            t->state = B5EP_REF;
            t->sub = 1u; /* claim already provided the full first-pose dwell */
            t->sample_phase = B5EP_PH_REF;
            t->npings = 0;
            t->ping_active = false;
            t->sample_ms = 0;
        }
        return;
    }

    switch (t->state) {
    case B5EP_REF: case B5EP_PRE_L: case B5EP_POST_L: case B5EP_BACK_L:
    case B5EP_PRE_R: case B5EP_POST_R: case B5EP_BACK_R: case B5EP_FINAL:
        sample_tick(t, s, now);
        return;
    default:
        finish(t, B5EP_END_MOVE, now);
        return;
    }
}

const char *b5_ep_state_name(unsigned state) {
    static const char *const names[] = {
        "IDLE", "CLAIM", "REF", "TURN_L", "PRE_L", "DRIVE_L", "POST_L", "RETRACE_L",
        "BACK_L", "TURN_R", "PRE_R", "DRIVE_R", "POST_R", "RETRACE_R", "BACK_R",
        "TURN_BACK", "FINAL", "DONE", "HOLD", "TRIM"
    };
    return state < sizeof(names) / sizeof(names[0]) ? names[state] : "UNKNOWN";
}

const char *b5_ep_end_name(unsigned end) {
    static const char *const names[] = {
        "NONE", "COMPLETE", "CANCELLED", "MOVE", "SENSOR", "LEASE", "TIMEOUT",
        "REFERENCE", "ALIGNMENT"
    };
    return end < sizeof(names) / sizeof(names[0]) ? names[end] : "UNKNOWN";
}
