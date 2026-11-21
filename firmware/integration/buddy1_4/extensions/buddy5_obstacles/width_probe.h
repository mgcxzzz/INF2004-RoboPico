#ifndef B5_WIDTH_PROBE_H
#define B5_WIDTH_PROBE_H

/* v0.2 candidate: wire10, 220 mm outer ceiling (clearance still binds first),
 * one bounded stationary retry for TRANSITION at the returning IN edge stop,
 * and provenance-labelled corroborated edge display estimates. Earlier version
 * comments below describe historical baselines, not the current wire policy. */

/* width_long_probe v0.1.1 = v0.1 + LOGGING ONLY: every accepted-or-rejected stopped
 * record of the run is also kept in arch[] (never dropped by the radio queue); with the
 * retained D2/D5/D4/D3 state it is saved to one flash sector after the run has stopped
 * and can be exported over USB (width_probe_log.h, width_probe_runtime.c). Motion,
 * clearance, sampling, gates, retries, estimator and all wire versions are unchanged. */

/* width_long_probe v0.1 (separate EXPERIMENTAL image, based on width_calibrated_probe v0.2):
 * the arbitrary 100 mm diagonal limit is replaced by a CLEARANCE-LIMITED pass with a
 * 190 mm measured outward ceiling per side. The countertracked 45 deg PRE leg, the
 * +-46 deg servo cap, the ladder, three-ping sampling, gates, stationary retries,
 * dwell, retrace of visited stops only, holds and the display estimator (coefficients,
 * -10 mm correction, 100-200 mm limits) are unchanged. Each outward step is the
 * smallest of: the 20 mm nominal step, the 190 mm ceiling less the 5 mm overshoot
 * reserve, and the travel that keeps the modelled worst-turn corner radius (and the
 * heading-dependent front corner) >= 50 mm from the modelled face plane after the
 * worst accepted segment, using the integrated pose and a 5 deg heading reserve, so
 * the trim/turn/retrace at the new stop stays inside the same model. The pass stops at
 * the first of: confirmed disappearance (first all-FAR, unchanged), ceiling, clearance,
 * step bound or fault. A ceiling/clearance/step limit retraces normally and is
 * reported (summary cap flags; new D5 limits frame); a fault keeps its hold.
 * Wire: D1/D2 v9 (v4 layout and semantics, long-probe sequence bounds), D3 v2, D4 v2
 * (layouts unchanged), D5 v1 (new). Older decoders reject these versions; v4/D3 v1
 * captures keep their 100 mm meaning. The estimator's completion rule still requires
 * the TRAVEL CEILING (cap_budget), never a clearance or step limit. */

/* width_calibrated_probe v0.1 (separate image): width probe v0.2.1 motion, sampling,
 * clearance, Stop/fault handling and D1/D2 wire v4 UNCHANGED. Added: at the end of
 * every attempted run (any end code) the Pico computes a DISPLAY width from the
 * stopped records (width_estimate.c: frozen stepped range-rise + calibration) and
 * sends it in a new D3 result frame (version B5WP_RESULT_WIRE) after the D2 summary.
 * The result is never read by motion, clearance or any hold/fault decision.
 *
 * EXPERIMENTAL width-measurement diagnostic v0.2.1 (Pico SDK, not RTOS), wire v4.
 * v0.2.1 = v0.2 with ONE change: the plane-association near allowance is
 * 25 mm (was 15) and the same 25 mm is subtracted from the modelled face
 * plane in every footprint/turn clearance check (conservative propagation).
 * Far +60, faults/timeouts/invalid/too-near stops and everything else are
 * unchanged. Wire v3 data keep their original -15/+60 meaning.
 * Separate image from the direction edge probe v0.1.1 and from width probe
 * v0.1 / v0.1.1, all unchanged. Observes and reports only: it never resumes
 * navigation, never commands a bypass and holds stopped at the end or after
 * any abort.
 *
 * v0.2 changes (see CHANGES.md):
 *  - Range model: fixed frontal REF plane + continuous quantized encoder pose
 *    for EVERY sample, expected = d0 + 62(1 - cos h) - netY. No side anchors.
 *  - Stopped countertracked turn ladder per side: heading 0 (REF / MID),
 *    +-15, +-30, PRE +-45, then the existing adaptive forward steps.
 *  - Symmetric return: only stops actually visited are revisited, in reverse;
 *    MID (heading 0) between sides, FINAL at the end.
 *  - TRANSITION: a triplet of only clean IN and clean FAR pings, mixed. It is
 *    neither IN nor FAR; it never bounds or corroborates an edge.
 *
 * Coordinate signs (all model values, never physical pose feedback):
 *   x = RIGHT, y = FORWARD, mm, origin = drive-axle midpoint at REF.
 *   Body heading + = LEFT (degrees, encoder model).
 *   Servo + = RIGHT relative to the body (0.1 deg); beam world angle
 *   (+ = left) = heading - servo.
 *   "Axis x" = lateral position where the modelled sonar beam axis meets
 *   the modelled face plane (shaft position + beam direction x expected range).
 */
#include <stdbool.h>
#include <stdint.h>
#include "extensions/buddy5_obstacles/automatic_bypass.h" /* B5SensorPort */
#include "include/interfaces/nav_api.h"
#include "extensions/buddy5_obstacles/width_estimate.h"

#define B5WP_RETRIES_PER_RUN  4u
#ifndef B5WP_RUN_ARCHIVE         /* SDK v0.1.1+ run-log copy for the USB/flash export (default on).
                                    0 = no copy: RTOS build, which has no archive exporter and
                                    needs the RAM; probe, estimator and wire output are unchanged. */
#define B5WP_RUN_ARCHIVE      1
#endif
#define B5WP_RESULT_WIRE      3u   /* D3 v3: rule2, basis4, budget flags = 220 mm ceiling */
#define B5WP_RETRY_WIRE       3u   /* D4 v3: includes bounded boundary-TRANSITION retry */
#define B5WP_LIMIT_WIRE       2u   /* D5 v2: same layout, 220 mm outer ceiling */

#define B5WP_WIRE_VERSION 10u   /* v1 rolling; v2 stable PRE; v3 REF plane + ladder (-15/+60);
                                  v4 = v3 with near allowance 25 + face-plane margin 25;
                                  (5-8 other experiments); v9 = v4 semantics, long
                                  clearance-limited diagonal: 190 mm ceiling, 12 steps */
#define B5WP_NEAR_MM          25       /* plane association: IN if expected-25 <= echo <= expected+60 */
#define B5WP_FAR_MM           60       /* unchanged */
#define B5WP_FACE_MARGIN_MM   25.0f    /* modelled face plane moved 25 mm nearer in clearance checks */
#define B5WP_SERVO_LIMIT_DD   460      /* shared verified +/-46 deg command cap */
#define B5WP_BUDGET_MM        220.0f   /* travel CEILING: cumulative MEASURED outward drive per side */
#define B5WP_TURN_RESERVE_DEG 5.0f     /* heading reserve in the clearance projection */
#define B5WP_STEP_MM          20.0f    /* nominal adaptive step */
#define B5WP_OVERSHOOT_MM     5.0f     /* allowed measured-minus-command; also budget reserve */
#define B5WP_MIN_STEP_MM      5.0f     /* a smaller remaining step means budget exhausted */
#define B5WP_MAX_STEPS        12u      /* hard bound on outward segments per side (10 nominal) */
#define B5WP_RETURN_RESIDUAL_MM 2.0f   /* path residual allowed after retrace */
#define B5WP_RUNG_DEG         15.0f    /* ladder rung spacing: +-15, +-30, then PRE +-45 */
#define B5WP_PRE_DEG          45.0f
#ifndef B5WP_D0_MIN_MM                 /* override only in the host clearance test */
#define B5WP_D0_MIN_MM        270u     /* 300 mm manual setup; rejects the 200 mm setup */
#endif
#ifndef B5WP_D0_MAX_MM
#define B5WP_D0_MAX_MM        330u
#endif
#define B5WP_FRONT_MARGIN_MM  50.0f    /* modelled front corner to modelled face plane */
#define B5WP_CAR_FRONT_MM     130.0f   /* axle mid -> front IR bar tip, measured 2026-11-01 */
#define B5WP_CAR_REAR_MM      60.0f    /* axle mid -> rear (<60 measured; 190 total) */
#define B5WP_CAR_HALF_W_MM    67.5f    /* 135 mm widest point, measured 2026-11-01 */
#define B5WP_SHAFT_MM         62.0f    /* same model shaft as the direction probe */
#define B5WP_MAX_TRIMS        24u      /* bounded stopped-pose corrections per run (16 at 45 records) */
/* Stops per side: 0 = frontal (REF for L, MID for R), 1/2 = rungs, 3 = PRE,
 * 4.. = forward steps. */
#define B5WP_STOP_PRE         3u
#define B5WP_MAX_STOPS        (B5WP_STOP_PRE + 1u + B5WP_MAX_STEPS)
/* REF + per side (2 rungs + PRE + 12 steps outward, 12 step/PRE returns, 2 rung
 * returns) + MID + FINAL = 61. */
#define B5WP_MAX_RECORDS      (3u + 2u * (2u + 1u + 2u * B5WP_MAX_STEPS + 2u))
#define B5WP_QUEUE_CAP        64u
_Static_assert(B5WP_QUEUE_CAP >= B5WP_MAX_RECORDS, "queue holds a whole run");
_Static_assert(B5WE_MAX_POINTS >= B5WP_MAX_RECORDS, "estimator keeps every record of a run");
_Static_assert(B5WP_MAX_STEPS <= 255u && B5WP_MAX_RECORDS <= 255u, "u8 step / point counts");
#define B5WP_NO_ANCHOR        0xFFFFu

enum {
    B5WP_IDLE, B5WP_CLAIM, B5WP_REF, B5WP_TURN, B5WP_PRE, B5WP_STEP, B5WP_OUT,
    B5WP_RETRACE, B5WP_RET, B5WP_TURN_BACK, B5WP_FINAL, B5WP_DONE, B5WP_HOLD, B5WP_TRIM,
    B5WP_LADDER, B5WP_MID
};
enum {
    B5WP_END_NONE, B5WP_END_COMPLETE, B5WP_END_CANCELLED, B5WP_END_MOVE,
    B5WP_END_SENSOR, B5WP_END_LEASE, B5WP_END_TIMEOUT, B5WP_END_REFERENCE,
    B5WP_END_ALIGNMENT, B5WP_END_TRAVEL, B5WP_END_CLEARANCE
};
/* Record phases (wire v3 adds 5..7). OUT/RET step = forward stop (0 = PRE). */
enum { B5WP_PH_REF, B5WP_PH_PRE, B5WP_PH_OUT, B5WP_PH_RET, B5WP_PH_FINAL,
       B5WP_PH_LAD_OUT, B5WP_PH_LAD_RET, B5WP_PH_MID };
enum { B5WP_SIDE_NONE, B5WP_SIDE_L, B5WP_SIDE_R };   /* record side field */
enum { B5WP_UNKNOWN, B5WP_IN, B5WP_FAR, B5WP_TRANSITION };
enum {
    B5WP_ST_UNKNOWN,        /* not concluded: aborted, faulty or missing evidence     */
    B5WP_ST_NO_INNER_IN,    /* first all-FAR came before any all-IN: not bracketed     */
    B5WP_ST_BOUNDARY,       /* IN->FAR outward, IN-bound stop all-IN again on return   */
    B5WP_ST_NOT_REACHED,    /* no all-FAR before the budget/clearance/step bound       */
    B5WP_ST_CONTRADICTORY,  /* a return FAR where the outward pass was all-IN          */
    B5WP_ST_UNCORROBORATED  /* IN->FAR outward but the IN-bound stop not IN on return  */
};
enum { B5WP_WIDTH_INCOMPLETE, B5WP_WIDTH_MEASURED };
/* D5: why a side's outward pass ended (first that applied). */
enum { B5WP_LIM_NONE, B5WP_LIM_FAR, B5WP_LIM_CEILING, B5WP_LIM_CLEARANCE, B5WP_LIM_STEPS,
       B5WP_LIM_ABORT };
enum {
    B5WP_FLAG_REF_OK = 1u << 0, B5WP_FLAG_FINAL_REF_OK = 1u << 1,
    B5WP_FLAG_L_RETURN_OK = 1u << 2, B5WP_FLAG_R_RETURN_OK = 1u << 3,
    B5WP_FLAG_SERVO_IN_RANGE = 1u << 4,
    B5WP_FLAG_L_CAP_BUDGET = 1u << 5, B5WP_FLAG_R_CAP_BUDGET = 1u << 6,
    B5WP_FLAG_L_CAP_CLEAR = 1u << 7, B5WP_FLAG_R_CAP_CLEAR = 1u << 8,
    B5WP_FLAG_L_CAP_STEPS = 1u << 9, B5WP_FLAG_R_CAP_STEPS = 1u << 10,
    /* wire v3 */
    B5WP_FLAG_L_FAR_NO_STEP = 1u << 11, B5WP_FLAG_R_FAR_NO_STEP = 1u << 12, /* FAR on ladder/PRE */
    B5WP_FLAG_L_TRANSITION = 1u << 13, B5WP_FLAG_R_TRANSITION = 1u << 14,
    B5WP_FLAG_MID_OK = 1u << 15
};

typedef struct {
    uint32_t ms;
    uint16_t run, index;
    uint8_t phase, side, step, n;       /* step: OUT/RET forward stop, LAD rung 1/2 */
    int16_t heading_cdeg, servo_dd, applied_dd;
    int16_t path_mm10, seg_cmd_mm10, seg_meas_mm10;   /* measured along-leg path */
    uint16_t expected_mm, anchor_mm, anchor_index;    /* v3: anchor = REF (d0, index 0) */
    uint8_t cls[3], evidence;
    uint16_t value[3];
    int16_t net_x_mm10, net_y_mm10, correction_cdeg;
    int16_t axis_x_mm10, front_gap_mm10;
} B5WPRec;

typedef struct {
    uint32_t ms;
    uint16_t run;
    uint8_t end, width_result, state_l, state_r, steps_l, steps_r;
    uint16_t records, lost, d0;
    int16_t out_l_mm10, out_r_mm10, res_l_mm10, res_r_mm10;
    int16_t turn_l_cdeg, turn_r_cdeg, final_heading_cdeg;   /* v3: outermost ladder/PRE heading */
    int16_t net_x_mm10, net_y_mm10;
    uint16_t flags, correction_count;
    int16_t correction_total_cdeg;
    int16_t l_in_mm10, l_far_mm10, r_in_mm10, r_far_mm10;  /* union brackets, axis x */
    uint16_t width_lo_mm10, width_hi_mm10;
    int16_t min_front_gap_mm10;   /* footprint extents: decoder, from records */
} B5WPSummary;

typedef struct {
    uint8_t out_cls, ret_cls;
    bool visited, revisited;
    float path_mm, out_axis, ret_axis;
} B5WPStop;

typedef struct {
    uint8_t state;
    bool boundary, contradictory, transition, cap_budget, cap_clear, cap_steps;
    bool return_ok, ret_in_ok;
    uint8_t limit;                 /* B5WP_LIM_*: why the outward pass ended */
    float clear_total_mm;          /* clearance-allowed outward travel at the last plan (out + remaining) */
    float min_turn_gap;            /* smallest modelled worst-turn gap at a visited outward stop */
    unsigned steps, last, ret_k;   /* last = outermost visited stop; ret_k = stop being revisited */
    int in_stop, far_stop, last_in; /* stop indices, -1 = none */
    float path_mm, out_mm, turn_deg;
    float in_x, far_x, ret_in_x;
    B5WPStop stop[B5WP_MAX_STOPS];
} B5WPSide;

typedef struct {
    uint32_t ms;
    uint16_t run, expected_mm;
    int16_t heading_cdeg, applied_dd;
    uint8_t phase, side, step, attempt, cls[3];
    uint16_t value[3];
} B5WPRetry;

typedef struct {
    B5SensorPort sensor;
    int (*applied)(void); /* applied servo PWM command, never pose feedback */
    unsigned state, sub, end, cur;  /* cur: 0 = left side, 1 = right side */
    uint32_t lease, command;
    uint64_t began, since, ping_since, sample_since;
    float heading_deg, move_start_heading, seg_cmd_mm, seg_meas_mm;
    uint16_t d0, run;
    b5_ping_t ping[3];
    unsigned npings;
    bool command_active, pose_timing, ping_active, pose_trimmed;
    unsigned trim_return_state, correction_count;
    int16_t correction_total_cdeg;
    float trim_target_deg;
    int servo_dd;
    uint8_t sample_phase, sample_step;
    uint8_t after_phase, after_step;  /* pose to sample when the pending turn completes */
    unsigned after_state;
    uint32_t sample_ms;
    int16_t sample_heading_cdeg, sample_servo_dd, sample_applied_dd;
    float net_x_mm, net_y_mm, turn_back_deg;
    B5WPSide side[2];
    bool ref_ok, mid_ok, final_ref_ok, servo_in_range;
    bool allow_borderline_ref;   /* RTOS mission already warned on a short echo */
    float last_axis_x;              /* axis x of the most recent record */
    float min_front_gap;
    float width_lo, width_hi;
    uint8_t width_result;
    B5WPRec rec[B5WP_QUEUE_CAP];
    uint16_t rec_head, rec_count, rec_index, lost;
#ifdef APP_RTOS_BUDDY5
    B5WPRec terminal_record; /* retained rejected triplet after FIFO drains */
    bool terminal_record_valid;
#endif
    B5WPSummary summary;
    bool summary_pending;
    /* width_calibrated_probe v0.1: display estimate, never used by motion */
    B5WEInput est_in;
    B5WEResult result;
    uint32_t result_ms;
    bool result_pending;
    bool pose_retried;
    B5WPRetry retries[B5WP_RETRIES_PER_RUN];
    unsigned retry_count, retry_sent;
    bool limits_pending;          /* D5 after D2 */
    /* v0.1.1 logging only: a copy of every pushed record, independent of the radio queue */
#if B5WP_RUN_ARCHIVE
    B5WPRec arch[B5WP_MAX_RECORDS];
#endif
    uint16_t arch_n, arch_lost;
} B5WidthProbe;

void b5_wp_init(B5WidthProbe *t, const B5SensorPort *port);
bool b5_wp_ready(const B5WidthProbe *t);
bool b5_wp_begin(B5WidthProbe *t, uint64_t now);
void b5_wp_tick(B5WidthProbe *t, uint64_t now);
void b5_wp_cancel(B5WidthProbe *t, uint64_t now);
void b5_wp_reset(B5WidthProbe *t);
bool b5_wp_peek(const B5WidthProbe *t, B5WPRec *out);
void b5_wp_drop(B5WidthProbe *t);
const char *b5_wp_state_name(unsigned state);
const char *b5_wp_end_name(unsigned end);

/* Pure model helpers (also mirrored by the Python decoder). */
float b5_wp_axis_x(float net_x_mm, float heading_deg, float applied_deg, float range_mm);
float b5_wp_front_gap(float d0_mm, float pivot_y_mm, float heading_deg);
float b5_wp_turn_front_gap(float d0_mm, float pivot_y_mm);
float b5_wp_next_step(float out_mm, unsigned steps); /* ceiling only; 0 = ceiling/step bound reached */
/* Outward travel still allowed by the clearance model from the current integrated pose
 * (before the overshoot reserve); see width_probe.c. */
float b5_wp_clear_travel(float d0_mm, float net_y_mm, float heading_deg);
bool  b5_wp_segment_ok(float command_mm, float measured_mm);
/* REF-plane expected range from quantized wire values (heading cdeg, net y 0.1 mm). */
uint16_t b5_wp_expected_mm(uint16_t d0, int16_t heading_cdeg, int16_t net_y_mm10);
/* v3 triplet rule: IN / FAR / TRANSITION (clean mix) / UNKNOWN (fault, invalid, near). */
uint8_t b5_wp_classify(const b5_ping_t p[3], uint16_t expected_mm);

#endif
