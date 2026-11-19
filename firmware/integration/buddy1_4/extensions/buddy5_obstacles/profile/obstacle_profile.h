/**
 * @file    obstacle_profile.h
 * @brief   Obstacle profiling and bypass planning (pure logic, no RTOS).
 *
 * This module turns a set of (angle, range) samples into an obstacle
 * profile and a bypass plan. It has no hardware or kernel dependency, so
 * it is unit-tested on the host (see test/).
 *
 * Coordinate frame, seen from the sensor looking forward. Angles are
 * signed tenths of a degree with the same sign rule as Motion Control's
 * heading (+ = right):
 *   angle  -900 dd = left, 0 dd = straight ahead, +900 dd = right
 *   x = r * sin(angle)  (positive to the RIGHT)
 *   y = r * cos(angle)  (positive AHEAD)
 *
 * Minimum-bit representation (Tribunal 1): samples, profiles and plans
 * are C bit-field records sized to each value's valid range (b5_pack.h).
 * They are written only by the obstacle task; anything another task
 * reads is a copy. Values are clamped before they are stored.
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#ifndef B5_PROFILE_H
#define B5_PROFILE_H

#include <stdbool.h>
#include <stdint.h>

#include "b5_pack.h"

#define B5_MAX_SAMPLES      (48U)   /* dense sweep (21) + refinement + spare */

/** One range sample taken at one servo angle: 28 bits in one word.
 *  b_valid = 1 means the angle has a result: an echo range (b_weak = 0)
 *  or a weak no-return clearance (b_weak = 1, range FAR).
 *  b_partial = 1: an echo was seen and ONE ping faulted; the range blocks,
 *  and space nearer than it is only weak clearance evidence.
 *  b_block_only = 1: an echo was seen but two or more pings faulted; the
 *  range blocks, and the angle is otherwise unknown (never clearance). */
typedef struct
{
    int32_t  angle_dd  : 11; /**< -900..+900, tenths of a degree        */
    uint32_t range_mm  : 12; /**< nearest echo; B5_RANGE_FAR_MM if none  */
    uint32_t b_valid   : 1;  /**< 0 = unknown (faults / incomplete)      */
    uint32_t b_stable  : 1;  /**< every ping echoed, spread within limit */
    uint32_t b_weak    : 1;  /**< clean no-returns only: weak clearance  */
    uint32_t b_partial : 1;  /**< echo + 1 fault: blocks; nearer = weak  */
    uint32_t b_block_only : 1; /**< echo + >=2 faults: blocks, else unknown */
} b5_sample_t;

/** Outcome class of one ping (rule B). */
typedef enum
{
    B5_PING_ECHO = 0,       /**< measured echo, range valid            */
    B5_PING_NO_RETURN,      /**< completed measurement, nothing heard  */
    B5_PING_FAULT           /**< sensor / transaction fault            */
} b5_ping_class_t;

/** Why a ping is a fault (diagnostics). */
typedef enum
{
    B5_PING_WHY_OK = 0,
    B5_PING_WHY_STUCK_HIGH, /**< ECHO high before TRIG, never went low  */
    B5_PING_WHY_NO_RISE,    /**< no rising edge (silent / disconnected) */
    B5_PING_WHY_NO_FALL,    /**< rose but never fell inside the window  */
    B5_PING_WHY_SHORT       /**< only pulses shorter than ECHO_MIN      */
} b5_ping_why_t;

/** One classified ping: 34 bits. */
typedef struct
{
    uint32_t cls      : 2;  /**< b5_ping_class_t                       */
    uint32_t why      : 3;  /**< b5_ping_why_t                         */
    uint32_t range_mm : 12; /**< ECHO only; FAR otherwise              */
    uint32_t width_us : 17; /**< measured width, saturated (diagnostic)*/
} b5_ping_t;

/** Ping classes seen at one angle (diagnostics and tests). */
typedef struct
{
    uint8_t n_echo;
    uint8_t n_no_return;
    uint8_t n_fault;
} b5_ping_counts_t;

/** All samples of one scan, kept sorted by angle. */
typedef struct
{
    b5_sample_t sample[B5_MAX_SAMPLES];
    uint8_t     count;
} b5_scan_t;

/** Tunables the profiler needs (copied from b5_config.h by the caller). */
typedef struct
{
    int32_t near_mm;            /**< range below which a sample is "near" */
    int16_t alpha_dd;           /**< beam half-angle                     */
    int16_t scan_min_dd;        /**< lowest angle the sweep reaches      */
    int16_t scan_max_dd;        /**< highest angle the sweep reaches     */
    int32_t car_width_mm;
    int32_t car_length_mm;
    int32_t margin_mm;
    int32_t obst_depth_mm;
    int32_t open_clearance_mm;
    bool    b_prefer_right;
} b5_params_t;

/** Which way to go round the obstacle. */
typedef enum
{
    B5_SIDE_NONE = 0,
    B5_SIDE_RIGHT,
    B5_SIDE_LEFT
} b5_side_t;

typedef enum
{
    B5_DIRECTION_NO_FRONT = 0,
    B5_DIRECTION_INCOMPLETE,
    B5_DIRECTION_CLIPPED,
    B5_DIRECTION_UNSTABLE,
    B5_DIRECTION_AMBIGUOUS,
    B5_DIRECTION_EDGES,
    B5_DIRECTION_CONFLICT,
    B5_DIRECTION_TENTATIVE           /* small apparent lean; not a reliable offset */
} b5_direction_reason_t;

/* Apparent front-return arc, separate from physical width and path safety.
 * direction is the suggested BYPASS side (NONE = UNKNOWN offset evidence).
 * preferred_side includes the configured fallback, never permission to move. */
typedef struct
{
    uint32_t span_mm          : 12;
    uint32_t stable_span_mm   : 12;
    uint32_t samples          : 6;

    int32_t left_edge_dd      : 11;
    int32_t right_edge_dd     : 11;
    uint32_t direction        : 2;
    uint32_t preferred_side   : 2;
    uint32_t reason           : 3;

    int32_t bias_dd           : 11; /* arc midpoint, + suggests LEFT */
    uint32_t uncertainty_dd   : 9; /* model band; not calibrated accuracy */
    uint32_t range_mm         : 12;
} b5_direction_t;

/* Evidence flags for the conservative navigation result. */
#define B5_PROFILE_EVIDENCE_SUPPORTED   (1U << 0)
#define B5_PROFILE_EVIDENCE_SCAN_LEFT   (1U << 1)
#define B5_PROFILE_EVIDENCE_SCAN_RIGHT  (1U << 2)
#define B5_PROFILE_EVIDENCE_CLEAR_SIDE  (1U << 3)
#define B5_PROFILE_EVIDENCE_WEAK_CLEAR  (1U << 4)  /* chosen side relied on no-return rays */
#define B5_PROFILE_EVIDENCE_SWEEP_ASSUMED (1U << 5) /* chosen side's footprint left the scan */
#define B5_PROFILE_EVIDENCE_SWEEP_FAIL_L (1U << 6) /* left manoeuvre footprint failed */
#define B5_PROFILE_EVIDENCE_SWEEP_FAIL_R (1U << 7) /* right manoeuvre footprint failed */

/** Swept-footprint result for one side. */
typedef enum
{
    B5_SWEEP_NOT_RUN = 0,   /**< side had too little clearance to try  */
    B5_SWEEP_FAIL,          /**< footprint meets an obstacle / unknown */
    B5_SWEEP_PASS_ASSUMED,  /**< passes, but parts were never scanned  */
    B5_SWEEP_PASS           /**< every footprint point in seen free space */
} b5_sweep_result_t;

/** Why a footprint failed (first failing point). */
typedef enum
{
    B5_SWEEP_WHY_NONE = 0,
    B5_SWEEP_WHY_ENVELOPE,  /**< overlaps the planning envelope        */
    B5_SWEEP_WHY_RETURN,    /**< a measured return lies inside it      */
    B5_SWEEP_WHY_BEYOND,    /**< reaches behind a measured side return */
    B5_SWEEP_WHY_UNKNOWN,   /**< scanned but no usable evidence there  */
    B5_SWEEP_WHY_POLICY     /**< weak/assumed evidence not allowed     */
} b5_sweep_why_t;

/** Detail of one side's footprint check (diagnostics). */
typedef struct
{
    uint8_t  result;        /**< b5_sweep_result_t                       */
    uint8_t  why;           /**< b5_sweep_why_t                          */
    uint8_t  b_weak;        /**< relied on a weak (no-return) ray        */
    uint8_t  fail_leg;      /**< 1 spin out, 2 diagonal, 3 spin back,
                                 4 pass, 5 rejoin spin, 6 rejoin leg     */
    int16_t  fail_x_mm;     /**< first failing point, sensor frame       */
    int16_t  fail_y_mm;
    uint16_t n_points;      /**< outline points checked                  */
    uint16_t n_assumed;     /**< ... in unscanned / occluded space       */
    uint16_t n_weak;        /**< ... free only by a no-return ray        */
} b5_sweep_report_t;

/**
 * Result of Planning: what the obstacle looks like and what to do.
 * Legacy geometry plus explicit conservative navigation extents.
 */
typedef struct
{
    /* Grouped so no field straddles a 32-bit unit: 8 words exactly. */
    uint32_t b_found          : 1;  /**< at least one near sample        */
    uint32_t b_right_at_limit : 1;  /**< obstacle runs past the scan arc */
    uint32_t b_left_at_limit  : 1;
    uint32_t side             : 2;  /**< b5_side_t, NONE = blocked       */
    uint32_t confidence_pct   : 7;  /**< share of samples with a MEASURED
                                         echo (weak clears excluded)     */
    uint32_t coverage_pct     : 7;  /**< share with any usable result    */
    uint32_t closest_mm       : 12; /**< closest range in the run        */

    int32_t  right_edge_dd    : 11; /**< raw right edge (bracket middle) */
    int32_t  left_edge_dd     : 11; /**< raw left edge (bracket middle)  */
    uint32_t sweep_left       : 2;  /**< b5_sweep_result_t               */
    uint32_t sweep_right      : 2;

    int32_t  closest_dd       : 11;
    int32_t  x_right_mm       : 12; /**< planning right bound, lateral   */
    uint32_t evidence_flags   : 8;  /**< B5_PROFILE_EVIDENCE_*           */

    int32_t  x_left_mm        : 12; /**< planning left bound, lateral    */
    uint32_t width_raw_mm     : 12; /**< width before beam correction    */

    uint32_t width_mm         : 12; /**< legacy corrected / nav observed */
    uint32_t clear_right_mm   : 12; /**< free lateral space right of it  */

    uint32_t clear_left_mm    : 12; /**< free lateral space left of it   */
    uint32_t lateral_mm       : 12; /**< sideways shift for the bypass   */

    uint32_t forward_mm       : 12; /**< forward travel to pass + margin */
    uint32_t span_uncertainty_mm : 12; /**< total model growth, not accuracy */

    uint32_t planning_width_mm   : 12; /**< conservative bypass envelope */
    uint32_t stable_span_mm      : 12; /**< span of STABLE near returns only
                                            (provisional; not planning width) */
    /* Diagnostic-only: record the branch actually taken, not a re-evaluation. */
    uint32_t decision_branch     : 3; /* 0 none, 1 direction, 2 tie, 3 gap, 4 L, 5 R */
    uint32_t decision_left_ok    : 1;
    uint32_t decision_right_ok   : 1;
    uint32_t decision_sweep      : 1;
} b5_profile_t;

/* Trigonometry in Q15 fixed point (32768 = 1.0), angle in tenths of deg. */
int32_t b5_cos_q15(int32_t angle_dd);
int32_t b5_sin_q15(int32_t angle_dd);

/* Read-only: every original sample remains available to the path checks. */
void b5_estimate_direction(b5_scan_t const * p_scan,
                           b5_params_t const * p_param,
                           b5_direction_t * p_direction);
char const * b5_direction_reason_name(uint32_t reason);

/* Scan bookkeeping. */
void    b5_scan_clear(b5_scan_t * p_scan);
bool    b5_scan_add(b5_scan_t * p_scan, int16_t angle_dd,
                    int32_t range_mm, bool b_valid);
bool    b5_scan_add_sample(b5_scan_t * p_scan, b5_sample_t const * p_s);
int32_t b5_median3(int32_t a, int32_t b, int32_t c);

/* Rule B: classify one ping from what the driver observed, then combine
 * the pings at one angle into a sample. Pure: shared by the RTOS task,
 * the SDK bench and the host tests. */
b5_ping_t b5_classify_ping(bool b_pretrig_low, bool b_rise, bool b_fall,
                           bool b_short_seen, uint32_t width_us);
void    b5_combine_pings(int16_t angle_dd, b5_ping_t const * p_ping,
                         uint8_t n_pings, b5_sample_t * p_out,
                         b5_ping_counts_t * p_counts);

/* Perception ONLY: median of three classified pings, with a majority of
 * echoes agreeing within the spread allowance. Any fault stays unknown.
 * Here b_stable means majority echo consensus, not all-ping agreement.
 * Never substitute this result for b5_combine_pings in a safety scan. */
void    b5_combine_perception_pings(int16_t angle_dd,
                                    b5_ping_t const * p_ping, uint8_t n_pings,
                                    b5_sample_t * p_out);

/* Prints the apparent OBSTACLE position, opposite to the bypass hint.
 * UNKNOWN includes a centred arc: neither is a detected lateral offset. */
char const * b5_obstacle_position_name(b5_direction_t const * p_direction);

/* Flat-face projection using centre range and apparent edge angles. Returns
 * zero if a forward echo or complete edge brackets are unavailable. This is
 * an apparent acoustic span, not a beam-corrected physical measurement. */
uint16_t b5_centre_span_estimate(b5_scan_t const * p_perception,
                                 b5_direction_t const * p_direction);

/* Coarse Sweep -> Edge Refinement decision. */
bool    b5_scan_has_near(b5_scan_t const * p_scan, int32_t near_mm);
bool    b5_next_refine_angle(b5_scan_t const * p_scan, int32_t near_mm,
                             int16_t alpha_dd, int16_t * p_angle_dd);

/* Bounded acquisition history, angles rather than indices (insertion shifts
 * indices). Fixed-policy perception only; never reinterpret unknown as clear. */
typedef struct
{
    int16_t angle_dd[6];
    uint8_t count;
} b5_edge_retry_t;
bool b5_next_edge_retry(b5_scan_t const * p_perception, int32_t near_mm,
                         b5_edge_retry_t * p_retry, int16_t * p_angle_dd);
/* Merge a repeat conservatively: retain every nearer echo and uncertainty.
 * Repeat observations can resolve perception, never erase safety evidence. */
bool b5_scan_add_retry(b5_scan_t * p_safety, b5_sample_t const * p_repeat);

/* Planning. */
void    b5_build_profile(b5_scan_t const * p_scan,
                         b5_params_t const * p_param,
                         b5_profile_t * p_prof);

/* Conservative navigation estimate; leaves the legacy profiler unchanged. */
void    b5_build_navigation_profile(b5_scan_t const * p_scan,
                                    b5_params_t const * p_param,
                                    b5_profile_t * p_prof);

void    b5_build_navigation_profile_ex(b5_scan_t const * p_scan,
                                       b5_params_t const * p_param,
                                       b5_profile_t * p_prof,
                                       b5_sweep_report_t * p_rep_l,
                                       b5_sweep_report_t * p_rep_r);

/* Width/envelope/clearance/footprint still use EVERY conservative sample.
 * The separate median scan supplies the front-arc observation only. A
 * supported direction may choose among two passing paths, never revive
 * a rejected one. NULL perception falls back to the conservative scan. */
void    b5_build_observed_navigation_profile(b5_scan_t const * p_safety,
                                             b5_scan_t const * p_perception,
                                             b5_params_t const * p_param,
                                             b5_profile_t * p_prof,
                                             b5_direction_t * p_direction,
                                             b5_sweep_report_t * p_rep_l,
                                             b5_sweep_report_t * p_rep_r);

/* Fixed 45-degree bypass policy requested for the demo: enforce the existing
 * measured side-gap checks, report the swept-footprint model as ADVISORY.
 * Explicit opt-in: existing strict entry points above are not changed.
 * An apparent lean above half a coarse sample step may supply a TENTATIVE
 * preference between two gap-usable sides; it is never called a trusted offset.
 * A chosen side is a provisional fixed-route plan, not proof of a safe path. */
void    b5_build_fixed_bypass_profile(b5_scan_t const * p_safety,
                                      b5_scan_t const * p_perception,
                                      b5_params_t const * p_param,
                                      b5_profile_t * p_prof,
                                      b5_direction_t * p_direction,
                                      b5_sweep_report_t * p_rep_l,
                                      b5_sweep_report_t * p_rep_r);

/* v0.6.3 DIAGNOSTIC ONLY: why the side-gap test returned its value in the
 * last nav_build_profile_policy() call. Recording it never changes a result.
 * reason: the zero/limit cause, or (non-zero result) why the outward walk
 * stopped. angle_dd/range_mm/flags: the offending (or last walked) safety
 * sample; value: reason-specific (forward depth y mm for SHALLOW, angular
 * gap dd for *_GAP, lateral overlap mm for OVERLAP/LIMITED). */
enum
{
    B5_CLR_NOT_RUN = 0,      /* no supported obstacle: side not evaluated */
    B5_CLR_AT_LIMIT,         /* obstacle (near) sample at the sweep limit   */
    B5_CLR_NO_START,         /* no sample beyond the obstacle edge          */
    B5_CLR_START_UNUSABLE,   /* first sample beyond the edge unknown/block   */
    B5_CLR_START_WINDOW,     /* ... outside the symmetric comparison window  */
    B5_CLR_START_GAP,        /* ... more than MAX_GAP from the edge          */
    B5_CLR_WALK_UNUSABLE,    /* walk: unknown/block-only sample              */
    B5_CLR_WALK_GAP,         /* walk: angular gap too large / out of order   */
    B5_CLR_WALK_WINDOW,      /* walk: sample outside the comparison window   */
    B5_CLR_WALK_SHALLOW,     /* walk: return nearer than corridor depth      */
    B5_CLR_WALK_PLANE,       /* walk: ray does not reach the corridor plane  */
    B5_CLR_SCAN_END,         /* walk reached the last sample (sweep end)     */
    B5_CLR_OVERLAP,          /* outer return inside the envelope: zero       */
    B5_CLR_LIMITED           /* outer return in the corridor limits the gap  */
};
#define B5_CLR_ANGLE_NONE (0x7FFF)
typedef struct
{
    uint8_t  reason;
    uint8_t  pairs;          /* adjacent pairs accepted by the walk         */
    uint8_t  flags;          /* sample: b0 valid b1 weak b2 partial b3 block b4 stable */
    uint8_t  edge_index;     /* safety-scan index of the obstacle edge       */
    int16_t  edge_dd;        /* obstacle edge sample angle (safety scan)     */
    int16_t  near_edge_x;    /* envelope edge x, mm (sensor frame, +right)   */
    int16_t  corridor_y;     /* forward depth the walk must clear, mm        */
    int16_t  angle_dd;       /* offending / last sample, B5_CLR_ANGLE_NONE   */
    uint16_t range_mm;
    int16_t  value;
    int16_t  result_mm;      /* value returned (before the at-limit rule)    */
    int16_t  lim_dd;         /* comparison window used                       */
} b5_clear_diag_t;
/* Copy of the last decision's left [0] / right [1] diagnosis. */
void b5_clearance_diag(b5_clear_diag_t p_out[2]);

/* Swept-footprint check of the bypass manoeuvre on one side, using the
 * envelope in p_prof. search_mm is the first rejoin search extension. */
uint8_t b5_check_sweep(b5_scan_t const * p_scan, b5_params_t const * p_param,
                       b5_profile_t const * p_prof, b5_side_t side,
                       int32_t lateral_mm, int32_t forward_mm,
                       int32_t search_mm, b5_sweep_report_t * p_rep);

#endif /* B5_PROFILE_H */
