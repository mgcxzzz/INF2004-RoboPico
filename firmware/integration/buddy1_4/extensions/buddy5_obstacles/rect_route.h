#ifndef B5_RECT_ROUTE_H
#define B5_RECT_ROUTE_H
/* Automatic v0.3 rectangular bypass geometry and return-line acquisition.
 * Pure arithmetic: no motor, sensor or coordinator access.
 *
 * Frame: origin = drive-axle midpoint (in-place pivot) at the scan pose,
 * +y = original approach heading, +x = towards the SELECTED bypass side.
 * Left and right routes are mirror images in this frame.
 *
 * All body dimensions are the CONFIGURED provisional values in b5_config.h,
 * not new measurements. Sonar face is on the centreline. The line sensors are
 * assumed at the front IR bar tip. The obstacle is assumed to be a rectangle
 * no wider than 200 mm and no deeper than 200 mm whose near face is the
 * forward echo. The acoustic span (direction.span_mm) is NOT used. */
#include <stdbool.h>
#include <stdint.h>
#include "extensions/buddy5_obstacles/profile/obstacle_profile.h"
#include "extensions/buddy5_obstacles/profile/b5_config.h"

#define B5R_HALF_WIDTH_MM   ((float)B5_CAR_WIDTH_MM * 0.5f)                         /* 67.5 */
#define B5R_PIVOT_FRONT_MM  ((float)B5_PIVOT_TO_FRONT_MM)                           /* 130 */
#define B5R_PIVOT_REAR_MM   ((float)(B5_CAR_LENGTH_MM - B5_PIVOT_TO_FRONT_MM))      /* 60 */
#define B5R_SONAR_AHEAD_MM  ((float)(B5_PIVOT_TO_FRONT_MM - B5_SENSOR_BEHIND_FRONT_MM)) /* 65 */
#define B5R_LINE_AHEAD_MM   B5R_PIVOT_FRONT_MM
#define B5R_MARGIN_MM       ((float)B5_MARGIN_MM)                                   /* 50 */
#define B5R_RANGE_ALLOW_MM  ((float)B5_PROFILE_RANGE_ALLOW_MM)                      /* 10 */
#define B5R_ANGLE_ALLOW_DD  (B5_PROFILE_SERVO_ALLOW_DD)                             /* 4 deg */
#define B5R_OBST_WIDTH_MM   ((float)B5_EXPECTED_MAX_WIDTH_MM)                       /* 200 */
#define B5R_OBST_DEPTH_MM   ((float)B5_OBST_DEPTH_MM)                               /* 200 (full build) */
#define B5R_GEOM_DEPTH_MM   (150.0f) /* v0.4 geometry-only test: obstacle <= 200 W x 150 D */
#define B5R_SIDE_MM         (250.0f) /* provisional; requested demo value */
#define B5R_BACKOFF_MAX_MM  ((float)B5_REVERSE_MM)  /* larger need -> BLOCKED, not a long reverse */
#define B5R_RETURN_WINDOW_MM (100.0f) /* line may be this much beyond the expected crossing */
#define B5R_RETURN_MAX_MM   (300.0f)  /* commanded cap of the return leg */
#define B5R_ADVANCE_MAX_MM  (200.0f)

typedef struct {
    unsigned forward_range_mm;       /* stopped forward sonar range used */
    float near_y_mm, far_y_mm;       /* conservative obstacle faces (pivot frame) */
    float sweep_radius_mm;           /* in-place turn: farthest body corner */
    float rear_corner_radius_mm;     /* in-place turn: rear corners */
    float obstacle_depth_mm;         /* depth limit the plan was sized for */
    float backoff_mm;                /* 0 unless the measured points need it (never when
                                        reverse is disallowed) */
    float reverse_needed_mm;         /* minimum reverse the first turn would need */
    bool turn_clearance_ok;          /* first-turn sweep keeps MARGIN at the stop pose */
    float first_turn_clear_mm;       /* min sonar point distance - sweep radius */
    float side_mm, forward_mm, pass_y_mm;
    float rear_past_far_mm, side_past_far_mm;
    float max_offset_mm;             /* obstacle centre shift towards the pass side
                                        still giving MARGIN beside the forward leg */
    float return_expect_mm, return_search_mm, return_max_mm;
} B5RectPlan;

/* forward_range_mm: stopped sonar range straight ahead (direction.range_mm).
 * scan: every echo in the safety scan contributes to the first-turn backoff.
 * Returns false when the plan would need >150 mm reverse or exceeds API caps. */
bool b5_rect_plan(const b5_scan_t *scan, unsigned forward_range_mm, B5RectPlan *out);
/* As above with an explicit depth limit. allow_reverse=false (v0.4 geometry
 * test): backoff is always 0; turn_clearance_ok reports whether the first
 * in-place turn keeps MARGIN from every echo at the ACTUAL stopped pose. The
 * caller must hold stopped when it is false; the margin is never reduced. */
bool b5_rect_plan_ex(const b5_scan_t *scan, unsigned forward_range_mm, float depth_mm,
                     bool allow_reverse, B5RectPlan *out);

/* ---- Return-line acquisition (both front reflectance sensors) ----
 * Normalised calibrated levels 0(white)..1000(black). Saved calibration is
 * only read. Evidence required, in order, within a bounded travel:
 *   1. baseline: first 30 mm of the return leg, spread <=150 on each sensor and
 *      at least 300 below full scale (a uniformly dark floor is rejected here);
 *   2. rise: each sensor >= max(400, baseline+300) for 3 consecutive samples;
 *   3. fall: each sensor back <= baseline+150 for 3 samples after 5..80 mm;
 *   4. both sensors' stripe centres agree within 40 mm.
 * No rise by the search distance, a rise without a fall (edge of a dark
 * region), unstable background or disagreement -> rejection. */
enum { B5L_PENDING, B5L_FOUND, B5L_NO_RISE, B5L_EDGE, B5L_UNSTABLE,
       B5L_LOW_CONTRAST, B5L_MISMATCH, B5L_NO_SAMPLES };
#define B5L_ABS_LEVEL     (400)
#define B5L_CONTRAST      (300)
#define B5L_FALL_MARGIN   (150)
#define B5L_BASE_MM       (30.0f)
#define B5L_BASE_SAMPLES  (10U)
#define B5L_BASE_SPREAD   (150)
#define B5L_CONFIRM_N     (3U)
#define B5L_WIDTH_MIN_MM  (5.0f)
#define B5L_WIDTH_MAX_MM  (80.0f)
#define B5L_SKEW_MM       (40.0f)
typedef struct {
    unsigned phase, count; /* 0 background, 1 on stripe, 2 crossed, 3 edge */
    float first, rise, fall;
} B5LineEdge;
typedef struct {
    float sum[2];
    uint16_t lo[2], hi[2], base[2];
    unsigned n, result;
    bool based;
    B5LineEdge s[2];
    float centre, search_mm;
} B5LineCross;
void b5_cross_init(B5LineCross *c, float search_mm);
unsigned b5_cross_step(B5LineCross *c, float progress_mm, uint16_t left, uint16_t right);
/* Stationary/turning confirmation against the SAME baseline. */
bool b5_cross_evidence(const B5LineCross *c, uint16_t left, uint16_t right);
const char *b5_cross_name(unsigned result);
#endif
