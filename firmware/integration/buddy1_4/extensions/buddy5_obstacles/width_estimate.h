#ifndef B5_WIDTH_ESTIMATE_H
#define B5_WIDTH_ESTIMATE_H
/* On-Pico width estimate for the width-probe v0.2.1 motion (width_calibrated_probe v0.1).
 * DISPLAY ESTIMATE ONLY: nothing in the firmware reads this result for motion,
 * clearance, bypass or handback.
 *
 * Exact integer port of the frozen stepped range-rise replay
 * (width_method_review_2026_11_18/replay_range_walk.py, SHA256 f2d73424...9207,
 * width_probe_json/onset/rel/two_sided) and of the calibration rule in
 * width_calibrated_v0_1/estimate_width.py (SHA256 c0c654d9...6c70):
 *
 *  residual per stopped record = median(clean+unclean ECHO values, >= 2 needed)
 *      - firmware expected range; > 150 mm -> F; fewer than 2 echoes -> excluded.
 *  left baseline  = median residual of REF/MID/FINAL;
 *  right baseline = median residual of right records with |axis x| < 60 mm
 *                   (else the left/centre baseline).
 *  points per side sorted by (|axis x|, residual); HIGH = F or residual-baseline >= 10 mm;
 *  onset = first point after which every point stays HIGH:
 *      i = 0 -> HIGH_FROM_START [0, x0]; none -> WALK_FREE (reach = last |x|);
 *      else ONSET [x(i-1), x(i)].
 *  A  both ONSET, complete consistent run, MID and FINAL IN:
 *      raw = mean(La+Ra, Lb+Rb); display = clamp(raw - 10, 100, 200).
 *  B  exactly one WALK_FREE side, the other ONSET, the WALK_FREE side exhausted its
 *      travel budget, its outermost outward triplet IN, its retrace accepted, state
 *      NOT_REACHED, and 100 <= sampled span <= 200:
 *      sampled = near onset inner end + far reach; display = (sampled + 200) / 2.
 *  C  anything else: display 150 (project range midpoint), reason preserved.
 *
 * Units: positions mm10 (as recorded on the wire), residuals quarter-mm (exact for
 * medians of integer echoes), widths mm100 (exact for the mean of two tenths). */
#include <stdbool.h>
#include <stdint.h>

#define B5WE_MAX_POINTS   48u
#define B5WE_HIGH_Q       40      /* 10 mm, quarter-mm units */
#define B5WE_FAR_Q        600     /* residual > 150 mm is F */
#define B5WE_RIGHT_BASE_X10 600   /* right baseline window: |x| < 60 mm */
#define B5WE_OFFSET_MM100 1000    /* calibrated correction 10 mm */
#define B5WE_MIN_MM100    10000   /* project range 100..200 mm */
#define B5WE_MAX_MM100    20000
#define B5WE_MID_MM100    15000
#define B5WE_NONE         0xFFFFu
#define B5WE_RULE_ID      1u      /* frozen replay f2d73424 + estimate_width c0c654d9 */

enum { B5WE_K_NONE, B5WE_K_NUM, B5WE_K_FAR };
enum { B5WE_PSIDE_NONE, B5WE_PSIDE_L, B5WE_PSIDE_R };
enum { B5WE_NO_DATA, B5WE_ONSET, B5WE_HIGH_FROM_START, B5WE_WALK_FREE };
enum { B5WE_BASIS_NONE, B5WE_BASIS_CALIBRATED_RANGE_RISE, B5WE_BASIS_EMPIRICAL_REACH_COMPLETION,
       B5WE_BASIS_PROJECT_RANGE_ASSUMPTION };
enum { B5WE_STATUS_NONE, B5WE_STATUS_ESTIMATE, B5WE_STATUS_REACH_LIMIT, B5WE_STATUS_INVALID };
enum { B5WE_REASON_NONE, B5WE_REASON_NO_RECORDS, B5WE_REASON_INCONSISTENT,
       B5WE_REASON_RUN_OR_REFERENCE, B5WE_REASON_EDGES_UNRESOLVED };
enum {
    B5WE_F_LIMIT_APPLIED = 1u << 0,       /* A: project range changed the display */
    B5WE_F_COMPLETION_MET = 1u << 1,      /* B conditions all met */
    B5WE_F_L_BUDGET = 1u << 2, B5WE_F_R_BUDGET = 1u << 3,
    B5WE_F_L_RETURN_OK = 1u << 4, B5WE_F_R_RETURN_OK = 1u << 5,
    B5WE_F_L_NOT_REACHED = 1u << 6, B5WE_F_R_NOT_REACHED = 1u << 7,
    B5WE_F_L_LAST_OUT_IN = 1u << 8, B5WE_F_R_LAST_OUT_IN = 1u << 9,
    B5WE_F_MID_IN = 1u << 10, B5WE_F_FINAL_IN = 1u << 11,
    B5WE_F_CONSISTENT = 1u << 12,
    B5WE_F_SIDES_INCOMPLETE_RUN = 1u << 13,  /* brackets shown for an invalid run: info only */
    B5WE_F_SORT_TIE = 1u << 14,             /* equal |x| with F and number: reference undefined */
    B5WE_F_SAMPLED_OUT_OF_RANGE = 1u << 15
};

typedef struct {
    uint8_t side;      /* B5WE_PSIDE_* */
    uint8_t centre;    /* REF, MID or FINAL record */
    uint8_t kind;      /* B5WE_K_* */
    uint16_t absx10;   /* |axis x| in 0.1 mm, as recorded */
    int32_t res4;      /* residual in quarter-mm (K_NUM only) */
} B5WEPoint;

typedef struct {
    bool not_reached, last_out_in, return_ok, cap_budget;
} B5WESideIn;

typedef struct {
    uint8_t end;              /* width-probe end code; 1 = COMPLETE */
    bool consistent, mid_in, final_in;
    uint16_t records;
    B5WESideIn side[2];       /* 0 = left, 1 = right */
    uint16_t n;
    B5WEPoint pt[B5WE_MAX_POINTS];
} B5WEInput;

typedef struct {
    uint8_t status;           /* B5WE_NO_DATA .. B5WE_WALK_FREE */
    uint8_t n;                /* points used */
    uint16_t a10, b10;        /* ONSET [a,b]; HIGH_FROM_START [0,b]; WALK_FREE a = reach */
    int32_t base4;
} B5WESide;

typedef struct {
    uint8_t basis, status, reason;
    uint16_t display_mm100;
    uint16_t raw_mm100;       /* A: raw midpoint */
    uint16_t corrected_mm100; /* A: raw - 10 before the project limits */
    uint16_t sampled_mm10;    /* apparent sampled span (lower-bound arithmetic of the replay) */
    uint16_t span_lo_mm10, span_hi_mm10;   /* A: summed onset brackets */
    uint16_t flags;
    B5WESide side[2];
} B5WEResult;

/* Feature of one stopped record (exactly the replay's res()). */
B5WEPoint b5_we_point(uint8_t side, bool centre, const uint8_t cls[3], const uint16_t value[3],
                      uint16_t expected_mm, int16_t axis_x_mm10);
void b5_we_estimate(const B5WEInput *in, B5WEResult *out);

#endif
