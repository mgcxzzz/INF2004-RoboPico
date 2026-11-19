#ifndef B5_STRIPE_DETECT_H
#define B5_STRIPE_DETECT_H
/* v0.5 return-leg stripe detector. Pure; no hardware/coordinator access.
 *
 * Input: one sample per control tick = return-leg encoder progress (mm) and
 * the two front sensors' normalised calibrated levels (0 white .. 1000
 * black; saved calibration is only read). The robot drives towards the line
 * roughly perpendicular to it, so both sensors cross the stripe together.
 *
 * Evidence chain (thresholds chosen from the 2026-11-04 v0.4.1 return-leg
 * captures, where the off-mat floor read 0..104 and the stripe went to 1000
 * on both sensors for ~14-17 mm within ~2 mm of each other):
 *  1. Quiet background: >=15 mm of travel, >=5 samples, each sensor spread
 *     <=150 and max <=700. The latest quiet snapshot must end <=12 mm before a
 *     rise (allows the sensor's own ramp). A rolling window, not a fixed start
 *     window: a dark start (run4: right sensor fading from 1000) only delays.
 *  2. Rise: a sensor >= max(400, bg+300) for >=3 samples and >=2 mm. Shorter
 *     excursions are isolated spikes and are ignored.
 *  3. Both sensors rise within 20 mm of each other (single-sensor features,
 *     e.g. run3 at ~233 mm, are rejected).
 *  4. Each falls back to <= bg+150 for >=3 samples and >=2 mm, with width
 *     5..45 mm (steps / mat edges / long junction branches are rejected).
 *  5. Stripe centres agree within 20 mm; then 8 mm of clean background
 *     (rejects closely spaced multi-stripe texture such as barcodes).
 * A rejected candidate needs a fresh quiet background. No accepted rise by
 * search_mm -> NO_LINE. Two reflectance sensors cannot tell a course line from
 * any other isolated dark stripe of similar width. */
#include <stdbool.h>
#include <stdint.h>

enum { B5S_PENDING, B5S_FOUND, B5S_NO_LINE };
enum { B5SR_NONE, B5SR_SINGLE_SENSOR, B5SR_WIDE_OR_STEP, B5SR_NARROW, B5SR_SKEW,
       B5SR_NOT_CLEAN_AFTER };
#define B5S_QUIET_MM       (15.0f)
#define B5S_QUIET_SAMPLES  (5U)
#define B5S_QUIET_SPREAD   (150)
#define B5S_BG_MAX         (700)
#define B5S_SNAP_GAP_MM    (12.0f)
#define B5S_RISE_ABS       (400)
#define B5S_CONTRAST       (300)
#define B5S_FALL_MARGIN    (150)
#define B5S_SUSTAIN_N      (3U)
#define B5S_SUSTAIN_MM     (2.0f)
#define B5S_WIDTH_MIN_MM   (5.0f)
#define B5S_WIDTH_MAX_MM   (45.0f)
#define B5S_SKEW_MM        (20.0f)
#define B5S_POST_MM        (8.0f)

typedef struct {
    uint8_t st, cnt, fcnt;  /* st: 0 not risen, 1 on stripe, 2 fallen */
    float run, frun, rise, fall;
} B5StripeSensor;
typedef struct {
    float search_mm;
    /* rolling quiet window */
    float q_start;
    uint32_t q_sum[2];
    uint16_t q_min[2], q_max[2], q_n;
    bool snap_ok;
    float snap_p;
    uint16_t snap_bg[2];
    /* candidate */
    uint8_t phase;           /* 0 idle, 1 candidate, 2 clean-after */
    uint16_t bg[2];          /* background frozen for the candidate / result */
    float first_rise, post_start, post_run;
    uint8_t post_cnt;
    B5StripeSensor s[2];
    /* result and counters */
    uint8_t result, last_reject;
    uint16_t rejects, spikes, samples;
    float centre, width[2], last_p;
} B5Stripe;

void b5_stripe_init(B5Stripe *d, float search_mm);
unsigned b5_stripe_step(B5Stripe *d, float progress_mm, uint16_t left, uint16_t right);
/* Stationary/turning confirmation against the FOUND stripe's background. */
bool b5_stripe_evidence(const B5Stripe *d, uint16_t left, uint16_t right);
uint16_t b5_stripe_threshold(const B5Stripe *d, unsigned sensor);
const char *b5_stripe_result_name(unsigned r);
const char *b5_stripe_reject_name(unsigned r);
#endif
