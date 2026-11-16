/**
 * @file ramp_assist.h
 * @brief Ramp assist.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* v2.3 + Buddy4 ramp-assist v0.6 — decision logic (pure C, host-testable).
 *
 * CURRENT BEHAVIOUR (v0.6, hardware-tested 2026-10-31 on the 10 mm ramp):
 *  - Detect: Buddy4 side tilt |h| >= 4 mm for 4 consecutive valid 20 ms
 *    samples during ordinary line following (AUTO, continuous drive).
 *  - Assist (2.5 s max per ramp): +20 % of the existing feedforward (cap 40
 *    PWM), held through ACCEL/SPREAD-only exclusions up to 150 ms; ends on
 *    level, descent, invalid/stale tilt or timeout.
 *  - Stall push: +60 PWM on both wheels if both stop for 200 ms while
 *    assisting (1.5 s budget per ramp).
 *  - Speed hold (until the tilt has been flat for 200 ms, 5 s max): minimum
 *    speed target 35 mm/s per wheel, integral limit 300 and 3x integral gain
 *    in the existing continuous speed controller (continuous_drive.h hold_*).
 *  - Stop / Pause / ESTOP / line search / junction remove everything at once;
 *    the 0..420 PWM clamp and 1200/s slew are never changed.
 * Version history follows (newest first).
 *
 * v0.6 (2026-10-31): speed hold replaces the v0.5 300 PWM floor. Runs
 * 02:07-02:08 (v0.5): the fixed floor drove the car up the climb at 107-131
 * mm/s (3x the 45 target); ~0.5 s in, Buddy4 reported a non-ACCEL/SPREAD
 * invalid tilt, which ended the floor (single use); the car hit the crest
 * fast, nose on the mat (barcode ADC ~1000), stopped, and on the descent
 * steering starved the ramp wheel again (0-25 PWM) until the stall push.
 * Buddy4 rejected every hump (quality budget) at that speed.
 * v0.6: from activation, while the episode lasts, the continuous drive gets
 * (scaled by hold_level, ramped 0..1 in 0.2 s): a minimum speed target of
 * RA_HOLD_MIN_MM_S per wheel (steering can no longer ask a wheel to stop), an
 * integral limit of RA_HOLD_I_MAX instead of 100 and RA_HOLD_KI x the integral
 * gain, so the existing speed loop adds whatever PWM (to 420) it needs to
 * hold the speed instead of a fixed PWM. It ends when the filtered tilt has
 * been flat for 200 ms (episode end), after RA_HOLD_MAX_US per episode, on
 * stale Buddy4 (RA_HOLD_STALE_US) or invalid tilt of any reason longer than
 * RA_HOLD_INVALID_US (shorter invalid only keeps it); a re-activation in the
 * same episode restarts it within the budget. Stop/Pause/not following
 * remove it at once. +20 % assist, v0.3 hold and v0.4 push unchanged.
 *
 * v0.5 (2026-10-31): tilt floor. Run 01:50:15 (bracket tilted up; 4 of 5 runs
 * cleared): the final stall at 225 mm had the left wheel at 295 PWM but the
 * RAMP wheel at 12-51 PWM, because line steering asked it for 2-9 mm/s while
 * the tilted car saw the line off-centre. v0.5: from assist activation until
 * the tilt is back to flat (filtered |h| <= 2 mm for 100 ms), each wheel's
 * requested output is at least RA_FLOOR_PWM (ramped at RA_FLOOR_SLEW), so
 * steering cannot starve the climbing wheel. Bounded by RA_FLOOR_MAX_US per
 * ramp; ends on stale (60 ms), non-skip invalid (40 ms) or ACCEL/SPREAD
 * exclusion > 150 ms; Stop/Pause/not-following remove it at once. The +20 %
 * assist, the 150 ms hold and the stall push are unchanged and still apply.
 *
 * v0.4 (2026-10-31): stall push. Run at 01:21 (10 mm ramp): the car stopped at
 * 201 mm with the right wheel still up (tilt 8-11 mm) and the front settling
 * onto the mat, for ~20 s, at only 295/165 PWM of the 420 limit (output is
 * ff + error + integral capped at 100; steering asked the ramp wheel for 13
 * mm/s), while the +20 % assist had timed out. v0.4: while ASSISTING, if both
 * wheels measure < RA_STALL_MM_S for RA_STALL_US, add RA_PUSH_PWM (absolute,
 * the same on both wheels so steering is kept) ramped at RA_PUSH_SLEW, until a
 * wheel exceeds RA_PUSH_RELEASE_MM_S, the assist ends (level/descent/timeout/
 * invalid/stale) or RA_PUSH_MAX_US of push is used in this episode. Push time
 * does not count toward the 2.5 s assist timeout, so one ramp gets at most
 * 2.5 s + 1.5 s. Stop/Pause/fault/search/junction remove everything at once;
 * the controller's 0..420 clamp, slew and anti-windup still apply.
 *
 * v0.3 (2026-10-31): hold through Buddy4 acceleration/spread exclusions.
 * Run 20 (lower bracket, 10 mm ramp): at the crest the car slowed sharply;
 * Buddy4 marked tilt invalid with source-2 quality reasons ACCEL (4) and
 * SPREAD|ACCEL (5), 3 samples (60 ms) at a time, so the 40 ms invalid rule
 * ended the assist four times exactly when the car needed it; the integral
 * then surged the car to ~80 mm/s and the hump was rejected (detail 3).
 * v0.3: while ASSISTING, a non-valid tilt sample whose ONLY reasons are
 * ACCEL and/or SPREAD (Buddy4 healthy, tracking) holds the current assist
 * (detector state frozen) for up to RA_SKIP_HOLD_US = 150 ms; longer ends
 * with INVALID. Any other invalid reason keeps the unchanged 40 ms rule.
 * Not assisting: such samples still block activation (unchanged). Boost,
 * cap, slew, timeout (holds count toward it), stale, Stop/pause/fault/search/
 * junction ends, activation and the v0.2 level/descent detector are unchanged.
 *
 * v0.2 (2026-10-30): the v0.1 descent rule (3 mm below the raw peak for 2
 * samples) ended and latched assistance 241 ms into the lower-bracket climb
 * while the tilt rocked 4..7 mm, before the crest. v0.2 judges level and
 * descent on a 3-sample median of valid heights and needs a sustained fall
 * to half of the filtered peak (100 ms); level needs 100 ms. Activation,
 * boost magnitude, cap, ramp rates, timeout, invalid/stale and all
 * eligibility (pause/stop/fault/search/junction) rules are unchanged.
 *
 * Original purpose (v0.1): while the car climbs a one-sided ramp, raise the
 * EXISTING continuous-drive feedforward of both wheels by a small fraction.
 * From v0.4/v0.6 the stall push and speed hold also add PWM, raise the
 * minimum speed target and widen the integral limit while on the ramp (see
 * continuous_drive.h boost_add / hold_*); the PWM clamp and slew stay fixed.
 *
 * Tilt input: Buddy4's per-20 ms quasistatic side estimate
 * (right-minus-left height, mm) and its own validity flag (MAD/gravity/
 * timing/acceleration/turn/impact guards, Tracking state). A completed or
 * accepted ramp event is NOT required.
 *
 * Constants were tuned on one car and one 10 mm ramp (2026-10-30/31 runs,
 * see B4_PEAK_V0_5.md); other ramps or cars may need different values. */
#ifndef RAMP_ASSIST_H
#define RAMP_ASSIST_H
#include <stdbool.h>
#include <stdint.h>

#define RA_ENTER_MM 4    /* |height| to start counting persistence      */
#define RA_ENTER_TICKS 4 /* consecutive valid 20 ms samples (80 ms)     */
#define RA_EXIT_MM 2     /* level band                                  */
#define RA_EXIT_TICKS 5  /* v0.2: 100 ms filtered level ends (LEVEL)    */
#define RA_LEVEL_REARM_US                                                      \
    200000u /* 200 ms level ends the episode, clears latch */
#define RA_DESCENT_MIN_PEAK                                                    \
    6 /* v0.2: descent rule armed once filtered peak >= 6 mm */
#define RA_DESCENT_NUM                                                         \
    1 /* v0.2: descent when filtered |h| <= peak * 1/2 ...  */
#define RA_DESCENT_DEN 2
#define RA_DESCENT_TICKS                                                       \
    5 /* ... for 100 ms (5 consecutive valid samples)       */
#define RA_DETECTOR_VERSION 6
#define RA_TIMEOUT_US 2500000u /* cumulative assisted time per episode        */
#define RA_INVALID_US 40000u   /* invalid tilt longer than this ends assist   */
#define RA_SKIP_HOLD_US                                                        \
    150000u /* v0.3: max hold through ACCEL/SPREAD-only exclusion */
#define RA_Q_SPREAD                                                            \
    1u /* = B4_Q_SPREAD (buddy4/subsystem4/imu_proc.h; checked by the host test) */
#define RA_Q_ACCEL 4u      /* = B4_Q_ACCEL */
#define RA_STALE_US 60000u /* no Buddy4 sample for this long ends assist  */
#define RA_ALPHA_MILLI 200 /* +20 % of the existing feedforward           */
#define RA_SLEW_MILLI_PER_S                                                    \
    1000                   /* 0 -> 200 milli in 200 ms (and back)         */
#define RA_PWM_CAP 40      /* per-wheel absolute cap on added PWM         */
#define RA_STALL_MM_S 5.0f /* v0.4: both wheels slower than this ...          */
#define RA_STALL_US                                                            \
    200000u               /* ... for this long while assisting = stalled     */
#define RA_PUSH_PWM 60.0f /* v0.4: absolute stall push per wheel             */
#define RA_PUSH_SLEW                                                           \
    200.0f /* PWM/s: 0 -> 60 in 0.3 s and back                 */
#define RA_PUSH_RELEASE_MM_S                                                   \
    15.0f /* moving again: release the push                   */
#define RA_PUSH_MAX_US                                                         \
    1500000u /* push budget per ramp episode                     */
#define RA_HOLD_MIN_MM_S                                                       \
    35.0f /* v0.6: minimum speed target per wheel while held      */
#define RA_HOLD_I_MAX                                                          \
    300.0f /* v0.6: integral limit while held (normal 100)        */
#define RA_HOLD_KI                                                             \
    3.0f /* v0.6: integral gain multiplier while held            */
#define RA_HOLD_SLEW                                                           \
    5.0f /* hold_level per second: 0 -> 1 in 0.2 s and back      */
#define RA_HOLD_MAX_US                                                         \
    5000000u /* v0.6: hold budget per ramp episode                   */
#define RA_HOLD_INVALID_US                                                     \
    500000u /* v0.6: invalid tilt (any reason) longer than this ends */
#define RA_HOLD_STALE_US                                                       \
    200000u /* v0.6: no Buddy4 sample for this long ends the hold    */

enum
{
    RA_END_NONE = 0,
    RA_END_LEVEL = 1,
    RA_END_DESCENT = 2,
    RA_END_INVALID = 3,
    RA_END_STALE = 4,
    RA_END_TIMEOUT = 5,
    RA_END_NOT_FOLLOWING = 6,
    RA_END_PAUSED = 7,
    RA_END_STOPPED = 8
};
enum
{
    RA_EVT_PERIODIC = 0,
    RA_EVT_ACTIVATE = 1,
    RA_EVT_END = 2,
    RA_EVT_PUSH = 3 /* v0.4 stall push started */,
    RA_EVT_HOLD_END =
        4 /* v0.6 speed hold ended (v0.5: tilt floor ended); it starts at activation */
};

typedef struct
{
    uint64_t now_us;
    float dt; /* seconds since the previous call (control period) */
    bool
        run_active; /* v2.3 guard active (cleared by Stop, ESTOP, faults, run end) */
    bool
        auto_mode; /* coordinator session active in MOTION_AUTO (not paused)       */
    bool
        following; /* existing continuous drive selected and not in line recovery  */
    uint32_t
        tilt_seq; /* increments on every Buddy4 20 ms period                      */
    uint64_t tilt_us;
    bool tilt_valid;
    int16_t
        tilt_mm; /* right-minus-left, only meaningful when valid                  */
    bool
        tilt_skip; /* v0.3: not valid ONLY because of Buddy4 ACCEL/SPREAD exclusion  */
    float speed_mm_s
        [2]; /* v0.4: measured wheel speeds (v2.3 velocity[])                 */
} ramp_assist_input_t;

typedef struct
{
    /* activation */
    bool assisting;
    float alpha_milli; /* ramped value actually requested (0..RA_ALPHA_MILLI) */
    uint64_t active_since_us;
    uint32_t active_ms; /* current or last activation */
    uint8_t last_end;
    uint8_t activations; /* this run, saturating */
    /* episode (one ramp) */
    bool latched; /* after DESCENT/TIMEOUT: wait for level before re-arming */
    uint64_t episode_us; /* cumulative assisted time this episode */
    int16_t peak_mm;     /* filtered |height| peak of the current activation */
    int16_t med[3];
    uint8_t med_n; /* v0.2: last valid |heights| for the median */
    int8_t sign;   /* sign of the counted/assisting tilt */
    uint8_t over_ticks;
    uint8_t level_ticks;
    uint8_t drop_ticks;
    uint64_t level_since_us;
    uint64_t invalid_since_us;
    uint64_t skip_since_us; /* v0.3: start of the current hold (0 = none) */
    uint8_t holds;          /* v0.3: holds started this run (saturating) */
    float push_pwm; /* v0.4: ramped stall push actually requested (PWM) */
    bool pushing;   /* v0.4: push wanted this step */
    uint64_t stall_since_us;
    uint64_t push_used_us;
    uint8_t pushes;   /* v0.4: pushes started this run (saturating) */
    bool hold_on;     /* v0.6 speed hold wanted */
    float hold_level; /* v0.6: ramped 0..1, scales the hold parameters */
    uint64_t hold_since_us;
    uint64_t hold_used_us;
    uint64_t hold_inv_since_us;
    bool have_sample;
    uint32_t last_seq;
    uint64_t last_sample_us;
    int16_t last_mm;
    bool last_valid;
    /* events for telemetry (consumed by the caller) */
    uint8_t pending_event; /* RA_EVT_* or 0xff none */
    /* 500 ms interval statistics (reset by the caller after each report) */
    int16_t int_max_mm;
    int16_t int_min_mm;
    uint8_t int_valid;
    uint8_t int_invalid;
    uint8_t int_run;
    uint8_t int_longest;
} ramp_assist_t;

static inline void ra_interval_reset (ramp_assist_t * p_first)
{
    p_first->int_max_mm = -128;
    p_first->int_min_mm = 127;
    p_first->int_valid = p_first->int_invalid = p_first->int_run =
        p_first->int_longest = 0;
}
static inline void ra_reset (ramp_assist_t * p_first)
{
    *p_first = (ramp_assist_t){0};
    p_first->pending_event = 0xff;
    ra_interval_reset(p_first);
}
static inline int16_t ra_abs16 (int16_t value)
{
    return (int16_t)(value < 0 ? -value : value);
}
/* v0.2: median of the last (up to) 3 valid |heights|; a single dip or spike
 * cannot move it. Cleared on any invalid sample. */
static inline int16_t ra_median_push (int16_t * p_m_value, uint8_t * p_count,
                                      int16_t value)
{
    p_m_value[0] = p_m_value[1];
    p_m_value[1] = p_m_value[2];
    p_m_value[2] = value;
    if (*p_count < 3)
    {
        (*p_count)++;
    }
    if (*p_count < 3)
    {
        return value;
    }
    int16_t first = p_m_value[0], second = p_m_value[1],
            character = p_m_value[2];
    return (first > second) ? ((second > character)
                                   ? second
                                   : (first > character ? character : first))
                            : ((first > character)
                                   ? first
                                   : (second > character ? character : second));
}
/* v0.3: a non-valid Buddy4 sample that is ONLY an acceleration/spread exclusion while Buddy4 is
 * tracking (no turn, reverse, impact, timing, gravity or unavailable reason). */
static inline bool ra_tilt_skip (bool tracking, bool valid, uint8_t reasons)
{
    return tracking && !valid && reasons != 0u &&
           (reasons & (uint8_t) ~(RA_Q_SPREAD | RA_Q_ACCEL)) == 0u;
}
static inline uint8_t ra_inc8 (uint8_t value)
{
    return (uint8_t)(value < 255 ? value + 1 : 255);
}

static inline void ra_end (ramp_assist_t * p_first, uint8_t reason,
                           uint64_t now, bool latch)
{
    if (!p_first->assisting)
    {
        return;
    }
    p_first->assisting = false;
    p_first->active_ms = (uint32_t)((now - p_first->active_since_us) / 1000u);
    p_first->last_end = reason;
    if (latch)
    {
        p_first->latched = true;
    }
    p_first->over_ticks = p_first->drop_ticks = 0;
    p_first->skip_since_us = 0;
    p_first->pending_event = RA_EVT_END;
}
static inline void ra_hold_off (ramp_assist_t * p_first, uint64_t now)
{ /* v0.6 */
    if (!p_first->hold_on)
    {
        return;
    }
    p_first->hold_on = false;
    p_first->hold_used_us += now - p_first->hold_since_us;
    if (p_first->pending_event == 0xff)
    {
        p_first->pending_event = RA_EVT_HOLD_END;
    }
}
static inline void ra_end_episode (ramp_assist_t * p_first, uint64_t now)
{
    ra_hold_off(p_first, now);
    p_first->latched = false;
    p_first->episode_us = 0;
    p_first->peak_mm = 0;
    p_first->push_used_us = 0;
    p_first->hold_used_us = 0;
}

/* One call per control step (10 ms). Returns the alpha (milli) to apply to
 * both wheels this step. Ineligible states force 0 immediately (the existing
 * controller is being reset or left at that moment anyway). */
static inline float ramp_assist_step (ramp_assist_t * p_first,
                                      const ramp_assist_input_t * p_in_value)
{
    const uint64_t now = p_in_value->now_us;
    if (!p_in_value->run_active)
    { /* Stop, ESTOP, fault, run end */
        ra_end(p_first, RA_END_STOPPED, now, false);
        uint8_t ev_value = p_first->pending_event, acts = p_first->activations,
                end = p_first->last_end;
        uint32_t milliseconds = p_first->active_ms;
        bool had = p_first->have_sample;
        ra_reset(p_first); /* a new run starts a fresh state */
        p_first->pending_event = ev_value;
        p_first->activations = acts;
        p_first->last_end = end;
        p_first->active_ms = milliseconds;
        p_first->have_sample = had;
        return 0.0f;
    }
    /* new Buddy4 sample? */
    if (p_in_value->tilt_seq != p_first->last_seq || !p_first->have_sample)
    {
        bool fresh =
            !p_first->have_sample || p_in_value->tilt_seq != p_first->last_seq;
        p_first->have_sample = true;
        p_first->last_seq = p_in_value->tilt_seq;
        if (fresh)
        {
            p_first->last_sample_us = p_in_value->tilt_us;
            p_first->last_valid = p_in_value->tilt_valid;
            p_first->last_mm = p_in_value->tilt_mm;
            if (p_in_value->tilt_valid)
            {
                const int16_t h_value = p_in_value->tilt_mm,
                              mag = ra_abs16(h_value);
                const int8_t sg_value = (int8_t)(h_value > 0   ? 1
                                                 : h_value < 0 ? -1
                                                               : 0);
                p_first->invalid_since_us = 0;
                p_first->skip_since_us = 0;
                p_first->hold_inv_since_us = 0; /* v0.6 */
                p_first->int_valid = ra_inc8(p_first->int_valid);
                if (h_value > p_first->int_max_mm)
                {
                    p_first->int_max_mm = h_value;
                }
                if (h_value < p_first->int_min_mm)
                {
                    p_first->int_min_mm = h_value;
                }
                if (mag >= RA_ENTER_MM)
                {
                    p_first->int_run = ra_inc8(p_first->int_run);
                    if (p_first->int_run > p_first->int_longest)
                    {
                        p_first->int_longest = p_first->int_run;
                    }
                }
                else
                {
                    p_first->int_run = 0;
                }
                const int16_t hf_value = ra_median_push(
                    p_first->med, &p_first->med_n, mag); /* v0.2 filtered |h| */
                /* level band (filtered): ends activations after 100 ms, episodes after 200 ms */
                if (hf_value <= RA_EXIT_MM)
                {
                    p_first->level_ticks = ra_inc8(p_first->level_ticks);
                    if (!p_first->level_since_us)
                    {
                        p_first->level_since_us = p_in_value->tilt_us;
                    }
                    if (p_in_value->tilt_us - p_first->level_since_us >=
                            RA_LEVEL_REARM_US &&
                        !p_first->assisting)
                    {
                        ra_end_episode(p_first, now);
                    }
                }
                else
                {
                    p_first->level_ticks = 0;
                    p_first->level_since_us = 0;
                }
                if (p_first->assisting)
                {
                    if (hf_value > p_first->peak_mm)
                    {
                        p_first->peak_mm = hf_value;
                    }
                    if (p_first->level_ticks >= RA_EXIT_TICKS)
                    {
                        ra_end(p_first, RA_END_LEVEL, now, false);
                    }
                    else
                    {
                        bool falling =
                            p_first->peak_mm >= RA_DESCENT_MIN_PEAK &&
                            (int32_t)hf_value * RA_DESCENT_DEN <=
                                (int32_t)p_first->peak_mm * RA_DESCENT_NUM;
                        p_first->drop_ticks =
                            falling ? ra_inc8(p_first->drop_ticks) : 0;
                        if (p_first->drop_ticks >= RA_DESCENT_TICKS)
                        {
                            ra_end(p_first, RA_END_DESCENT, now, true);
                        }
                    }
                }
                else if (!p_first->latched && p_in_value->auto_mode &&
                         p_in_value->following &&
                         p_first->episode_us < RA_TIMEOUT_US)
                {
                    if (mag >= RA_ENTER_MM &&
                        (p_first->over_ticks == 0 || sg_value == p_first->sign))
                    {
                        p_first->sign = sg_value;
                        p_first->over_ticks = ra_inc8(p_first->over_ticks);
                    }
                    else
                    {
                        p_first->over_ticks = 0;
                    }
                    if (p_first->over_ticks >= RA_ENTER_TICKS)
                    {
                        p_first->assisting = true;
                        p_first->active_since_us = now;
                        p_first->peak_mm = hf_value > mag ? mag : hf_value;
                        p_first->drop_ticks = p_first->level_ticks = 0;
                        p_first->activations = ra_inc8(p_first->activations);
                        p_first->pending_event = RA_EVT_ACTIVATE;
                        if (!p_first->hold_on &&
                            p_first->hold_used_us < RA_HOLD_MAX_US)
                        { /* v0.6 */
                            p_first->hold_on = true;
                            p_first->hold_since_us = now;
                            p_first->hold_inv_since_us = 0;
                        }
                    }
                }
                else
                {
                    p_first->over_ticks = 0;
                }
            }
            if (!p_in_value->tilt_valid && !p_first->hold_inv_since_us)
            {
                p_first->hold_inv_since_us =
                    p_in_value->tilt_us ? p_in_value->tilt_us : 1; /* v0.6 */
            }
            if (p_in_value->tilt_valid)
            {
                /* handled above */
            }
            else if (p_in_value->tilt_skip && p_first->assisting &&
                     !p_first->invalid_since_us)
            {
                /* v0.3: acceleration/spread exclusion only: hold, freeze the detector */
                p_first->int_invalid = ra_inc8(p_first->int_invalid);
                p_first->int_run = 0;
                if (!p_first->skip_since_us)
                {
                    p_first->skip_since_us =
                        p_in_value->tilt_us ? p_in_value->tilt_us : 1;
                    p_first->holds = ra_inc8(p_first->holds);
                }
            }
            else
            {
                p_first->skip_since_us = 0;
                p_first->int_invalid = ra_inc8(p_first->int_invalid);
                p_first->int_run = 0;
                p_first->med_n = 0;
                p_first->over_ticks = 0;
                p_first->level_ticks = 0;
                p_first->level_since_us = 0;
                if (!p_first->invalid_since_us)
                {
                    p_first->invalid_since_us =
                        p_in_value->tilt_us ? p_in_value->tilt_us : 1;
                }
            }
        }
    }
    /* time-based terminations */
    if (p_first->assisting)
    {
        if (!p_in_value->auto_mode)
        {
            ra_end(p_first, RA_END_PAUSED, now, false);
        }
        else if (!p_in_value->following)
        {
            ra_end(p_first, RA_END_NOT_FOLLOWING, now, false);
        }
        else if (now - p_first->last_sample_us > RA_STALE_US)
        {
            ra_end(p_first, RA_END_STALE, now, false);
        }
        else if (p_first->invalid_since_us &&
                 now - p_first->invalid_since_us >= RA_INVALID_US)
        {
            ra_end(p_first, RA_END_INVALID, now, false);
        }
        else if (p_first->skip_since_us &&
                 now - p_first->skip_since_us >= RA_SKIP_HOLD_US)
        {
            ra_end(p_first, RA_END_INVALID, now, false);
        }
        else
        {
            float delta_time =
                p_in_value->dt > 0.0f
                    ? (p_in_value->dt > 0.1f ? 0.1f : p_in_value->dt)
                    : 0.0f;
            if (!p_first->pushing)
            {
                p_first->episode_us +=
                    (uint64_t)(delta_time *
                               1e6f); /* v0.4: push time not counted */
            }
            if (p_first->episode_us >= RA_TIMEOUT_US)
            {
                ra_end(p_first, RA_END_TIMEOUT, now, true);
            }
        }
    }
    if (p_first->hold_on)
    { /* v0.6 hold end conditions */
        bool end = p_first->hold_used_us + (now - p_first->hold_since_us) >=
                       RA_HOLD_MAX_US ||
                   now - p_first->last_sample_us > RA_HOLD_STALE_US ||
                   (p_first->hold_inv_since_us &&
                    now - p_first->hold_inv_since_us >= RA_HOLD_INVALID_US) ||
                   (p_first->level_since_us &&
                    p_first->last_sample_us - p_first->level_since_us >=
                        RA_LEVEL_REARM_US);
        if (end)
        {
            ra_hold_off(p_first, now);
        }
    }
    if (!p_in_value->auto_mode || !p_in_value->following)
    {
        p_first->alpha_milli = 0.0f;
        p_first->over_ticks = 0;
        p_first->push_pwm = 0.0f;
        p_first->pushing = false;
        p_first->stall_since_us = 0; /* v0.4: removed at once */
        ra_hold_off(p_first, now);
        p_first->hold_level = 0.0f; /* v0.6: removed at once */
        return 0.0f;
    }
    { /* v0.6 hold level ramp */
        float delta_time_value =
            p_in_value->dt > 0.0f
                ? (p_in_value->dt > 0.1f ? 0.1f : p_in_value->dt)
                : 0.0f;
        float goal = p_first->hold_on ? 1.0f : 0.0f,
              step = RA_HOLD_SLEW * delta_time_value,
              demand = goal - p_first->hold_level;
        p_first->hold_level +=
            demand > step ? step : (demand < -step ? -step : demand);
        if (p_first->hold_level < 0.0f)
        {
            p_first->hold_level = 0.0f;
        }
        if (p_first->hold_level > 1.0f)
        {
            p_first->hold_level = 1.0f;
        }
    }
    { /* v0.4 stall push */
        float delta_time_value_value =
            p_in_value->dt > 0.0f
                ? (p_in_value->dt > 0.1f ? 0.1f : p_in_value->dt)
                : 0.0f;
        float v0_value = fabsf(p_in_value->speed_mm_s[0]),
              v1_value = fabsf(p_in_value->speed_mm_s[1]),
              vmax = v0_value > v1_value ? v0_value : v1_value;
        if (p_first->assisting && vmax < RA_STALL_MM_S)
        {
            if (!p_first->stall_since_us)
            {
                p_first->stall_since_us = now ? now : 1;
            }
        }
        else
        {
            p_first->stall_since_us = 0;
        }
        bool budget = p_first->push_used_us < RA_PUSH_MAX_US;
        bool want =
            p_first->assisting && budget &&
            (p_first->pushing ? vmax < RA_PUSH_RELEASE_MM_S
                              : (p_first->stall_since_us &&
                                 now - p_first->stall_since_us >= RA_STALL_US));
        if (want && !p_first->pushing)
        {
            p_first->pushes = ra_inc8(p_first->pushes);
            if (p_first->pending_event == 0xff)
            {
                p_first->pending_event = RA_EVT_PUSH;
            }
        }
        p_first->pushing = want;
        if (want)
        {
            p_first->push_used_us += (uint64_t)(delta_time_value_value * 1e6f);
        }
        float goal = want ? RA_PUSH_PWM : 0.0f,
              step = RA_PUSH_SLEW * delta_time_value_value,
              demand_value = goal - p_first->push_pwm;
        p_first->push_pwm +=
            demand_value > step ? step
                                : (demand_value < -step ? -step : demand_value);
        if (p_first->push_pwm < 0.0f)
        {
            p_first->push_pwm = 0.0f;
        }
    }
    {
        float goal = p_first->assisting ? (float)RA_ALPHA_MILLI : 0.0f;
        float delta_time_value_value_value =
            p_in_value->dt > 0.0f
                ? (p_in_value->dt > 0.1f ? 0.1f : p_in_value->dt)
                : 0.0f;
        float step = (float)RA_SLEW_MILLI_PER_S * delta_time_value_value_value;
        float demand_value_value = goal - p_first->alpha_milli;
        p_first->alpha_milli +=
            demand_value_value > step
                ? step
                : (demand_value_value < -step ? -step : demand_value_value);
    }
    return p_first->alpha_milli;
}

/* Source-15 telemetry payload (little endian, 28 bytes; frame 31 bytes):
 *  0 u32 uptime_ms         4 u8 event (RA_EVT_*; 3 = stall push started, 4 = speed hold ended (v0.5: tilt floor ended))    5 u8 flags (bit0 active build,
 *  bit1 assisting, bit2 latched, bit3 last sample valid, bits4-6 detector
 *  version: 0 = v0.1, 2-6 = v0.2-v0.6; bit7 powered-estimator v0.2) 6 u8 last end reason
 *  7 u8 activations        8 u16 alpha_milli         10 i16 added PWM left
 * 12 i16 added PWM right (shadow build: what WOULD be added; v0.4 includes the stall push)
 * 14 i8 current height mm (-128 = invalid)          15 i8 activation peak |mm| (v0.2: median-filtered)
 * 16 i8 interval max mm (-128 none)  17 i8 interval min mm (127 none)
 * 18 u8 valid samples  19 u8 invalid samples  20 u8 longest run >= enter (samples)
 * 21 u8 max |I| left   22 u8 max |I| right (continuous controller, 0..100; v0.6: 0..255, held limit 300 reported as 255)
 * 23 u8 control ticks with either |I| at its active limit - 1 (controller authority limit)
 * 24 u16 active_ms (current or last activation)  26 i16 peak added PWM (activation)
 */
#define RA_TLM_SOURCE 15u
#define RA_TLM_PAYLOAD_BYTES 28u
#define RA_TLM_FRAME_BYTES (3u + RA_TLM_PAYLOAD_BYTES)
typedef struct
{
    uint32_t uptime_ms;
    uint8_t event;
    uint8_t flags;
    uint8_t last_end;
    uint8_t activations;
    uint16_t alpha_milli;
    int16_t pwm_l;
    int16_t pwm_r;
    int8_t height_mm;
    int8_t peak_mm;
    int8_t int_max;
    int8_t int_min;
    uint8_t valid;
    uint8_t invalid;
    uint8_t longest;
    uint8_t max_i_l;
    uint8_t max_i_r;
    uint8_t sat_ticks;
    uint16_t active_ms;
    int16_t peak_pwm;
} ramp_assist_tlm_t;
static inline int8_t ra_i8 (int value)
{
    return (int8_t)(value < -127 ? -127 : value > 126 ? 126 : value);
}
static inline void ra_put16 (uint8_t * p_data, uint16_t value)
{
    p_data[0] = (uint8_t)value;
    p_data[1] = (uint8_t)(value >> 8);
}
static inline void ra_tlm_pack (const ramp_assist_tlm_t * p_sample, uint8_t seq,
                                uint8_t out[RA_TLM_FRAME_BYTES])
{
    uint8_t * p_data = out + 3;
    out[0] = (uint8_t)((RA_TLM_SOURCE << 4) | 2u);
    out[1] = (uint8_t)RA_TLM_PAYLOAD_BYTES;
    out[2] = seq;
    p_data[0] = (uint8_t)p_sample->uptime_ms;
    p_data[1] = (uint8_t)(p_sample->uptime_ms >> 8);
    p_data[2] = (uint8_t)(p_sample->uptime_ms >> 16);
    p_data[3] = (uint8_t)(p_sample->uptime_ms >> 24);
    p_data[4] = p_sample->event;
    p_data[5] = p_sample->flags;
    p_data[6] = p_sample->last_end;
    p_data[7] = p_sample->activations;
    ra_put16(p_data + 8, p_sample->alpha_milli);
    ra_put16(p_data + 10, (uint16_t)p_sample->pwm_l);
    ra_put16(p_data + 12, (uint16_t)p_sample->pwm_r);
    p_data[14] = (uint8_t)p_sample->height_mm;
    p_data[15] = (uint8_t)p_sample->peak_mm;
    p_data[16] = (uint8_t)p_sample->int_max;
    p_data[17] = (uint8_t)p_sample->int_min;
    p_data[18] = p_sample->valid;
    p_data[19] = p_sample->invalid;
    p_data[20] = p_sample->longest;
    p_data[21] = p_sample->max_i_l;
    p_data[22] = p_sample->max_i_r;
    p_data[23] = p_sample->sat_ticks;
    ra_put16(p_data + 24, p_sample->active_ms);
    ra_put16(p_data + 26, (uint16_t)p_sample->peak_pwm);
}
#endif

/* End of file. */
