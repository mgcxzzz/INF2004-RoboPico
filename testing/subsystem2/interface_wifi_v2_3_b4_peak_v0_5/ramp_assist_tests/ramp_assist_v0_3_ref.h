/* v2.3 + Buddy4 ramp-assist v0.3 — decision logic (pure C, host-testable).
 *
 * v0.3 (2026-10-31): hold through Buddy4 acceleration/spread exclusions.
 * Run 20 (lower bracket, 10 mm ramp): at the crest the car slowed sharply;
 * Buddy4 marked tilt invalid with source-2 quality reasons ACCEL (4) and
 * SPREAD|ACCEL (5), 3 samples (60 ms) at a time, so the 40 ms invalid rule
 * ended the assist four times exactly when the car needed it; the integral
 * then surged the car to ~80 mm/s and the hump was rejected (detail 3).
 * v0.3: while ASSISTING, a non-valid tilt sample whose ONLY reasons are
 * ACCEL and/or SPREAD (Buddy4 healthy, tracking) holds the current assist
 * (detector state frozen) for up to RA03_SKIP_HOLD_US = 150 ms; longer ends
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
 * Purpose: while the car climbs a one-sided ramp, raise the EXISTING
 * continuous-drive feedforward of both wheels by a small fraction, so the
 * existing per-wheel speed controller needs less integral build-up to hold
 * the unchanged target speed. It never changes targets, gains, PWM limits,
 * slew or anti-windup (see continuous_drive.h, boost_*).
 *
 * Tilt input: Buddy4's per-20 ms quasistatic side estimate
 * (right-minus-left height, mm) and its own validity flag (MAD/gravity/
 * timing/acceleration/turn/impact guards, Tracking state). A completed or
 * accepted ramp event is NOT required.
 *
 * All constants below are PROVISIONAL (RAMP_ASSIST_V0_1.md): they come from
 * the controller limits and two 500 ms-resolution ramp captures; the shadow
 * build exists to measure what they could not. */
#ifndef RAMP_ASSIST03_H
#define RAMP_ASSIST03_H
#include <stdbool.h>
#include <stdint.h>

#define RA03_ENTER_MM          4        /* |height| to start counting persistence      */
#define RA03_ENTER_TICKS       4        /* consecutive valid 20 ms samples (80 ms)     */
#define RA03_EXIT_MM           2        /* level band                                  */
#define RA03_EXIT_TICKS        5        /* v0.2: 100 ms filtered level ends (LEVEL)    */
#define RA03_LEVEL_REARM_US    200000u  /* 200 ms level ends the episode, clears latch */
#define RA03_DESCENT_MIN_PEAK  6        /* v0.2: descent rule armed once filtered peak >= 6 mm */
#define RA03_DESCENT_NUM       1        /* v0.2: descent when filtered |h| <= peak * 1/2 ...  */
#define RA03_DESCENT_DEN       2
#define RA03_DESCENT_TICKS     5        /* ... for 100 ms (5 consecutive valid samples)       */
#define RA03_DETECTOR_VERSION  3
#define RA03_TIMEOUT_US        2500000u /* cumulative assisted time per episode        */
#define RA03_INVALID_US        40000u   /* invalid tilt longer than this ends assist   */
#define RA03_SKIP_HOLD_US      150000u  /* v0.3: max hold through ACCEL/SPREAD-only exclusion */
#define RA03_Q_SPREAD          1u       /* = B4_Q_SPREAD (buddy4/subsystem4/imu_proc.h; checked by the host test) */
#define RA03_Q_ACCEL           4u       /* = B4_Q_ACCEL */
#define RA03_STALE_US          60000u   /* no Buddy4 sample for this long ends assist  */
#define RA03_ALPHA_MILLI       200      /* +20 % of the existing feedforward           */
#define RA03_SLEW_MILLI_PER_S  1000     /* 0 -> 200 milli in 200 ms (and back)         */
#define RA03_PWM_CAP           40       /* per-wheel absolute cap on added PWM         */

enum {
    RA03_END_NONE = 0, RA03_END_LEVEL = 1, RA03_END_DESCENT = 2, RA03_END_INVALID = 3,
    RA03_END_STALE = 4, RA03_END_TIMEOUT = 5, RA03_END_NOT_FOLLOWING = 6,
    RA03_END_PAUSED = 7, RA03_END_STOPPED = 8
};
enum { RA03_EVT_PERIODIC = 0, RA03_EVT_ACTIVATE = 1, RA03_EVT_END = 2 };

typedef struct {
    uint64_t now_us;
    float    dt;            /* seconds since the previous call (control period) */
    bool     run_active;    /* v2.3 guard active (cleared by Stop, ESTOP, faults, run end) */
    bool     auto_mode;     /* coordinator session active in MOTION_AUTO (not paused)       */
    bool     following;     /* existing continuous drive selected and not in line recovery  */
    uint32_t tilt_seq;      /* increments on every Buddy4 20 ms period                      */
    uint64_t tilt_us;
    bool     tilt_valid;
    int16_t  tilt_mm;       /* right-minus-left, only meaningful when valid                  */
    bool     tilt_skip;     /* v0.3: not valid ONLY because of Buddy4 ACCEL/SPREAD exclusion  */
} RampAssist03Input;

typedef struct {
    /* activation */
    bool     assisting;
    float    alpha_milli;   /* ramped value actually requested (0..RA03_ALPHA_MILLI) */
    uint64_t active_since_us;
    uint32_t active_ms;     /* current or last activation */
    uint8_t  last_end;
    uint8_t  activations;   /* this run, saturating */
    /* episode (one ramp) */
    bool     latched;       /* after DESCENT/TIMEOUT: wait for level before re-arming */
    uint64_t episode_us;    /* cumulative assisted time this episode */
    int16_t  peak_mm;       /* filtered |height| peak of the current activation */
    int16_t  med[3]; uint8_t med_n; /* v0.2: last valid |heights| for the median */
    int8_t   sign;          /* sign of the counted/assisting tilt */
    uint8_t  over_ticks, level_ticks, drop_ticks;
    uint64_t level_since_us;
    uint64_t invalid_since_us;
    uint64_t skip_since_us; /* v0.3: start of the current hold (0 = none) */
    uint8_t  holds;         /* v0.3: holds started this run (saturating) */
    bool     have_sample;
    uint32_t last_seq;
    uint64_t last_sample_us;
    int16_t  last_mm; bool last_valid;
    /* events for telemetry (consumed by the caller) */
    uint8_t  pending_event; /* RA03_EVT_* or 0xff none */
    /* 500 ms interval statistics (reset by the caller after each report) */
    int16_t  int_max_mm, int_min_mm;
    uint8_t  int_valid, int_invalid, int_run, int_longest;
} RampAssist03;

static inline void ra03_interval_reset(RampAssist03 *a) {
    a->int_max_mm = -128; a->int_min_mm = 127;
    a->int_valid = a->int_invalid = a->int_run = a->int_longest = 0;
}
static inline void ra03_reset(RampAssist03 *a) {
    *a = (RampAssist03){0};
    a->pending_event = 0xff;
    ra03_interval_reset(a);
}
static inline int16_t ra03_abs16(int16_t v) { return (int16_t)(v < 0 ? -v : v); }
/* v0.2: median of the last (up to) 3 valid |heights|; a single dip or spike
 * cannot move it. Cleared on any invalid sample. */
static inline int16_t ra03_median_push(int16_t *m, uint8_t *n, int16_t v) {
    m[0] = m[1]; m[1] = m[2]; m[2] = v; if (*n < 3) (*n)++;
    if (*n < 3) return v;
    int16_t a = m[0], b = m[1], c = m[2];
    return (a > b) ? ((b > c) ? b : (a > c ? c : a)) : ((a > c) ? a : (b > c ? c : b));
}
/* v0.3: a non-valid Buddy4 sample that is ONLY an acceleration/spread exclusion while Buddy4 is
 * tracking (no turn, reverse, impact, timing, gravity or unavailable reason). */
static inline bool ra03_tilt_skip(bool tracking, bool valid, uint8_t reasons) {
    return tracking && !valid && reasons != 0u && (reasons & (uint8_t)~(RA03_Q_SPREAD | RA03_Q_ACCEL)) == 0u;
}
static inline uint8_t ra03_inc8(uint8_t v) { return (uint8_t)(v < 255 ? v + 1 : 255); }

static inline void ra03_end(RampAssist03 *a, uint8_t reason, uint64_t now, bool latch) {
    if (!a->assisting) return;
    a->assisting = false;
    a->active_ms = (uint32_t)((now - a->active_since_us) / 1000u);
    a->last_end = reason;
    if (latch) a->latched = true;
    a->over_ticks = a->drop_ticks = 0; a->skip_since_us = 0;
    a->pending_event = RA03_EVT_END;
}
static inline void ra03_end_episode(RampAssist03 *a) {
    a->latched = false; a->episode_us = 0; a->peak_mm = 0;
}

/* One call per control step (10 ms). Returns the alpha (milli) to apply to
 * both wheels this step. Ineligible states force 0 immediately (the existing
 * controller is being reset or left at that moment anyway). */
static inline float ramp_assist03_step(RampAssist03 *a, const RampAssist03Input *in) {
    const uint64_t now = in->now_us;
    if (!in->run_active) {                     /* Stop, ESTOP, fault, run end */
        ra03_end(a, RA03_END_STOPPED, now, false);
        uint8_t ev = a->pending_event, acts = a->activations, end = a->last_end;
        uint32_t ms = a->active_ms;
        bool had = a->have_sample;
        ra03_reset(a);                           /* a new run starts a fresh state */
        a->pending_event = ev; a->activations = acts; a->last_end = end; a->active_ms = ms;
        a->have_sample = had;
        return 0.0f;
    }
    /* new Buddy4 sample? */
    if (in->tilt_seq != a->last_seq || !a->have_sample) {
        bool fresh = !a->have_sample || in->tilt_seq != a->last_seq;
        a->have_sample = true; a->last_seq = in->tilt_seq;
        if (fresh) {
            a->last_sample_us = in->tilt_us; a->last_valid = in->tilt_valid; a->last_mm = in->tilt_mm;
            if (in->tilt_valid) {
                const int16_t h = in->tilt_mm, mag = ra03_abs16(h);
                const int8_t sg = (int8_t)(h > 0 ? 1 : h < 0 ? -1 : 0);
                a->invalid_since_us = 0; a->skip_since_us = 0;
                a->int_valid = ra03_inc8(a->int_valid);
                if (h > a->int_max_mm) a->int_max_mm = h;
                if (h < a->int_min_mm) a->int_min_mm = h;
                if (mag >= RA03_ENTER_MM) { a->int_run = ra03_inc8(a->int_run); if (a->int_run > a->int_longest) a->int_longest = a->int_run; }
                else a->int_run = 0;
                const int16_t hf = ra03_median_push(a->med, &a->med_n, mag);   /* v0.2 filtered |h| */
                /* level band (filtered): ends activations after 100 ms, episodes after 200 ms */
                if (hf <= RA03_EXIT_MM) {
                    a->level_ticks = ra03_inc8(a->level_ticks);
                    if (!a->level_since_us) a->level_since_us = in->tilt_us;
                    if (in->tilt_us - a->level_since_us >= RA03_LEVEL_REARM_US && !a->assisting) ra03_end_episode(a);
                } else { a->level_ticks = 0; a->level_since_us = 0; }
                if (a->assisting) {
                    if (hf > a->peak_mm) a->peak_mm = hf;
                    if (a->level_ticks >= RA03_EXIT_TICKS) ra03_end(a, RA03_END_LEVEL, now, false);
                    else {
                        bool falling = a->peak_mm >= RA03_DESCENT_MIN_PEAK &&
                                       (int32_t)hf * RA03_DESCENT_DEN <= (int32_t)a->peak_mm * RA03_DESCENT_NUM;
                        a->drop_ticks = falling ? ra03_inc8(a->drop_ticks) : 0;
                        if (a->drop_ticks >= RA03_DESCENT_TICKS) ra03_end(a, RA03_END_DESCENT, now, true);
                    }
                } else if (!a->latched && in->auto_mode && in->following && a->episode_us < RA03_TIMEOUT_US) {
                    if (mag >= RA03_ENTER_MM && (a->over_ticks == 0 || sg == a->sign)) {
                        a->sign = sg; a->over_ticks = ra03_inc8(a->over_ticks);
                    } else a->over_ticks = 0;
                    if (a->over_ticks >= RA03_ENTER_TICKS) {
                        a->assisting = true; a->active_since_us = now; a->peak_mm = hf > mag ? mag : hf;
                        a->drop_ticks = a->level_ticks = 0;
                        a->activations = ra03_inc8(a->activations);
                        a->pending_event = RA03_EVT_ACTIVATE;
                    }
                } else a->over_ticks = 0;
            } else if (in->tilt_skip && a->assisting && !a->invalid_since_us) {
                /* v0.3: acceleration/spread exclusion only: hold, freeze the detector */
                a->int_invalid = ra03_inc8(a->int_invalid); a->int_run = 0;
                if (!a->skip_since_us) { a->skip_since_us = in->tilt_us ? in->tilt_us : 1; a->holds = ra03_inc8(a->holds); }
            } else {
                a->skip_since_us = 0;
                a->int_invalid = ra03_inc8(a->int_invalid); a->int_run = 0; a->med_n = 0;
                a->over_ticks = 0; a->level_ticks = 0; a->level_since_us = 0;
                if (!a->invalid_since_us) a->invalid_since_us = in->tilt_us ? in->tilt_us : 1;
            }
        }
    }
    /* time-based terminations */
    if (a->assisting) {
        if (!in->auto_mode) ra03_end(a, RA03_END_PAUSED, now, false);
        else if (!in->following) ra03_end(a, RA03_END_NOT_FOLLOWING, now, false);
        else if (now - a->last_sample_us > RA03_STALE_US) ra03_end(a, RA03_END_STALE, now, false);
        else if (a->invalid_since_us && now - a->invalid_since_us >= RA03_INVALID_US) ra03_end(a, RA03_END_INVALID, now, false);
        else if (a->skip_since_us && now - a->skip_since_us >= RA03_SKIP_HOLD_US) ra03_end(a, RA03_END_INVALID, now, false);
        else {
            float dt = in->dt > 0.0f ? (in->dt > 0.1f ? 0.1f : in->dt) : 0.0f;
            a->episode_us += (uint64_t)(dt * 1e6f);
            if (a->episode_us >= RA03_TIMEOUT_US) ra03_end(a, RA03_END_TIMEOUT, now, true);
        }
    }
    if (!in->auto_mode || !in->following) { a->alpha_milli = 0.0f; a->over_ticks = 0; return 0.0f; }
    {
        float goal = a->assisting ? (float)RA03_ALPHA_MILLI : 0.0f;
        float dt = in->dt > 0.0f ? (in->dt > 0.1f ? 0.1f : in->dt) : 0.0f;
        float step = (float)RA03_SLEW_MILLI_PER_S * dt;
        float d = goal - a->alpha_milli;
        a->alpha_milli += d > step ? step : (d < -step ? -step : d);
    }
    return a->alpha_milli;
}

/* Source-15 telemetry payload (little endian, 28 bytes; frame 31 bytes):
 *  0 u32 uptime_ms         4 u8 event (RA03_EVT_*)    5 u8 flags (bit0 active build,
 *  bit1 assisting, bit2 latched, bit3 last sample valid, bits4-6 detector
 *  version: 0 = v0.1, 2 = v0.2, 3 = v0.3; bit7 powered-estimator v0.2) 6 u8 last end reason
 *  7 u8 activations        8 u16 alpha_milli         10 i16 added PWM left
 * 12 i16 added PWM right (shadow build: what WOULD be added)
 * 14 i8 current height mm (-128 = invalid)          15 i8 activation peak |mm| (v0.2: median-filtered)
 * 16 i8 interval max mm (-128 none)  17 i8 interval min mm (127 none)
 * 18 u8 valid samples  19 u8 invalid samples  20 u8 longest run >= enter (samples)
 * 21 u8 max |I| left   22 u8 max |I| right (continuous controller, 0..100)
 * 23 u8 control ticks with either |I| >= 99 (controller authority limit)
 * 24 u16 active_ms (current or last activation)  26 i16 peak added PWM (activation)
 */
#define RA03_TLM_SOURCE        15u
#define RA03_TLM_PAYLOAD_BYTES 28u
#define RA03_TLM_FRAME_BYTES   (3u + RA03_TLM_PAYLOAD_BYTES)
typedef struct {
    uint32_t uptime_ms; uint8_t event, flags, last_end, activations;
    uint16_t alpha_milli; int16_t pwm_l, pwm_r; int8_t height_mm, peak_mm, int_max, int_min;
    uint8_t valid, invalid, longest, max_i_l, max_i_r, sat_ticks;
    uint16_t active_ms; int16_t peak_pwm;
} RampAssist03Tlm;
static inline int8_t ra03_i8(int v) { return (int8_t)(v < -127 ? -127 : v > 126 ? 126 : v); }
static inline void ra03_put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static inline void ra03_tlm_pack(const RampAssist03Tlm *t, uint8_t seq, uint8_t out[RA03_TLM_FRAME_BYTES]) {
    uint8_t *p = out + 3;
    out[0] = (uint8_t)((RA03_TLM_SOURCE << 4) | 2u); out[1] = (uint8_t)RA03_TLM_PAYLOAD_BYTES; out[2] = seq;
    p[0] = (uint8_t)t->uptime_ms; p[1] = (uint8_t)(t->uptime_ms >> 8); p[2] = (uint8_t)(t->uptime_ms >> 16); p[3] = (uint8_t)(t->uptime_ms >> 24);
    p[4] = t->event; p[5] = t->flags; p[6] = t->last_end; p[7] = t->activations;
    ra03_put16(p + 8, t->alpha_milli); ra03_put16(p + 10, (uint16_t)t->pwm_l); ra03_put16(p + 12, (uint16_t)t->pwm_r);
    p[14] = (uint8_t)t->height_mm; p[15] = (uint8_t)t->peak_mm; p[16] = (uint8_t)t->int_max; p[17] = (uint8_t)t->int_min;
    p[18] = t->valid; p[19] = t->invalid; p[20] = t->longest; p[21] = t->max_i_l; p[22] = t->max_i_r; p[23] = t->sat_ticks;
    ra03_put16(p + 24, t->active_ms); ra03_put16(p + 26, (uint16_t)t->peak_pwm);
}
#endif
