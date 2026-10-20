/**
 * @file    imu_proc.c
 * @brief   Buddy 4: statechart 6.4 and all IMU processing (pure C).
 *
 * Every 20 ms (NFR-IMU-01) b4_tick() gets the accelerometer samples the
 * GY-511 collected in its FIFO (400 Hz, about 8) and Buddy 2's motion
 * estimate, and runs one step of the statechart:
 *
 *   Initialising -> Calibrating -> Sampling { Idle, Motion Tracking,
 *                                             Hump Processing,
 *                                             Impact / Shock }
 *   any of them -> Fault / Recovery -> (authorised reset) -> Initialising
 *
 * Inside Sampling, every period, in this order (Week 6: "checks for
 * impact before terrain on every sample"):
 *   1. data health: I2C errors, no new data, stuck data -> Fault;
 *   2. shock: any FIFO sample far from the filtered gravity vector ->
 *      Impact / Shock, a partial hump is invalidated, Safety is warned
 *      (task_imu.c does that in the same period: well inside 50 ms);
 *   3. terrain: pitch, hump profile, motion class.
 *
 * Hump height (FR-IMU-02). The GY-511 has no gyroscope, so pitch comes
 * from the gravity component along the car's forward axis, with Buddy 2's
 * longitudinal acceleration taken out:
 *     sin(pitch) = (a_fwd - a_fwd_at_rest - a_long) / |g|
 * The rear contact follows the front contact one wheelbase L behind, so
 * the height of the front contact over the ground it started on is
 *     h(s) = h(s - L) + L * sin(pitch(s))
 * rebuilt every B4_HUMP_BIN_MM of Buddy 2's distance s. The peak of h is
 * the hump height. This uses distance, not time, so it does not drift
 * with speed the way h = v^2 / 2g does (Tribunal 1 review), and it is
 * checked: once the car is past the hump h must come back to about 0,
 * otherwise the estimate is rejected rather than published.
 *
 * No floating point: the RP2040 has no FPU.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#include <string.h>

#include "imu_proc.h"

#define AX_FWD          (0)
#define AX_LAT          (1)
#define AX_UP           (2)
#define LP_ONE          (256)               /* filter state is mg x 256 */
#define PERIOD_US       (B4_PERIOD_MS * 1000U)
#define L_BINS          ((uint32_t)(B4_WHEELBASE_MM / B4_HUMP_BIN_MM))
#define BIN_MASK        (B4_HUMP_BINS - 1U)
#define FILL_MAX        (32U)               /* bins per period, at most */
#define CAL_PERIODS     (B4_CAL_WINDOW_MS / B4_PERIOD_MS)
#define MM_S2_PER_G     (9807)

#if ((B4_HUMP_BINS & (B4_HUMP_BINS - 1U)) != 0U)
#error "B4_HUMP_BINS must be a power of two"
#endif

/* asin(i / 32) in 0.1 degree, i = 0..32. */
static int16_t const g_asin_dd[33] =
{
      0,  18,  36,  54,  72,  90, 108, 126, 145, 163, 182, 201, 220,
    240, 259, 280, 300, 321, 342, 364, 387, 410, 434, 460, 486, 514,
    543, 575, 610, 650, 696, 756, 900
};

/* =====================================================================
 * Small helpers
 * ================================================================== */

static int32_t
iabs (int32_t v)
{
    return (v < 0) ? -v : v;
}

/**
 * @brief  Arcsine of a Q15 value (32767 = 1.0) in 0.1 degree.
 *         Table and linear interpolation: within 0.1 deg up to 60 deg.
 */
int16_t
b4_asin_dd (int32_t sin_q15)
{
    int32_t x = iabs(sin_q15);
    int32_t idx;
    int32_t frac;
    int32_t v;

    if (x >= 32768)
    {
        v = 900;
    }
    else
    {
        idx  = x / 1024;                    /* 32768 / 32 per step */
        frac = x % 1024;
        v    = (int32_t)g_asin_dd[idx] +
               ((((int32_t)g_asin_dd[idx + 1] - (int32_t)g_asin_dd[idx]) *
                 frac) / 1024);
    }

    return (int16_t)((sin_q15 < 0) ? -v : v);
}

/**
 * @brief  Integer square root (floor).
 */
uint32_t
b4_isqrt (uint32_t value)
{
    uint32_t root = 0U;
    uint32_t bit  = 1UL << 30;

    while (bit > value)
    {
        bit >>= 2;
    }

    while (bit != 0U)
    {
        if (value >= (root + bit))
        {
            value -= root + bit;
            root   = (root >> 1) + bit;
        }
        else
        {
            root >>= 1;
        }

        bit >>= 2;
    }

    return root;
}

/**
 * @brief  NFR-IMU-02 as clarified after Tribunal 1: the error may be up to
 *         max(5 mm, 10 % of the reference height).
 */
bool
b4_nfr02_ok (int32_t est_mm, int32_t ref_mm)
{
    int32_t err = iabs(est_mm - ref_mm);

    return ((err <= B4_NFR02_ABS_MM) ||
            ((err * 100) <= (iabs(ref_mm) * B4_NFR02_PCT)));
}

static int32_t
clamp (int32_t v, int32_t lo, int32_t hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static bool
sampling (imu_state_t s)
{
    return ((s == IMU_ST_IDLE) || (s == IMU_ST_TRACKING) ||
            (s == IMU_ST_HUMP) || (s == IMU_ST_IMPACT));
}

static void
emit (b4_ctx_t const * p_ctx, b4_out_t * p_out, imu_evt_type_t type,
      uint32_t detail, int32_t height_mm, uint32_t t_us)
{
    imu_event_t * p_e;

    if (p_out->n_events >= B4_OUT_EVENTS_MAX)
    {
        return;                 /* cannot happen: at most 3 per period */
    }

    p_e = &p_out->events[p_out->n_events];
    p_out->n_events++;

    p_e->type      = (uint32_t)type & 0x7U;
    p_e->motion    = (uint32_t)p_ctx->motion & 0x7U;
    p_e->detail    = detail & 0xFU;
    p_e->height_mm = (uint32_t)clamp(height_mm, 0, IMU_HEIGHT_MAX_MM);
    p_e->pitch_dd  = clamp(p_ctx->pitch_dd, -IMU_PITCH_MAX_DD,
                           IMU_PITCH_MAX_DD);
    p_e->b_valid   = sampling(p_ctx->state) ? 1U : 0U;
    p_e->b_odo     = p_ctx->b_odo_now ? 1U : 0U;
    p_e->reserved  = 0U;
    p_e->t_ms      = (uint16_t)((t_us / 1000U) & 0xFFFFU);
}

/* =====================================================================
 * State changes
 * ================================================================== */

static void
enter_fault (b4_ctx_t * p_ctx, uint8_t code, uint32_t t_us, b4_out_t * p_out)
{
    if (p_ctx->state == IMU_ST_FAULT)
    {
        return;
    }

    p_ctx->hump.b_active = false;       /* data invalid: no hump kept  */
    p_ctx->state  = IMU_ST_FAULT;
    p_ctx->fault  = code;
    p_ctx->motion = IMU_MOTION_UNKNOWN;
    p_out->fault  = code;
    emit(p_ctx, p_out, IMU_EVT_FAULT, code, 0, t_us);
}

static void
cal_restart (b4_ctx_t * p_ctx)
{
    uint32_t i;

    p_ctx->cal_n = 0U;

    for (i = 0U; i < 3U; i++)
    {
        p_ctx->cal_sum[i] = 0;
        p_ctx->cal_min[i] = INT16_MAX;
        p_ctx->cal_max[i] = INT16_MIN;
    }
}

static void
enter_tracking (b4_ctx_t * p_ctx)
{
    p_ctx->state          = IMU_ST_TRACKING;
    p_ctx->hump.b_active  = false;
    p_ctx->hump.level_mm  = 0;
    p_ctx->hump.level_us  = 0U;
    p_ctx->hump.b_zeroed  = false;
    p_ctx->hump.up_us     = 0U;
}

/* Leave Hump Processing or Impact / Shock for wherever the mission says. */
static void
back_to_sampling (b4_ctx_t * p_ctx)
{
    if (p_ctx->b_mission)
    {
        enter_tracking(p_ctx);
    }
    else
    {
        p_ctx->hump.b_active = false;
        p_ctx->state         = IMU_ST_IDLE;
    }
}

static void
reject_hump (b4_ctx_t * p_ctx, imu_reject_t why, uint32_t t_us,
             b4_out_t * p_out)
{
    int32_t peak_mm = (p_ctx->hump.peak + 5) / 10;

    p_ctx->rejected++;
    p_ctx->hump.b_active = false;
    emit(p_ctx, p_out, IMU_EVT_HUMP_REJECTED, (uint32_t)why, peak_mm, t_us);
}

/* =====================================================================
 * Public: statechart events
 * ================================================================== */

/**
 * @brief  Power-up: everything cleared, Initialising.
 */
void
b4_reset (b4_ctx_t * p_ctx)
{
    (void)memset(p_ctx, 0, sizeof(*p_ctx));
    p_ctx->state  = IMU_ST_INIT;
    p_ctx->motion = IMU_MOTION_UNKNOWN;
    p_ctx->cand   = IMU_MOTION_UNKNOWN;
    cal_restart(p_ctx);
}

/**
 * @brief  Result of configuring the GY-511 (task_imu.c, drv_lsm303.c).
 *         OK -> Calibrating, which is always redone after a fault.
 */
void
b4_init_done (b4_ctx_t * p_ctx, bool b_ok, uint32_t t_us, b4_out_t * p_out)
{
    if (p_ctx->state != IMU_ST_INIT)
    {
        return;
    }

    if (!b_ok)
    {
        enter_fault(p_ctx, IMU_FAULT_INIT, t_us, p_out);
        return;
    }

    if (p_ctx->fault != IMU_FAULT_NONE)
    {
        p_out->b_clear_fault = true;    /* reset accepted and the sensor
                                         * answers again              */
        p_ctx->fault = IMU_FAULT_NONE;
    }

    p_ctx->i2c_run      = 0U;
    p_ctx->stale_run    = 0U;
    p_ctx->stuck_run    = 0U;
    p_ctx->state        = IMU_ST_CALIBRATING;
    p_ctx->cal_start_us = t_us;
    cal_restart(p_ctx);
}

/**
 * @brief  Operating-state change from the Navigation Coordinator.
 */
void
b4_mission (b4_ctx_t * p_ctx, b4_mission_t event, uint32_t t_us,
            b4_out_t * p_out)
{
    switch (event)
    {
        case B4_MISSION_START:          /* new run resets the maximum  */
            p_ctx->max_height_mm  = 0U;
            p_ctx->last_height_mm = 0U;
            p_ctx->humps          = 0U;
            p_ctx->rejected       = 0U;
            /* fall through */
        case B4_MISSION_RESUME:
            p_ctx->b_mission = true;

            if (p_ctx->state == IMU_ST_IDLE)
            {
                enter_tracking(p_ctx);
            }
            break;

        case B4_MISSION_STOP:
        case B4_MISSION_DONE:
            p_ctx->b_mission = false;

            if (p_ctx->state == IMU_ST_HUMP)
            {
                reject_hump(p_ctx, IMU_REJ_STOPPED, t_us, p_out);
            }

            if ((p_ctx->state == IMU_ST_TRACKING) ||
                (p_ctx->state == IMU_ST_HUMP))
            {
                p_ctx->state = IMU_ST_IDLE;
            }

            if (event == B4_MISSION_DONE)
            {
                /* Completed: the final results include the highest
                 * hump peak (Week 6 Section 5). */
                emit(p_ctx, p_out, IMU_EVT_RUN_SUMMARY, 0U,
                     (int32_t)p_ctx->max_height_mm, t_us);
            }
            break;

        case B4_MISSION_RESET:
            p_ctx->b_mission = false;

            if (p_ctx->state == IMU_ST_FAULT)
            {
                /* Authorised reset [fault cleared]: Initialising, so
                 * the baseline is measured again, never reused. */
                p_ctx->state  = IMU_ST_INIT;
                p_out->b_init = true;
            }
            break;

        default:
            break;
    }
}

/* =====================================================================
 * Processing steps
 * ================================================================== */

/* Buddy 2's estimate: accumulate distance and heading (differences of
 * wrapping int16 values), keep a short history, derive the rates. */
static void
update_motion (b4_ctx_t * p_ctx, b4_in_t const * p_in)
{
    int32_t  dd = 0;
    int32_t  dh = 0;
    uint32_t last = B4_RATE_N - 1U;
    uint32_t mid  = B4_RATE_N / 2U;
    uint32_t i;

    p_ctx->b_odo_now = false;

    if (p_in->b_motion)
    {
        if (p_ctx->b_have_last)
        {
            dd = (int32_t)(int16_t)(uint16_t)((uint16_t)p_in->distance_mm -
                                              (uint16_t)p_ctx->last_dist);
            dh = (int32_t)(int16_t)(uint16_t)((uint16_t)p_in->heading_dd -
                                              (uint16_t)p_ctx->last_head);
        }

        p_ctx->b_have_last = true;
        p_ctx->last_dist   = p_in->distance_mm;
        p_ctx->last_head   = p_in->heading_dd;
        p_ctx->hump.d_mm  += dd;
        p_ctx->head_dd    += dh;
        p_ctx->b_odo_now   = true;

        for (i = 0U; i < last; i++)
        {
            p_ctx->hist_t[i] = p_ctx->hist_t[i + 1U];
            p_ctx->hist_d[i] = p_ctx->hist_d[i + 1U];
            p_ctx->hist_h[i] = p_ctx->hist_h[i + 1U];
        }

        p_ctx->hist_t[last] = p_in->t_us;
        p_ctx->hist_d[last] = p_ctx->hump.d_mm;
        p_ctx->hist_h[last] = p_ctx->head_dd;

        if (p_ctx->hist_n < B4_RATE_N)
        {
            p_ctx->hist_n++;
        }
    }
    else
    {
        p_ctx->b_have_last = false;
        p_ctx->hist_n      = 0U;

#if (B4_BENCH_ASSUME_MM_S > 0)
        if (p_ctx->b_mission)
        {
            /* Bench only: steady assumed speed (b4_config.h). */
            p_ctx->hump.d_mm += (int32_t)(((uint32_t)B4_BENCH_ASSUME_MM_S *
                                           p_in->gap_us) / 1000000U);
            p_ctx->b_odo_now  = true;
        }
#endif
    }

    p_ctx->b_rates = (p_ctx->hist_n >= B4_RATE_N);

    if (p_ctx->b_rates)
    {
        int64_t t_all = (int64_t)(uint32_t)(p_ctx->hist_t[last] -
                                            p_ctx->hist_t[0]);
        int64_t t_old = (int64_t)(uint32_t)(p_ctx->hist_t[mid] -
                                            p_ctx->hist_t[0]);
        int64_t t_new = (int64_t)(uint32_t)(p_ctx->hist_t[last] -
                                            p_ctx->hist_t[mid]);
        int64_t v_old;
        int64_t v_new;

        if ((t_old <= 0) || (t_new <= 0))
        {
            p_ctx->b_rates = false;
            return;
        }

        v_old = ((int64_t)(p_ctx->hist_d[mid] - p_ctx->hist_d[0]) *
                 1000000) / t_old;
        v_new = ((int64_t)(p_ctx->hist_d[last] - p_ctx->hist_d[mid]) *
                 1000000) / t_new;

        p_ctx->speed_mm_s  = (int32_t)(((int64_t)(p_ctx->hist_d[last] -
                                                  p_ctx->hist_d[0]) *
                                        1000000) / t_all);
        p_ctx->accel_mm_s2 = (int32_t)(((v_new - v_old) * 2000000) / t_all);
        p_ctx->turn_dd_s   = (int32_t)(((int64_t)(p_ctx->hist_h[last] -
                                                  p_ctx->hist_h[0]) *
                                        1000000) / t_all);
    }
}

/* Pitch from the filtered forward component (see the file header). */
static int32_t
update_pitch (b4_ctx_t * p_ctx)
{
    int32_t fwd = p_ctx->lp[AX_FWD] / LP_ONE;
    int32_t up  = p_ctx->lp[AX_UP] / LP_ONE;
    int32_t b_f = p_ctx->base[AX_FWD];
    int32_t b_u = p_ctx->base[AX_UP];
    int64_t den;
    int32_t sin_q15;

    if (p_ctx->b_rates)
    {
        fwd -= (p_ctx->accel_mm_s2 * 1000) / MM_S2_PER_G;
    }

    /* Angle of the (fwd, up) vector against the same vector at rest:
     *   sin(pitch) = (fwd * b_u - up * b_f) / (|now| * |rest|)
     * Using the direction, not the size, cancels vertical acceleration
     * (going over the hump's edges scales both components alike). */
    den = (int64_t)b4_isqrt((uint32_t)((fwd * fwd) + (up * up))) *
          (int64_t)b4_isqrt((uint32_t)((b_f * b_f) + (b_u * b_u)));

    if (den < (int64_t)(300 * 300))
    {
        sin_q15 = 0;                    /* free fall or a jolt: no angle */
    }
    else
    {
        int64_t num = ((int64_t)fwd * b_u) - ((int64_t)up * b_f);

        sin_q15 = clamp((int32_t)((num * 32768) / den),
              -32767, 32767);
    }

    p_ctx->pitch_dd = b4_asin_dd(sin_q15);

    /* Tilt with hysteresis: climbing / descending / level. */
    if (p_ctx->tilt == 0)
    {
        if (p_ctx->pitch_dd > B4_TILT_ON_DD)
        {
            p_ctx->tilt = 1;
        }
        else if (p_ctx->pitch_dd < -B4_TILT_ON_DD)
        {
            p_ctx->tilt = -1;
        }
        else
        {
            /* level */
        }
    }
    else if (p_ctx->tilt > 0)
    {
        if (p_ctx->pitch_dd < B4_TILT_OFF_DD)
        {
            p_ctx->tilt = 0;
        }
    }
    else
    {
        if (p_ctx->pitch_dd > -B4_TILT_OFF_DD)
        {
            p_ctx->tilt = 0;
        }
    }

    return sin_q15;
}

/* Rebuild the front-contact height for every bin of distance travelled.
 * Returns the distance covered in whole bins (mm). */
static int32_t
fill_bins (b4_hump_t * p_h, int32_t sin_q15)
{
    uint32_t n = 0U;

    while ((p_h->d_mm >= p_h->next_bin_mm) && (n < FILL_MAX))
    {
        uint32_t next = (p_h->head + 1U) & BIN_MASK;
        int32_t  back = p_h->bins[(next - L_BINS) & BIN_MASK];
        int32_t  h    = back + ((B4_WHEELBASE_MM * 10 * sin_q15) / 32768);

        p_h->head       = next;
        p_h->bins[next] = h;
        p_h->next_bin_mm += B4_HUMP_BIN_MM;
        n++;

        if (p_h->b_active)
        {
            /* Peak of a 5-bin (20 mm) average: the maximum of single
             * noisy bins would be biased upwards by the noise. */
            int32_t avg = (h + p_h->bins[(next - 1U) & BIN_MASK] +
                           p_h->bins[(next - 2U) & BIN_MASK] +
                           p_h->bins[(next - 3U) & BIN_MASK] +
                           p_h->bins[(next - 4U) & BIN_MASK]) / 5;

            if (avg > p_h->peak)
            {
                p_h->peak = avg;
            }
        }
    }

    if (n == FILL_MAX)
    {
        p_h->next_bin_mm = p_h->d_mm + B4_HUMP_BIN_MM;   /* resync */
    }

    return (int32_t)n * B4_HUMP_BIN_MM;
}

static void
finish_hump (b4_ctx_t * p_ctx, uint32_t t_us, b4_out_t * p_out)
{
    b4_hump_t * p_h     = &p_ctx->hump;
    int32_t     peak_mm = (p_h->peak + 5) / 10;
    int32_t     end     = 0;                            /* 0.1 mm */
    int32_t     tol     = (p_h->peak * B4_HUMP_CLOSE_PCT) / 100;
    uint32_t    i;

    /* Height under the car now: the mean over the last wheelbase of bins
     * (one bin alone carries the noise of every step that built it). */
    for (i = 0U; i < L_BINS; i++)
    {
        end += p_h->bins[(p_h->head - i) & BIN_MASK];
    }

    end /= (int32_t)L_BINS;

    if (tol < (B4_HUMP_CLOSE_MM * 10))
    {
        tol = B4_HUMP_CLOSE_MM * 10;
    }

    if (!p_h->b_odo_ok)
    {
        reject_hump(p_ctx, IMU_REJ_NO_ODOMETRY, t_us, p_out);
    }
    else if (p_h->b_tainted)
    {
        /* Began before the ground reference was re-established after a
         * shock: its height is not trusted either. */
        reject_hump(p_ctx, IMU_REJ_SHOCK, t_us, p_out);
    }
    else if ((peak_mm < B4_HUMP_MIN_MM) || (peak_mm > B4_HUMP_MAX_MM))
    {
        reject_hump(p_ctx, IMU_REJ_RANGE, t_us, p_out);
    }
    else if (iabs(end) > tol)
    {
        reject_hump(p_ctx, IMU_REJ_NOT_CLOSED, t_us, p_out);
    }
    else
    {
        p_ctx->last_height_mm = (uint8_t)peak_mm;
        p_ctx->humps++;

        if ((uint8_t)peak_mm > p_ctx->max_height_mm)
        {
            p_ctx->max_height_mm = (uint8_t)peak_mm;    /* run maximum */
        }

        p_h->b_active = false;
        emit(p_ctx, p_out, IMU_EVT_HUMP_PEAK, 0U, peak_mm, t_us);
    }

    enter_tracking(p_ctx);
}

/* Motion Tracking and Hump Processing (terrain part of one period). */
static void
terrain (b4_ctx_t * p_ctx, b4_in_t const * p_in, int32_t adv_mm,
         b4_out_t * p_out)
{
    b4_hump_t * p_h   = &p_ctx->hump;
    int32_t     pitch = p_ctx->pitch_dd;

    if (p_ctx->state == IMU_ST_TRACKING)
    {
        /* Level for a wheelbase: both contacts on the same ground again,
         * so this ground becomes the height reference (0). */
        if (iabs(pitch) < B4_TILT_OFF_DD)
        {
            p_h->level_mm += adv_mm;
            p_h->level_us += p_in->gap_us;

            if (((p_h->level_mm >= B4_WHEELBASE_MM) ||
                 (!p_ctx->b_odo_now &&
                  (p_h->level_us >= (B4_HUMP_END_MS * 1000U)))) &&
                !p_h->b_zeroed)
            {
                (void)memset(p_h->bins, 0, sizeof(p_h->bins));
                p_h->b_zeroed = true;
                p_h->b_ref_ok = true;
            }
        }
        else
        {
            p_h->level_mm = 0;
            p_h->level_us = 0U;
            p_h->b_zeroed = false;
        }

        /* Hump start [sustained change]: nose up long enough. */
        if (pitch > B4_TILT_ON_DD)
        {
            p_h->up_us += p_in->gap_us;
        }
        else
        {
            p_h->up_us = 0U;
        }

        if (p_h->up_us >= (B4_HUMP_START_MS * 1000U))
        {
            p_h->b_active   = true;
            p_h->b_tainted  = !p_h->b_ref_ok;
            p_h->b_descent  = false;
            p_h->b_odo_ok   = p_ctx->b_odo_now;
            p_h->t_start_us = p_in->t_us;
            p_h->d_start_mm = p_h->d_mm;
            p_h->peak       = p_h->bins[p_h->head];
            p_h->after_mm   = 0;
            p_h->after_us   = 0U;
            p_ctx->state    = IMU_ST_HUMP;
            emit(p_ctx, p_out, IMU_EVT_HUMP_START, 0U, 0, p_in->t_us);
        }

        return;
    }

    /* Hump Processing */
    if (!p_ctx->b_odo_now)
    {
        p_h->b_odo_ok = false;
    }

    if (pitch < -B4_TILT_ON_DD)
    {
        p_h->b_descent = true;
    }

    if (p_h->b_descent)
    {
        if (iabs(pitch) < B4_TILT_OFF_DD)
        {
            p_h->after_mm += adv_mm;
            p_h->after_us += p_in->gap_us;
        }
        else
        {
            p_h->after_mm = 0;
            p_h->after_us = 0U;
        }
    }

    if ((p_h->b_odo_ok && (p_h->after_mm >= B4_HUMP_END_MM)) ||
        (!p_h->b_odo_ok && (p_h->after_us >= (B4_HUMP_END_MS * 1000U))))
    {
        finish_hump(p_ctx, p_in->t_us, p_out);
    }
    else if (((uint32_t)(p_in->t_us - p_h->t_start_us) >=
              (B4_HUMP_TIMEOUT_MS * 1000U)) ||
             ((p_h->d_mm - p_h->d_start_mm) > B4_HUMP_MAX_LEN_MM))
    {
        /* Profile timeout: reject, never keep a doubtful peak. */
        reject_hump(p_ctx, p_h->b_descent ? IMU_REJ_TIMEOUT
                                          : IMU_REJ_NO_DESCENT,
                    p_in->t_us, p_out);
        enter_tracking(p_ctx);
    }
    else
    {
        /* still profiling */
    }
}

/* Motion class (FR-IMU-03), with a hold time against flicker. */
static void
classify (b4_ctx_t * p_ctx, b4_in_t const * p_in, bool b_quiet,
          b4_out_t * p_out)
{
    imu_motion_t cand;

    if (p_ctx->state == IMU_ST_IMPACT)
    {
        cand = IMU_MOTION_IMPACT;
    }
    else if (p_ctx->tilt > 0)
    {
        cand = IMU_MOTION_CLIMBING;
    }
    else if (p_ctx->tilt < 0)
    {
        cand = IMU_MOTION_DESCENDING;
    }
    else if (p_ctx->b_rates)
    {
        /* Buddy 2's estimate (Tribunal 1: not recomputed here). */
        if (iabs(p_ctx->turn_dd_s) > B4_TURN_DD_S)
        {
            cand = IMU_MOTION_TURNING;
        }
        else if (iabs(p_ctx->accel_mm_s2) > B4_ACCEL_MM_S2)
        {
            cand = IMU_MOTION_ACCELERATING;
        }
        else if (iabs(p_ctx->speed_mm_s) < B4_STILL_MM_S)
        {
            cand = IMU_MOTION_STATIONARY;
        }
        else
        {
            cand = IMU_MOTION_STEADY;
        }
    }
    else
    {
        /* No fresh estimate: the IMU alone can only tell "still". */
        cand = b_quiet ? IMU_MOTION_STATIONARY : IMU_MOTION_UNKNOWN;
    }

    if (cand == p_ctx->motion)
    {
        p_ctx->cand    = cand;
        p_ctx->cand_us = 0U;
        return;
    }

    if ((cand == IMU_MOTION_IMPACT) || (p_ctx->motion == IMU_MOTION_IMPACT))
    {
        p_ctx->cand_us = B4_CLASS_HOLD_MS * 1000U;     /* at once */
    }
    else if (cand != p_ctx->cand)
    {
        p_ctx->cand_us = p_in->gap_us;
    }
    else
    {
        p_ctx->cand_us += p_in->gap_us;
    }

    p_ctx->cand = cand;

    if (p_ctx->cand_us >= (B4_CLASS_HOLD_MS * 1000U))
    {
        p_ctx->motion  = cand;
        p_ctx->cand_us = 0U;

        if (p_ctx->b_mission)
        {
            emit(p_ctx, p_out, IMU_EVT_MOTION, 0U, 0, p_in->t_us);
        }
    }
}

/* Calibrating: a stable, stationary 2.0 s window gives the baseline. */
static void
calibrate (b4_ctx_t * p_ctx, int16_t const mean[3], b4_in_t const * p_in,
           b4_out_t * p_out)
{
    bool     b_restart = false;
    uint32_t i;

    if ((uint32_t)(p_in->t_us - p_ctx->cal_start_us) >=
        (B4_CAL_DEADLINE_MS * 1000U))
    {
        enter_fault(p_ctx, IMU_FAULT_CAL_TIMEOUT, p_in->t_us, p_out);
        return;
    }

    update_motion(p_ctx, p_in);

    if (p_ctx->b_rates && (iabs(p_ctx->speed_mm_s) >= B4_STILL_MM_S))
    {
        b_restart = true;               /* [stationary] guard          */
    }

    for (i = 0U; i < 3U; i++)
    {
        int16_t lo = (mean[i] < p_ctx->cal_min[i]) ? mean[i]
                                                   : p_ctx->cal_min[i];
        int16_t hi = (mean[i] > p_ctx->cal_max[i]) ? mean[i]
                                                   : p_ctx->cal_max[i];

        if (((int32_t)hi - (int32_t)lo) > B4_CAL_STABLE_MG)
        {
            b_restart = true;           /* moved: start the window again */
        }
    }

    if (b_restart)
    {
        cal_restart(p_ctx);
    }

    for (i = 0U; i < 3U; i++)
    {
        p_ctx->cal_sum[i] += mean[i];

        if (mean[i] < p_ctx->cal_min[i])
        {
            p_ctx->cal_min[i] = mean[i];
        }

        if (mean[i] > p_ctx->cal_max[i])
        {
            p_ctx->cal_max[i] = mean[i];
        }
    }

    p_ctx->cal_n++;

    if (p_ctx->cal_n >= CAL_PERIODS)
    {
        int32_t  b[3];
        uint32_t sq = 0U;

        for (i = 0U; i < 3U; i++)
        {
            b[i] = p_ctx->cal_sum[i] / (int32_t)p_ctx->cal_n;
            sq  += (uint32_t)(b[i] * b[i]);
        }

        p_ctx->g_mg = (int32_t)b4_isqrt(sq);

        if ((p_ctx->g_mg < B4_CAL_G_MIN_MG) ||
            (p_ctx->g_mg > B4_CAL_G_MAX_MG) ||
            ((b[AX_UP] * 100) < (B4_CAL_UP_MIN_PCT * p_ctx->g_mg)))
        {
            /* |g| wrong, or gravity is not on the UP axis (mounting
             * settings wrong, or the car is on a slope). */
            enter_fault(p_ctx, IMU_FAULT_CAL, p_in->t_us, p_out);
            return;
        }

        for (i = 0U; i < 3U; i++)
        {
            p_ctx->base[i] = (int16_t)b[i];
            p_ctx->lp[i]   = b[i] * LP_ONE;
        }

        p_ctx->pitch_dd         = 0;
        p_ctx->tilt             = 0;
        p_ctx->motion           = IMU_MOTION_STATIONARY;
        p_ctx->cand             = IMU_MOTION_STATIONARY;
        p_ctx->cand_us          = 0U;
        (void)memset(p_ctx->hump.bins, 0, sizeof(p_ctx->hump.bins));
        p_ctx->hump.next_bin_mm = p_ctx->hump.d_mm + B4_HUMP_BIN_MM;
        p_ctx->hump.b_ref_ok    = true;  /* the ground it stands on   */
        p_ctx->state            = IMU_ST_IDLE;   /* calibration valid */

        if (p_ctx->b_mission)
        {
            enter_tracking(p_ctx);
        }
    }
}

/* =====================================================================
 * Public: one period
 * ================================================================== */

/**
 * @brief  One 20 ms period. p_out is added to (clear it before).
 */
void
b4_tick (b4_ctx_t * p_ctx, b4_in_t const * p_in, b4_out_t * p_out)
{
    int32_t  sum[3] = { 0, 0, 0 };
    int16_t  mean[3];
    uint32_t i;

    if ((p_ctx->state == IMU_ST_INIT) || (p_ctx->state == IMU_ST_FAULT))
    {
        return;
    }

    /* NFR-IMU-01 evidence: periods that came late, or data lost. */
    if (p_in->gap_us > B4_LATE_US)
    {
        p_ctx->missed += (uint16_t)(((p_in->gap_us + (PERIOD_US / 2U)) /
                                     PERIOD_US) - 1U);
    }
    else if (p_in->b_overrun)
    {
        p_ctx->missed++;
    }
    else
    {
        /* on time */
    }

    /* 1. Data health */
    if (!p_in->b_read_ok)
    {
        p_ctx->i2c_errors++;
        p_ctx->i2c_run++;

        if (p_ctx->i2c_run >= B4_I2C_ERR_MAX)
        {
            enter_fault(p_ctx, IMU_FAULT_I2C, p_in->t_us, p_out);
        }
        return;
    }

    p_ctx->i2c_run = 0U;

    if (p_in->n == 0U)
    {
        p_ctx->stale_run++;

        if (p_ctx->stale_run >= B4_STALE_MAX)
        {
            enter_fault(p_ctx, IMU_FAULT_STALE, p_in->t_us, p_out);
        }
        return;
    }

    p_ctx->stale_run = 0U;

    for (i = 0U; i < (uint32_t)p_in->n; i++)
    {
        sum[AX_FWD] += p_in->s[i].fwd_mg;
        sum[AX_LAT] += p_in->s[i].lat_mg;
        sum[AX_UP]  += p_in->s[i].up_mg;
    }

    for (i = 0U; i < 3U; i++)
    {
        mean[i] = (int16_t)(sum[i] / (int32_t)p_in->n);
    }

    if ((mean[AX_FWD] == p_ctx->last_mean.fwd_mg) &&
        (mean[AX_LAT] == p_ctx->last_mean.lat_mg) &&
        (mean[AX_UP] == p_ctx->last_mean.up_mg))
    {
        p_ctx->stuck_run++;

        if (p_ctx->stuck_run >= B4_STUCK_PERIODS)
        {
            enter_fault(p_ctx, IMU_FAULT_INVALID, p_in->t_us, p_out);
            return;
        }
    }
    else
    {
        p_ctx->stuck_run = 0U;
    }

    p_ctx->last_mean.fwd_mg = mean[AX_FWD];
    p_ctx->last_mean.lat_mg = mean[AX_LAT];
    p_ctx->last_mean.up_mg  = mean[AX_UP];

    if (p_ctx->state == IMU_ST_CALIBRATING)
    {
        calibrate(p_ctx, mean, p_in, p_out);
        return;
    }

    /* 2. Shock, against the filtered gravity vector of the last period */
    {
        uint32_t max_d2 = 0U;
        uint32_t spread = 0U;
        uint32_t at     = 0U;
        int32_t  g[3];
        bool     b_quiet;

        for (i = 0U; i < 3U; i++)
        {
            g[i] = p_ctx->lp[i] / LP_ONE;
        }

        for (i = 0U; i < (uint32_t)p_in->n; i++)
        {
            int32_t  dx = p_in->s[i].fwd_mg - g[AX_FWD];
            int32_t  dy = p_in->s[i].lat_mg - g[AX_LAT];
            int32_t  dz = p_in->s[i].up_mg  - g[AX_UP];
            uint32_t d2 = (uint32_t)((dx * dx) + (dy * dy) + (dz * dz));
            uint32_t sx = (uint32_t)iabs(p_in->s[i].fwd_mg - mean[AX_FWD]);
            uint32_t sy = (uint32_t)iabs(p_in->s[i].lat_mg - mean[AX_LAT]);
            uint32_t sz = (uint32_t)iabs(p_in->s[i].up_mg  - mean[AX_UP]);

            if (d2 > max_d2)
            {
                max_d2 = d2;
                at     = i;
            }

            spread = (sx > spread) ? sx : spread;
            spread = (sy > spread) ? sy : spread;
            spread = (sz > spread) ? sz : spread;
        }

        b_quiet = (spread < (uint32_t)B4_QUIET_MG);

        if ((max_d2 > (uint32_t)(B4_SHOCK_MG * B4_SHOCK_MG)) &&
            (p_ctx->state != IMU_ST_IMPACT))
        {
            uint32_t mg = b4_isqrt(max_d2);

            /* A shock invalidates a partial hump: never a false peak. */
            if (p_ctx->state == IMU_ST_HUMP)
            {
                reject_hump(p_ctx, IMU_REJ_SHOCK, p_in->t_us, p_out);
            }

            p_ctx->state        = IMU_ST_IMPACT;
            p_ctx->hump.b_ref_ok = false;   /* re-establish when level */
            p_ctx->clear_us     = 0U;
            p_ctx->shocks++;
            p_ctx->shock_mg     = (uint16_t)((mg > 4095U) ? 4095U : mg);
            p_out->b_shock      = true;
            p_out->shock_age_us = ((uint32_t)p_in->n - 1U - at) *
                                  B4_SAMPLE_US;
            p_ctx->motion       = IMU_MOTION_IMPACT;
            p_ctx->cand         = IMU_MOTION_IMPACT;
            emit(p_ctx, p_out, IMU_EVT_IMPACT, 0U, 0, p_in->t_us);
        }
        else if (p_ctx->state == IMU_ST_IMPACT)
        {
            if (max_d2 < (uint32_t)(B4_SHOCK_CLEAR_MG * B4_SHOCK_CLEAR_MG))
            {
                p_ctx->clear_us += p_in->gap_us;
            }
            else
            {
                p_ctx->clear_us = 0U;
            }

            if (p_ctx->clear_us >= (B4_SHOCK_HOLD_MS * 1000U))
            {
                back_to_sampling(p_ctx);        /* shock cleared */
            }
        }
        else
        {
            /* no shock */
        }

        /* 3. Terrain */
        for (i = 0U; i < 3U; i++)
        {
            p_ctx->lp[i] += ((mean[i] * LP_ONE) - p_ctx->lp[i]) /
                            (1 << B4_LP_SHIFT);
        }

        update_motion(p_ctx, p_in);

        {
            int32_t sin_q15 = update_pitch(p_ctx);
            int32_t adv     = fill_bins(&p_ctx->hump, sin_q15);

            if ((p_ctx->state == IMU_ST_TRACKING) ||
                (p_ctx->state == IMU_ST_HUMP))
            {
                terrain(p_ctx, p_in, adv, p_out);
            }
        }

        classify(p_ctx, p_in, b_quiet, p_out);
    }
}

/**
 * @brief  Telemetry view of the current state plus the frame's event.
 */
void
b4_fill_tlm (b4_ctx_t const * p_ctx, imu_event_t const * p_evt,
             imu_tlm_t * p_tlm)
{
    p_tlm->state         = (uint8_t)p_ctx->state;
    p_tlm->motion        = (uint8_t)p_ctx->motion;
    p_tlm->b_valid       = sampling(p_ctx->state);
    p_tlm->b_odo         = p_ctx->b_odo_now;
    p_tlm->pitch_dd      = p_ctx->pitch_dd;
    p_tlm->fault         = p_ctx->fault;
    p_tlm->max_height_mm = p_ctx->max_height_mm;
    p_tlm->humps         = p_ctx->humps;
    p_tlm->rejected      = p_ctx->rejected;
    p_tlm->shock_mg      = p_ctx->shock_mg;
    p_tlm->shock_ms      = p_ctx->shock_ms;
    p_tlm->shocks        = p_ctx->shocks;
    p_tlm->i2c_errors    = p_ctx->i2c_errors;
    p_tlm->missed        = p_ctx->missed;

    if (p_evt != NULL)
    {
        p_tlm->evt       = (uint8_t)p_evt->type;
        p_tlm->detail    = (uint8_t)p_evt->detail;
        p_tlm->height_mm = (uint8_t)p_evt->height_mm;
        p_tlm->motion    = (uint8_t)p_evt->motion;
        p_tlm->pitch_dd  = (int16_t)p_evt->pitch_dd;
        p_tlm->t_ms      = p_evt->t_ms;
    }
    else
    {
        p_tlm->evt       = (uint8_t)IMU_EVT_STATUS;
        p_tlm->detail    = 0U;
        p_tlm->height_mm = p_ctx->last_height_mm;
        p_tlm->t_ms      = 0U;
    }
}
