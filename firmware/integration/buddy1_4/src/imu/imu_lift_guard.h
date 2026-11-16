/**
 * @file imu_lift_guard.h
 * @brief Imu lift guard.
 * @details Sensor processing owns calibration/terrain state; RTOS readers use
 * copies. Height acceptance and motor assistance are separate from end-run
 * safety.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Release-only accelerometer lift/upset guard. Pure integer logic. */
#ifndef IMU_LIFT_GUARD_H
#define IMU_LIFT_GUARD_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "src/imu/imu_proc.h"
#include "src/platform/compact_signed.h"

/* Reuse the processor's car-frame sample type: forward, lateral, up, mg. */
typedef b4_vec_t imu_lift_vec_t;

enum
{
    IMU_LIFT_REASON_UP_ACCEL = 1U,
    IMU_LIFT_REASON_TILT = 2U,
    IMU_LIFT_CANDIDATE_NONE = 0U,
    IMU_LIFT_CANDIDATE_UP_ACCEL = 1U,
    IMU_LIFT_CANDIDATE_TILT = 2U,
    IMU_LIFT_CANDIDATE_BOTH = 3U,
    IMU_LIFT_REJECT_NONE = 0U,
    IMU_LIFT_REJECT_NORM = 1U,    /* sample magnitude outside gravity band */
    IMU_LIFT_REJECT_INVALID = 2U, /* failed, empty, late, or overrun batch */
    IMU_LIFT_REJECT_NONCANDIDATE =
        3U,                         /* valid sample crossed neither trigger */
    IMU_LIFT_UP_SAMPLES = 12U,      /* 30 ms at 400 Hz */
    IMU_LIFT_TILT_SAMPLES = 40U,    /* 100 ms at 400 Hz */
    IMU_LIFT_UPRIGHT_SAMPLES = 200U /* 500 ms at 400 Hz */
};

#define IMU_LIFT_ACCEL_EXTRA_MG (400)
#define IMU_LIFT_UPRIGHT_EXTRA_MG (200)

typedef struct
{
    unsigned
        up_projection_bits : 13; /* current projection on calibrated gravity */
    unsigned norm_mg : 12;       /* current acceleration magnitude */
    int16_t
        min_cos_q15; /* minimum dot/(gravity*norm) among gravity-like samples */
    uint16_t
        min_cos_norm_mg; /* magnitude at that valid minimum-cosine sample */
    int16_t
        peak_up_projection_mg; /* lifetime maximum calibrated-up projection */
    uint16_t event_count;      /* saturating lifetime event count */
    uint32_t event_time_ms; /* first sample that qualified, wraps naturally */
    uint32_t min_cos_sample_ms;     /* hardware timer epoch; wraps naturally */
    uint32_t peak_window_start_ms;  /* first valid sample since reset */
    uint32_t peak_up_samples : 4;   /* greatest upward dwell seen since reset */
    uint32_t peak_tilt_samples : 6; /* greatest tilt dwell seen since reset */
    uint32_t
        up_samples : 4; /* consecutive upward-acceleration samples, <= 12 */
    uint32_t tilt_samples : 6;    /* consecutive >=45-degree samples, <= 40 */
    uint32_t upright_samples : 8; /* consecutive reset-ready samples, <= 200 */
    uint32_t reason : 2;          /* 1 = up accel, 2 = tilt; 3 reserved */
    uint32_t last_candidate : 2;  /* latest nonzero candidate since Reset */
    uint32_t
        last_rejection : 2; /* latest norm/invalid rejection; 3 is initial */
    uint32_t armed : 1;     /* calibrated baseline has been observed */
    uint32_t current_valid : 1; /* projection/norm describe a fresh sample */
    uint32_t latched : 1;       /* retained until authorized upright reset */
    uint32_t saturated : 1;     /* current metric exceeded its packed range */
    uint32_t window_set : 1;    /* lifetime peak window initialized */
    uint32_t min_cos_set : 1;   /* at least one gravity-like sample seen */
    uint32_t peak_up_set : 1;   /* upward projection high-water initialized */
    uint32_t
        peak_saturated : 1; /* any sample exceeded the packed metric range */
} imu_lift_guard_t;

/** @brief Read the signed 13-bit calibrated-up projection (-4096..4095 mg). */
static inline int32_t imu_lift_projection (const imu_lift_guard_t * p_guard)
{
    return compact_signed_unpack(p_guard->up_projection_bits, 13U);
}

static inline uint32_t imu_lift_isqrt_u64 (uint64_t value)
{
    uint64_t remainder = value;
    uint64_t result = 0U;
    uint64_t bit = UINT64_C(1) << 62;
    while (bit > remainder)
    {
        bit >>= 2;
    }
    while (bit != 0U)
    {
        if (remainder >= (result + bit))
        {
            remainder -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (result > UINT32_MAX) ? UINT32_MAX : (uint32_t)result;
}

static inline void imu_lift_guard_init (imu_lift_guard_t * p_guard)
{
    if (p_guard != NULL)
    {
        (void)memset(p_guard, 0, sizeof(*p_guard));
        p_guard->last_rejection = IMU_LIFT_REJECT_NONCANDIDATE;
    }
}

static inline void imu_lift_guard_no_fresh_batch (imu_lift_guard_t * p_guard)
{
    if (NULL == p_guard)
    {
        return;
    }
    /* `armed` means the calibrated baseline exists, not that this batch did. */
    p_guard->current_valid = false;
    p_guard->up_samples = 0U;
    p_guard->tilt_samples = 0U;
    p_guard->upright_samples = 0U;
    p_guard->last_rejection = IMU_LIFT_REJECT_INVALID;
}

static inline bool imu_lift_guard_reset_ready (const imu_lift_guard_t * p_guard)
{
    return (((p_guard != NULL) && (p_guard->armed)) &&
            (p_guard->current_valid)) &&
           (p_guard->upright_samples >= IMU_LIFT_UPRIGHT_SAMPLES);
}

/* Called only after the caller has accepted an authoritative reset request.
 * Upright freshness is checked again here; the lifetime event count survives.
 */
static inline bool imu_lift_guard_clear_latch (imu_lift_guard_t * p_guard)
{
    if (!imu_lift_guard_reset_ready(p_guard))
    {
        return false;
    }
    p_guard->latched = false;
    p_guard->reason = 0U;
    p_guard->event_time_ms = 0U;
    p_guard->up_samples = 0U;
    p_guard->tilt_samples = 0U;
    return true;
}

static inline uint16_t imu_lift_clamp_u12 (uint32_t value, bool * p_saturated)
{
    if (value > 4095U)
    {
        *p_saturated = true;
        return 4095U;
    }
    return (uint16_t)value;
}

static inline int16_t imu_lift_clamp_s13 (int64_t value, bool * p_saturated)
{
    if (value > 4095)
    {
        *p_saturated = true;
        return 4095;
    }
    if (value < -4096)
    {
        *p_saturated = true;
        return -4096;
    }
    return (int16_t)value;
}

/*
 * Consume one oldest-first FIFO batch. `fresh_batch` must be false for failed,
 * empty, late, or overrun reads and before calibration/after an IMU fault.
 * Those cases clear consecutive dwell and readiness without advancing time or
 * clearing an already latched event. The intended sensor sample period is
 * 2,500 us (400 Hz); a mismatched period is rejected rather than miscounted.
 */
static inline void
imu_lift_guard_batch (imu_lift_guard_t * p_guard, const int16_t baseline_mg[3],
                      int32_t gravity_mg, const imu_lift_vec_t * p_samples,
                      uint8_t count, uint32_t sample_period_us, uint32_t now_ms,
                      bool fresh_batch)
{
    if (NULL == p_guard)
    {
        return;
    }
    if ((!fresh_batch) || NULL == baseline_mg || NULL == p_samples ||
        (0U == count) || (count > B4_FIFO_DEPTH) ||
        (sample_period_us != B4_SAMPLE_US) || (gravity_mg < B4_CAL_G_MIN_MG) ||
        (gravity_mg > B4_CAL_G_MAX_MG))
    {
        imu_lift_guard_no_fresh_batch(p_guard);
        return;
    }

    p_guard->armed = true;
    for (uint32_t index = 0U; index < count; ++index)
    {
        int64_t const dot = (int64_t)baseline_mg[0] * p_samples[index].fwd_mg +
                            (int64_t)baseline_mg[1] * p_samples[index].lat_mg +
                            (int64_t)baseline_mg[2] * p_samples[index].up_mg;
        uint64_t const norm2 = (uint64_t)((int64_t)p_samples[index].fwd_mg *
                                          p_samples[index].fwd_mg) +
                               (uint64_t)((int64_t)p_samples[index].lat_mg *
                                          p_samples[index].lat_mg) +
                               (uint64_t)((int64_t)p_samples[index].up_mg *
                                          p_samples[index].up_mg);
        uint32_t const norm = imu_lift_isqrt_u64(norm2);
        int64_t const projection = dot / gravity_mg;
        bool saturated = false;
        bool const norm_in_band =
            ((uint64_t)norm * 4U >= (uint64_t)gravity_mg * 3U) &&
            ((uint64_t)norm * 4U <= (uint64_t)gravity_mg * 5U);
        /* Cos(45 deg) ~= 0.7071; compare integer dot products, no atan/trig. */
        bool const steep_tilt =
            norm_in_band &&
            ((dot * 32768) <= ((int64_t)gravity_mg * norm * 23171));
        /* Cos(20 deg) ~= 0.94; reset requires a stable, level platform. */
        bool const upright =
            (norm_in_band &&
             ((dot * 100) >= ((int64_t)gravity_mg * norm * 94))) &&
            (projection <= (int64_t)gravity_mg + IMU_LIFT_UPRIGHT_EXTRA_MG);
        bool const upward_accel =
            projection >= (int64_t)gravity_mg + IMU_LIFT_ACCEL_EXTRA_MG;

        p_guard->up_projection_bits = compact_signed_pack(
            imu_lift_clamp_s13(projection, &saturated), 13U);
        p_guard->norm_mg = imu_lift_clamp_u12(norm, &saturated);
        p_guard->saturated = saturated;
        p_guard->current_valid = true;

        int64_t const cosine_denominator = (int64_t)gravity_mg * norm;
        int64_t cosine_q15 =
            cosine_denominator > 0 ? (dot * 32768) / cosine_denominator : 0;
        if (cosine_q15 > INT16_MAX)
        {
            cosine_q15 = INT16_MAX;
        }
        if (cosine_q15 < INT16_MIN)
        {
            cosine_q15 = INT16_MIN;
        }
        uint32_t const age_ms =
            ((uint32_t)count - 1U - index) * sample_period_us / 1000U;
        uint32_t const sample_ms = now_ms - age_ms;
        if (!p_guard->window_set)
        {
            p_guard->window_set = true;
            p_guard->peak_window_start_ms = sample_ms;
        }
        /* Keep the orientation high-water only for gravity-like magnitudes.
         * Dynamic/out-of-band samples report a norm rejection, not a valid
         * peak. */
        if (norm_in_band &&
            ((!p_guard->min_cos_set) || (cosine_q15 < p_guard->min_cos_q15)))
        {
            p_guard->min_cos_set = true;
            p_guard->min_cos_q15 = (int16_t)cosine_q15;
            p_guard->min_cos_norm_mg =
                (uint16_t)((norm > UINT16_MAX) ? UINT16_MAX : norm);
            p_guard->min_cos_sample_ms = sample_ms;
        }
        if ((!p_guard->peak_up_set) ||
            (imu_lift_projection(p_guard) > p_guard->peak_up_projection_mg))
        {
            p_guard->peak_up_set = true;
            p_guard->peak_up_projection_mg = imu_lift_projection(p_guard);
        }
        p_guard->peak_saturated = (p_guard->peak_saturated) || saturated;

        if (upright)
        {
            if (p_guard->upright_samples < IMU_LIFT_UPRIGHT_SAMPLES)
            {
                p_guard->upright_samples++;
            }
        }
        else
        {
            p_guard->upright_samples = 0U;
        }

        if (p_guard->latched)
        {
            continue; /* retain first event; continue measuring reset readiness
                       */
        }

        if (upward_accel)
        {
            if (p_guard->up_samples < IMU_LIFT_UP_SAMPLES)
            {
                p_guard->up_samples++;
            }
        }
        else
        {
            p_guard->up_samples = 0U;
        }
        if (steep_tilt)
        {
            if (p_guard->tilt_samples < IMU_LIFT_TILT_SAMPLES)
            {
                p_guard->tilt_samples++;
            }
        }
        else
        {
            p_guard->tilt_samples = 0U;
        }

        if (p_guard->up_samples > p_guard->peak_up_samples)
        {
            p_guard->peak_up_samples = p_guard->up_samples;
        }
        if (p_guard->tilt_samples > p_guard->peak_tilt_samples)
        {
            p_guard->peak_tilt_samples = p_guard->tilt_samples;
        }
        uint32_t const candidate = (upward_accel && steep_tilt)
                                       ? IMU_LIFT_CANDIDATE_BOTH
                                   : upward_accel ? IMU_LIFT_CANDIDATE_UP_ACCEL
                                   : steep_tilt   ? IMU_LIFT_CANDIDATE_TILT
                                                  : IMU_LIFT_CANDIDATE_NONE;
        if (candidate != IMU_LIFT_CANDIDATE_NONE)
        {
            p_guard->last_candidate = candidate;
        }
        if (!norm_in_band)
        {
            p_guard->last_rejection = IMU_LIFT_REJECT_NORM;
        }

        if ((p_guard->up_samples >= IMU_LIFT_UP_SAMPLES) ||
            (p_guard->tilt_samples >= IMU_LIFT_TILT_SAMPLES))
        {
            p_guard->latched = true;
            /* The categorical wire field permits only 1 or 2; tilt wins if
             * both qualifications complete in the same batch. */
            p_guard->reason = (p_guard->tilt_samples >= IMU_LIFT_TILT_SAMPLES)
                                  ? IMU_LIFT_REASON_TILT
                                  : IMU_LIFT_REASON_UP_ACCEL;
            if (p_guard->event_count < UINT16_MAX)
            {
                p_guard->event_count++;
            }
            /* Timestamp the sample that completed dwell; FIFO is oldest-first.
             */
            p_guard->event_time_ms = now_ms - age_ms;
        }
    }
}

#endif /* IMU_LIFT_GUARD_H */

/* End of file. */
