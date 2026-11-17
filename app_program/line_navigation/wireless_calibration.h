/**
 * @file wireless_calibration.h
 * @brief Staged, owner-only front IR calibration without motor commands.
 * @details Captures average 32 fresh owner frames. Active calibration is never
 * changed here; the owner commits it only after validation/verified saving.
 */
#ifndef ROBOPICO_WIRELESS_CALIBRATION_H
#define ROBOPICO_WIRELESS_CALIBRATION_H
#include "src/line_following/line_logic.h"
#include "src/network/wifi_protocol.h"
#include <string.h>

enum
{
    RL_CAL_READ,
    RL_CAL_BEGIN,
    RL_CAL_WHITE,
    RL_CAL_LEFT,
    RL_CAL_RIGHT,
    RL_CAL_APPLY,
    RL_CAL_SAVE,
    RL_CAL_CANCEL,
    RL_CAL_SAMPLES = 32U,
    RL_CAL_SESSION_MS = 120000U,
    RL_CAL_MAX_SPREAD = 128U,
    RL_CAL_FRAME_BYTES = 40U
};
enum
{
    RL_CAL_IDLE,
    RL_CAL_STAGING,
    RL_CAL_SAMPLING,
    RL_CAL_CAPTURED,
    RL_CAL_APPLIED,
    RL_CAL_SAVED,
    RL_CAL_CANCELLED,
    RL_CAL_INVALID,
    RL_CAL_BUSY,
    RL_CAL_ADC,
    RL_CAL_UNSTABLE,
    RL_CAL_TIMEOUT,
    RL_CAL_SAVE_FAILED
};
typedef struct
{
    uint32_t deadline_ms;
    uint32_t sample_ms;
    uint32_t serial;
    uint32_t request_ms;
    uint16_t revision;
    unsigned white_l : 12;
    unsigned white_r : 12;
    unsigned black_l : 12;
    unsigned black_r : 12;
    unsigned sum_l : 17;
    unsigned sum_r : 17;
    unsigned min_l : 12;
    unsigned min_r : 12;
    unsigned max_l : 12;
    unsigned max_r : 12;
    unsigned count : 6;
    unsigned captured : 3;
    unsigned capture : 2;
    unsigned operation : 3;
    unsigned request_id : 8;
    unsigned result : 4;
    unsigned ack : 3;
    unsigned session : 1;
    unsigned applied : 1;
    unsigned saved : 1;
    unsigned have_request : 1;
} rl_cal_session_t;

/** @brief Reconstruct the staged values; capture flags prevent partial use. */
static inline line_calibration_t
rl_cal_candidate (const rl_cal_session_t * p_cal)
{
    return (line_calibration_t){.white = {p_cal->white_l, p_cal->white_r},
                                .black = {p_cal->black_l, p_cal->black_r},
                                .mask = p_cal->captured};
}

/** @brief Discard a transaction without changing the active calibration. */
static inline void rl_cal_abort (rl_cal_session_t * p_cal, unsigned result)
{
    p_cal->session = 0U;
    p_cal->capture = 0U;
    p_cal->applied = 0U;
    p_cal->saved = 0U;
    p_cal->result = result;
}

/**
 * @brief Validate a fresh request; commit is true only for a new Apply/Save.
 * @details Duplicate id/operation within 5 seconds never repeats sampling or
 * flash writes. Read is nonmutating; Cancel remains possible with bad ADC.
 */
static inline uint8_t rl_cal_request (rl_cal_session_t * p_cal,
                                      unsigned operation, uint8_t request_id,
                                      uint32_t now, uint32_t serial,
                                      bool allowed, bool adc_ok,
                                      bool * p_commit)
{
    *p_commit = false;
    if (operation > RL_CAL_CANCEL)
    {
        return WP_ACK_FORMAT;
    }
    if (RL_CAL_READ == operation)
    {
        return WP_ACK_ACCEPTED;
    }
    if (p_cal->have_request && p_cal->request_id == request_id &&
        p_cal->operation == operation && now - p_cal->request_ms < 5000U)
    {
        return p_cal->ack;
    }
    if (p_cal->capture && RL_CAL_CANCEL != operation)
    {
        return WP_ACK_BUSY;
    }
    p_cal->have_request = 1U;
    p_cal->request_id = request_id;
    p_cal->operation = operation;
    p_cal->request_ms = now;
    p_cal->ack = WP_ACK_ACCEPTED;
    ++p_cal->revision;
    if (RL_CAL_CANCEL == operation)
    {
        rl_cal_abort(p_cal, RL_CAL_CANCELLED);
        return WP_ACK_ACCEPTED;
    }
    if (!allowed || !adc_ok)
    {
        p_cal->result = !allowed ? RL_CAL_BUSY : RL_CAL_ADC;
        p_cal->ack = WP_ACK_BUSY;
        return WP_ACK_BUSY;
    }
    if (RL_CAL_BEGIN == operation)
    {
        p_cal->captured = p_cal->count = p_cal->capture = 0U;
        p_cal->white_l = p_cal->white_r = p_cal->black_l = p_cal->black_r = 0U;
        p_cal->applied = p_cal->saved = 0U;
        p_cal->session = 1U;
        p_cal->serial = serial;
        p_cal->deadline_ms = now + RL_CAL_SESSION_MS;
        p_cal->result = RL_CAL_STAGING;
        return WP_ACK_ACCEPTED;
    }
    if (RL_CAL_APPLY == operation || RL_CAL_SAVE == operation)
    {
        line_calibration_t candidate = rl_cal_candidate(p_cal);
        if ((p_cal->session || p_cal->applied) && calibration_valid(&candidate))
        {
            *p_commit = true;
            return WP_ACK_ACCEPTED;
        }
    }
    else if (p_cal->session &&
             (RL_CAL_WHITE == operation || (p_cal->captured & 1U)))
    {
        p_cal->captured &= RL_CAL_WHITE == operation  ? 0U
                           : RL_CAL_LEFT == operation ? ~2U
                                                      : ~4U;
        p_cal->applied = p_cal->saved = 0U;
        p_cal->capture = operation - RL_CAL_WHITE + 1U;
        p_cal->count = p_cal->sum_l = p_cal->sum_r = 0U;
        p_cal->min_l = p_cal->min_r = 4095U;
        p_cal->max_l = p_cal->max_r = 0U;
        p_cal->deadline_ms = now + RL_CAL_SESSION_MS;
        p_cal->result = RL_CAL_SAMPLING;
        return WP_ACK_ACCEPTED;
    }
    p_cal->result = RL_CAL_INVALID;
    p_cal->ack = WP_ACK_FORMAT;
    return WP_ACK_FORMAT;
}

/** @brief Consume one fresh all-channel frame, or abort unsafe/stale staging.
 */
static inline void rl_cal_sample (rl_cal_session_t * p_cal, uint32_t now,
                                  uint32_t serial, bool allowed, bool adc_ok,
                                  uint16_t left, uint16_t right)
{
    if (adc_ok && left <= 4095U && right <= 4095U)
    {
        p_cal->sample_ms = now;
    }
    if (!p_cal->session)
    {
        return;
    }
    if (serial != p_cal->serial || !allowed)
    {
        rl_cal_abort(p_cal, RL_CAL_CANCELLED);
        return;
    }
    if ((int32_t)(now - p_cal->deadline_ms) >= 0)
    {
        rl_cal_abort(p_cal, RL_CAL_TIMEOUT);
        return;
    }
    if (!p_cal->capture)
    {
        return;
    }
    if (!adc_ok || left > 4095U || right > 4095U)
    {
        p_cal->capture = 0U;
        p_cal->result = RL_CAL_ADC;
        return;
    }
    p_cal->sum_l += left;
    p_cal->sum_r += right;
    if (left < p_cal->min_l)
    {
        p_cal->min_l = left;
    }
    if (right < p_cal->min_r)
    {
        p_cal->min_r = right;
    }
    if (left > p_cal->max_l)
    {
        p_cal->max_l = left;
    }
    if (right > p_cal->max_r)
    {
        p_cal->max_r = right;
    }
    if (++p_cal->count < RL_CAL_SAMPLES)
    {
        return;
    }
    unsigned capture = p_cal->capture;
    p_cal->capture = 0U;
    if (((1U == capture || 2U == capture) &&
         p_cal->max_l - p_cal->min_l > RL_CAL_MAX_SPREAD) ||
        ((1U == capture || 3U == capture) &&
         p_cal->max_r - p_cal->min_r > RL_CAL_MAX_SPREAD))
    {
        p_cal->result = RL_CAL_UNSTABLE;
        return;
    }
    unsigned mean_l = (p_cal->sum_l + RL_CAL_SAMPLES / 2U) / RL_CAL_SAMPLES;
    unsigned mean_r = (p_cal->sum_r + RL_CAL_SAMPLES / 2U) / RL_CAL_SAMPLES;
    if (1U == capture)
    {
        p_cal->white_l = mean_l;
        p_cal->white_r = mean_r;
        p_cal->captured = 1U;
    }
    else if (2U == capture)
    {
        p_cal->black_l = mean_l;
        p_cal->captured |= 2U;
    }
    else
    {
        p_cal->black_r = mean_r;
        p_cal->captured |= 4U;
    }
    p_cal->result = RL_CAL_CAPTURED;
    ++p_cal->revision;
}

/** @brief Pack up to 12 bits; inputs are ADC-domain values, bits unsigned. */
static inline void rl_cal_bits (uint8_t * p_out, unsigned at, unsigned width,
                                uint32_t value)
{
    for (unsigned index = 0U; index < width; ++index)
    {
        p_out[(at + index) / 8U] |=
            (uint8_t)(((value >> index) & 1U) << ((at + index) % 8U));
    }
}

/** @brief Encode source-15 CC/28; native structure bytes never cross the wire.
 */
static inline void rl_cal_encode_status (const rl_cal_session_t * p_cal,
                                         const line_calibration_t * p_active,
                                         unsigned origin, uint32_t now,
                                         bool allowed, bool adc_ok,
                                         uint16_t left, uint16_t right,
                                         uint8_t out[RL_CAL_FRAME_BYTES])
{
    memset(out, 0, RL_CAL_FRAME_BYTES);
    out[0] = 0xf2U;
    out[1] = 37U;
    uint8_t * p_data = out + 3U;
    p_data[0] = 0xccU;
    p_data[1] = 0x28U;
    for (unsigned index = 0U; index < 4U; ++index)
    {
        p_data[2U + index] = (uint8_t)(now >> (8U * index));
        p_data[29U + index] = (uint8_t)(p_cal->sample_ms >> (8U * index));
    }
    p_data[6] = (uint8_t)p_cal->revision;
    p_data[7] = (uint8_t)(p_cal->revision >> 8U);
    p_data[8] = p_cal->request_id;
    p_data[9] = (uint8_t)(p_cal->operation | (p_cal->result << 4U));
    line_calibration_t candidate = rl_cal_candidate(p_cal);
    p_data[10] =
        (uint8_t)(p_cal->session | (calibration_valid(&candidate) << 1U) |
                  (adc_ok << 2U) | (allowed << 3U) | (p_cal->applied << 4U) |
                  (p_cal->saved << 5U));
    p_data[11] = (uint8_t)(p_cal->captured | (p_cal->capture << 3U));
    p_data[12] = (uint8_t)p_cal->count;
    p_data[13] = (uint8_t)(origin & 3U);
    const uint16_t values[8] = {p_active->white[0], p_active->white[1],
                                p_active->black[0], p_active->black[1],
                                candidate.white[0], candidate.white[1],
                                candidate.black[0], candidate.black[1]};
    for (unsigned index = 0U; index < 8U; ++index)
    {
        rl_cal_bits(p_data + 14U, index * 12U, 12U, values[index]);
    }
    rl_cal_bits(p_data + 26U, 0U, 12U, left);
    rl_cal_bits(p_data + 26U, 12U, 12U, right);
    rl_cal_bits(p_data + 33U, 0U, 12U,
                p_cal->count ? (unsigned)(p_cal->max_l - p_cal->min_l) : 0U);
    rl_cal_bits(p_data + 33U, 12U, 12U,
                p_cal->count ? (unsigned)(p_cal->max_r - p_cal->min_r) : 0U);
    p_data[36] = p_cal->ack;
}
#endif

/* End of file. */
