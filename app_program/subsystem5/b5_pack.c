/**
 * @file    b5_pack.c
 * @brief   Binary telemetry frame for Buddy 5 (masks and shifts, fixed
 *          little-endian layout; see b5_pack.h for the bit map).
 *
 * Pure C, no kernel or hardware dependency, so it is unit-tested on the
 * host (testing/host/test_b5_pack.c) and on target (S5-U07..).
 *
 * Group IS28, Buddy 5. Coding standard: Barr-C:2018.
 */

#include "b5_pack.h"

/* Clamp an unsigned field to its width before packing. */
static uint32_t
fit_u(uint32_t value, uint32_t bits)
{
    uint32_t max = b5_mask(bits);

    return (value > max) ? max : value;
}

static void
put_le32(uint8_t * p, uint32_t word)
{
    p[0] = (uint8_t)(word & 0xFFU);
    p[1] = (uint8_t)((word >> 8) & 0xFFU);
    p[2] = (uint8_t)((word >> 16) & 0xFFU);
    p[3] = (uint8_t)((word >> 24) & 0xFFU);
}

static uint32_t
get_le32(uint8_t const * p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/**
 * @brief  Pack a status view into the 16-byte telemetry frame.
 *         Every field is clamped to its width first, so nothing wraps.
 */
void
b5_tlm_pack (b5_tlm_t const * p_in, uint8_t p_frame[B5_TLM_FRAME_BYTES])
{
    uint32_t w0 = 0U;
    uint32_t w1 = 0U;
    uint32_t w2 = 0U;
    uint32_t w3 = 0U;
    int32_t  servo = b5_clamp(p_in->servo_dd, B5_ANGLE_MIN_DD,
                              B5_ANGLE_MAX_DD);

    w0 = b5_put_u(w0,  0U, 4U, B5_TLM_VERSION);
    w0 = b5_put_u(w0,  4U, B5_BITS_STATE, fit_u(p_in->state, B5_BITS_STATE));
    w0 = b5_put_u(w0,  8U, B5_BITS_FAULT, fit_u(p_in->fault, B5_BITS_FAULT));
    w0 = b5_put_u(w0, 11U, B5_BITS_SIDE,  fit_u(p_in->side, B5_BITS_SIDE));
    w0 = b5_put_u(w0, 13U, B5_BITS_COUNT,
                  fit_u(p_in->blocked_tries, B5_BITS_COUNT));
    w0 = b5_put_u(w0, 15U, B5_BITS_COUNT,
                  fit_u(p_in->recover_tries, B5_BITS_COUNT));
    w0 = b5_put_s(w0, 17U, B5_BITS_ANGLE, servo);
    w0 = b5_put_u(w0, 28U, 1U, p_in->b_found ? 1U : 0U);

    w1 = b5_put_u(w1,  0U, B5_BITS_MM, fit_u(p_in->range_mm, B5_BITS_MM));
    w1 = b5_put_u(w1, 12U, B5_BITS_MM, fit_u(p_in->closest_mm, B5_BITS_MM));
    w1 = b5_put_u(w1, 24U, B5_BITS_PCT,
                  fit_u(p_in->confidence_pct, B5_BITS_PCT));

    w2 = b5_put_u(w2,  0U, B5_BITS_MM, fit_u(p_in->width_mm, B5_BITS_MM));
    w2 = b5_put_u(w2, 12U, B5_BITS_MM, fit_u(p_in->lateral_mm, B5_BITS_MM));

    w3 = b5_put_u(w3,  0U, B5_BITS_COUNTER, p_in->pings);
    w3 = b5_put_u(w3, 16U, B5_BITS_COUNTER, p_in->no_echo);

    put_le32(&p_frame[0],  w0);
    put_le32(&p_frame[4],  w1);
    put_le32(&p_frame[8],  w2);
    put_le32(&p_frame[12], w3);
}

/**
 * @brief  Decode a telemetry frame (used by tests and the bench console;
 *         the PC side does the same with the bit map in b5_pack.h).
 * @return false if the frame version is not one this code understands.
 */
bool
b5_tlm_unpack (uint8_t const p_frame[B5_TLM_FRAME_BYTES], b5_tlm_t * p_out)
{
    uint32_t w0 = get_le32(&p_frame[0]);
    uint32_t w1 = get_le32(&p_frame[4]);
    uint32_t w2 = get_le32(&p_frame[8]);
    uint32_t w3 = get_le32(&p_frame[12]);

    p_out->version        = (uint8_t)b5_get_u(w0, 0U, 4U);
    p_out->state          = (uint8_t)b5_get_u(w0, 4U, B5_BITS_STATE);
    p_out->fault          = (uint8_t)b5_get_u(w0, 8U, B5_BITS_FAULT);
    p_out->side           = (uint8_t)b5_get_u(w0, 11U, B5_BITS_SIDE);
    p_out->blocked_tries  = (uint8_t)b5_get_u(w0, 13U, B5_BITS_COUNT);
    p_out->recover_tries  = (uint8_t)b5_get_u(w0, 15U, B5_BITS_COUNT);
    p_out->servo_dd       = (int16_t)b5_get_s(w0, 17U, B5_BITS_ANGLE);
    p_out->b_found        = (b5_get_u(w0, 28U, 1U) != 0U);
    p_out->range_mm       = (uint16_t)b5_get_u(w1, 0U, B5_BITS_MM);
    p_out->closest_mm     = (uint16_t)b5_get_u(w1, 12U, B5_BITS_MM);
    p_out->confidence_pct = (uint8_t)b5_get_u(w1, 24U, B5_BITS_PCT);
    p_out->width_mm       = (uint16_t)b5_get_u(w2, 0U, B5_BITS_MM);
    p_out->lateral_mm     = (uint16_t)b5_get_u(w2, 12U, B5_BITS_MM);
    p_out->pings          = (uint16_t)b5_get_u(w3, 0U, B5_BITS_COUNTER);
    p_out->no_echo        = (uint16_t)b5_get_u(w3, 16U, B5_BITS_COUNTER);

    return (p_out->version == B5_TLM_VERSION);
}
