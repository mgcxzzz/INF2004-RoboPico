/**
 * @file    b5_pack.c
 * @brief   Binary telemetry frame for Buddy 5 (masks and shifts, fixed
 *          little-endian layout; schema in ../system/sys_tlm.h, source 5).
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
    return sys_sat_u(value, bits);
}

/**
 * @brief  Pack a status view into the telemetry frame: the 3-byte common
 *         header (source 5) and the 16-byte payload (layout in
 *         ../system/sys_tlm.h). Every field is clamped first, so nothing
 *         wraps.
 */
void
b5_tlm_pack (b5_tlm_t const * p_in, uint8_t p_frame[B5_TLM_FRAME_BYTES])
{
    uint8_t * p  = &p_frame[TLM_HDR_BYTES];
    uint32_t  w0 = 0U;
    uint32_t  w1 = 0U;
    uint32_t  w2 = 0U;
    uint32_t  w3 = 0U;
    int32_t   servo = b5_clamp(p_in->servo_dd, B5_ANGLE_MIN_DD,
                               B5_ANGLE_MAX_DD);

    w0 = b5_put_u(w0,  0U, B5_BITS_STATE, fit_u(p_in->state, B5_BITS_STATE));
    w0 = b5_put_u(w0,  4U, B5_BITS_FAULT, fit_u(p_in->fault, B5_BITS_FAULT));
    w0 = b5_put_u(w0,  7U, B5_BITS_SIDE,  fit_u(p_in->side, B5_BITS_SIDE));
    w0 = b5_put_u(w0,  9U, B5_BITS_COUNT,
                  fit_u(p_in->blocked_tries, B5_BITS_COUNT));
    w0 = b5_put_u(w0, 11U, B5_BITS_COUNT,
                  fit_u(p_in->recover_tries, B5_BITS_COUNT));
    w0 = b5_put_s(w0, 13U, B5_BITS_ANGLE, servo);
    w0 = b5_put_u(w0, 24U, 1U, p_in->b_found ? 1U : 0U);

    w1 = b5_put_u(w1,  0U, B5_BITS_MM, fit_u(p_in->range_mm, B5_BITS_MM));
    w1 = b5_put_u(w1, 12U, B5_BITS_MM, fit_u(p_in->closest_mm, B5_BITS_MM));
    w1 = b5_put_u(w1, 24U, B5_BITS_PCT,
                  fit_u(p_in->confidence_pct, B5_BITS_PCT));

    w2 = b5_put_u(w2,  0U, B5_BITS_MM, fit_u(p_in->width_mm, B5_BITS_MM));
    w2 = b5_put_u(w2, 12U, B5_BITS_MM, fit_u(p_in->lateral_mm, B5_BITS_MM));

    w3 = b5_put_u(w3,  0U, B5_BITS_COUNTER, p_in->pings);
    w3 = b5_put_u(w3, 16U, B5_BITS_COUNTER, p_in->no_echo);

    tlm_put_header(p_frame, TLM_SRC_OBSTACLE, B5_TLM_PAYLOAD_BYTES,
                   p_in->seq);
    sys_put_le32(&p[0],  w0);
    sys_put_le32(&p[4],  w1);
    sys_put_le32(&p[8],  w2);
    sys_put_le32(&p[12], w3);
}

/**
 * @brief  Decode a telemetry frame (used by tests and the bench console;
 *         the PC does the same with the schema in ../system/sys_tlm.h).
 * @return false (and nothing decoded) if the header is not a version-2
 *         Buddy 5 frame of the right length.
 */
bool
b5_tlm_unpack (uint8_t const * p_frame, uint32_t len, b5_tlm_t * p_out)
{
    uint8_t const * p = &p_frame[TLM_HDR_BYTES];
    uint32_t        w0;
    uint32_t        w1;
    uint32_t        w2;
    uint32_t        w3;

    if (!tlm_check_header(p_frame, len, TLM_SRC_OBSTACLE,
                          B5_TLM_PAYLOAD_BYTES, &p_out->seq))
    {
        return false;
    }

    w0 = sys_get_le32(&p[0]);
    w1 = sys_get_le32(&p[4]);
    w2 = sys_get_le32(&p[8]);
    w3 = sys_get_le32(&p[12]);

    p_out->state          = (uint8_t)b5_get_u(w0, 0U, B5_BITS_STATE);
    p_out->fault          = (uint8_t)b5_get_u(w0, 4U, B5_BITS_FAULT);
    p_out->side           = (uint8_t)b5_get_u(w0, 7U, B5_BITS_SIDE);
    p_out->blocked_tries  = (uint8_t)b5_get_u(w0, 9U, B5_BITS_COUNT);
    p_out->recover_tries  = (uint8_t)b5_get_u(w0, 11U, B5_BITS_COUNT);
    p_out->servo_dd       = (int16_t)b5_get_s(w0, 13U, B5_BITS_ANGLE);
    p_out->b_found        = (b5_get_u(w0, 24U, 1U) != 0U);
    p_out->range_mm       = (uint16_t)b5_get_u(w1, 0U, B5_BITS_MM);
    p_out->closest_mm     = (uint16_t)b5_get_u(w1, 12U, B5_BITS_MM);
    p_out->confidence_pct = (uint8_t)b5_get_u(w1, 24U, B5_BITS_PCT);
    p_out->width_mm       = (uint16_t)b5_get_u(w2, 0U, B5_BITS_MM);
    p_out->lateral_mm     = (uint16_t)b5_get_u(w2, 12U, B5_BITS_MM);
    p_out->pings          = (uint16_t)b5_get_u(w3, 0U, B5_BITS_COUNTER);
    p_out->no_echo        = (uint16_t)b5_get_u(w3, 16U, B5_BITS_COUNTER);

    return true;
}
