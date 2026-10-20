/**
 * @file    s3_pack.c
 * @brief   Buddy 3 telemetry frame: pack and unpack (schema in
 *          ../system/sys_tlm.h, source 3). Masks and shifts, fixed
 *          little-endian layout, every field saturated to its width.
 *
 * Group IS28, Buddy 3. Coding standard: Barr-C:2018.
 */

#include "../system/sys_tlm.h"

void
s3_tlm_pack (s3_tlm_t const * p_in, uint8_t p_frame[S3_TLM_FRAME_BYTES])
{
    uint8_t * p  = &p_frame[TLM_HDR_BYTES];
    uint32_t  w0 = 0U;
    uint32_t  w1 = 0U;
    uint32_t  w2 = 0U;

    w0 = sys_put_u(w0,  0U, 3U, sys_sat_u(p_in->state, 3U));
    w0 = sys_put_u(w0,  3U, 3U, sys_sat_u(p_in->sensors, 3U));
    w0 = sys_put_u(w0,  6U, 1U, p_in->b_granted ? 1U : 0U);
    w0 = sys_put_u(w0,  7U, 2U, sys_sat_u(p_in->side, 2U));
    w0 = sys_put_u(w0,  9U, 6U, sys_sat_u(p_in->last_char, 6U));
    w0 = sys_put_u(w0, 15U, 3U, sys_sat_u(p_in->last_result, 3U));
    w0 = sys_put_u(w0, 18U, 3U, sys_sat_u(p_in->last_cmd, 3U));
    w0 = sys_put_u(w0, 21U, 1U, p_in->b_dist ? 1U : 0U);

    w1 = sys_put_u(w1,  0U, 8U, p_in->barcodes_ok);
    w1 = sys_put_u(w1,  8U, 8U, p_in->bad_reads);
    w1 = sys_put_u(w1, 16U, 8U, p_in->backwards);
    w1 = sys_put_u(w1, 24U, 8U, p_in->junctions);

    w2 = sys_put_u(w2,  0U, 8U, p_in->lost);
    w2 = sys_put_u(w2,  8U, 8U, p_in->overflows);

    tlm_put_header(p_frame, TLM_SRC_LINE, S3_TLM_PAYLOAD_BYTES, p_in->seq);
    sys_put_le32(&p[0], w0);
    sys_put_le32(&p[4], w1);
    sys_put_le32(&p[8], w2);
}

/**
 * @return false (and nothing decoded) if the header is not a version-2
 *         Line and Barcode frame of the right length.
 */
bool
s3_tlm_unpack (uint8_t const * p_frame, uint32_t len, s3_tlm_t * p_out)
{
    uint8_t const * p = &p_frame[TLM_HDR_BYTES];
    uint32_t        w0;
    uint32_t        w1;
    uint32_t        w2;

    if (!tlm_check_header(p_frame, len, TLM_SRC_LINE, S3_TLM_PAYLOAD_BYTES,
                          &p_out->seq))
    {
        return false;
    }

    w0 = sys_get_le32(&p[0]);
    w1 = sys_get_le32(&p[4]);
    w2 = sys_get_le32(&p[8]);

    p_out->state       = (uint8_t)sys_get_u(w0, 0U, 3U);
    p_out->sensors     = (uint8_t)sys_get_u(w0, 3U, 3U);
    p_out->b_granted   = (sys_get_u(w0, 6U, 1U) != 0U);
    p_out->side        = (uint8_t)sys_get_u(w0, 7U, 2U);
    p_out->last_char   = (uint8_t)sys_get_u(w0, 9U, 6U);
    p_out->last_result = (uint8_t)sys_get_u(w0, 15U, 3U);
    p_out->last_cmd    = (uint8_t)sys_get_u(w0, 18U, 3U);
    p_out->b_dist      = (sys_get_u(w0, 21U, 1U) != 0U);
    p_out->barcodes_ok = (uint8_t)sys_get_u(w1, 0U, 8U);
    p_out->bad_reads   = (uint8_t)sys_get_u(w1, 8U, 8U);
    p_out->backwards   = (uint8_t)sys_get_u(w1, 16U, 8U);
    p_out->junctions   = (uint8_t)sys_get_u(w1, 24U, 8U);
    p_out->lost        = (uint8_t)sys_get_u(w2, 0U, 8U);
    p_out->overflows   = (uint8_t)sys_get_u(w2, 8U, 8U);

    return true;
}
