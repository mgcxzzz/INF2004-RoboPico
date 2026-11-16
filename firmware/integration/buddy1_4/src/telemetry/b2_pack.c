/**
 * @file b2_pack.c
 * @brief B2 pack.
 * @details Diagnostic records have explicit lengths, signed domains and
 * reserved bits. Never substitute native structure memory for the documented
 * byte encoding.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
/* Source-2 sensing snapshot: pure packing, no network or hardware calls. */
#include "src/telemetry/sys_tlm.h"
static void put16 (uint8_t * p_data, uint16_t value)
{
    p_data[0] = (uint8_t)value;
    p_data[1] = (uint8_t)(value >> 8);
}
static uint8_t axis_mad_byte (uint16_t mg_value)
{
    uint32_t value = ((uint32_t)mg_value + B2_TLM_AXIS_MAD_LSB_MG - 1U) /
                     B2_TLM_AXIS_MAD_LSB_MG;
    return (uint8_t)((value > 255U) ? 255U : value);
}
static uint16_t get16 (uint8_t const * p_data)
{
    return (uint16_t)((uint16_t)p_data[0] | ((uint16_t)p_data[1] << 8));
}
void b2_tlm_pack (b2_tlm_t const * p_in_value_value,
                  uint8_t frame[B2_TLM_FRAME_BYTES])
{
    uint8_t * p_data = &frame[TLM_HDR_BYTES];
    tlm_put_header(frame, TLM_SRC_MOTION, B2_TLM_PAYLOAD_BYTES,
                   p_in_value_value->seq);
    p_data[0] = p_in_value_value->flags & 15U;
    p_data[1] = 0U;
    put16(p_data + 2, (uint16_t)p_in_value_value->left_mm_s);
    put16(p_data + 4, (uint16_t)p_in_value_value->right_mm_s);
    put16(p_data + 6, (uint16_t)p_in_value_value->centre_mm_s);
    put16(p_data + 8, (uint16_t)p_in_value_value->roll_dd);
    put16(p_data + 10, (uint16_t)p_in_value_value->side_height_mm);
    sys_put_le32(p_data + 12, (uint32_t)p_in_value_value->distance_mm);
    put16(p_data + 16, p_in_value_value->invalid_left);
    put16(p_data + 18, p_in_value_value->invalid_right);
    p_data[20] = p_in_value_value->quality_reasons;
    p_data[21] = p_in_value_value->ramp_state & 15U;
    p_data[22] = p_in_value_value->quality_bad_samples;
    p_data[23] = p_in_value_value->quality_total_samples;
    put16(p_data + 24, p_in_value_value->max_spread_mg);
    put16(p_data + 26, p_in_value_value->max_accel_mm_s2);
    put16(p_data + 28, p_in_value_value->max_turn_dd_s);
    p_data[30] = p_in_value_value->rate_samples;
    p_data[31] = axis_mad_byte(p_in_value_value->max_axis_mad_mg[0]);
    put16(p_data + 32, p_in_value_value->max_mad_mg);
    p_data[34] = p_in_value_value->min_fifo_samples;
    p_data[35] = axis_mad_byte(p_in_value_value->max_axis_mad_mg[1]);
    p_data[36] = axis_mad_byte(p_in_value_value->max_axis_mad_mg[2]);
}
bool b2_tlm_unpack (uint8_t const * p_frame_value, uint32_t len,
                    b2_tlm_t * p_out_value)
{
    uint8_t const * p_data;
    bool legacy, reasons, numeric, robust;
    if (!p_out_value)
    {
        return false;
    }
    legacy = tlm_check_header(p_frame_value, len, TLM_SRC_MOTION,
                              B2_TLM_LEGACY_PAYLOAD_BYTES, &p_out_value->seq);
    reasons = tlm_check_header(p_frame_value, len, TLM_SRC_MOTION,
                               B2_TLM_REASON_PAYLOAD_BYTES, &p_out_value->seq);
    numeric = tlm_check_header(p_frame_value, len, TLM_SRC_MOTION,
                               B2_TLM_NUMERIC_PAYLOAD_BYTES, &p_out_value->seq);
    robust = tlm_check_header(p_frame_value, len, TLM_SRC_MOTION,
                              B2_TLM_ROBUST_PAYLOAD_BYTES, &p_out_value->seq);
    if (((((!legacy) && (!reasons)) && (!numeric)) && (!robust)) &&
        (!tlm_check_header(p_frame_value, len, TLM_SRC_MOTION,
                           B2_TLM_PAYLOAD_BYTES, &p_out_value->seq)))
    {
        return false;
    }
    p_data = &p_frame_value[TLM_HDR_BYTES];
    if (((p_data[0] & 240U) != 0U) || (p_data[1] != 0U))
    {
        return false;
    }
    p_out_value->flags = p_data[0];
    p_out_value->left_mm_s = (int16_t)get16(p_data + 2);
    p_out_value->right_mm_s = (int16_t)get16(p_data + 4);
    p_out_value->centre_mm_s = (int16_t)get16(p_data + 6);
    p_out_value->roll_dd = (int16_t)get16(p_data + 8);
    p_out_value->side_height_mm = (int16_t)get16(p_data + 10);
    p_out_value->distance_mm = (int32_t)sys_get_le32(p_data + 12);
    p_out_value->invalid_left = get16(p_data + 16);
    p_out_value->invalid_right = get16(p_data + 18);
    p_out_value->quality_reasons = p_out_value->ramp_state =
        p_out_value->quality_bad_samples = p_out_value->quality_total_samples =
            0U;
    p_out_value->max_spread_mg = p_out_value->max_accel_mm_s2 =
        p_out_value->max_turn_dd_s = 0U;
    p_out_value->rate_samples = 0U;
    p_out_value->max_mad_mg = 0U;
    p_out_value->min_fifo_samples = 0U;
    p_out_value->max_axis_mad_mg[0] = p_out_value->max_axis_mad_mg[1] =
        p_out_value->max_axis_mad_mg[2] = 0U;
    if (!legacy)
    {
        if (((p_data[21] & 240U) != 0U) || (p_data[22] > p_data[23]))
        {
            return false;
        }
        p_out_value->quality_reasons = p_data[20];
        p_out_value->ramp_state = p_data[21];
        p_out_value->quality_bad_samples = p_data[22];
        p_out_value->quality_total_samples = p_data[23];
        if (!reasons)
        {
            if (((p_data[31] != 0U) && (numeric || robust)) ||
                (p_data[30] > p_data[23]))
            {
                return false;
            }
            p_out_value->max_spread_mg = get16(p_data + 24);
            p_out_value->max_accel_mm_s2 = get16(p_data + 26);
            p_out_value->max_turn_dd_s = get16(p_data + 28);
            p_out_value->rate_samples = p_data[30];
            if (!numeric)
            {
                if ((robust && (p_data[35] != 0U)) || (p_data[34] > 32U))
                {
                    return false;
                }
                p_out_value->max_mad_mg = get16(p_data + 32);
                p_out_value->min_fifo_samples = p_data[34];
                if (!robust)
                {
                    uint8_t top = p_data[31];
                    if (p_data[35] > top)
                    {
                        top = p_data[35];
                    }
                    if (p_data[36] > top)
                    {
                        top = p_data[36];
                    }
                    /* Same ticks, so the largest axis byte must encode max MAD.
                     */
                    if (top != axis_mad_byte(p_out_value->max_mad_mg))
                    {
                        return false;
                    }
                    p_out_value->max_axis_mad_mg[0] =
                        (uint16_t)(p_data[31] * B2_TLM_AXIS_MAD_LSB_MG);
                    p_out_value->max_axis_mad_mg[1] =
                        (uint16_t)(p_data[35] * B2_TLM_AXIS_MAD_LSB_MG);
                    p_out_value->max_axis_mad_mg[2] =
                        (uint16_t)(p_data[36] * B2_TLM_AXIS_MAD_LSB_MG);
                }
            }
        }
    }
    return true;
}

/* End of file. */
