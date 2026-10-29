/* Source-2 sensing snapshot: pure packing, no network or hardware calls. */
#include "../system/sys_tlm.h"
static void put16(uint8_t *p, uint16_t v) { p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); }
static uint8_t axis_mad_byte(uint16_t mg) {
    uint32_t v=((uint32_t)mg+B2_TLM_AXIS_MAD_LSB_MG-1U)/B2_TLM_AXIS_MAD_LSB_MG;
    return (uint8_t)((v>255U)?255U:v);
}
static uint16_t get16(uint8_t const *p) { return (uint16_t)((uint16_t)p[0]|((uint16_t)p[1]<<8)); }
void b2_tlm_pack(b2_tlm_t const *in, uint8_t frame[B2_TLM_FRAME_BYTES]) {
    uint8_t *p=&frame[TLM_HDR_BYTES];
    tlm_put_header(frame,TLM_SRC_MOTION,B2_TLM_PAYLOAD_BYTES,in->seq);
    p[0]=in->flags & 15U; p[1]=0U;
    put16(p+2,(uint16_t)in->left_mm_s); put16(p+4,(uint16_t)in->right_mm_s);
    put16(p+6,(uint16_t)in->centre_mm_s); put16(p+8,(uint16_t)in->roll_dd);
    put16(p+10,(uint16_t)in->side_height_mm); sys_put_le32(p+12,(uint32_t)in->distance_mm);
    put16(p+16,in->invalid_left); put16(p+18,in->invalid_right);
    p[20]=in->quality_reasons; p[21]=in->ramp_state & 15U;
    p[22]=in->quality_bad_samples; p[23]=in->quality_total_samples;
    put16(p+24,in->max_spread_mg); put16(p+26,in->max_accel_mm_s2);
    put16(p+28,in->max_turn_dd_s); p[30]=in->rate_samples;
    p[31]=axis_mad_byte(in->max_axis_mad_mg[0]);
    put16(p+32,in->max_mad_mg);p[34]=in->min_fifo_samples;
    p[35]=axis_mad_byte(in->max_axis_mad_mg[1]);
    p[36]=axis_mad_byte(in->max_axis_mad_mg[2]);
}
bool b2_tlm_unpack(uint8_t const *frame, uint32_t len, b2_tlm_t *out) {
    uint8_t const *p;
    bool legacy, reasons, numeric, robust;
    if (!out) return false;
    legacy=tlm_check_header(frame,len,TLM_SRC_MOTION,B2_TLM_LEGACY_PAYLOAD_BYTES,&out->seq);
    reasons=tlm_check_header(frame,len,TLM_SRC_MOTION,B2_TLM_REASON_PAYLOAD_BYTES,&out->seq);
    numeric=tlm_check_header(frame,len,TLM_SRC_MOTION,B2_TLM_NUMERIC_PAYLOAD_BYTES,&out->seq);
    robust=tlm_check_header(frame,len,TLM_SRC_MOTION,B2_TLM_ROBUST_PAYLOAD_BYTES,&out->seq);
    if (!legacy && !reasons && !numeric && !robust && !tlm_check_header(frame,len,TLM_SRC_MOTION,B2_TLM_PAYLOAD_BYTES,&out->seq)) return false;
    p=&frame[TLM_HDR_BYTES]; if ((p[0]&240U)!=0U || p[1]!=0U) return false;
    out->flags=p[0]; out->left_mm_s=(int16_t)get16(p+2); out->right_mm_s=(int16_t)get16(p+4);
    out->centre_mm_s=(int16_t)get16(p+6); out->roll_dd=(int16_t)get16(p+8);
    out->side_height_mm=(int16_t)get16(p+10); out->distance_mm=(int32_t)sys_get_le32(p+12);
    out->invalid_left=get16(p+16); out->invalid_right=get16(p+18);
    out->quality_reasons=out->ramp_state=out->quality_bad_samples=out->quality_total_samples=0U;
    out->max_spread_mg=out->max_accel_mm_s2=out->max_turn_dd_s=0U;
    out->rate_samples=0U;
    out->max_mad_mg=0U;out->min_fifo_samples=0U;
    out->max_axis_mad_mg[0]=out->max_axis_mad_mg[1]=out->max_axis_mad_mg[2]=0U;
    if (!legacy) {
        if ((p[21]&240U)!=0U || p[22]>p[23]) return false;
        out->quality_reasons=p[20]; out->ramp_state=p[21];
        out->quality_bad_samples=p[22]; out->quality_total_samples=p[23];
        if (!reasons) {
            if ((p[31]!=0U && (numeric || robust)) || p[30]>p[23]) return false;
            out->max_spread_mg=get16(p+24); out->max_accel_mm_s2=get16(p+26);
            out->max_turn_dd_s=get16(p+28); out->rate_samples=p[30];
            if (!numeric) {
                if((robust && p[35]!=0U) || p[34]>32U) return false;
                out->max_mad_mg=get16(p+32);out->min_fifo_samples=p[34];
                if (!robust) {
                    uint8_t top=p[31];
                    if (p[35]>top) top=p[35];
                    if (p[36]>top) top=p[36];
                    /* Same ticks, so the largest axis byte must encode max MAD. */
                    if (top!=axis_mad_byte(out->max_mad_mg)) return false;
                    out->max_axis_mad_mg[0]=(uint16_t)(p[31]*B2_TLM_AXIS_MAD_LSB_MG);
                    out->max_axis_mad_mg[1]=(uint16_t)(p[35]*B2_TLM_AXIS_MAD_LSB_MG);
                    out->max_axis_mad_mg[2]=(uint16_t)(p[36]*B2_TLM_AXIS_MAD_LSB_MG);
                }
            }
        }
    }
    return true;
}
