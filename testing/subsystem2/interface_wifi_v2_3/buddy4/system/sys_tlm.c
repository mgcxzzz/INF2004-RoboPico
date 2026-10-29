/**
 * @file    sys_tlm.c
 * @brief   Pack and unpack the Navigation Coordinator and Safety Supervisor
 *          telemetry frames (schema in sys_tlm.h).
 *
 * Pure C, no kernel or hardware dependency, so the same code is tested on
 * the host (testing/host/test_nav.c, test_safety.c) and runs on the Pico.
 *
 * Group IS28 (team). Coding standard: Barr-C:2018.
 */

#include "sys_tlm.h"

/* ---------------------------------------------------------------------
 * Navigation Coordinator (source 6)
 * ------------------------------------------------------------------ */

void
nav_tlm_pack (nav_tlm_t const * p_in, uint8_t p_frame[NAV_TLM_FRAME_BYTES])
{
    uint8_t * p   = &p_frame[TLM_HDR_BYTES];
    uint32_t  w0  = 0U;

    w0 = sys_put_u(w0,  0U, NAV_BITS_STATE,
                   sys_sat_u(p_in->state, NAV_BITS_STATE));
    w0 = sys_put_u(w0,  3U, NAV_BITS_OWNER,
                   sys_sat_u(p_in->owner, NAV_BITS_OWNER));
    w0 = sys_put_u(w0,  6U, 1U, p_in->b_slow ? 1U : 0U);
    w0 = sys_put_u(w0,  7U, NAV_BITS_STEP,
                   sys_sat_u(p_in->search_step, NAV_BITS_STEP));
    w0 = sys_put_u(w0, 10U, NAV_BITS_DROPPED, p_in->dropped);

    tlm_put_header(p_frame, TLM_SRC_NAV, NAV_TLM_PAYLOAD_BYTES, p_in->seq);
    sys_put_le32(&p[0], w0);
    p[4] = (uint8_t)(p_in->commands_sent & 0xFFU);
    p[5] = (uint8_t)((p_in->commands_sent >> 8) & 0xFFU);
}

/**
 * @return false (and nothing decoded) if the header is not a version-2
 *         Navigation Coordinator frame of the right length.
 */
bool
nav_tlm_unpack (uint8_t const * p_frame, uint32_t len, nav_tlm_t * p_out)
{
    uint8_t const * p = &p_frame[TLM_HDR_BYTES];
    uint32_t        w0;

    if (!tlm_check_header(p_frame, len, TLM_SRC_NAV, NAV_TLM_PAYLOAD_BYTES,
                          &p_out->seq))
    {
        return false;
    }

    w0 = sys_get_le32(&p[0]);
    p_out->state         = (uint8_t)sys_get_u(w0, 0U, NAV_BITS_STATE);
    p_out->owner         = (uint8_t)sys_get_u(w0, 3U, NAV_BITS_OWNER);
    p_out->b_slow        = (sys_get_u(w0, 6U, 1U) != 0U);
    p_out->search_step   = (uint8_t)sys_get_u(w0, 7U, NAV_BITS_STEP);
    p_out->dropped       = (uint8_t)sys_get_u(w0, 10U, NAV_BITS_DROPPED);
    p_out->commands_sent = (uint16_t)((uint16_t)p[4] |
                                      ((uint16_t)p[5] << 8));

    return true;
}

/* ---------------------------------------------------------------------
 * Safety Supervisor (source 7)
 * ------------------------------------------------------------------ */

void
safety_tlm_pack (safety_tlm_t const * p_in,
                 uint8_t p_frame[SAFETY_TLM_FRAME_BYTES])
{
    uint8_t * p  = &p_frame[TLM_HDR_BYTES];
    uint32_t  w0 = 0U;
    uint32_t  w1 = 0U;
    uint32_t  w2 = 0U;

    w0 = sys_put_u(w0,  0U, SAFETY_BITS_STATE,
                   sys_sat_u(p_in->state, SAFETY_BITS_STATE));
    w0 = sys_put_u(w0,  2U, 1U, p_in->b_inhibit ? 1U : 0U);
    w0 = sys_put_u(w0,  3U, SAFETY_BITS_SRC,
                   sys_sat_u(p_in->fault_src, SAFETY_BITS_SRC));
    w0 = sys_put_u(w0,  6U, SAFETY_BITS_SRC,
                   sys_sat_u(p_in->warn_src, SAFETY_BITS_SRC));
    w0 = sys_put_u(w0,  9U, SAFETY_BITS_CODE, p_in->fault_code);
    w0 = sys_put_u(w0, 17U, SAFETY_BITS_CODE, p_in->warn_code);

    w1 = sys_put_u(w1,  0U, SAFETY_BITS_COUNTER, p_in->estops);
    w1 = sys_put_u(w1,  8U, SAFETY_BITS_COUNTER, p_in->faults);
    w1 = sys_put_u(w1, 16U, SAFETY_BITS_COUNTER, p_in->warnings);
    w1 = sys_put_u(w1, 24U, SAFETY_BITS_COUNTER, p_in->slow_stops);

    w2 = sys_put_u(w2,  0U, SAFETY_BITS_STOP_MS,
                   sys_sat_u(p_in->last_stop_ms, SAFETY_BITS_STOP_MS));
    w2 = sys_put_u(w2, 10U, SAFETY_BITS_STOP_MS,
                   sys_sat_u(p_in->max_stop_ms, SAFETY_BITS_STOP_MS));
    w2 = sys_put_u(w2, 20U, SAFETY_BITS_MASK, p_in->fault_mask);

    tlm_put_header(p_frame, TLM_SRC_SAFETY, SAFETY_TLM_PAYLOAD_BYTES,
                   p_in->seq);
    sys_put_le32(&p[0], w0);
    sys_put_le32(&p[4], w1);
    sys_put_le32(&p[8], w2);
}

/**
 * @return false (and nothing decoded) if the header is not a version-2
 *         Safety Supervisor frame of the right length.
 */
bool
safety_tlm_unpack (uint8_t const * p_frame, uint32_t len,
                   safety_tlm_t * p_out)
{
    uint8_t const * p = &p_frame[TLM_HDR_BYTES];
    uint32_t        w0;
    uint32_t        w1;
    uint32_t        w2;

    if (!tlm_check_header(p_frame, len, TLM_SRC_SAFETY,
                          SAFETY_TLM_PAYLOAD_BYTES, &p_out->seq))
    {
        return false;
    }

    w0 = sys_get_le32(&p[0]);
    w1 = sys_get_le32(&p[4]);
    w2 = sys_get_le32(&p[8]);

    p_out->state        = (uint8_t)sys_get_u(w0, 0U, SAFETY_BITS_STATE);
    p_out->b_inhibit    = (sys_get_u(w0, 2U, 1U) != 0U);
    p_out->fault_src    = (uint8_t)sys_get_u(w0, 3U, SAFETY_BITS_SRC);
    p_out->warn_src     = (uint8_t)sys_get_u(w0, 6U, SAFETY_BITS_SRC);
    p_out->fault_code   = (uint8_t)sys_get_u(w0, 9U, SAFETY_BITS_CODE);
    p_out->warn_code    = (uint8_t)sys_get_u(w0, 17U, SAFETY_BITS_CODE);
    p_out->estops       = (uint8_t)sys_get_u(w1, 0U, SAFETY_BITS_COUNTER);
    p_out->faults       = (uint8_t)sys_get_u(w1, 8U, SAFETY_BITS_COUNTER);
    p_out->warnings     = (uint8_t)sys_get_u(w1, 16U, SAFETY_BITS_COUNTER);
    p_out->slow_stops   = (uint8_t)sys_get_u(w1, 24U, SAFETY_BITS_COUNTER);
    p_out->last_stop_ms = (uint16_t)sys_get_u(w2, 0U, SAFETY_BITS_STOP_MS);
    p_out->max_stop_ms  = (uint16_t)sys_get_u(w2, 10U, SAFETY_BITS_STOP_MS);
    p_out->fault_mask   = (uint8_t)sys_get_u(w2, 20U, SAFETY_BITS_MASK);

    return true;
}
