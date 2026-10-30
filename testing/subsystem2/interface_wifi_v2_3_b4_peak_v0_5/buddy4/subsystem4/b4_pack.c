/**
 * @file    b4_pack.c
 * @brief   Buddy 4: pack and unpack the IMU telemetry frame (source 4,
 *          layout in imu_iface.h), and the names used on consoles.
 *
 * Pure C, no kernel or hardware dependency: the same code runs in the
 * host tests (testing/subsystem4/host) and on the Pico.
 *
 * Group IS28, Buddy 4. Coding standard: Barr-C:2018.
 */

#include "imu_iface.h"

/**
 * @brief  Clamp every value to its field, then pack (sys_tlm.h rules).
 */
void
imu_tlm_pack (imu_tlm_t const * p_in, uint8_t p_frame[IMU_TLM_FRAME_BYTES])
{
    uint8_t * p  = &p_frame[TLM_HDR_BYTES];
    uint32_t  w0 = 0U;
    uint32_t  w1 = 0U;
    uint32_t  w2 = 0U;
    uint32_t  w3 = 0U;

    w0 = sys_put_u(w0,  0U, IMU_BITS_STATE,
                   sys_sat_u(p_in->state, IMU_BITS_STATE));
    w0 = sys_put_u(w0,  3U, IMU_BITS_MOTION,
                   sys_sat_u(p_in->motion, IMU_BITS_MOTION));
    w0 = sys_put_u(w0,  6U, IMU_BITS_EVT,
                   sys_sat_u(p_in->evt, IMU_BITS_EVT));
    w0 = sys_put_u(w0,  9U, IMU_BITS_DETAIL,
                   sys_sat_u(p_in->detail, IMU_BITS_DETAIL));
    w0 = sys_put_u(w0, 13U, 1U, p_in->b_valid ? 1U : 0U);
    w0 = sys_put_u(w0, 14U, 1U, p_in->b_odo ? 1U : 0U);
    w0 = sys_put_s(w0, 15U, IMU_BITS_PITCH,
                   sys_clamp(p_in->pitch_dd, -IMU_PITCH_MAX_DD,
                             IMU_PITCH_MAX_DD));
    w0 = sys_put_u(w0, 26U, IMU_BITS_FAULT,
                   sys_sat_u(p_in->fault, IMU_BITS_FAULT));

    w1 = sys_put_u(w1,  0U, IMU_BITS_HEIGHT, p_in->height_mm);
    w1 = sys_put_u(w1,  8U, IMU_BITS_HEIGHT, p_in->max_height_mm);
    w1 = sys_put_u(w1, 16U, IMU_BITS_COUNT8, p_in->humps);
    w1 = sys_put_u(w1, 24U, IMU_BITS_COUNT8, p_in->rejected);

    w2 = sys_put_u(w2,  0U, IMU_BITS_SHOCK_MG,
                   sys_sat_u(p_in->shock_mg, IMU_BITS_SHOCK_MG));
    w2 = sys_put_u(w2, 12U, IMU_BITS_LATENCY,
                   sys_sat_u(p_in->shock_ms, IMU_BITS_LATENCY));
    w2 = sys_put_u(w2, 18U, IMU_BITS_COUNT8, p_in->shocks);
    w2 = sys_put_u(w2, 26U, IMU_BITS_I2C_ERR, p_in->i2c_errors); /* wraps */

    w3 = sys_put_u(w3,  0U, IMU_BITS_MISSED, p_in->missed);
    w3 = sys_put_u(w3, 16U, IMU_BITS_TIME, p_in->t_ms);

    tlm_put_header(p_frame, TLM_SRC_IMU, IMU_TLM_PAYLOAD_BYTES, p_in->seq);
    sys_put_le32(&p[0],  w0);
    sys_put_le32(&p[4],  w1);
    sys_put_le32(&p[8],  w2);
    sys_put_le32(&p[12], w3);
}

/**
 * @return false (and nothing decoded) if the header is not a version-2
 *         IMU frame of the right length.
 */
bool
imu_tlm_unpack (uint8_t const * p_frame, uint32_t len, imu_tlm_t * p_out)
{
    uint8_t const * p = &p_frame[TLM_HDR_BYTES];
    uint32_t        w0;
    uint32_t        w1;
    uint32_t        w2;
    uint32_t        w3;

    if (!tlm_check_header(p_frame, len, TLM_SRC_IMU, IMU_TLM_PAYLOAD_BYTES,
                          &p_out->seq))
    {
        return false;
    }

    w0 = sys_get_le32(&p[0]);
    w1 = sys_get_le32(&p[4]);
    w2 = sys_get_le32(&p[8]);
    w3 = sys_get_le32(&p[12]);

    p_out->state         = (uint8_t)sys_get_u(w0, 0U, IMU_BITS_STATE);
    p_out->motion        = (uint8_t)sys_get_u(w0, 3U, IMU_BITS_MOTION);
    p_out->evt           = (uint8_t)sys_get_u(w0, 6U, IMU_BITS_EVT);
    p_out->detail        = (uint8_t)sys_get_u(w0, 9U, IMU_BITS_DETAIL);
    p_out->b_valid       = (sys_get_u(w0, 13U, 1U) != 0U);
    p_out->b_odo         = (sys_get_u(w0, 14U, 1U) != 0U);
    p_out->pitch_dd      = (int16_t)sys_get_s(w0, 15U, IMU_BITS_PITCH);
    p_out->fault         = (uint8_t)sys_get_u(w0, 26U, IMU_BITS_FAULT);
    p_out->height_mm     = (uint8_t)sys_get_u(w1, 0U, IMU_BITS_HEIGHT);
    p_out->max_height_mm = (uint8_t)sys_get_u(w1, 8U, IMU_BITS_HEIGHT);
    p_out->humps         = (uint8_t)sys_get_u(w1, 16U, IMU_BITS_COUNT8);
    p_out->rejected      = (uint8_t)sys_get_u(w1, 24U, IMU_BITS_COUNT8);
    p_out->shock_mg      = (uint16_t)sys_get_u(w2, 0U, IMU_BITS_SHOCK_MG);
    p_out->shock_ms      = (uint8_t)sys_get_u(w2, 12U, IMU_BITS_LATENCY);
    p_out->shocks        = (uint8_t)sys_get_u(w2, 18U, IMU_BITS_COUNT8);
    p_out->i2c_errors    = (uint8_t)sys_get_u(w2, 26U, IMU_BITS_I2C_ERR);
    p_out->missed        = (uint16_t)sys_get_u(w3, 0U, IMU_BITS_MISSED);
    p_out->t_ms          = (uint16_t)sys_get_u(w3, 16U, IMU_BITS_TIME);

    return true;
}

char const *
imu_state_name (imu_state_t state)
{
    static char const * const names[IMU_ST_COUNT] =
    {
        "Initialising", "Calibrating", "Idle", "Motion Tracking",
        "Hump Processing", "Impact / Shock", "Fault"
    };

    return ((uint32_t)state < (uint32_t)IMU_ST_COUNT) ? names[state] : "?";
}

char const *
imu_motion_name (imu_motion_t motion)
{
    static char const * const names[IMU_MOTION_COUNT] =
    {
        "unknown", "stationary", "steady", "accelerating", "turning",
        "climbing", "descending", "impact"
    };

    return ((uint32_t)motion < (uint32_t)IMU_MOTION_COUNT) ? names[motion]
                                                           : "?";
}

char const *
imu_evt_name (imu_evt_type_t evt)
{
    static char const * const names[IMU_EVT_COUNT] =
    {
        "status", "motion", "hump start", "hump peak", "hump rejected",
        "impact", "fault", "run summary"
    };

    return ((uint32_t)evt < (uint32_t)IMU_EVT_COUNT) ? names[evt] : "?";
}

char const *
imu_reject_name (imu_reject_t reason)
{
    static char const * const names[8] =
    {
        "none", "timeout", "no descent (a ramp?)", "height out of range",
        "height did not return to 0", "no odometry", "shock", "stopped"
    };

    return ((uint32_t)reason < 8U) ? names[reason] : "?";
}
