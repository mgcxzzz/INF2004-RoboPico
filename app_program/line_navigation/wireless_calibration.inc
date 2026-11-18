/**
 * @file wireless_calibration.inc
 * @brief Control-owner adapter and protected calibration status snapshot.
 * @details Included by line_service.c. Never sample, apply or save on the
 * network task. Flash I/O is outside the copy lock and uses the existing port.
 */

typedef struct
{
    rl_cal_session_t session;
    line_calibration_t active;
    uint16_t raw_left;
    uint16_t raw_right;
    uint8_t origin;
    bool adc_ok;
} rl_cal_view_t;

static rl_cal_view_t g_calibration_view;

/** @brief Require complete Stop, no countdown/export and released buttons. */
static bool wireless_cal_allowed (void)
{
    bringup_status_t state;
    bringup_get_status(&state);
    return rtos_line_mode_change_allowed() && !(state.flags & BR_LATCHED) &&
           !g_bringup_irq_stop && !board_button_pressed(20U) &&
           !board_button_pressed(21U);
}

/** @brief Copy owner values; packet encoding runs only when reporting. */
static void wireless_cal_update_view (void)
{
    UINT key = enter();
    g_calibration_view.session = g_wireless_cal;
    g_calibration_view.active = g_calibration_value;
    g_calibration_view.raw_left = g_adc[0];
    g_calibration_view.raw_right = g_adc[2];
    g_calibration_view.origin = (uint8_t)g_cal_origin;
    g_calibration_view.adc_ok = g_adc_ok;
    leave(key);
}

/** @brief Consume only this control cycle's newly acquired frame. */
static void wireless_cal_tick (void)
{
    uint32_t now = board_kernel_ms();
    bool allowed = true;
    uint32_t serial = 0U;
    if (g_wireless_cal.session)
    {
        allowed = wireless_cal_allowed();
        UINT key = enter();
        serial = g_cancel_serial;
        leave(key);
    }
    rl_cal_sample(&g_wireless_cal, now, serial, allowed, g_adc_ok, g_adc[0],
                  g_adc[2]);
    wireless_cal_update_view();
}

/** @brief Execute a current wireless request on the ADC/control owner only. */
uint8_t rtos_line_cal_command (unsigned operation, uint8_t request_id)
{
    bool allowed = wireless_cal_allowed();
    uint32_t now = board_kernel_ms();
    UINT key = enter();
    uint32_t serial = g_cancel_serial;
    leave(key);
    bool commit = false;
    bool fresh = g_adc_ok && now - g_wireless_cal.sample_ms <= 100U;
    uint8_t result = rl_cal_request(&g_wireless_cal, operation, request_id, now,
                                    serial, allowed, fresh, &commit);
    if (WP_ACK_ACCEPTED == result && commit)
    {
        line_calibration_t candidate = rl_cal_candidate(&g_wireless_cal);
        bool verified = true;
        if (RL_CAL_SAVE == operation)
        {
            uint8_t page[256];
            rl_cal_encode(page, &candidate);
            verified = board_line_save(page);
        }
        if (verified)
        {
            g_calibration_value = candidate;
            g_cal_origin = RL_CAL_SAVE == operation ? 2U : 3U;
            g_filter_ready = false;
            g_wireless_cal.session = 0U;
            g_wireless_cal.applied = 1U;
            g_wireless_cal.saved = RL_CAL_SAVE == operation;
            g_wireless_cal.result =
                RL_CAL_SAVE == operation ? RL_CAL_SAVED : RL_CAL_APPLIED;
            ++g_wireless_cal.revision;
        }
        else
        {
            g_wireless_cal.result = RL_CAL_SAVE_FAILED;
            g_wireless_cal.ack = WP_ACK_BUSY;
            result = WP_ACK_BUSY;
        }
    }
    wireless_cal_update_view();
    return result;
}

/** @brief Encode a copied status outside the lock for a periodic or reply
 * frame. */
void rtos_line_cal_info (uint8_t sequence, uint8_t out[RL_CAL_FRAME_BYTES])
{
    UINT key = enter();
    rl_cal_view_t view = g_calibration_view;
    leave(key);
    uint32_t now = board_kernel_ms();
    bool fresh = view.adc_ok && now - view.session.sample_ms <= 100U;
    rl_cal_encode_status(&view.session, &view.active, view.origin, now,
                         wireless_cal_allowed(), fresh, view.raw_left,
                         view.raw_right, out);
    out[2] = sequence;
}

/* End of file. */
