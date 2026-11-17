/**
 * @file wireless_calibration.inc
 * @brief Control-owner adapter and protected calibration status mailbox.
 * @details Included by line_service.c. Never sample, apply or save on the
 * network task. Flash I/O is outside the copy lock and uses the existing port.
 */

/** @brief Require complete Stop, no countdown/export and released buttons. */
static bool wireless_cal_allowed (void)
{
    bringup_status_t state;
    bringup_get_status(&state);
    return rtos_line_mode_change_allowed() && !(state.flags & BR_LATCHED) &&
           !g_bringup_irq_stop && !board_button_pressed(20U) &&
           !board_button_pressed(21U);
}

/** @brief Publish a coherent owner snapshot, with no I/O inside the lock. */
static void wireless_cal_publish (uint32_t now, bool allowed)
{
    uint8_t frame[RL_CAL_FRAME_BYTES];
    bool fresh = g_adc_ok && now - g_wireless_cal.sample_ms <= 100U;
    rl_cal_encode_status(&g_wireless_cal, &g_calibration_value, g_cal_origin,
                         now, allowed, fresh, g_adc[0], g_adc[2], frame);
    UINT key = enter();
    memcpy(g_calibration_frame, frame, sizeof(frame));
    leave(key);
}

/** @brief Consume only this control cycle's newly acquired frame. */
static void wireless_cal_tick (void)
{
    bool allowed = wireless_cal_allowed();
    uint32_t now = board_kernel_ms();
    UINT key = enter();
    uint32_t serial = g_cancel_serial;
    leave(key);
    rl_cal_sample(&g_wireless_cal, now, serial, allowed, g_adc_ok, g_adc[0],
                  g_adc[2]);
    wireless_cal_publish(now, allowed);
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
    wireless_cal_publish(board_kernel_ms(), allowed);
    return result;
}

/** @brief Network readers copy encoded bytes; they never inspect live state. */
void rtos_line_cal_info (uint8_t sequence, uint8_t out[RL_CAL_FRAME_BYTES])
{
    UINT key = enter();
    memcpy(out, g_calibration_frame, RL_CAL_FRAME_BYTES);
    leave(key);
    out[2] = sequence;
}

/* End of file. */
