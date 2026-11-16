/**
 * @file line_service.h
 * @brief Line service.
 * @details ADC/control ownership covers sensor acquisition, planning and calibration.
 * Other tasks consume protected copies; flash and stored exports require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_LINE_SERVICE_H
#define RTOS_LINE_SERVICE_H
#include "../rtos_bringup/bringup.h"
#include "line_model.h"
#include "adc_reader.h"

#ifdef APP_RTOS_NETWORK
#include "src/network/wifi_packets.h"
void rtos_line_telemetry (wifi_motion_packet_t * p_out);
void rtos_line_resume (void);
#endif
#ifdef APP_RTOS_RELEASE
void rtos_line_release_info (uint8_t seq, uint8_t out[15]);
void rtos_line_barcode_attempt_info (uint8_t seq, uint32_t now_ms,
                                     uint8_t out[30]);
void rtos_line_junction_info (uint8_t seq, uint32_t now_ms, uint8_t out[20]);
void rtos_line_drive_info (uint8_t seq, uint32_t now_ms, uint8_t out[18]);
uint8_t rtos_line_log_reply (const wifi_command_t * p_command);
#endif
bool rtos_line_request_fresh (uint32_t received_us);
bool rtos_line_request_start (uint32_t received_us);
bool rtos_line_pending (void);
bool rtos_line_mode_change_allowed (void);
#if defined(APP_RTOS_RELEASE) && defined(APP_RTOS_NETWORK)
void rtos_line_drive_mode_info (uint8_t seq, uint32_t now_ms, uint8_t out[17]);
#endif
ER rtos_line_init (void);
bool rtos_line_command (unsigned command, uint32_t received_us);
void rtos_line_cancel (void);
bool rtos_line_start_valid (void);
void rtos_line_side_poll (uint64_t now);
bool rtos_line_prepare (const motion_feedback_t * p_flags, double path,
                        float delta_time, bool active);
bool rtos_line_ready (void);
void rtos_line_begin (void);
analog_demand_t rtos_line_follow (float delta_time);
void rtos_line_feedback (motion_feedback_t * p_flags,
                         const analog_demand_t * p_demand);
line_envelope_t rtos_line_power (const motion_output_t * p_output,
                                 analog_demand_t * p_demand, float delta_time);
void rtos_line_actual_output (unsigned wheel, int32_t output);
void rtos_line_record (const bringup_status_t * p_state,
                       const analog_demand_t * p_demand, uint32_t cost);
void rtos_line_report (void);
void rtos_line_dump_if_requested (void);
void rtos_line_flash_task (INT code, void * p_exinf);
bool board_line_adc_init (void);
bool board_line_adc_read (unsigned channel, unsigned count, bool spaced,
                          uint16_t * p_value);
void board_line_adc_diagnostics (rl_adc_diagnostics_t * p_out);
bool board_line_save (const uint8_t page[256]);
void board_line_flash_worker (void);
void board_motor_levels (unsigned wheel, uint16_t first, uint16_t second);
dual_levels_t board_motor_current (unsigned wheel);
extern UW tm_usb_pending (void);
#endif

/* End of file. */
