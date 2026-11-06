#ifndef RTOS_LINE_SERVICE_H
#define RTOS_LINE_SERVICE_H
#include "../rtos_bringup/bringup.h"
#include "line_model.h"
#include "adc_reader.h"

#ifdef APP_RTOS_NETWORK
#include "src/network/wifi_packets.h"
void rtos_line_telemetry(WifiMotionPacket *out);
void rtos_line_resume(void);
#endif
#ifdef APP_RTOS_RELEASE
void rtos_line_release_info(uint8_t seq, uint8_t out[15]);
void rtos_line_barcode_attempt_info(uint8_t seq, uint32_t now_ms, uint8_t out[30]);
uint8_t rtos_line_log_reply(const WifiCommand *command);
#endif
bool rtos_line_request_fresh(uint32_t received_us);
bool rtos_line_request_start(uint32_t received_us);
bool rtos_line_pending(void);
ER rtos_line_init(void);
bool rtos_line_command(unsigned command, uint32_t received_us);
void rtos_line_cancel(void);
bool rtos_line_start_valid(void);
void rtos_line_side_poll(uint64_t now);
bool rtos_line_prepare(const MotionFeedback *f, double path, float dt, bool active);
bool rtos_line_ready(void);
void rtos_line_begin(void);
AnalogDemand rtos_line_follow(float dt);
void rtos_line_feedback(MotionFeedback *f, const AnalogDemand *d);
LineEnvelope rtos_line_power(const MotionOutput *output, AnalogDemand *d, float dt);
void rtos_line_actual_output(unsigned w, int32_t output);
void rtos_line_record(const BringupStatus *s, const AnalogDemand *d, uint32_t cost);
void rtos_line_report(void);
void rtos_line_dump_if_requested(void);
void rtos_line_flash_task(INT code, void *exinf);
bool board_line_adc_init(void);
bool board_line_adc_read(unsigned channel, unsigned count, bool spaced, uint16_t *value);
void board_line_adc_diagnostics(RlAdcDiagnostics *out);
bool board_line_save(const uint8_t page[256]);
void board_line_flash_worker(void);
void board_motor_levels(unsigned wheel, uint16_t a, uint16_t b);
DualLevels board_motor_current(unsigned wheel);
extern UW tm_usb_pending(void);
#endif
