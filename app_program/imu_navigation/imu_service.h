#ifndef RTOS_IMU_SERVICE_H
#define RTOS_IMU_SERVICE_H
#include "../rtos_bringup/bringup.h"
#include "imu_motion.h"
/* Short 50Hz acquisition work precedes background USB3/WiFi4 on core0.
 * Motor/navigation/safety priorities and sole output owner stay on core1. */
#define RI_TASK_PRIORITY 2
#define RI_TASK_PROCESSOR TP_PRC1
void ri_init(void);
void ri_task(INT code, void *exinf);
void ri_report(void);
#ifdef APP_RTOS_RELEASE
void ri_release_diagnostics(uint8_t seq, uint32_t now_ms, uint8_t out[63]);
bool ri_safety_blocked(void);
unsigned ri_safety_reason(void); /* 0 safe, 1 lift, 2 overturn, 3 unready/stale */
bool ri_safety_can_reset(void);
bool ri_safety_reset_latch(void); /* authoritative atomic eligibility/request */
void ri_safety_packet(uint8_t seq, uint32_t now_ms, uint16_t run, uint8_t out[30]);
#endif
void ri_reset_request(void);
void ri_observe(const MotionFeedback *f, const uint32_t invalid[2], uint32_t dt_us, bool running);
void ri_tilt_snapshot(b4v23_tilt_t *out);
#endif
