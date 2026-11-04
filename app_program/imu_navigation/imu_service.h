#ifndef RTOS_IMU_SERVICE_H
#define RTOS_IMU_SERVICE_H
#include "../rtos_bringup/bringup.h"
#include "imu_motion.h"
#define RI_TASK_PRIORITY 16
#define RI_TASK_PROCESSOR TP_PRC2
void ri_init(void);
void ri_task(INT code, void *exinf);
void ri_report(void);
void ri_reset_request(void);
void ri_observe(const MotionFeedback *f, const uint32_t invalid[2], uint32_t dt_us, bool running);
void ri_tilt_snapshot(b4v23_tilt_t *out);
#endif
