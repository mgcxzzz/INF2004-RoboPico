#ifndef RTOS_IMU_MOTION_H
#define RTOS_IMU_MOTION_H
/* Motion-core calls only. No hardware/RTOS types leak into the controller. */
#include "src/imu/imu_service.h"
#include "src/motion/ramp_drive.h"
#include "include/interfaces/nav_api.h"
void ri_new_run(void);
MotionRampAssist ri_ramp_step(uint64_t now, float dt, bool following, const float speed[2],
                              const int16_t target[2], const ContinuousDrive drive[2]);
void ri_ramp_halt(uint64_t now);
void ri_ramp_report(void);
static inline int32_t ri_unboosted(const ContinuousDrive *s, int32_t output) {
    int32_t b = (int32_t)(s->boost_pwm + .5f);
    return output > b ? output - b : 0;
}
#endif
