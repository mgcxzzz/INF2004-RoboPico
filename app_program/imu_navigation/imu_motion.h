/**
 * @file imu_motion.h
 * @brief Imu motion.
 * @details The IMU task owns sensor acquisition and calibration; readers use snapshots.
 * Ramp requests expire through the coordinator; lift safety may remove output.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_IMU_MOTION_H
#define RTOS_IMU_MOTION_H
/* Motion-core calls only. No hardware/RTOS types leak into the controller. */
#include "src/imu/imu_service.h"
#include "src/motion/ramp_drive.h"
#include "include/interfaces/nav_api.h"
void ri_new_run (void);
/* Snapshot the IMU first, then read hardware time internally for its age. */
motion_ramp_assist_t ri_ramp_step (float delta_time, bool following,
                                   const float speed[2],
                                   const int16_t target[2],
                                   const continuous_drive_t drive[2]);
void ri_ramp_halt (uint64_t now);
void ri_ramp_report (void);
static inline int32_t ri_unboosted (const continuous_drive_t * p_state,
                                    int32_t output)
{
    int32_t second = (int32_t)(p_state->boost_pwm + .5f);
    return output > second ? output - second : 0;
}
#endif

/* End of file. */
