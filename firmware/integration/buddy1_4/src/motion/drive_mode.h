#ifndef BUDDY_DRIVE_MODE_H
#define BUDDY_DRIVE_MODE_H

#include <stdbool.h>
#include <stdint.h>

/* Runtime line-following cruise choices. These are RAM-only settings and do
 * not select the legacy circuit profile. */
typedef enum
{
    DRIVE_MODE_BASELINE45 = 0,
    DRIVE_MODE_NORMAL60 = 1,
    DRIVE_MODE_FASTER_TRIAL75 = 2,
    DRIVE_MODE_NONE = 3
} DriveMode;

enum
{
    DRIVE_MODE_BASELINE_SPEED = 45,
    DRIVE_MODE_NORMAL_SPEED = 60,
    DRIVE_MODE_TRIAL_SPEED = 75,
    DRIVE_MODE_LOAD_DWELL_US = 180000,
    DRIVE_MODE_LOAD_REARM_US = 300000,
    DRIVE_MODE_LOAD_MAX_PWM = 40
};

static inline bool drive_mode_valid (unsigned mode)
{
    return mode <= DRIVE_MODE_FASTER_TRIAL75;
}

static inline unsigned drive_mode_speed (unsigned mode)
{
    switch (mode)
    {
    case DRIVE_MODE_NORMAL60:
        return DRIVE_MODE_NORMAL_SPEED;
    case DRIVE_MODE_FASTER_TRIAL75:
        return DRIVE_MODE_TRIAL_SPEED;
    case DRIVE_MODE_BASELINE45:
    default:
        return DRIVE_MODE_BASELINE_SPEED;
    }
}

/* A smooth per-wheel reserve is limited to sustained underspeed while the
 * wheel is already moving. A true stall is left to existing startup/encoder
 * supervision. */
static inline float drive_mode_load_pwm_target (float commanded, float measured)
{
    if (!(commanded >= 25.0f) || !(measured >= 3.0f) ||
        measured >= commanded * 0.80f)
    {
        return 0.0f;
    }
    float deficit = commanded - measured;
    float target = deficit * 1.5f;
    return target > DRIVE_MODE_LOAD_MAX_PWM ? (float)DRIVE_MODE_LOAD_MAX_PWM
                                            : target;
}

#endif

/* End of file. */
