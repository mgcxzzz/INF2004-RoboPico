/**
 * @file line_logic.h
 * @brief Line logic.
 * @details The control task owns mutable steering/recovery state.
 * Keep straight, scan, turn and recovery behavior distinct; no direct PWM here.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef ANALOG_LINE_LOGIC_H
#define ANALOG_LINE_LOGIC_H
#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint16_t white[2];
    uint16_t black[2];
    uint8_t mask;
} line_calibration_t;
/* Standalone test targets may use a measured lower-contrast calibration.
 * Existing targets keep the historical100-count minimum by default. */
#ifndef LINE_CAL_MIN_SPAN
#define LINE_CAL_MIN_SPAN 100
#endif
_Static_assert((LINE_CAL_MIN_SPAN > 0) && (LINE_CAL_MIN_SPAN <= 4095),
               "Invalid front calibration minimum");
enum
{
    AL_EDGE,
    AL_RIGHT_GUARD,
    AL_BOTH_BLACK,
    AL_SEEK_LEFT,
    AL_LOST
};
typedef struct
{
    uint64_t white_since;
    uint64_t both_since;
    uint32_t white_mm;
    bool white;
    bool both;
    bool black_hold;
} analog_tracker_t;
typedef struct
{
    int16_t target[2];
    uint16_t level[2];
    uint8_t decision;
    bool stop;
} analog_demand_t;
enum
{
    LINE_BASE_MM_S = 45,
    LINE_DIFFERENTIAL_MM_S = 15
};
enum
{
    LINE_USB_HOLD,
    LINE_BANK,
    LINE_USB_FREE
};
enum
{
    START_OK,
    START_ACTIVE,
    START_SETTLING,
    START_CAL,
    START_GP20,
    START_REARM,
    START_LEFT_WHITE,
    START_RIGHT_DARK,
    START_HOLD_GP21,
    START_RELEASE_GP21,
    START_GUARD,
    START_LINE_ABSENT
};
typedef struct
{
    uint64_t since;
    bool timing;
    bool ready;
} line_rearm_t;
/* Readiness is permission for a NEW explicit request, never auto-start.
 * Holding GP21 after readiness is allowed for the original USB hold mode. */
static inline void line_rearm_step (line_rearm_t * p_result, uint64_t now,
                                    bool released, bool stationary, bool busy)
{
    if (busy || (!stationary))
    {
        *p_result = (line_rearm_t){0};
        return;
    }
    if (p_result->ready)
    {
        return;
    }
    if (!released)
    {
        p_result->timing = false;
        return;
    }
    if (!p_result->timing)
    {
        p_result->timing = true;
        p_result->since = now;
    }
    if (now - p_result->since >= 1000000)
    {
        p_result->ready = true;
    }
}
static inline unsigned line_start_block (bool active, bool settling, bool valid,
                                         bool stop, bool ready, uint16_t left,
                                         uint16_t right, bool hold,
                                         unsigned mode)
{
    if (active)
    {
        return START_ACTIVE;
    }
    if (settling)
    {
        return START_SETTLING;
    }
    if (!valid)
    {
        return START_CAL;
    }
    if (stop)
    {
        return START_GP20;
    }
    if (!ready)
    {
        return START_REARM;
    }
    /* Full left black is now permitted. All-white / right-dark still reject. */
    if (left < 150)
    {
        return START_LEFT_WHITE;
    }
    if (right > 450)
    {
        return START_RIGHT_DARK;
    }
    if (LINE_USB_HOLD == mode && (!hold))
    {
        return START_HOLD_GP21;
    }
    if ((mode != LINE_USB_HOLD) && hold)
    {
        return START_RELEASE_GP21;
    }
    return START_OK;
}
static inline bool line_countdown_cancel (bool connected, bool held, bool stop)
{
    return ((!connected) || held) || stop;
}

/* Profile3 isolates the existing wheel controllers from optical steering.
 * Deliberately forward-only; two-second stages include the normal speed ramp.
 * Caller retains button, feedback, direction, distance and deadline guards. */
enum
{
    RESPONSE_US = 6000000,
    RESPONSE_MM = 450
};
static inline analog_demand_t response_demand (uint64_t elapsed)
{
    analog_demand_t demand = {0};
    unsigned phase = (unsigned)(elapsed / 2000000u);
    demand.decision = (uint8_t)(5 + (phase > 2 ? 2 : phase));
    if (elapsed >= RESPONSE_US)
    {
        demand.stop = true;
        return demand;
    }
    demand.target[0] = 1 == phase ? 35 : 2 == phase ? 95 : 65;
    demand.target[1] = 1 == phase ? 95 : 2 == phase ? 35 : 65;
    return demand;
}
static inline unsigned run_start_block (bool response, bool active,
                                        bool settling, bool valid, bool stop,
                                        bool ready, uint16_t left,
                                        uint16_t right, bool hold,
                                        unsigned mode)
{
    /* Only optical alignment is bypassed. Calibration and deliberate rearm
       remain required, including for a diagnostic on a plain white surface. */
    return line_start_block(active, settling, valid, stop, ready,
                            response ? 500 : left, response ? 0 : right, hold,
                            mode);
}
static inline unsigned profile_start_block (unsigned profile, bool active,
                                            bool settling, bool valid,
                                            bool stop, bool ready,
                                            uint16_t left, uint16_t right,
                                            bool hold, unsigned mode)
{
    unsigned reason =
        run_start_block(2 == profile || 5 == profile, active, settling, valid,
                        stop, ready, left, right, hold, mode);
    if ((!reason) && 3 == profile && (left < 850))
    {
        return START_LEFT_WHITE;
    }
    if ((!reason) && 5 == profile && (left < 650) && (right < 650))
    {
        return START_LINE_ABSENT;
    }
    return reason;
}
enum
{
    AL_KICK_PWM = 250,
    AL_KICK_US = 120000,
    AL_KICK_COOLDOWN_US = 100000
};
typedef struct
{
    uint64_t end_us;
    uint64_t cooldown_until;
    bool active;
} line_breakaway_t;
/* Forward-only floor breakaway. Independent drive-off guards still own the
 * outputs. Pulse ends on the first control update at120ms or speed>=20mm/s;
 * it cannot trigger again until100ms cooldown and near-stationary feedback. */
static inline int32_t line_breakaway (line_breakaway_t * p_second, uint64_t now,
                                      int16_t wanted, float ramp, float speed,
                                      int32_t pwm)
{
    if (wanted <= 0)
    {
        *p_second = (line_breakaway_t){0};
        return 0;
    }
    if ((p_second->active) &&
        (((now >= p_second->end_us) || (speed >= 20)) || (speed < -8)))
    {
        p_second->active = false;
        p_second->cooldown_until = now + AL_KICK_COOLDOWN_US;
    }
    if ((((((!p_second->active) && (now >= p_second->cooldown_until)) &&
           (wanted >= 20)) &&
          (ramp >= 20)) &&
         (speed >= -8)) &&
        (speed < 8))
    {
        p_second->active = true;
        p_second->end_us = now + AL_KICK_US;
    }
    if ((p_second->active) && (pwm < AL_KICK_PWM))
    {
        pwm = AL_KICK_PWM;
    }
    if (pwm < 0)
    {
        pwm = 0;
    }
    if (pwm > AL_KICK_PWM)
    {
        pwm = AL_KICK_PWM;
    }
    return pwm;
}
static inline bool calibration_valid (const line_calibration_t * p_character)
{
    return 7 == p_character->mask && (p_character->white[0] <= 4095) &&
           (p_character->white[1] <= 4095) && (p_character->black[0] <= 4095) &&
           (p_character->black[1] <= 4095) &&
           (p_character->black[0] >=
            p_character->white[0] + LINE_CAL_MIN_SPAN) &&
           (p_character->black[1] >= p_character->white[1] + LINE_CAL_MIN_SPAN);
}
static inline uint16_t line_level (uint16_t raw, uint16_t white, uint16_t black)
{
    if ((black <= white) || (raw <= white))
    {
        return 0;
    }
    if (raw >= black)
    {
        return 1000;
    }
    return (uint16_t)(((uint32_t)(raw - white) * 1000u) / (black - white));
}
/* Follow the RIGHT edge of the strip with the physical LEFT sensor:
 * blackness above midpoint -> steer right, below midpoint -> steer left.
 * Positive differential means left wheel faster, which turns chassis right.
 * Right sensor assists after larger rightward line displacement. No barcode. */
static inline analog_demand_t
analog_demand (analog_tracker_t * p_sample,
               const line_calibration_t * p_character, uint16_t l_value,
               uint16_t result, uint64_t now, uint32_t mm_value)
{
    analog_demand_t demand = {0};
    if (!calibration_valid(p_character))
    {
        demand.stop = true;
        return demand;
    }
    demand.level[0] =
        line_level(l_value, p_character->white[0], p_character->black[0]);
    demand.level[1] =
        line_level(result, p_character->white[1], p_character->black[1]);
    int32_t error = (int32_t)demand.level[0] - 500;
    if ((error > -60) && (error < 60))
    {
        error = 0;
    }
    /* V6 halves the proportional steering differential and lowers cruise.
       Keep the same optical target/deadband and all loss/crossing guards. */
    int32_t differential = error * (2 * LINE_DIFFERENTIAL_MM_S) / 1000;
    if (demand.level[1] >= 650)
    {
        differential = LINE_DIFFERENTIAL_MM_S;
        demand.decision = AL_RIGHT_GUARD;
    }
    bool both = (demand.level[0] >= 750) && (demand.level[1] >= 750);
    if (both)
    {
        if (!p_sample->both)
        {
            p_sample->both_since = now;
        }
        p_sample->both = true;
        demand.decision = AL_BOTH_BLACK;
        demand.target[0] = demand.target[1] = 35;
        if (now - p_sample->both_since >= 120000)
        {
            demand.stop = true;
        }
    }
    else
    {
        p_sample->both = false;
        demand.target[0] = (int16_t)(LINE_BASE_MM_S + differential);
        demand.target[1] = (int16_t)(LINE_BASE_MM_S - differential);
    }
    bool white = (demand.level[0] < 150) && (demand.level[1] < 150);
    if (white)
    {
        if (!p_sample->white)
        {
            p_sample->white_since = now;
            p_sample->white_mm = mm_value;
        }
        p_sample->white = true;
        demand.decision = AL_SEEK_LEFT;
        if ((now - p_sample->white_since >= 650000) ||
            (mm_value - p_sample->white_mm >= 40))
        {
            demand.stop = true;
            demand.decision = AL_LOST;
        }
    }
    else
    {
        p_sample->white = false;
    }
    if (demand.stop)
    {
        demand.target[0] = demand.target[1] = 0;
    }
    return demand;
}
/* Experimental profile4: black plateau means straight, not an estimated centre.
 * Reuse the existing white-loss and both-dark guards, then replace edge
 * demands. Hysteresis prevents flickering near a single black threshold.
 * All-white recovery assumes the strip is to the left; two widely spaced spots
 * cannot disambiguate every departure. This is deliberately a short test, not a
 * search. */
static inline analog_demand_t
black_straight_demand (analog_tracker_t * p_sample,
                       const line_calibration_t * p_character, uint16_t l_value,
                       uint16_t result, uint64_t now, uint32_t mm_value)
{
    analog_demand_t demand =
        analog_demand(p_sample, p_character, l_value, result, now, mm_value);
    if ((demand.stop) || AL_BOTH_BLACK == demand.decision)
    {
        p_sample->black_hold = false;
        return demand;
    }
    if (demand.level[1] >= 650)
    {
        p_sample->black_hold = false;
        demand.target[0] = 60;
        demand.target[1] = 30;
        demand.decision = AL_RIGHT_GUARD;
        return demand;
    }
    if (demand.level[0] >= 850)
    {
        p_sample->black_hold = true;
    }
    else if (demand.level[0] <= 650)
    {
        p_sample->black_hold = false;
    }
    if (p_sample->black_hold)
    {
        demand.target[0] = demand.target[1] = 45;
        demand.decision = 8;
    }
    else
    {
        demand.target[0] = 30;
        demand.target[1] = 60;
        if (demand.decision != AL_SEEK_LEFT)
        {
            demand.decision = 9;
        }
    }
    return demand;
}
/* Profile5 only: tighter left recovery on BOTH WHITE after edge tracking.
 * No reverse drive. Left output becomes zero, right speed45mm/s. Recovery
 * remains bounded by1200ms OR30mm average absolute wheel path. With no slip,
 * a stationary-left pivot using30mm average path turns about31degrees.
 * Both-white direction is ambiguous: this experiment assumes a LEFT bend. */
enum
{
    CURVE_RECOVERY_US = 1200000,
    CURVE_RECOVERY_MM = 30
};
static inline analog_demand_t
curve_demand (analog_tracker_t * p_sample,
              const line_calibration_t * p_character, uint16_t l_value,
              uint16_t result, uint64_t now, uint32_t mm_value)
{
    analog_demand_t demand =
        analog_demand(p_sample, p_character, l_value, result, now, mm_value);
    if (!calibration_valid(p_character))
    {
        return demand;
    }
    if ((demand.level[0] < 150) && (demand.level[1] < 150))
    {
        demand.stop = (now - p_sample->white_since >= CURVE_RECOVERY_US) ||
                      (mm_value - p_sample->white_mm >= CURVE_RECOVERY_MM);
        demand.decision = demand.stop ? AL_LOST : 10;
        demand.target[0] = 0;
        demand.target[1] = demand.stop ? 0 : 45;
    }
    return demand;
}
#endif

/* End of file. */
