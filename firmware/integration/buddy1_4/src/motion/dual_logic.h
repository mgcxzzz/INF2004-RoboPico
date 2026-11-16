/**
 * @file dual_logic.h
 * @brief Dual logic.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef DUAL_LOGIC_H
#define DUAL_LOGIC_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Provisional user measurements, not manufacturer specifications. */
#define LEFT_CPR 2524.05f
#define RIGHT_CPR 2528.30f
#define CIRCUMFERENCE_MM 200.0f
#define TRACK_MM 110.0f
#define PWM_LIMIT 250 /* permille: 25%, tested on each wheel */
#define CONTROL_PERIOD_US 10000u
#define CONTROL_LATE_US 100000u
#define MAX_RUN_US 3000000u
#define REST_US 1000000u

enum
{
    WHY_NONE,
    WHY_GP20,
    WHY_RELEASE,
    WHY_LIMIT,
    WHY_SERIAL,
    WHY_USB,
    WHY_LATE,
    WHY_FEEDBACK,
    WHY_DIRECTION,
    WHY_ENCODER,
    WHY_TARGET,
    WHY_BUTTON
};
typedef struct
{
    bool active;
    bool consumed;
    uint32_t generation;
    uint32_t stops;
    uint8_t reason;
    uint64_t start_us;
    uint64_t deadline_us;
    uint64_t last_control_us;
    uint64_t next_us;
    uint64_t duration_us;
} dual_guard_t;
static inline void guard_stop (volatile dual_guard_t * p_g_value, uint64_t now,
                               uint8_t why)
{
    if (p_g_value->active)
    {
        p_g_value->duration_us = now - p_g_value->start_us;
        p_g_value->next_us = now + REST_US;
        p_g_value->reason = why;
        ++p_g_value->stops;
    }
    p_g_value->active = false;
    p_g_value->consumed = true;
}
static inline void guard_tick (volatile dual_guard_t * p_g_value, uint64_t now,
                               bool held, bool stop)
{
    if (stop)
    {
        guard_stop(p_g_value, now, WHY_GP20);
    }
    else if (!held)
    {
        guard_stop(p_g_value, now, WHY_RELEASE);
    }
    else if ((p_g_value->active) && (now >= p_g_value->deadline_us))
    {
        guard_stop(p_g_value, now, WHY_LIMIT);
    }
    else if ((p_g_value->active) &&
             (now - p_g_value->last_control_us >= CONTROL_LATE_US))
    {
        guard_stop(p_g_value, now, WHY_LATE);
    }
    if ((!held) && (!stop))
    {
        p_g_value->consumed = false;
    }
}
static inline bool guard_start (volatile dual_guard_t * p_g_value, uint64_t now,
                                bool held, bool stop)
{
    if (((((p_g_value->active) || (p_g_value->consumed)) || (!held)) || stop) ||
        (now < p_g_value->next_us))
    {
        return false;
    }
    p_g_value->active = true;
    p_g_value->consumed = true;
    ++p_g_value->generation;
    p_g_value->start_us = p_g_value->last_control_us = now;
    p_g_value->deadline_us = now + MAX_RUN_US;
    return true;
}

static inline float mm_per_tick (unsigned wheel)
{
    return CIRCUMFERENCE_MM / (0 == wheel ? LEFT_CPR : RIGHT_CPR);
}
/* Raw encoder sign: left forward negative, right forward positive. */
static inline float raw_to_mm (unsigned wheel, int64_t ticks)
{
    return (float)ticks * mm_per_tick(wheel) * (0 == wheel ? -1.0f : 1.0f);
}
static inline float heading_deg (float left_mm, float right_mm)
{
    return (left_mm - right_mm) / TRACK_MM * (180.0f / 3.14159265358979323846f);
}
typedef struct
{
    uint16_t a;
    uint16_t b;
} dual_levels_t;
/* Positive command means chassis-forward on BOTH wheels. */
static inline dual_levels_t dual_levels (unsigned wheel,
                                         int32_t signed_permille)
{
    dual_levels_t value = {0, 0};
    if (wheel > 1)
    {
        return value;
    }
    if (signed_permille > PWM_LIMIT)
    {
        signed_permille = PWM_LIMIT;
    }
    if (signed_permille < -PWM_LIMIT)
    {
        signed_permille = -PWM_LIMIT;
    }
    uint16_t level =
        (uint16_t)(signed_permille < 0 ? -signed_permille : signed_permille);
    bool a_side = (0 == wheel) ? signed_permille > 0 : signed_permille < 0;
    if (a_side)
    {
        value.a = level;
    }
    else
    {
        value.b = level;
    }
    return value;
}
static inline float clampf (float value_x, float low, float high)
{
    return value_x < low ? low : value_x > high ? high : value_x;
}
typedef struct
{
    float integral;
    float ramp;
} wheel_pi_t;
/* Positive magnitudes: the command supplies direction separately. Gains are
   conservative initial values, still requiring physical tuning. */
static inline float feedforward (unsigned wheel, int direction, float speed)
{
    if (speed <= 0)
    {
        return 0;
    }
    float v15 = 0 == wheel ? (direction > 0 ? 101.0f : 96.0f)
                           : (direction > 0 ? 93.0f : 96.0f);
    float v25 = 0 == wheel ? (direction > 0 ? 185.0f : 181.0f)
                           : (direction > 0 ? 177.0f : 176.0f);
    return clampf(150.0f + (speed - v15) * 100.0f / (v25 - v15), 0, PWM_LIMIT);
}
static inline int32_t pi_step (wheel_pi_t * p_data, unsigned wheel,
                               int direction, float demand_mm_s,
                               float measured_along_mm_s, float delta_time)
{
    if (demand_mm_s <= 0)
    {
        p_data->ramp = p_data->integral = 0;
        return 0;
    }
    /* 100 mm/s^2 demand ramp; no automatic polarity reversal. */
    p_data->ramp += clampf(demand_mm_s - p_data->ramp, -100.0f * delta_time,
                           100.0f * delta_time);
    float error = p_data->ramp - measured_along_mm_s;
    float ff_value = feedforward(wheel, direction, p_data->ramp);
    float proposed_i =
        clampf(p_data->integral + 0.8f * error * delta_time, -80, 80);
    float proposed = ff_value + 0.6f * error + proposed_i;
    if (!(((proposed > PWM_LIMIT) && (error > 0)) ||
          ((proposed < 0) && (error < 0))))
    {
        p_data->integral = proposed_i;
    }
    int32_t pwm = (int32_t)(clampf(ff_value + 0.6f * error + p_data->integral,
                                   0, PWM_LIMIT) +
                            0.5f);
    return direction * pwm;
}

typedef struct
{
    char key;
    bool bounded;
    int8_t sign[2];
    float speed[2];
    float target_mm[2];
} dual_command_t;
static inline bool command_make (char key, float timed_speed,
                                 dual_command_t * p_character)
{
    *p_character = (dual_command_t){.key = key};
    if ('f' == key || 'b' == key)
    {
        p_character->sign[0] = p_character->sign[1] = 'f' == key ? 1 : -1;
        p_character->speed[0] = p_character->speed[1] =
            clampf(timed_speed, 60, 100);
    }
    else if ('l' == key || 'r' == key)
    {
        p_character->sign[0] = 'r' == key ? 1 : -1;
        p_character->sign[1] = -p_character->sign[0];
        p_character->speed[0] = p_character->speed[1] = 40;
    }
    else if ('d' == key || 'g' == key || 'q' == key || 'e' == key)
    {
        p_character->bounded = true;
        if ('d' == key || 'g' == key)
        {
            p_character->sign[0] = p_character->sign[1] = 1;
            p_character->target_mm[0] = p_character->target_mm[1] =
                'g' == key ? 300 : 100;
            p_character->speed[0] = p_character->speed[1] =
                'g' == key ? 100 : 60;
        }
        else
        {
            p_character->sign[0] = 'e' == key ? 1 : -1;
            p_character->sign[1] = -p_character->sign[0];
            p_character->target_mm[0] = p_character->target_mm[1] =
                TRACK_MM * 3.14159265358979323846f / 8.0f;
            p_character->speed[0] = p_character->speed[1] = 40;
        }
    }
    else
    {
        return false;
    }
    return true;
}
static inline float position_demand (const dual_command_t * p_character,
                                     unsigned wheel, float wheel_mm)
{
    if (!p_character->bounded)
    {
        return p_character->speed[wheel];
    }
    float remaining =
        p_character->target_mm[wheel] - wheel_mm * p_character->sign[wheel];
    if (remaining <= 2.0f)
    {
        return 0;
    }
    float minimum =
        p_character->sign[0] != p_character->sign[1] ? 18.0f : 25.0f;
    return fminf(p_character->speed[wheel], fmaxf(minimum, remaining * 3.0f));
}

/* Bench-only terminal approach: 15% was physically tested on both wheels.
   Keep enough torque through the final 12 mm rather than tapering into
   stiction. Cut at the existing 2 mm threshold, then measure coasting/overshoot
   physically. */
static inline int32_t finish_output (const dual_command_t * p_character,
                                     unsigned wheel, float pos, int32_t u_value)
{
    float remaining =
        p_character->target_mm[wheel] - pos * p_character->sign[wheel];
    if (((((p_character->bounded) &&
           (p_character->sign[0] == p_character->sign[1])) &&
          (remaining > 2)) &&
         (remaining <= 12)) &&
        (u_value * p_character->sign[wheel] < 150))
    {
        return p_character->sign[wheel] * 150;
    }
    return u_value;
}
/* Turn-only breakaway assistance. 20% is below the existing 25% cap.
   A pulse lasts at most 100 ms, ends early once speed reaches 20 mm/s,
   and cannot repeat for 100 ms. No unconditional boost near the target. */
typedef struct
{
    uint32_t pulse_us;
    uint32_t cooldown_us;
} turn_assist_t;
static inline uint32_t subtract_us (uint32_t first, uint32_t second)
{
    return first > second ? first - second : 0;
}
static inline int32_t turn_assist_step (turn_assist_t * p_sample,
                                        const dual_command_t * p_character,
                                        unsigned wheel, float ramp, float along,
                                        int32_t u_value, uint32_t elapsed,
                                        bool done)
{
    if (done || (p_character->sign[0] == p_character->sign[1]))
    {
        *p_sample = (turn_assist_t){0};
        return done ? 0 : u_value;
    }
    p_sample->cooldown_us = subtract_us(p_sample->cooldown_us, elapsed);
    if (p_sample->pulse_us)
    {
        p_sample->pulse_us = subtract_us(p_sample->pulse_us, elapsed);
        if ((along >= 20) || (!p_sample->pulse_us))
        {
            p_sample->pulse_us = 0;
            p_sample->cooldown_us = 100000;
        }
    }
    if (((((!p_sample->pulse_us) && (!p_sample->cooldown_us)) &&
          (ramp >= 20)) &&
         (along >= -8)) &&
        (along < 8))
    {
        p_sample->pulse_us = 100000;
    }
    int32_t magnitude = u_value * p_character->sign[wheel];
    if ((p_sample->pulse_us) && (magnitude < 200))
    {
        magnitude = 200;
    }
    if (magnitude < 0)
    {
        magnitude = 0;
    }
    if (magnitude > PWM_LIMIT)
    {
        magnitude = PWM_LIMIT;
    }
    return magnitude * p_character->sign[wheel];
}
static inline uint32_t command_limit_us (char key)
{
    return 'g' == key ? 6000000u : MAX_RUN_US;
}
typedef struct
{
    bool selecting;
    bool last_stop;
    bool ready;
    uint8_t index;
} profile_select_t;
static inline char profile_key (uint8_t index)
{
    const char keys[] = {'d', 'g', 'q', 'e'};
    return keys[index % 4];
}
/* GP20 cycles only before the start tap. Outside selection it remains STOP. */
static inline bool profile_step (profile_select_t * p_state, bool usb,
                                 bool hold, bool stop)
{
    if (usb)
    {
        p_state->selecting = false;
    }
    if (!p_state->selecting)
    {
        return false;
    }
    if (!p_state->ready)
    {
        if ((!hold) && (!stop))
        {
            p_state->ready = true;
            p_state->last_stop = false;
        }
        return false;
    }
    if (stop && (!p_state->last_stop))
    {
        p_state->index = (p_state->index + 1) % 4;
    }
    p_state->last_stop = stop;
    if (hold && (!stop))
    {
        p_state->selecting = false;
        return true;
    }
    return false;
}

enum
{
    OFF_NONE,
    OFF_UNPLUG,
    OFF_BASE_RELEASE,
    OFF_PRESS,
    OFF_RELEASE,
    OFF_COUNTDOWN
};
typedef struct
{
    uint8_t state;
    uint64_t start_at;
} offline_arm_t;
/* button is debounced by hardware adapter. Returns one start request only. */
static inline bool offline_step (offline_arm_t * p_o_value, uint64_t now,
                                 bool usb, bool button, bool stop)
{
    if (stop || (usb && (p_o_value->state != OFF_UNPLUG)))
    {
        p_o_value->state = OFF_NONE;
        return false;
    }
    switch (p_o_value->state)
    {
    case OFF_UNPLUG:
        if (!usb)
        {
            p_o_value->state = OFF_BASE_RELEASE;
        }
        break;
    case OFF_BASE_RELEASE:
        if (!button)
        {
            p_o_value->state = OFF_PRESS;
        }
        break;
    case OFF_PRESS:
        if (button)
        {
            p_o_value->state = OFF_RELEASE;
        }
        break;
    case OFF_RELEASE:
        if (!button)
        {
            p_o_value->state = OFF_COUNTDOWN;
            p_o_value->start_at = now + 2000000;
        }
        break;
    case OFF_COUNTDOWN:
        if (button)
        {
            p_o_value->state = OFF_NONE;
        }
        else if (now >= p_o_value->start_at)
        {
            p_o_value->state = OFF_NONE;
            return true;
        }
        break;
    default:
        break;
    }
    return false;
}
/* Deliberate one-shot standalone arm: two-button hold, full release, separate
 * tap. */
typedef struct
{
    uint64_t since;
    bool ready;
    bool used;
} boot_arm_t;
static inline bool boot_arm_step (boot_arm_t * p_second, uint64_t now, bool usb,
                                  bool busy, bool hold, bool stop)
{
    if (usb || busy)
    {
        p_second->since = 0;
        p_second->ready = false;
        return false;
    }
    if (p_second->used)
    {
        return false;
    }
    if (hold && stop)
    {
        if (!p_second->since)
        {
            p_second->since = now;
        }
        if (now - p_second->since >= 2000000)
        {
            p_second->ready = true;
        }
    }
    else if (p_second->ready)
    {
        if ((!hold) && (!stop))
        {
            p_second->used = true;
            p_second->ready = false;
            return true;
        }
    }
    else
    {
        p_second->since = 0;
    }
    return false;
}
#endif

/* End of file. */
