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
#define PWM_LIMIT 250                 /* permille: 25%, tested on each wheel */
#define CONTROL_PERIOD_US 10000u
#define CONTROL_LATE_US 100000u
#define MAX_RUN_US 3000000u
#define REST_US 1000000u

enum { WHY_NONE, WHY_GP20, WHY_RELEASE, WHY_LIMIT, WHY_SERIAL, WHY_USB,
       WHY_LATE, WHY_FEEDBACK, WHY_DIRECTION, WHY_ENCODER, WHY_TARGET, WHY_BUTTON };
typedef struct {
    bool active, consumed;
    uint32_t generation, stops;
    uint8_t reason;
    uint64_t start_us, deadline_us, last_control_us, next_us, duration_us;
} DualGuard;
static inline void guard_stop(volatile DualGuard *g, uint64_t now, uint8_t why) {
    if (g->active) {
        g->duration_us = now - g->start_us;
        g->next_us = now + REST_US;
        g->reason = why;
        ++g->stops;
    }
    g->active = false;
    g->consumed = true;
}
static inline void guard_tick(volatile DualGuard *g, uint64_t now, bool held, bool stop) {
    if (stop) guard_stop(g, now, WHY_GP20);
    else if (!held) guard_stop(g, now, WHY_RELEASE);
    else if (g->active && now >= g->deadline_us) guard_stop(g, now, WHY_LIMIT);
    else if (g->active && now - g->last_control_us >= CONTROL_LATE_US)
        guard_stop(g, now, WHY_LATE);
    if (!held && !stop) g->consumed = false;
}
static inline bool guard_start(volatile DualGuard *g, uint64_t now, bool held, bool stop) {
    if (g->active || g->consumed || !held || stop || now < g->next_us) return false;
    g->active = true; g->consumed = true; ++g->generation;
    g->start_us = g->last_control_us = now;
    g->deadline_us = now + MAX_RUN_US;
    return true;
}

static inline float mm_per_tick(unsigned w) {
    return CIRCUMFERENCE_MM / (w == 0 ? LEFT_CPR : RIGHT_CPR);
}
/* Raw encoder sign: left forward negative, right forward positive. */
static inline float raw_to_mm(unsigned w, int64_t ticks) {
    return (float)ticks * mm_per_tick(w) * (w == 0 ? -1.0f : 1.0f);
}
static inline float heading_deg(float left_mm, float right_mm) {
    return (left_mm - right_mm) / TRACK_MM * (180.0f / 3.14159265358979323846f);
}
typedef struct { uint16_t a, b; } DualLevels;
/* Positive command means chassis-forward on BOTH wheels. */
static inline DualLevels dual_levels(unsigned w, int32_t signed_permille) {
    DualLevels v = {0, 0};
    if (w > 1) return v;
    if (signed_permille > PWM_LIMIT) signed_permille = PWM_LIMIT;
    if (signed_permille < -PWM_LIMIT) signed_permille = -PWM_LIMIT;
    uint16_t level = (uint16_t)(signed_permille < 0 ? -signed_permille : signed_permille);
    bool a_side = (w == 0) ? signed_permille > 0 : signed_permille < 0;
    if (a_side) v.a = level; else v.b = level;
    return v;
}
static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : x > hi ? hi : x;
}
typedef struct { float integral, ramp; } WheelPI;
/* Positive magnitudes: the command supplies direction separately. Gains are
   conservative initial values, still requiring physical tuning. */
static inline float feedforward(unsigned w, int direction, float speed) {
    if (speed <= 0) return 0;
    float v15 = w == 0 ? (direction > 0 ? 101.0f : 96.0f) : (direction > 0 ? 93.0f : 96.0f);
    float v25 = w == 0 ? (direction > 0 ? 185.0f : 181.0f) : (direction > 0 ? 177.0f : 176.0f);
    return clampf(150.0f + (speed-v15)*100.0f/(v25-v15), 0, PWM_LIMIT);
}
static inline int32_t pi_step(WheelPI *p, unsigned w, int direction,
                              float demand_mm_s, float measured_along_mm_s, float dt) {
    if (demand_mm_s <= 0) { p->ramp = p->integral = 0; return 0; }
    /* 100 mm/s^2 demand ramp; no automatic polarity reversal. */
    p->ramp += clampf(demand_mm_s-p->ramp, -100.0f*dt, 100.0f*dt);
    float error = p->ramp - measured_along_mm_s;
    float ff = feedforward(w, direction, p->ramp);
    float proposed_i = clampf(p->integral + 0.8f*error*dt, -80, 80);
    float proposed = ff + 0.6f*error + proposed_i;
    if (!((proposed > PWM_LIMIT && error > 0) || (proposed < 0 && error < 0)))
        p->integral = proposed_i;
    int32_t pwm = (int32_t)(clampf(ff + 0.6f*error+p->integral, 0, PWM_LIMIT)+0.5f);
    return direction * pwm;
}

typedef struct {
    char key;
    bool bounded;
    int8_t sign[2];
    float speed[2], target_mm[2];
} DualCommand;
static inline bool command_make(char key, float timed_speed, DualCommand *c) {
    *c = (DualCommand){.key=key};
    if (key == 'f' || key == 'b') {
        c->sign[0] = c->sign[1] = key == 'f' ? 1 : -1;
        c->speed[0] = c->speed[1] = clampf(timed_speed, 60, 100);
    } else if (key == 'l' || key == 'r') {
        c->sign[0] = key == 'r' ? 1 : -1;
        c->sign[1] = -c->sign[0];
        c->speed[0] = c->speed[1] = 40;
    } else if (key == 'd' || key == 'q' || key == 'e') {
        c->bounded = true;
        if (key == 'd') {
            c->sign[0] = c->sign[1] = 1;
            c->target_mm[0] = c->target_mm[1] = 100;
            c->speed[0] = c->speed[1] = 60;
        } else {
            c->sign[0] = key == 'e' ? 1 : -1;
            c->sign[1] = -c->sign[0];
            c->target_mm[0] = c->target_mm[1] = TRACK_MM*3.14159265358979323846f/8.0f;
            c->speed[0] = c->speed[1] = 40;
        }
    } else return false;
    return true;
}
static inline float position_demand(const DualCommand *c, unsigned w, float wheel_mm) {
    if (!c->bounded) return c->speed[w];
    float remaining = c->target_mm[w] - wheel_mm*c->sign[w];
    if (remaining <= 2.0f) return 0;
    return fminf(c->speed[w], remaining*3.0f);
}

enum { OFF_NONE, OFF_UNPLUG, OFF_BASE_RELEASE, OFF_PRESS, OFF_RELEASE, OFF_COUNTDOWN };
typedef struct { uint8_t state; uint64_t start_at; } OfflineArm;
/* button is debounced by hardware adapter. Returns one start request only. */
static inline bool offline_step(OfflineArm *o,uint64_t now,bool usb,bool button,bool stop) {
    if(stop || (usb && o->state!=OFF_UNPLUG)) { o->state=OFF_NONE; return false; }
    switch(o->state) {
        case OFF_UNPLUG: if(!usb) o->state=OFF_BASE_RELEASE; break;
        case OFF_BASE_RELEASE: if(!button) o->state=OFF_PRESS; break;
        case OFF_PRESS: if(button) o->state=OFF_RELEASE; break;
        case OFF_RELEASE: if(!button) { o->state=OFF_COUNTDOWN; o->start_at=now+2000000; } break;
        case OFF_COUNTDOWN:
            if(button) o->state=OFF_NONE;
            else if(now>=o->start_at) { o->state=OFF_NONE; return true; }
            break;
        default: break;
    }
    return false;
}
#endif
