/**
 * @file line_model.h
 * @brief Line model.
 * @details ADC/control ownership covers sensor acquisition, planning and
 * calibration. Other tasks consume protected copies; flash and stored exports
 * require Stop.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef RTOS_LINE_MODEL_H
#define RTOS_LINE_MODEL_H

/* Hardware-free Step 2 adapter. The reference algorithms remain in main's
 * feature folders; only the RTOS acquisition/output boundary lives here. */
#ifdef APP_RTOS_BARCODE
/* Horizontal GP27 sensing point -> wheel axle, measured on the current car.
 * Other migration steps and the SDK reference retain their 92mm geometry. */
#define BARCODE_SENSOR_AHEAD_MM 110
#ifndef BARCODE_CONFIRM_GATE
#define BARCODE_CONFIRM_GATE 1
#endif
#endif
#include "src/line_following/continuous_follow.h"
#include "src/line_following/circuit_capture.h"
#include "src/line_following/circuit_load.h"
#include "src/motion/maneuver_power.h"
#include "src/motion/line_recovery_power.h"
#include "src/motion/ramp_profile.h"
#include "src/barcode/barcode_record.h"
#include "config/calibration_presets.h"
#ifdef APP_RTOS_BUDDY5
#include "../obstacle_navigation/b5_straight.h"
#endif
#ifdef APP_RTOS_IMU
#include "../imu_navigation/imu_motion.h"
#endif

enum
{
    RL_PROFILE = 1,
    RL_CAL_OFFSET = 0x1a9000,
    RL_MAIN_OFFSET = 0x1d0000,
    RL_CAL_MAGIC = 0x31434c52,
    RL_LOG_ROWS =
#ifdef APP_RTOS_BARCODE
        640 /* RAM is shared with the original63 banks and2ms raw capture. */
#else
        1536
#endif
};

enum
{
    RL_J_WHITE_NOW = 1,
    RL_J_WHITE_QUALIFIED = 2,
    RL_J_DEPARTURE_READY = 4,
    RL_J_BRANCH_ACCEPTED = 8,
    RL_J_ACCEPTED_WITHOUT_WHITE = 16,
    RL_J_ACCEPT_EVENT = 32,
    RL_J_WHITE_TIMING = 64
};
typedef struct
{
    uint64_t white_us;
    uint8_t flags;
} line_junction_diagnostic_t;

typedef struct
{
    circuit_t circuit;
    barcode_nav_t nav;
    continuous_steering_t steering;
    scan_smooth_t scan;
    analog_demand_t optical;
    uint16_t baseline;
    uint16_t side_adc;
    uint64_t candidate_us;
    double candidate_wheel[2];
    bool black;
    bool candidate;
    bool caster_assist_enabled;
    line_junction_diagnostic_t junction;
} line_planner_t;

#ifdef APP_RTOS_BUDDY5
static inline void rl_buddy5_straight_sample (const line_planner_t * p_planner,
                                              const circuit_input_t * p_input,
                                              const analog_demand_t * p_demand,
                                              b5_line_sample_t * p_out)
{
    const circuit_t * p_circuit = &p_planner->circuit;
    const barcode_nav_t * p_nav = &p_planner->nav;
    *p_out = (b5_line_sample_t){
        .now_us = p_input->now,
        .wheel_mm = {(float)p_input->wheel[0], (float)p_input->wheel[1]},
        .phase = p_circuit->phase,
        .decision = p_demand->decision,
        .valid = true,
        .busy = (p_circuit->busy) || (p_nav->busy),
        .fault = p_circuit->fault,
        .stop = p_demand->stop,
        .line_lost = p_circuit->follower.core.white,
#if defined(CIRCUIT_SWEEP_SEARCH) || defined(CIRCUIT_STATIONARY_SEARCH) ||     \
    defined(LINE_STATIONARY_SEARCH)
        .searching = p_circuit->follower.search.active,
#endif
        .junction_mark = ((p_circuit->candidate) || (p_nav->cross_candidate)) ||
                         (p_nav->cross_ready),
        /* The action and U-turn stage can remain populated after completion.
         * Barcode navigation clears busy when it returns to line following. */
        .route_turn = p_nav->busy,
        .steering = p_planner->steering.active};
}
#endif

/* Observation only: never modifies the planner's acceptance or motor targets.
 * White qualification reproduces the former <200, >10deg, 30ms condition.
 * For U-turns, acceptance is stage2 -> stage3, before the second settle. */
static inline void rl_junction_observe (line_planner_t * p_data,
                                        const line_calibration_t * p_cal,
                                        const circuit_input_t * p_index,
                                        unsigned before, unsigned stage)
{
    line_junction_diagnostic_t * p_other_data = &p_data->junction;
    const junction_t * p_other_index = &p_data->circuit.turn;
    if (J_CENTRE == before && J_TURN == p_data->circuit.phase)
    {
        *p_other_data = (line_junction_diagnostic_t){0};
    }
    p_other_data->flags &= (uint8_t) ~(RL_J_WHITE_NOW | RL_J_ACCEPT_EVENT);
    bool white =
        (line_level(p_index->adc[0], p_cal->white[0], p_cal->black[0]) < 200) &&
        (line_level(p_index->adc[1], p_cal->white[1], p_cal->black[1]) < 200);
    if (white)
    {
        p_other_data->flags |= RL_J_WHITE_NOW;
    }
    if (J_TURN == before && (p_other_index->turn_degrees > 10) && white)
    {
        if (!(p_other_data->flags & RL_J_WHITE_TIMING))
        {
            p_other_data->white_us = p_index->now;
            p_other_data->flags |= RL_J_WHITE_TIMING;
        }
        if (p_index->now - p_other_data->white_us >= 30000)
        {
            p_other_data->flags |= RL_J_WHITE_QUALIFIED;
        }
    }
    else
    {
        p_other_data->flags &= (uint8_t)~RL_J_WHITE_TIMING;
    }
    if (p_other_index->off_line)
    {
        p_other_data->flags |= RL_J_DEPARTURE_READY;
    }
    bool uturn = 'U' == barcode_nav_action(&p_data->nav, p_data->nav.action);
    bool accepted =
        J_TURN == before && (uturn ? 2 == stage && 3 == p_data->nav.uturn.stage
                                   : J_ACQUIRE == p_data->circuit.phase);
    if (accepted)
    {
        p_other_data->flags |= RL_J_BRANCH_ACCEPTED | RL_J_ACCEPT_EVENT;
        if (!(p_other_data->flags & RL_J_WHITE_QUALIFIED))
        {
            p_other_data->flags |= RL_J_ACCEPTED_WITHOUT_WHITE;
        }
    }
}

static inline void rl_begin (line_planner_t * p_data,
                             const circuit_input_t * p_index, uint16_t side_adc)
{
    *p_data = (line_planner_t){.baseline = side_adc,
                               .side_adc = side_adc,
                               .black = side_adc >= 212,
                               .candidate = side_adc >= 212};
    circuit_begin(&p_data->circuit, p_index);
    barcode_nav_begin(&p_data->nav, p_index);
    p_data->nav.external_reader =
        true; /* Code39 is deliberately deferred to Step 4. */
}

/* Junction-only edge qualification. Same 212 ADC preview, 3 ADC hysteresis,
 * 2 ms debounce, encoder width 11..30 mm and 4 mm white confirmation as the
 * reference. No call to either barcode decoder; test with a bare cross. */
static inline void rl_side (line_planner_t * p_data, uint16_t adc, uint64_t now,
                            const double wheel[2])
{
    p_data->side_adc = adc;
    bool candidate = p_data->black ? adc >= 209 : adc >= 215;
    if (candidate != p_data->candidate)
    {
        p_data->candidate = candidate;
        p_data->candidate_us = now;
        memcpy(p_data->candidate_wheel, wheel, sizeof(p_data->candidate_wheel));
    }
    if ((candidate == p_data->black) ||
        (now - p_data->candidate_us < AO_DEBOUNCE_US))
    {
        return;
    }
    p_data->black = candidate;
    barcode_nav_t * p_count = &p_data->nav;
    if (p_count->busy)
    {
        return;
    }
    double mm_value =
        (p_data->candidate_wheel[0] + p_data->candidate_wheel[1]) * .5;
    if (candidate)
    {
        barcode_cross_confirm(p_count, p_data->candidate_us, mm_value, true);
        p_count->cross_candidate = false;
        memcpy(p_count->leading_wheel, p_data->candidate_wheel,
               sizeof(p_count->leading_wheel));
        p_count->leading_us = p_data->candidate_us;
        p_count->leading_gate = p_count->allow_cross;
    }
    else if (p_count->leading_us)
    {
        double width =
            mm_value -
            (p_count->leading_wheel[0] + p_count->leading_wheel[1]) * .5;
        if ((width >= B_CROSS_MIN_MM) && (width <= B_CROSS_MAX_MM))
        {
            p_count->cross_width = width;
            p_count->cross_us = p_count->leading_us;
            memcpy(p_count->cross_wheel, p_count->leading_wheel,
                   sizeof(p_count->cross_wheel));
            p_count->cross_gate = p_count->leading_gate;
            p_count->cross_candidate = true;
            p_count->cross_trailing_mm = mm_value;
        }
        p_count->leading_us = 0;
    }
}

/* Profile 1 part of continuous_follow_step, without allocating 63 inactive
 * decoder banks. Host tests compare it against the unchanged full wrapper. */
static inline analog_demand_t
rl_follow_with_cruise (line_planner_t * p_data,
                       const line_calibration_t * p_cal,
                       const circuit_input_t * p_index, float delta_time,
                       unsigned cruise_speed_mm_s)
{
    unsigned before = p_data->circuit.phase, stage = p_data->nav.uturn.stage;
    analog_demand_t demand = barcode_circuit_step(
        &p_data->circuit, &p_data->nav, p_cal, p_index, RL_PROFILE);
    rl_junction_observe(p_data, p_cal, p_index, before, stage);
    p_data->optical = demand;
    bool activity = (p_data->nav.pending) ||
                    (p_data->side_adc >= (unsigned)p_data->baseline + 16);
    if (activity)
    {
        p_data->scan.activity_us = p_index->now;
    }
    bool scanning =
        ((!p_data->circuit.busy) && (!demand.stop)) &&
        (activity || ((p_data->scan.scanning) &&
                      (p_index->now - p_data->scan.activity_us < 600000)));
    p_data->scan.scanning = scanning;
    return continuous_steering_step_with_cruise(
        &p_data->steering, demand, &p_data->circuit, p_index, scanning,
        RL_PROFILE, delta_time, cruise_speed_mm_s);
}
static inline analog_demand_t rl_follow (line_planner_t * p_data,
                                         const line_calibration_t * p_cal,
                                         const circuit_input_t * p_index,
                                         float delta_time)
{
    return rl_follow_with_cruise(p_data, p_cal, p_index, delta_time,
                                 DRIVE_MODE_BASELINE_SPEED);
}

enum
{
    RL_CASTER_TARGET_MIN = 25,
    RL_CASTER_PWM = 55,
    RL_CASTER_WAIT_US = 300000,
    RL_CASTER_BOOST_US = 250000
};

typedef struct
{
    float anchor_mm[2];
    uint32_t progress_us;
    uint32_t boost_start_us;
    unsigned slow_samples : 2;
    unsigned rolling_wheel : 2;
    bool monitoring;
    bool moved;
    bool boosting;
    bool early_pulse;
} line_caster_t;

typedef struct
{
    continuous_drive_t drive[2];
    signed_wheel_t wheel[2];
    maneuver_power_t maneuver[2];
    line_recovery_wheel_t recovery[2];
    circuit_capture_t capture;
    circuit_load_t load;
    int32_t output[2];
    unsigned assist_code;
    line_caster_t caster;
} line_power_t;

/* A committed envelope is copied under the motion lock. The 1 ms safety
 * task can expire peaks without reading the controller's mutable PI state. */
typedef struct
{
    int32_t output[2];
    uint64_t until[2];
    unsigned continuous : 1;
    unsigned maneuver : 1;
    unsigned reverse : 1;
    unsigned recovery : 1;
    unsigned loaded : 2;
    unsigned caster_wheels : 2;
    bool no_progress;
} line_envelope_t;

/* Require travel before arming so Start and re-entry from recovery cannot
 * trigger an extra launch pulse. Use net encoder travel to ignore jitter. */
static inline void rl_caster_step (line_caster_t * p_state,
                                   const circuit_input_t * p_input,
                                   bool eligible, unsigned wheels,
                                   const analog_demand_t * p_demand,
                                   line_envelope_t * p_event)
{
    if ((!eligible) || (0U == wheels))
    {
        *p_state = (line_caster_t){0};
        return;
    }

    const uint32_t now = (uint32_t)p_input->now;
    const float left_mm = (float)p_input->wheel[0];
    const float right_mm = (float)p_input->wheel[1];
    const bool progress = fabsf(left_mm - p_state->anchor_mm[0]) >= 2.0f ||
                          fabsf(right_mm - p_state->anchor_mm[1]) >= 2.0f;
    if (!p_state->monitoring || progress)
    {
        p_state->anchor_mm[0] = left_mm;
        p_state->anchor_mm[1] = right_mm;
        p_state->progress_us = now;
        p_state->moved = p_state->monitoring &&
                         ((!p_state->boosting) || p_state->early_pulse);
        if (p_state->boosting)
        {
            p_state->rolling_wheel = 0U;
            p_state->slow_samples = 0U;
        }
        p_state->boosting = false;
        p_state->early_pulse = false;
        p_state->monitoring = true;
    }

    /* Arm the early pulse only after the outside wheel was rolling in this
     * correction. Three consecutive slow samples reject isolated jitter. */
    const unsigned side = (2U == wheels) ? 1U : 0U;
    const bool correction = ((1U == wheels) || (2U == wheels)) &&
                            (p_demand->target[1U - side] <= 10);
    if (p_state->early_pulse &&
        ((!correction) || (p_state->rolling_wheel != wheels)))
    {
        /* A change of steering direction ends the early pulse. */
        *p_state = (line_caster_t){0};
        return;
    }
    if (!correction || (p_state->rolling_wheel != wheels))
    {
        p_state->slow_samples = 0U;
        p_state->rolling_wheel = 0U;
    }
    if (correction && (p_input->speed[side] >= 0.75f * p_demand->target[side]))
    {
        p_state->rolling_wheel = wheels;
    }
    if (p_state->moved && (wheels == p_state->rolling_wheel) &&
        (p_input->speed[side] <= 0.5f * p_demand->target[side]))
    {
        if (p_state->slow_samples < 3U)
        {
            ++p_state->slow_samples;
        }
    }
    else
    {
        p_state->slow_samples = 0U;
    }

    const uint32_t age = now - p_state->progress_us;
    if ((!p_state->boosting) && p_state->moved &&
        ((age >= RL_CASTER_WAIT_US) || (3U == p_state->slow_samples)))
    {
        p_state->boosting = true;
        p_state->early_pulse = age < RL_CASTER_WAIT_US;
        p_state->boost_start_us = (age >= RL_CASTER_WAIT_US)
                                      ? p_state->progress_us + RL_CASTER_WAIT_US
                                      : now;
        p_state->anchor_mm[0] = left_mm;
        p_state->anchor_mm[1] = right_mm;
    }
    if (!p_state->boosting)
    {
        return;
    }
    const uint32_t boost_age = now - p_state->boost_start_us;
    if (boost_age >= RL_CASTER_BOOST_US)
    {
        p_event->no_progress = true;
        return;
    }
    p_event->caster_wheels = wheels;
    p_event->until[0] = p_input->now + (RL_CASTER_BOOST_US - boost_age);
}

static inline line_envelope_t rl_power (line_power_t * p_state,
                                        const line_planner_t * p_data,
                                        const circuit_input_t * p_index,
                                        const analog_demand_t * p_demand,
                                        unsigned before, float delta_time)
{
    const circuit_t * p_character = &p_data->circuit;
    if ((J_TURN == before ||
         ('U' == barcode_nav_action(&p_data->nav, p_data->nav.action) &&
          J_TURN_SETTLE == before)) &&
        J_ACQUIRE == p_character->phase)
    {
        circuit_capture_begin(&p_state->capture, p_index->now, p_index->path);
    }
    bool turning = J_TURN == p_character->phase &&
                   ((0 == p_demand->target[0] && (p_demand->target[1] > 0)) ||
                    (0 == p_demand->target[1] && (p_demand->target[0] != 0)));
    const circuit_tracker_t * p_tracker = J_ACQUIRE == p_character->phase
                                              ? &p_character->joining
                                              : &p_character->follower;
    bool recovering = p_tracker->core.white;
    int capture = circuit_capture_outer(&p_state->capture, p_index,
                                        p_character->phase, p_demand);
#if defined(CIRCUIT_STATIONARY_SEARCH) || defined(LINE_STATIONARY_SEARCH)
    bool recovery =
        ((p_tracker->search.active) && recovering) && (!p_demand->stop);
    int search = -1;
#else
    bool recovery = false;
    int search = line_search_outer(p_tracker->search.active && recovering &&
                                       !p_demand->stop,
                                   p_demand->target[0], p_demand->target[1]);
#endif
    bool maneuver = (turning && 'U' == barcode_nav_action(
                                           &p_data->nav, p_data->nav.action)) ||
                    (search >= 0);
    if (recovering)
    {
        capture = -1;
    }
    int load = circuit_load_outer(&p_state->load, p_index, p_character->phase,
                                  p_demand, (capture >= 0) || recovering);
    bool continuous = continuous_drive_selected(RL_PROFILE, p_character->phase,
                                                p_character->busy, false, false,
                                                maneuver, recovering, p_demand);
    if (continuous)
    {
        capture = load = -1;
        p_state->load = (circuit_load_t){0};
    }
    int effort = search >= 0 ? search : capture >= 0 ? capture : load;
    unsigned assist_code = search >= 0    ? (unsigned)search + 5
                           : capture >= 0 ? (unsigned)capture + 1
                           : load >= 0    ? (unsigned)load + 3
                                          : 0;
    line_envelope_t event = {
        .continuous = continuous,
        .maneuver = maneuver,
        .recovery = recovery,
        .reverse =
            'U' == barcode_nav_action(&p_data->nav, p_data->nav.action) &&
            J_TURN == p_character->phase};
    bool ramp_active = false;
#ifdef APP_RTOS_IMU
    motion_ramp_assist_t approved =
        ri_ramp_step(delta_time, continuous && (!recovering), p_index->speed,
                     p_demand->target, p_state->drive);
    ramp_active = (0U != approved.boost_milli) ||
                  (0U != approved.push_pwm_x100) || (0U != approved.hold_milli);
#endif
    for (unsigned wheel = 0; wheel < 2; ++wheel)
    {
        bool loaded = turning || (effort == (int)wheel);
        if (loaded)
        {
            event.loaded |= 1U << wheel;
        }
        if (!continuous)
        {
#ifdef APP_RTOS_IMU
            continuous_leave(
                &p_state->drive[wheel], &p_state->wheel[wheel], wheel,
                p_demand->target[wheel], p_index->speed[wheel],
                ri_unboosted(&p_state->drive[wheel], p_state->output[wheel]));
#else
            continuous_leave(&p_state->drive[wheel], &p_state->wheel[wheel],
                             wheel, p_demand->target[wheel],
                             p_index->speed[wheel], p_state->output[wheel]);
#endif
        }
#ifdef APP_RTOS_IMU
        motion_apply_ramp_profile(&p_state->drive[wheel], &approved);
#endif
        if (!maneuver)
        {
            maneuver_leave(&p_state->maneuver[wheel], &p_state->wheel[wheel],
                           p_demand->target[wheel]);
        }
        if (!recovery)
        {
            p_state->recovery[wheel] = (line_recovery_wheel_t){0};
        }
    }
    if (((!maneuver) && (effort >= 0)) && (assist_code != p_state->assist_code))
    {
        circuit_capture_prepare(&p_state->wheel[effort]);
    }
    p_state->assist_code = assist_code;
    for (unsigned wheel_value = 0; wheel_value < 2; ++wheel_value)
    {
        bool load_eligible =
            ((((!p_data->scan.scanning) && (!recovering)) && (!maneuver)) &&
             (!p_demand->stop)) &&
            C_FOLLOW == p_character->phase && (!p_character->busy) &&
            (SYM_STRAIGHT == p_demand->decision ||
             SYM_LEFT == p_demand->decision ||
             SYM_RIGHT == p_demand->decision || C_FAST == p_demand->decision) &&
            (p_demand->target[0] > 10) && (p_demand->target[1] > 10);
        event.output[wheel_value] =
            recovery ? line_recovery_wheel_step(
                           &p_state->recovery[wheel_value], wheel_value,
                           p_demand->target[wheel_value],
                           p_index->speed[wheel_value], delta_time,
                           p_index->now, p_state->output[wheel_value])
            : continuous
                ? continuous_drive_step_policy(
                      &p_state->drive[wheel_value],
                      &p_state->wheel[wheel_value], wheel_value,
                      p_demand->target[wheel_value],
                      p_index->speed[wheel_value], delta_time, p_index->now,
                      p_state->output[wheel_value], load_eligible)
            : maneuver
                ? maneuver_wheel_step(&p_state->maneuver[wheel_value],
                                      &p_state->wheel[wheel_value], wheel_value,
                                      p_demand->target[wheel_value],
                                      p_index->speed[wheel_value], delta_time,
                                      p_index->now)
                : circuit_wheel_step(
                      &p_state->wheel[wheel_value], wheel_value,
                      p_demand->target[wheel_value],
                      p_index->speed[wheel_value], delta_time, p_index->now,
                      (event.loaded & (1U << wheel_value)) != 0, RL_PROFILE);
        event.until[wheel_value] =
            (maneuver ? p_state->wheel[wheel_value].peak_active : turning)
                ? p_state->wheel[wheel_value].peak_end_us
                : 0;
    }
    const bool caster_eligible = p_data->caster_assist_enabled && continuous &&
                                 p_data->steering.active && (!recovering) &&
                                 (!p_data->scan.scanning) && (!ramp_active) &&
                                 (p_demand->target[0] >= 0) &&
                                 (p_demand->target[1] >= 0);
    const unsigned caster_wheels =
        ((p_demand->target[0] >= RL_CASTER_TARGET_MIN) ? 1U : 0U) |
        ((p_demand->target[1] >= RL_CASTER_TARGET_MIN) ? 2U : 0U);
    rl_caster_step(&p_state->caster, p_index, caster_eligible, caster_wheels,
                   p_demand, &event);
    return event;
}

static inline int32_t rl_cap (const line_envelope_t * p_event, unsigned wheel,
                              uint64_t now)
{
    int32_t output = p_event->output[wheel];
    if (((p_event->caster_wheels & (1U << wheel)) != 0U) &&
        (now < p_event->until[0]) && (output > 0))
    {
        output += RL_CASTER_PWM;
    }
    return p_event->recovery ? line_recovery_output_cap(p_event->output[wheel])
           : p_event->continuous ? continuous_output_cap(output)
           : p_event->maneuver
               ? maneuver_output_cap(p_event->output[wheel],
                                     p_event->until[wheel], now)
               : barcode_output_cap(p_event->output[wheel], false,
                                    p_event->until[wheel], now);
}

/* Feed the unassisted duty back to the PI and slew controller. Otherwise a
 * short boost could be carried into future control steps after expiry. */
static inline int32_t rl_controller_output (const line_envelope_t * p_event,
                                            unsigned wheel, uint64_t now)
{
    return p_event->continuous ? continuous_output_cap(p_event->output[wheel])
                               : rl_cap(p_event, wheel, now);
}

static inline dual_levels_t rl_levels (const line_envelope_t * p_event,
                                       unsigned wheel, uint64_t now)
{
    int32_t output = rl_cap(p_event, wheel, now);
    return p_event->recovery     ? line_recovery_levels(wheel, output)
           : p_event->continuous ? continuous_output_levels(wheel, output)
           : p_event->maneuver
               ? maneuver_levels(wheel, output, p_event->reverse)
           : p_event->reverse
               ? boosted_junction_levels(wheel, output, true)
               : barcode_output_levels(wheel, output, false,
                                       (p_event->loaded & (1U << wheel)) != 0,
                                       RL_PROFILE);
}

/* A supervisor may only lower live channel levels, never re-enable an API
 * stop or change direction using an older committed controller envelope. */
static inline dual_levels_t rl_reduce (dual_levels_t actual, dual_levels_t cap)
{
    return (dual_levels_t){.a = actual.a < cap.a ? actual.a : cap.a,
                           .b = actual.b < cap.b ? actual.b : cap.b};
}

/* Own sector; never erase the SDK motion log, its calibration or Wi-Fi
 * settings. Fixed-endian page, CRC over all bytes except the first magic+CRC
 * words. */
static inline bool rl_cal_decode (const uint8_t page[256],
                                  line_calibration_t * p_cal)
{
    uint32_t magic = 0, crc = 0;
    for (unsigned other_index = 0; other_index < 4; ++other_index)
    {
        magic |= (uint32_t)page[other_index] << (8 * other_index);
        crc |= (uint32_t)page[4 + other_index] << (8 * other_index);
    }
    if (((magic != RL_CAL_MAGIC) || (page[8] != 1)) ||
        (crc != log_crc(page + 8, 248)))
    {
        return false;
    }
    *p_cal = (line_calibration_t){.mask = page[9] & 7U};
    uint64_t packed = 0;
    for (unsigned count = 0; count < 6; ++count)
    {
        packed |= (uint64_t)page[10 + count] << (8 * count);
    }
    p_cal->white[0] = (uint16_t)(packed & 4095U);
    p_cal->white[1] = (uint16_t)((packed >> 12) & 4095U);
    p_cal->black[0] = (uint16_t)((packed >> 24) & 4095U);
    p_cal->black[1] = (uint16_t)((packed >> 36) & 4095U);
    return calibration_valid(p_cal);
}

static inline void rl_cal_encode (uint8_t page[256],
                                  const line_calibration_t * p_cal)
{
    memset(page, 0xff, 256);
    page[8] = 1;
    page[9] = p_cal->mask & 7U;
    uint64_t packed = (p_cal->white[0] & 4095U) |
                      ((uint64_t)(p_cal->white[1] & 4095U) << 12) |
                      ((uint64_t)(p_cal->black[0] & 4095U) << 24) |
                      ((uint64_t)(p_cal->black[1] & 4095U) << 36);
    for (unsigned count = 0; count < 6; ++count)
    {
        page[10 + count] = (uint8_t)(packed >> (8 * count));
    }
    uint32_t crc = log_crc(page + 8, 248);
    for (unsigned other_index = 0; other_index < 4; ++other_index)
    {
        page[other_index] = (uint8_t)(RL_CAL_MAGIC >> (8 * other_index));
        page[4 + other_index] = (uint8_t)(crc >> (8 * other_index));
    }
}

static inline line_calibration_t rl_boot_cal (const uint8_t page[256],
                                              const barcode_saved_t * p_saved,
                                              uint32_t * p_origin)
{
    line_calibration_t cal = successful_calibration();
    *p_origin = 0;
    if (CALIBRATION_RECORD_VERSION == p_saved->version &&
        (barcode_record_valid(p_saved)))
    {
        cal = p_saved->calibration;
        *p_origin = 1;
    }
    line_calibration_t local;
    if (rl_cal_decode(page, &local))
    {
        cal = local;
        *p_origin = 2;
    }
    return cal;
}

/* 56 bytes/row. ADCs use 12 bits each, calibrated levels 10 each, phase 4;
 * all packed words have one writer and copied snapshots under a spinlock. */
typedef struct
{
    uint32_t ms;
    int32_t ticks[2];
    uint32_t optics[2];
    int16_t speed[2];
    int16_t target[2];
    int16_t pwm[2];
    uint16_t dt_us;
    uint16_t cost_us;
    uint32_t meta;
    uint32_t sequence;
    uint32_t aux[2];
    int16_t optical_target[2];
} line_log_row_t;
_Static_assert(56 == sizeof(line_log_row_t), "line log layout");
static inline void rl_pack_optics (line_log_row_t * p_result,
                                   const uint16_t adc[3],
                                   const analog_demand_t * p_demand,
                                   unsigned phase)
{
    p_result->optics[0] = (adc[0] & 4095U) |
                          ((uint32_t)(adc[1] & 4095U) << 12) |
                          ((uint32_t)(adc[2] & 255U) << 24);
    p_result->optics[1] = ((uint32_t)adc[2] >> 8) |
                          ((uint32_t)(p_demand->level[0] & 1023U) << 4) |
                          ((uint32_t)(p_demand->level[1] & 1023U) << 14) |
                          ((phase & 15U) << 24);
}

static inline void rl_pack_steering (line_log_row_t * p_result,
                                     const uint16_t filtered[2],
                                     unsigned gain_milli,
                                     unsigned damping_milli)
{
    p_result->aux[0] = (filtered[0] & 4095U) |
                       ((uint32_t)(filtered[1] & 4095U) << 12) |
                       ((gain_milli & 255U) << 24);
    p_result->aux[1] =
        ((gain_milli >> 8) & 3U) | ((damping_milli & 1023U) << 2);
}

/* Use spare bits: six one-bit observations and signed13-bit angle x10.
 * The 56-byte row and existing steering fields retain their layout. */
static inline void rl_pack_junction (line_log_row_t * p_result,
                                     const line_planner_t * p_data)
{
    float degrees = p_data->circuit.turn.turn_degrees;
    int angle = degrees <= -409.6f ? -4096
                : degrees >= 409.5f
                    ? 4095
                    : (int)(degrees * 10 + (degrees < 0 ? -.5f : .5f));
    p_result->aux[1] = (p_result->aux[1] & 4095U) |
                       ((uint32_t)(p_data->junction.flags & 63U) << 12) |
                       ((uint32_t)(angle & 8191) << 18);
}
static inline int rl_log_angle (const line_log_row_t * p_result)
{
    unsigned value = (p_result->aux[1] >> 18) & 8191U;
    return value & 4096U ? (int)value - 8192 : (int)value;
}

static inline unsigned rl_route_bits (char route)
{
    return 'A' == route ? 1U : 'F' == route ? 2U : 'Z' == route ? 3U : 0U;
}
static inline unsigned rl_route_ascii (unsigned bits)
{
    return 1 == bits ? 'A' : 2 == bits ? 'F' : 3 == bits ? 'Z' : 0;
}

#endif

/* End of file. */
