/**
 * @file b5_straight.c
 * @brief Buddy5 takeover gate (see b5_straight.h for the rule).
 * @details Pure and hardware-free: stepped by the motion task with copied
 * Buddy3 values; host tests drive it with synthetic and real bend-monitor
 * states.
 */
#include "b5_straight.h"

#include <math.h>
#include <string.h>
#include "src/line_following/circuit_logic.h"
#include "src/line_following/line_logic.h"
#include "src/line_following/line_search.h"
#include "src/line_following/symmetric_logic.h"

#define B5S_CDEG_PER_RAD (18000.0f / 3.14159265358979323846f)

/* Output codes of continuous_steering_step_with_cruise() for an ordinary
 * following frame (continuous_steering.h; cruise_handover.h uses the same
 * literals): 24 while scanning for barcodes, 25 otherwise. */
enum
{
    B5S_DECISION_SCANNING = 24,
    B5S_DECISION_CONTINUOUS = 25
};

void b5_straight_reset (b5_straight_t * p_gate, uint8_t reason)
{
    (void)memset(p_gate, 0, sizeof(*p_gate));
    p_gate->reason = reason;
}

static bool turn_phase (unsigned phase)
{
    return (J_TURN == phase) || (J_TURN_SETTLE == phase) ||
           (J_ACQUIRE == phase);
}

static bool junction_phase (unsigned phase)
{
    return (phase >= (unsigned)J_APPROACH) && (phase <= (unsigned)J_ACQUIRE);
}

static uint8_t frame_reason (const b5_line_sample_t * p_sample)
{
    unsigned phase = p_sample->phase;
    unsigned decision = p_sample->decision;
    bool ordinary = (((SYM_STRAIGHT == decision) || (SYM_LEFT == decision)) ||
                     ((SYM_RIGHT == decision) || (C_FAST == decision))) ||
                    ((B5S_DECISION_SCANNING == decision) ||
                     (B5S_DECISION_CONTINUOUS == decision));
    bool recovering =
        (((SYM_RECOVER_LEFT == decision) || (SYM_RECOVER_RIGHT == decision)) ||
         ((C_PROBE_LEFT == decision) || (C_PROBE_RIGHT == decision))) ||
        ((AL_LOST == decision) ||
         ((decision >= (unsigned)LS_DECISION_FIRST) &&
          (decision < (unsigned)B5S_DECISION_SCANNING)));
    uint8_t reason = B5S_STRAIGHT;
    if (!p_sample->valid)
    {
        reason = B5S_UNAVAILABLE;
    }
    else if ((p_sample->route_turn) || (turn_phase(phase)))
    {
        reason = B5S_TURN;
    }
    else if (((p_sample->busy) || (p_sample->junction_mark)) ||
             (junction_phase(phase)))
    {
        reason = B5S_JUNCTION;
    }
    else if (((p_sample->line_lost) || (p_sample->searching)) || (recovering))
    {
        reason = B5S_LINE_LOST;
    }
    else if (((p_sample->stop) || (p_sample->fault)) || (C_FOLLOW != phase))
    {
        reason = B5S_OTHER;
    }
    else if (!p_sample->steering)
    {
        reason = B5S_UNAVAILABLE;
    }
    else if (!ordinary)
    {
        reason = B5S_OTHER;
    }
    return reason;
}

static void push (b5_straight_t * p_gate, const b5_line_sample_t * p_sample,
                  float path)
{
    p_gate->left[p_gate->head] = p_sample->wheel_mm[0];
    p_gate->right[p_gate->head] = p_sample->wheel_mm[1];
    p_gate->head = (uint8_t)((p_gate->head + 1U) % (unsigned)B5S_POINTS);
    if (p_gate->count < (uint8_t)B5S_POINTS)
    {
        p_gate->count++;
    }
    p_gate->last_path = path;
}

static bool sample_stale (uint64_t now, uint64_t previous)
{
    return (now < previous) || ((now - previous) > (uint64_t)B5S_SAMPLE_MAX_US);
}

static float heading_change (const b5_straight_t * p_gate, unsigned index,
                             float wheel_difference)
{
    float origin = p_gate->right[index] - p_gate->left[index];
    return fabsf(wheel_difference - origin) / B5S_TRACK_MM * B5S_CDEG_PER_RAD;
}

void b5_straight_step (b5_straight_t * p_gate,
                       const b5_line_sample_t * p_sample)
{
    uint8_t reason = frame_reason(p_sample);
    /* A new frame arrives before b5_straight_check() in the motion task.
     * Check the previous timestamp before replacing it. */
    if ((B5S_STRAIGHT == reason) && (0U != p_gate->count) &&
        sample_stale(p_sample->now_us, p_gate->sample_us))
    {
        reason = B5S_STALE;
    }
    float path = (p_sample->wheel_mm[0] + p_sample->wheel_mm[1]) * 0.5f;
    if ((B5S_STRAIGHT == reason) && (p_gate->count > 0U) &&
        (path < (p_gate->last_path - 1.0f)))
    {
        reason = B5S_OTHER; /* travel went backwards: not following */
    }
    if (B5S_STRAIGHT != reason)
    {
        b5_straight_reset(p_gate, reason);
        p_gate->sample_us = p_sample->now_us;
        return;
    }
    p_gate->sample_us = p_sample->now_us;
    if (0U == p_gate->count)
    {
        push(p_gate, p_sample, path);
        p_gate->since_us = p_sample->now_us;
    }
    else if ((path - p_gate->last_path) >= (float)B5S_STEP_MM)
    {
        push(p_gate, p_sample, path);
    }
    unsigned oldest =
        ((unsigned)p_gate->head + (unsigned)B5S_POINTS - p_gate->count) %
        (unsigned)B5S_POINTS;
    float wheel_difference = p_sample->wheel_mm[1] - p_sample->wheel_mm[0];
    p_gate->heading_cdeg = heading_change(p_gate, oldest, wheel_difference);
    p_gate->travel_mm =
        path - ((p_gate->left[oldest] + p_gate->right[oldest]) * 0.5f);
    /* The recent window catches entry into a tight curve when corrections
     * earlier in the long window happen to oppose the current turn. */
    unsigned recent = ((unsigned)p_gate->head + (unsigned)B5S_POINTS - 1U -
                       (unsigned)B5S_RECENT_STEPS) %
                      (unsigned)B5S_POINTS;
    bool recent_curve = (p_gate->count > (uint8_t)B5S_RECENT_STEPS) &&
                        (heading_change(p_gate, recent, wheel_difference) >
                         (float)B5S_RECENT_MAX_CDEG);
    if ((p_gate->heading_cdeg > (float)B5S_HEADING_MAX_CDEG) || (recent_curve))
    {
        /* Keep sliding the window: a straight after a curve needs a full
         * window of straight travel; the dwell restarts too. */
        p_gate->open = false;
        p_gate->reason = B5S_CURVE;
        p_gate->since_us = p_sample->now_us;
        return;
    }
    if ((p_gate->count < (uint8_t)B5S_POINTS) ||
        ((p_sample->now_us - p_gate->since_us) < (uint64_t)B5S_DWELL_US))
    {
        p_gate->open = false;
        p_gate->reason = B5S_SETTLING;
        return;
    }
    if (!p_gate->open)
    {
        p_gate->open = true;
        p_gate->open_us = p_sample->now_us;
    }
    p_gate->reason = B5S_STRAIGHT;
}

uint8_t b5_straight_check (b5_straight_t * p_gate, uint64_t now)
{
    if ((B5S_UNKNOWN != p_gate->reason) && sample_stale(now, p_gate->sample_us))
    {
        b5_straight_reset(p_gate, B5S_STALE);
    }
    return p_gate->reason;
}

/* End of file. */
