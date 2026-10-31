#ifndef BARCODE_UTURN_LOGIC_H
#define BARCODE_UTURN_LOGIC_H
#include "src/line_following/circuit_logic.h"

enum {
    U_SEGMENT_US = 8000000,
    U_TOTAL_US = 18000000,
    U_SETTLE_US = 2000000,
    U_STILL_US = 120000,
    U_FORWARD_FAST = 90,
    U_FORWARD_MID = 55,
    U_FORWARD_FINE = 35,
    U_REVERSE_FAST = 80,
    U_REVERSE_MID = 45,
    U_REVERSE_FINE = 30
};
typedef struct {
    uint64_t since, settle_since;
    float reverse_origin[2];
    uint8_t stage;
    bool started, settling;
} BarcodeUTurn;
/* Position is signed; travelled distance must keep reverse increments too.
 * Encoder deltas are32-bit, so widen before negating INT32_MIN. */
static inline uint64_t barcode_travel_ticks(int32_t delta) {
    return delta < 0 ? (uint64_t) - (int64_t)delta : (uint64_t)delta;
}
/* Two rolling quarter-turns with the same leftward yaw:
 * right forward/left OFF, settle, left reverse/right OFF, settle.
 * With equal wheel travel the final axle is on the original longitudinal
 * line. A single forward-only semicircle would instead be one track-width
 * beside it. Encoder geometry is provisional; optics must confirm return. */
static inline AnalogDemand barcode_uturn_step(BarcodeUTurn *u, Junction *j,
                                              const LineCalibration *cal, const JunctionInput *i) {
    if (j->phase == J_CENTRE) {
        AnalogDemand d = junction_step(j, cal, i);
        if (j->phase == J_TURN && !d.stop)
            d.target[1] = U_FORWARD_FAST;
        return d;
    }
    AnalogDemand d = {0};
    for (unsigned w = 0; w < 2; ++w)
        d.level[w] = line_level(i->adc[w], cal->white[w], cal->black[w]);
    if (!calibration_valid(cal))
        j_fail(j, J_LINE_LOST);
    if (!u->started && j->phase == J_TURN) {
        u->started = true;
        u->since = i->now;
    }
    j->turn_degrees = j_rotation(j, i);
    if (u->started && i->now - u->since >= U_TOTAL_US)
        j_fail(j, J_TURN_TIMEOUT);
    if (j->turn_degrees < -5 || j->turn_degrees >= 195)
        j_fail(j, J_BRANCH_MISSING);
    if (j->phase == J_TURN) {
        if (i->now - j->phase_us >= U_SEGMENT_US)
            j_fail(j, J_TURN_TIMEOUT);
        float left = i->wheel[0] - j->turn_origin[0], right = i->wheel[1] - j->turn_origin[1];
        bool white = d.level[0] < 200 && d.level[1] < 200;
        if (white && j->turn_degrees > 10) {
            if (!j->off_timing) {
                j->off_timing = true;
                j->off_us = i->now;
            }
            if (i->now - j->off_us >= 30000)
                j->off_line = true;
        } else
            j->off_timing = false;
        if (u->stage == 0) {
            if (fabsf(left) > 12 || right < -5 || right > 220)
                j_fail(j, J_CENTRE_DRIFT);
            if (j->phase != J_FAULT && j->turn_degrees >= 90) {
                u->stage = 1;
                u->settling = false;
                j_enter(j, J_TURN_SETTLE, i->now);
            } else {
                d.target[1] = j->turn_degrees >= 82   ? U_FORWARD_FINE
                              : j->turn_degrees >= 60 ? U_FORWARD_MID
                                                      : U_FORWARD_FAST;
            }
        } else if (u->stage == 2) {
            float reverse = u->reverse_origin[0] - i->wheel[0];
            float stationary = i->wheel[1] - u->reverse_origin[1];
            if (fabsf(stationary) > 12 || reverse < -5 || reverse > 220)
                j_fail(j, J_CENTRE_DRIFT);
            bool branch =
                j->off_line && j->turn_degrees >= 165 && (d.level[0] >= 400 || d.level[1] >= 400);
            if (!branch)
                j->line_timing = false;
            else {
                if (!j->line_timing) {
                    j->line_timing = true;
                    j->line_us = i->now;
                }
                if (i->now - j->line_us >= 30000 && j->phase != J_FAULT) {
                    u->stage = 3;
                    u->settling = false;
                    int error = (int)d.level[0] - d.level[1];
                    j->follower = (SymmetricTracker){.direction = error > 80    ? 1
                                                                  : error < -80 ? -1
                                                                                : 0,
                                                     .direction_us = i->now};
                    j_enter(j, J_TURN_SETTLE, i->now);
                }
            }
            if (j->phase == J_TURN)
                d.target[0] = -(j->turn_degrees >= 170   ? U_REVERSE_FINE
                                : j->turn_degrees >= 150 ? U_REVERSE_MID
                                                         : U_REVERSE_FAST);
        }
    } else if (j->phase == J_TURN_SETTLE) {
        if (i->now - j->phase_us >= U_SETTLE_US)
            j_fail(j, J_SETTLE_TIMEOUT);
        bool still = fabsf(i->speed[0]) < 5 && fabsf(i->speed[1]) < 5;
        if (!still)
            u->settling = false;
        else {
            if (!u->settling) {
                u->settling = true;
                u->settle_since = i->now;
            }
            if (i->now - u->settle_since >= U_STILL_US && j->phase != J_FAULT) {
                if (u->stage == 1) {
                    memcpy(u->reverse_origin, i->wheel, sizeof(u->reverse_origin));
                    u->stage = 2;
                    j->line_timing = false;
                    j_enter(j, J_TURN, i->now);
                } else if (u->stage == 3) {
                    u->stage = 4;
                    j->exit_origin = i->path;
                    j_enter(j, J_ACQUIRE, i->now);
                }
            }
        }
    }
    if (j->phase == J_FAULT) {
        d.stop = true;
        d.target[0] = d.target[1] = 0;
    }
    return d;
}
#endif
