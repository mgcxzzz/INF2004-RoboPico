/**
 * @file b5_straight.h
 * @brief Buddy5 takeover gate: positive, fresh evidence that Buddy3 is
 * stably following a straight.
 * @details The width probe, the direction probe and the bypass assume a
 * straight line segment, so Buddy5 may ask for the takeover only while this
 * gate is open. The motion task copies one Buddy3 frame per control period
 * (rtos_line_straight_sample(), same task, no lock) and steps the gate; the
 * mission guard reads it in the same task. The gate never changes Buddy3's
 * tuning or motor targets. Sonar is unaffected: any close reflector,
 * including a person's leg, still counts as a possible obstacle; the gate
 * only decides whether the manoeuvre may START here.
 *
 * Establish straight evidence across B5S_WINDOW_MM of travel and at least
 * B5S_DWELL_US:
 *  - Buddy3's provider ran this frame (fresh, <= B5S_SAMPLE_MAX_US old);
 *  - circuit phase C_FOLLOW, not busy, no fault, no stop;
 *  - ordinary decision: the continuous steering output (24 scanning,
 *    25 continuous) or straight / small left/right correction / fast;
 *  - line seen (no white loss, no recovery search, no recovery decision);
 *  - no junction candidate or cross mark, no junction/turn phase, no
 *    barcode route turn or U-turn in progress;
 *  - continuous steering active;
 *  - encoder heading change at most 6 degrees over the full window and
 *    3 degrees across the three most recent sample intervals. The shorter
 *    window catches tight curve entry. Optical bend-assist state is for
 *    steering, while this gate relies on measured heading and line state.
 * A curve keeps sliding the window and restarts the dwell.
 * @see app_program/obstacle_navigation/README.md
 */
#ifndef B5_STRAIGHT_H
#define B5_STRAIGHT_H

#include <stdbool.h>
#include <stdint.h>

/* Gate state / blocking reason (E5 v4 flags b12-15). Append only. */
typedef enum
{
    B5S_UNKNOWN = 0, /* never evaluated                                 */
    B5S_STRAIGHT,    /* open: takeover may be requested                 */
    B5S_SETTLING,    /* straight so far, window or dwell not complete   */
    B5S_STALE,       /* no fresh Buddy3 frame                           */
    B5S_UNAVAILABLE, /* Buddy3 provider not running / no bend monitor   */
    B5S_BEND_ENTRY,  /* legacy wire reason                               */
    B5S_BEND,        /* legacy wire reason                               */
    B5S_BEND_EXIT,   /* legacy wire reason                               */
    B5S_CURVE,       /* encoder heading change over the window too big  */
    B5S_JUNCTION,    /* junction phase, candidate edge or cross mark    */
    B5S_TURN,        /* junction turn, acquisition, route turn, U-turn  */
    B5S_LINE_LOST,   /* line loss, recovery search or recovery decision */
    B5S_OTHER,       /* stop, fault, other phase or decision            */
    B5S_REASON_COUNT
} b5_straight_reason_t;

enum
{
    B5S_WINDOW_MM = 60, /* straight travel needed before opening */
    B5S_STEP_MM = 10,   /* heading window sample spacing          */
    B5S_POINTS = (B5S_WINDOW_MM / B5S_STEP_MM) + 1,
    B5S_DWELL_US = 300000,      /* and at least this long                 */
    B5S_SAMPLE_MAX_US = 50000,  /* = B5MI_OBSERVATION_MAX_US              */
    B5S_HEADING_MAX_CDEG = 600, /* allows ordinary corrections over 60 mm */
    B5S_RECENT_STEPS = 3,       /* 30 mm plus sample spacing */
    B5S_RECENT_MAX_CDEG = 300   /* 3.0 deg over the recent window */
};
#define B5S_TRACK_MM 110.0f /* wheel track, = TRACK_MM (dual_logic.h) */

/* One Buddy3 frame, raw values copied in the motion task. */
typedef struct
{
    uint64_t now_us;
    float wheel_mm[2]; /* encoder travel left/right                     */
    uint8_t phase;     /* circuit phase (C_FOLLOW, J_*)                 */
    uint8_t decision;  /* Buddy3 demand decision (24/25, SYM_*, C_FAST) */
    bool valid;        /* Buddy3's provider ran in this frame           */
    bool busy;         /* circuit junction manoeuvre                    */
    bool fault;
    bool stop;
    bool line_lost;     /* follower tracker white (line not seen)        */
    bool searching;     /* recovery search active                        */
    bool junction_mark; /* qualified side edge or barcode cross mark     */
    bool route_turn;    /* barcode route action executing or U-turn      */
    bool steering;      /* continuous steering (bend monitor) active     */
} b5_line_sample_t;

typedef struct
{
    float left[B5S_POINTS]; /* wheel travel every B5S_STEP_MM of path */
    float right[B5S_POINTS];
    uint8_t head; /* next slot                              */
    uint8_t count;
    float last_path;    /* path of the newest window sample       */
    float heading_cdeg; /* |heading change| over the window       */
    float travel_mm;    /* travel covered by the window           */
    uint64_t sample_us; /* last valid frame                       */
    uint64_t since_us;  /* straight conditions held since         */
    uint64_t open_us;   /* gate opened (valid while open)         */
    uint8_t reason;     /* b5_straight_reason_t                   */
    bool open;
} b5_straight_t;

/* Closed, no evidence; reason given (normally B5S_UNKNOWN). */
void b5_straight_reset (b5_straight_t * p_gate, uint8_t reason);
/* One motion frame. An invalid sample closes the gate (UNAVAILABLE). */
void b5_straight_step (b5_straight_t * p_gate,
                       const b5_line_sample_t * p_sample);
/* Freshness check at use: closes the gate (STALE) when the last valid
 * frame is older than B5S_SAMPLE_MAX_US. Returns the current reason. */
uint8_t b5_straight_check (b5_straight_t * p_gate, uint64_t now);

#endif

/* End of file. */
