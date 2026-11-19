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
 * Open (B5S_STRAIGHT) only when ALL hold for every frame of the last
 * B5S_WINDOW_MM of travel and at least B5S_DWELL_US:
 *  - Buddy3's provider ran this frame (fresh, <= B5S_SAMPLE_MAX_US old);
 *  - circuit phase C_FOLLOW, not busy, no fault, no stop;
 *  - ordinary decision: the continuous steering output (24 scanning,
 *    25 continuous) or straight / small left/right correction / fast;
 *  - line seen (no white loss, no recovery search, no recovery decision);
 *  - no junction candidate or cross mark, no junction/turn phase, no
 *    barcode route turn or U-turn in progress;
 *  - continuous steering active (the bend monitor runs) and no sustained
 *    bend candidate, confirmed bend or bend exit. Brief corrections before
 *    candidate qualification are covered by the encoder heading check;
 *  - encoder heading change over the window <= B5S_HEADING_MAX_DEG, so a
 *    gentle curve below the bend-assist thresholds cannot qualify.
 * Any failed condition closes the gate and restarts the evidence (a curve
 * keeps sliding the heading window, so a straight after a curve needs a full
 * window of straight travel first).
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
    B5S_BEND_ENTRY,  /* sustained bend candidate                        */
    B5S_BEND,        /* confirmed bend                                  */
    B5S_BEND_EXIT,   /* bend exit / blend back                          */
    B5S_CURVE,       /* encoder heading change over the window too big  */
    B5S_JUNCTION,    /* junction phase, candidate edge or cross mark    */
    B5S_TURN,        /* junction turn, acquisition, route turn, U-turn  */
    B5S_LINE_LOST,   /* line loss, recovery search or recovery decision */
    B5S_OTHER,       /* stop, fault, other phase or decision            */
    B5S_REASON_COUNT
} b5_straight_reason_t;

enum
{
    B5S_WINDOW_MM = 200, /* straight travel needed before opening */
    B5S_STEP_MM = 10,    /* heading window sample spacing          */
    B5S_POINTS = (B5S_WINDOW_MM / B5S_STEP_MM) + 1,
    B5S_DWELL_US = 300000,     /* and at least this long                 */
    B5S_SAMPLE_MAX_US = 50000, /* = B5MI_OBSERVATION_MAX_US              */
    B5S_HEADING_MAX_CDEG = 600 /* 6.0 deg over the window: radius >= ~1.9 m */
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
    bool bend_candidate;
    bool bend_confirmed;
    bool bend_exiting;
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
/* Classify one Buddy3 frame (no state): B5S_STRAIGHT when the frame itself
 * is straight-following, otherwise the blocking reason. */
uint8_t b5_straight_frame_reason (const b5_line_sample_t * p_sample);
/* One motion frame. An invalid sample closes the gate (UNAVAILABLE). */
void b5_straight_step (b5_straight_t * p_gate,
                       const b5_line_sample_t * p_sample);
/* Freshness check at use: closes the gate (STALE) when the last valid
 * frame is older than B5S_SAMPLE_MAX_US. Returns the current reason. */
uint8_t b5_straight_check (b5_straight_t * p_gate, uint64_t now);

#endif

/* End of file. */
