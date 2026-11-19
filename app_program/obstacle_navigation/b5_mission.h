/**
 * @file b5_mission.h
 * @brief Buddy5 obstacle mission: guard, width probe, line alignment,
 * owned approach, direction probe and the compact bypass with fourth-turn
 * line acquisition.
 * @details Hardware-free sequencing over the shared coordinator (nav_api.h).
 * One owner steps it (the core1 motion task, right after motion_update()).
 * The frozen SDK algorithms (width_probe.c, edge_probe.c, automatic_bypass.c)
 * are stepped unchanged; this module decides only WHICH one runs and what may
 * happen between them. Buddy5 keeps one lease from the takeover at the
 * obstacle until the final hand-back or an operator Stop.
 * @see app_program/obstacle_navigation/README.md
 */
#ifndef B5_MISSION_H
#define B5_MISSION_H

#include <stdbool.h>
#include <stdint.h>
#include "extensions/buddy5_obstacles/automatic_bypass.h"
#include "extensions/buddy5_obstacles/width_probe.h"
#include "extensions/buddy5_obstacles/edge_probe.h"
#include "src/line_following/line_logic.h"
#include "b5_straight.h"
#ifndef MOTION_STEERED_DRIVE
#error "b5_mission needs the coordinator's lease-holder drive steering"
#endif

/* Mission stages. Values are transmitted (E5 status byte 8): append only. */
typedef enum
{
    B5MI_IDLE = 0,  /* no session (or not started)                     */
    B5MI_GUARD,     /* line following; fresh front sonar watched       */
    B5MI_HALTING,   /* takeover accepted; waiting for PAUSED/stopped   */
    B5MI_WIDTH,     /* width probe v0.2 running (stopped at ~300 mm)   */
    B5MI_ALIGN,     /* bounded alignment on fresh line evidence        */
    B5MI_APPROACH,  /* owned straight approach to ~200 mm              */
    B5MI_DIRECTION, /* direction probe (edge probe) running            */
    B5MI_BYPASS,    /* compact bypass, return and fourth-turn rejoin   */
    B5MI_HOLD,      /* stopped, lease kept; operator Stop ends it      */
    B5MI_CANCELLED, /* Stop/Pause/session end while Buddy5 was active  */
    B5MI_RANGE,     /* stationary fresh-range triplet                  */
    B5MI_STAGE_COUNT
} b5mi_stage_t;

/* Hold / end reasons (E5 byte 9). Append only. */
typedef enum
{
    B5MI_R_NONE = 0,
    B5MI_R_CANCELLED,
    B5MI_R_SENSOR,           /* sonar faults/timeout outside a probe          */
    B5MI_R_LEASE,            /* Buddy5 lease lost or refused                  */
    B5MI_R_MOVE,             /* a mission move was refused or interrupted     */
    B5MI_R_TIMEOUT,          /* stage deadline                                */
    B5MI_R_STALE,            /* line observations stale / coordinator fault   */
    B5MI_R_STALL,            /* no encoder progress for 3 s                   */
    B5MI_R_WIDTH_ABORT,      /* width probe ended after motion, not COMPLETE  */
    B5MI_R_WIDTH_BUSY,       /* width probe could not start                   */
    B5MI_R_NO_LINE_ALIGN,    /* bounded alignment found no line evidence      */
    B5MI_R_NO_LINE_APPROACH, /* line evidence lost during the approach       */
    B5MI_R_APPROACH_RANGE,   /* could not reach the direction-probe range     */
    B5MI_R_DIRECTION_ABORT,  /* direction probe did not complete              */
    B5MI_R_DIRECTION_UNKNOWN, /* complete but unknown/contradictory/unhealthy */
    B5MI_R_DIRECTION_BUSY, /* direction probe could not start               */
    B5MI_R_RANGE,          /* fresh range unusable for the next stage       */
    B5MI_R_BYPASS_BLOCKED, /* plan refused (range/clearance): BLOCKED       */
    B5MI_R_BYPASS_FAULT,   /* bypass/rejoin fault (its own reason recorded) */
    B5MI_R_NOT_STRAIGHT,   /* close obstacle while the takeover gate was
                              closed: stopped, no probe or bypass; module
                              reason = gate reason (b5_straight_reason_t) */
    B5MI_R_COUNT
} b5mi_reason_t;

/* Width policy for this mission run (E5 v2 byte 19). Append only. */
typedef enum
{
    B5MI_WIDTH_NONE = 0, /* no width decision yet                         */
    B5MI_WIDTH_MEASURED, /* probe attempted and COMPLETE                  */
    B5MI_WIDTH_SKIPPED,  /* clean stopped range outside 270-330 mm: probe
                            deliberately not started, no motion          */
    B5MI_WIDTH_FAILED,   /* probe attempted and did not complete: HOLD    */
    B5MI_WIDTH_RUNNING   /* probe attempted, not finished                 */
} b5mi_width_policy_t;

/* Direction basis (E5 byte 18). */
typedef enum
{
    B5MI_DIR_NONE = 0, /* no direction result in this mission            */
    B5MI_DIR_EVIDENCE, /* probe decision LEFT/RIGHT                      */
    B5MI_DIR_DEFAULT,  /* complete, healthy, equal sides: default side   */
    B5MI_DIR_HELD      /* incomplete/unknown/unhealthy: no movement      */
} b5mi_direction_basis_t;

/* Configuration, fixed at build time (see README for the evidence). */
enum
{
    B5MI_TRIGGER_MM = 300,         /* guard: three clean echoes <= this       */
    B5MI_TRIGGER_CONFIRM = 3,      /* v0.6.1 guard confirmation count         */
    B5MI_REARM_CLEAR_US = 1000000, /* clear this long before re-arming    */
    B5MI_APPROACH_TARGET_MM = 200, /* direction probe validated distance  */
    B5MI_APPROACH_STOP_MM = 205,   /* halt on a clean echo at or below    */
    B5MI_APPROACH_ACCEPT_MM = 215, /* range accepted without another leg  */
    B5MI_APPROACH_MAX_LEG_MM = 150, /* one leg never exceeds this         */
    B5MI_APPROACH_MAX_LEGS = 3,
    B5MI_APPROACH_SPEED = 20,        /* mm/s, API minimum                   */
    B5MI_APPROACH_NO_LINE_MM = 40,   /* travel without line evidence: halt */
    B5MI_STEER_MAX_DECIDEG = 80,     /* +/-8 deg setpoint (coordinator cap) */
    B5MI_ALIGN_TRIM_MAX_CDEG = 1200, /* larger width residual: hold       */
    B5MI_ALIGN_SPEED = 20,
    B5MI_ALIGN_STEPS = 6,           /* -4,+4,-8,+8,-12,+12 deg            */
    B5MI_LINE_LEVEL = 400,          /* coordinator hand-back level         */
    B5MI_EVIDENCE_US = 150000,      /* stationary line evidence            */
    B5MI_EVIDENCE_WAIT_US = 400000, /* per pose before the next search    */
    B5MI_RANGE_SPREAD_MM = 15,      /* fresh range triplet spread          */
    B5MI_RANGE_ATTEMPTS = 2,
    B5MI_PING_TIMEOUT_US = 400000,
    B5MI_HALT_LIMIT_US = 3000000,
    B5MI_ALIGN_LIMIT_US = 30000000,
    B5MI_APPROACH_LIMIT_US = 30000000,
    B5MI_STALL_US = 3000000,
    B5MI_OBSERVATION_MAX_US = 50000,
    B5MI_DEFAULT_SIDE = B5_SIDE_RIGHT /* v0.6.x deterministic fallback   */
};

typedef struct
{
    uint16_t range_mm; /* median of three clean echoes                 */
    uint8_t status;    /* 0 pending, 1 ok, 2 rejected                  */
    uint8_t attempts;
    uint8_t npings;
    bool ping_active;
    uint64_t ping_since;
    b5_ping_t ping[3];
} b5mi_range_t;

typedef struct
{
    /* Modules stepped by the mission (frozen algorithms). */
    B5WidthProbe width;
    B5EdgeProbe edge;
    B5Automatic bypass;
    B5SensorPort sensor;
    /* Mission state. */
    b5mi_stage_t stage;
    b5mi_stage_t after_range; /* stage that consumes the range result */
    b5mi_reason_t reason;
    uint8_t module_reason; /* width/edge end or bypass reason at a hold */
    bool session;
    bool rearm;
    bool clear_timing;
    uint64_t clear_since;
    uint32_t lease;
    uint32_t generation;
    uint32_t command;
    uint16_t run;
    uint64_t stage_since;
    uint64_t sub_since;
    unsigned sub;
    /* Sonar watch (guard / approach). */
    unsigned confirm;         /* fresh close echoes while the gate is open */
    unsigned close_count;     /* consecutive close echoes, gate ignored    */
    uint64_t taken_requested; /* request time of the ping just taken      */
    /* Straight-only takeover gate (Buddy3 frames, same task). */
    b5_straight_t straight;
    uint8_t gate_reason; /* b5_straight_reason_t at the last guard step */
    unsigned fault_run;
    bool ping_pending;
    uint64_t last_ping;
    uint64_t ping_requested;
    uint16_t trigger_mm;
    /* Line observations (copied each motion frame). */
    uint64_t observed;
    uint64_t used_observation;
    uint16_t line[2];
    uint8_t takeover_mask;
    bool evidence_timing;
    uint64_t evidence_since;
    /* Alignment and approach. */
    float align_heading_deg; /* encoder rotation since the width end */
    unsigned align_steps;
    float approach_last_line_mm;
    unsigned approach_legs;
    uint16_t approach_start_mm;
    bool halting_move;
    /* Bounded line steering during the approach (Buddy3 edge law). */
    bool steering; /* true in production; host tests may clear it */
    analog_tracker_t steer_tracker;
    float steer_mm; /* setpoint, right-minus-left wheel travel */
    uint64_t steer_us;
    int16_t steer_decideg;     /* last setpoint sent, 0.1 deg (+LEFT) */
    uint8_t steer_max_decideg; /* largest |setpoint| this mission run */
    uint8_t steer_saturated;   /* frames at the +/-8 deg bound (sat.) */
    /* Encoder progress watchdog for mission moves. */
    uint32_t progress_id;
    float last_progress;
    uint64_t progress_us;
    /* Fresh stationary range. */
    b5mi_range_t range;
    /* Results (retained until the next mission run / session). */
    uint16_t width_d0;
    uint8_t width_end;
    bool width_attempted;
    bool width_started;
    uint8_t width_policy;       /* b5mi_width_policy_t */
    uint16_t width_gate_mm;     /* fresh range that decided the policy */
    uint8_t direction_side;     /* B5_SIDE_* actually used                 */
    uint8_t direction_basis;    /* b5mi_direction_basis_t                  */
    uint8_t direction_decision; /* probe decision (B5EP_DECISION_*)        */
    uint8_t direction_end;
    bool direction_attempted;
    uint16_t bypass_range_mm;
    uint16_t stale_dropped; /* unsent records of a previous run discarded */
} b5_mission_t;

void b5_mission_init (b5_mission_t * p_mission, const B5SensorPort * p_port,
                      int (*p_applied)(void));
/* Session start: arm the guard (servo centred). Returns false while a
 * previous run still has unsent telemetry that cannot be discarded. */
bool b5_mission_begin (b5_mission_t * p_mission, uint64_t now);
/* Copy one line observation (normalised 0..1000) for this motion frame. */
void b5_mission_observe (b5_mission_t * p_mission, uint64_t now, uint16_t left,
                         uint16_t right);
/* Copy one Buddy3 line-following frame into the takeover gate (motion
 * task, before b5_mission_tick). */
void b5_mission_line (b5_mission_t * p_mission,
                      const b5_line_sample_t * p_sample);
/* One decision step, right after motion_update(). Bounded, nonblocking. */
void b5_mission_tick (b5_mission_t * p_mission, uint64_t now);
/* Stop/Pause/E-stop/network loss: cancel whichever module runs. */
void b5_mission_cancel (b5_mission_t * p_mission, uint64_t now);
/* True while Buddy5 owns or is acquiring the robot. */
bool b5_mission_active (const b5_mission_t * p_mission);
const char * b5_mission_stage_name (unsigned stage);
const char * b5_mission_reason_name (unsigned reason);

#endif

/* End of file. */
