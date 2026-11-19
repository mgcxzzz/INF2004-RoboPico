#ifndef B5_AUTOMATIC_BYPASS_H
#define B5_AUTOMATIC_BYPASS_H
/* rtos_buddy1_5 port of the SDK automatic v0.6.1 bypass (route execution,
 * return leg, sensor-guided fourth turn, stationary confirmation, bounded
 * search and hand-back), with its stationary-scan side selector REMOVED.
 *
 * What changed from automatic_bypass.c/h v0.6.1 (all else is copied verbatim):
 *  - GUARD/CONFIRM/HALTING/SCAN/REFINE/PLAN and the obstacle profiler are gone.
 *    The caller (b5_mission.c) supplies the side (from the direction probe or
 *    the labelled default) and the ACTUAL stopped forward range through
 *    b5_auto_start(); the route is planned by the unchanged b5_rect_plan_ex()
 *    with no safety scan (the direction probe has no stationary sweep), the
 *    v0.4.1 limits (depth 150 mm, no reverse) and the unchanged 50 mm margin.
 *  - The route adopts the encoder heading/lateral offset left by the probes
 *    (heading0/lateral0), so its absolute 90/0/-90 legs and the fourth-turn
 *    target stay referenced to the approach (line) heading.
 *  - After a successful hand-back the state is B5A_DONE (appended) instead of
 *    re-arming the v0.6.1 guard; the mission owns re-arming.
 *  - The geometry-only image variant is not built.
 * Values of the retained enums are unchanged so B5 event decoders keep their
 * meaning. Pure coordinator API: no PWM, sensor I/O only through the port. */
#include "extensions/buddy5_obstacles/b5_compat.h"
#include "extensions/buddy5_obstacles/manual_bypass.h"
#include "extensions/buddy5_obstacles/profile/obstacle_profile.h"
#include "extensions/buddy5_obstacles/rect_route.h"
#include "extensions/buddy5_obstacles/stripe_detect.h"
/* Values intentionally match the v0.6.1 source-5/B5-event states. 1-6 are no
 * longer entered by this module (the mission reports its own stage). */
enum { B5A_IDLE, B5A_GUARD, B5A_CONFIRM, B5A_HALTING, B5A_SCAN, B5A_REFINE,
       B5A_PLAN, B5A_BYPASS, B5A_REJOIN, B5A_BLOCKED, B5A_FAULT,
       B5A_HOLD /* v0.3 geometry-only build (not built here) */,
       B5A_DONE /* rtos_buddy1_5: handed back to line following */ };
enum { B5A_OK, B5A_SENSOR, B5A_LEASE, B5A_CANCELLED, B5A_NO_PATH,
       B5A_MOVE, B5A_NO_LINE, B5A_STALE, B5A_STALL, B5A_GEOMETRY_DONE,
       B5A_CLEARANCE /* v0.4: first turn cannot keep the margin without reverse */,
       B5A_FALLBACK_NO_LINE /* v0.6.1: no valid crossing, estimated-position sweep found no line */ };
/* Return/acquisition sub-phases inside B5A_REJOIN (v0.5 detector). */
enum { B5R_START, B5R_RETURN, B5R_RETURN_STOP, B5R_ADVANCE, B5R_RESTORE,
       B5R_RESTORE_TURN, B5R_CONFIRM, B5R_SEARCH, B5R_SEARCH_STOP, B5R_RESUME,
       /* v0.6 (appended so v0.5 phase numbers keep their meaning) */
       B5R_ACQUIRE, B5R_ACQUIRE_STOP };
/* v0.6 final turn onto the return line (unchanged; see the v0.6.1 header). */
#define B5R_ACQ_OPEN_DEG     (20.0f)
#define B5R_ACQ_PAST_DEG     (15.0f) /* = v0.5's largest search angle */
#define B5R_ACQ_CLEAR_US     100000u
#define B5R_ACQ_EVIDENCE_US  40000u
#define B5R_ACQ_MIN_SAMPLES  3u
#define B5R_SEARCH_STEPS 6U
#define B5R_FALLBACK_TOL_MM   (60.0f)
#define B5R_FALLBACK_MIN_SIN  (0.866f) /* |return heading| >= 60 deg */
#define B5R_FALLBACK_LEVEL    (B5S_RISE_ABS)
#define B5R_CONFIRM_US   150000u /* stationary stripe evidence before hand-back */
#define B5R_CONFIRM_WAIT_US 400000u /* per pose, then the next bounded search turn */
/* v0.4.1-tested route limits, carried explicitly (unchanged). */
#ifndef B5_PLAN_DEPTH_MM
#define B5_PLAN_DEPTH_MM (150)
#endif
#ifndef B5_ALLOW_REVERSE
#define B5_ALLOW_REVERSE (0)
#endif
#if B5_ALLOW_REVERSE != 0
#error "rtos_buddy1_5 keeps the v0.4.1 no-reverse bypass"
#endif
/* Forward range accepted for planning (v0.6.1 plan() gate, unchanged). */
#define B5_PLAN_RANGE_MIN_MM (80U)
#define B5_PLAN_RANGE_MAX_MM (220U)
/* v0.5 acquisition/hand-back events for the MQTT queue (see b5_event_pack.h). */
enum { B5V_NONE, B5V_TAKEOVER, B5V_PLAN, B5V_HOLD_CLEARANCE, B5V_LEG_END,
       B5V_RETURN_START, B5V_CANDIDATE, B5V_REJECT, B5V_STRIPE_FOUND, B5V_POSITIONED,
       B5V_RESTORED, B5V_SEARCH_TURN, B5V_CONFIRMED, B5V_HANDBACK, B5V_STOPPED,
       B5V_GEOMETRY_HOLD,
       /* v0.6 */ B5V_ACQUIRE, B5V_ACQUIRED,
       /* v0.6.1 */ B5V_FALLBACK };
typedef struct {
    uint32_t ms;
    int16_t a, b, heading_cdeg; /* a/b: event-specific, 0.1 mm or 0.1 deg */
    uint16_t level[2], bg[2];
    uint16_t seq;
    uint8_t code, state, phase, reason, result, detail, search, rejects;
} B5Event;
#define B5_EVENT_RING 24U
/* Sensor port shared by the width probe, direction probe and this bypass
 * (identical to the SDK v0.6.3+ port; latency_us is optional diagnostics). */
typedef struct {
    void (*angle)(int angle_dd, uint64_t now);
    bool (*settled)(uint64_t now);
    bool (*request)(uint64_t now);
    bool (*take)(b5_ping_t *out);
    void (*cancel)(void);
    uint16_t (*latency_us)(void);
} B5SensorPort;
typedef struct {
    B5SensorPort sensor;
    B5Manual route;
    uint64_t since, last_ping, last_good, observed;
    uint64_t search_since;
    uint64_t progress_us;
    uint32_t progress_id;
    float last_progress;
    uint32_t lease, generation, search_id;
    uint16_t line[2], range_mm, pings, faults;
    unsigned state, reason, fault_run, searches;
    int angle_dd;
    bool session, evidence_timing;
    float depth_mm;               /* planning obstacle depth (150) */
    B5RectPlan rect;
    B5Stripe stripe;
    float heading_deg, stop_progress, advance_mm, last_rotation;
    unsigned phase, stripe_result;
    uint16_t seen_rejects;
    uint8_t seen_phase, legs_reported, last_handback;
    B5Event events[B5_EVENT_RING];
    uint8_t ev_head, ev_count;
    uint16_t ev_seq, ev_dropped;
    uint64_t used_observation, evidence_since, phase_since;
    uint32_t observations;
    float acq_onset, search_centre;
    unsigned acq_samples;
    float pivot_return_mm;
    bool fallback;
    uint8_t armed_mask;
    uint8_t line_mask;
    uint64_t sensor_clear_since[2];
    bool sensor_clear_timing[2];
    /* rtos_buddy1_5 inputs, recorded for telemetry */
    uint8_t side;                    /* B5_SIDE_LEFT / B5_SIDE_RIGHT */
    uint16_t start_range_mm;         /* actual stopped forward range used */
    float heading0_deg, lateral0_mm; /* encoder offsets at adoption (+LEFT) */
} B5Automatic;
void b5_auto_init(B5Automatic *a, const B5SensorPort *port);
void b5_auto_observe(B5Automatic *a, uint64_t now, uint16_t left, uint16_t right);
/* Plans and adopts the compact route. Requires Buddy5 to hold `lease` with
 * the coordinator PAUSED and stopped. Returns false (and enters BLOCKED or
 * FAULT, stopped) when the plan or adoption is refused. */
bool b5_auto_start(B5Automatic *a, uint32_t lease, unsigned side, unsigned range_mm,
                   float heading0_deg, float lateral0_mm, uint64_t now);
void b5_auto_tick(B5Automatic *a, uint64_t now);
void b5_auto_cancel(B5Automatic *a, uint64_t now);
const char *b5_auto_name(unsigned state);
const char *b5_auto_phase_name(unsigned phase);
const char *b5_auto_reason_name(unsigned reason);
const char *b5_auto_event_name(unsigned code);
/* Oldest queued event (not removed) / remove it after a successful send. */
bool b5_auto_peek_event(const B5Automatic *a, B5Event *out);
void b5_auto_drop_event(B5Automatic *a);
#endif
