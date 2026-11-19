#ifndef BUDDY5_MANUAL_BYPASS_H
#define BUDDY5_MANUAL_BYPASS_H
#include "include/interfaces/nav_api.h"
#include "extensions/buddy5_obstacles/b5_compat.h" /* rtos_buddy1_5: SDK type names */

/* Stage 1: no sonar, obstacle detection, width claim or automatic line search.
 * Hardware-independent: every motor action uses the shared coordinator. */
enum {
    B5M_IDLE, B5M_ARMED, B5M_TAKEOVER, B5M_MOVING, B5M_NEXT,
    B5M_DONE, B5M_CANCELLED, B5M_FAILED, B5M_RESUMED
};
enum { B5M_FORWARD, B5M_REVERSE, B5M_LEFT, B5M_RIGHT, B5M_BYPASS_LEFT, B5M_BYPASS_RIGHT };
enum { B5M_ARM, B5M_CLAIM, B5M_START, B5M_FINISH, B5M_HOLD, B5M_CANCEL,
       B5M_ERROR, B5M_RESUME };
enum { B5M_OK, B5M_EXPIRED, B5M_OWNER_LOST, B5M_COMMAND_LOST, B5M_INTERRUPTED,
       B5M_TIMED_OUT, B5M_COORDINATOR_FAULT, B5M_BAD_REQUEST };
enum { B5M_MAX_STEPS = 8, B5M_EVENTS = 32, B5M_SPEED = 30 };
/* v0.6: clear straight legs of the automatic rectangle (sideways and the
 * forward pass) only. Turns, the return/acquisition, advance, search and the
 * manual console routes stay at B5M_SPEED or slower. Wheel-speed target only:
 * the PWM cap, slew, start aid and I300 limit are unchanged. */
enum { B5M_STRAIGHT_SPEED = 45, B5M_SLOW_SPEED = 20 /* API minimum */ };

typedef struct {
    float goal;
    unsigned turn : 1;
    /* v0.3 automatic only: goal is a heading (+LEFT deg) relative to the
     * adoption pose; the issued turn subtracts the accumulated ENCODER rotation
     * of completed legs so relative-turn errors do not accumulate. */
    unsigned absolute : 1;
    /* v0.6: clear straight leg driven at B5M_STRAIGHT_SPEED. */
    unsigned fast : 1;
} B5ManualStep;
typedef struct {
    uint64_t time_us;
    uint32_t command_id;
    float goal, progress_mm, rotation_deg;
    unsigned event : 4, step : 4, detail : 4, result : 4;
} B5ManualEvent;
/* v0.4 per-leg stopped diagnostics (recorded O(1) at leg start/end, printed
 * only by the stopped console). measured_* come from the coordinator's own
 * command record: encoder estimates, not independent body measurements. */
enum { B5LEG_PENDING, B5LEG_RUNNING, B5LEG_DONE, B5LEG_SKIPPED,
       B5LEG_FAILED /* manual executor reason */, B5LEG_INTERRUPTED /* automatic reason */ };
typedef struct {
    float issued, measured_mm, measured_deg;
    uint8_t outcome, reason;
} B5ManualLeg;
typedef struct {
    unsigned state : 4, kind : 3, index : 4, count : 4, reason : 4;
    uint32_t lease, command_id, expected_generation;
    uint64_t armed_us, began_us;
    B5ManualStep plan[B5M_MAX_STEPS];
    B5ManualEvent events[B5M_EVENTS];
    /* Sum of completed commands' encoder rotation (+LEFT). Not body heading. */
    float heading_deg;
    float issued[B5M_MAX_STEPS];
    B5ManualLeg leg[B5M_MAX_STEPS];
    uint8_t event_count;
    bool events_dropped;
} B5Manual;

void b5_manual_init(B5Manual *job);
bool b5_manual_arm(B5Manual *job, unsigned kind, uint64_t now_us);
/* Automatic wrapper may reuse the SAME route executor after its scan has
 * stopped and claimed a valid Buddy5 lease. Never starts a session. */
bool b5_manual_adopt(B5Manual *job, uint32_t lease, bool left,
                     float lateral_mm, uint64_t now_us);
/* v0.3 automatic rectangular route (absolute 90-degree headings). With
 * geometry_only the route also returns to the original line position and
 * restores the heading, then holds; it never requests line handover. */
bool b5_manual_adopt_rect(B5Manual *job, uint32_t lease, bool left, float backoff_mm,
                          float side_mm, float forward_mm, bool geometry_only,
                          uint64_t now_us);
void b5_manual_cancel(B5Manual *job, uint64_t now_us);
/* Close the leg that was running when an outside guard stopped the route. */
void b5_manual_mark_interrupted(B5Manual *job, unsigned reason);
const char *b5_manual_leg_outcome_name(unsigned outcome);
void b5_manual_poll(B5Manual *job, uint64_t now_us);
bool b5_manual_pause(B5Manual *job, uint64_t now_us);
/* Only after a completed full bypass. Existing stopped/line-present and
 * interrupted-navigation guards remain authoritative. No automatic retry. */
MotionResult b5_manual_resume(B5Manual *job, uint64_t now_us);
const char *b5_manual_state_name(unsigned state);
/* Console adapter status for stopped-only diagnostic capture, not motion. */
bool b5_manual_capture_ready(void);
#endif
