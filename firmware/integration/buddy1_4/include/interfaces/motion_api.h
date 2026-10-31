#ifndef BUDDY_SHARED_MOTION_API_H
#define BUDDY_SHARED_MOTION_API_H
#include <stdbool.h>
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* One shared instance in one firmware. Requests are non-blocking and callable
 * from normal task/poll context through the supplied port lock. The bundled
 * standalone harness uses cooperative core0 hooks; multicore/RTOS integration
 * additionally needs a flash-save lockout port and scheduling validation.
 * Do not call from an interrupt, or write motor GPIO/PWM from a buddy module. */
typedef enum {
    MOTION_BUDDY1 = 1,
    MOTION_BUDDY2,
    MOTION_BUDDY3,
    MOTION_BUDDY4,
    MOTION_BUDDY5
} MotionOwner;
typedef enum {
    MOTION_OK = 0,
    MOTION_INACTIVE,
    MOTION_BUSY,
    MOTION_NOT_OWNER,
    MOTION_INVALID,
    MOTION_NOT_STOPPED,
    MOTION_UNSUPPORTED,
    MOTION_NEED_LINE,
    MOTION_NEED_RECOVERY
} MotionResult;
typedef enum {
    MOTION_AUTO = 0,
    MOTION_STOPPING,
    MOTION_PAUSED,
    MOTION_DRIVING,
    MOTION_TURNING,
    MOTION_SETTLING,
    MOTION_FAULT,
    MOTION_OFF
} MotionMode;
typedef enum {
    MOTION_CMD_NONE = 0,
    MOTION_CMD_RUNNING,
    MOTION_CMD_DONE,
    MOTION_CMD_CANCELLED,
    MOTION_CMD_FAILED
} MotionCommandState;
typedef enum { MOTION_ROLLING = 0, MOTION_IN_PLACE } MotionTurnStyle;
typedef enum {
    MOTION_TERRAIN_UNKNOWN = 0,
    MOTION_FLAT,
    MOTION_ASCENDING,
    MOTION_DESCENDING
} MotionTerrain;
enum {
    NAV_FAULT_TIMEOUT = 1u,
    NAV_FAULT_SETTLE = 2u,
    NAV_FAULT_HEADING = 4u,
    NAV_FAULT_DIRECTION = 8u,
    NAV_FAULT_ENCODER = 16u,
    NAV_FAULT_IMU = 32u,
    NAV_FAULT_NO_ECHO = 64u,
    NAV_FAULT_WIFI = 128u,
    NAV_FAULT_CONTROL = 256u,
    NAV_FAULT_STALL = 512u,
    NAV_FAULT_MASK = 1023u
};
typedef struct {
#ifdef INTERFACE_UNPACKED_REFERENCE
    unsigned session_active, stopped, navigation_interrupted, mode, owner, command_state, terrain,
        fault, stop_reason;
#else
    unsigned session_active : 1, stopped : 1, navigation_interrupted : 1;
    unsigned mode : 3, owner : 3, command_state : 3, terrain : 2, fault : 10, stop_reason : 6;
#endif
    uint32_t lease, generation, command_id;
    float progress_mm, rotation_deg;
    int16_t target_mm_s[2];
    char pending_route;
} MotionStatus;
typedef struct {
    uint32_t id;
#ifdef INTERFACE_UNPACKED_REFERENCE
    unsigned state, fault;
#else
    unsigned state : 3, fault : 10;
#endif
    float progress_mm, rotation_deg;
} MotionCommandStatus;

/* Takes control and removes drive immediately; wait for PAUSED/stopped before
 * a move. Buddy5 may pre-empt another buddy; equal/lesser owners cannot steal
 * an existing lease. Returned lease must accompany every movement/resume. */
MotionResult motion_pause(MotionOwner owner, uint32_t *lease);
/* Any buddy can request a soft stop. It cancels the active move and never
 * resumes navigation automatically. Existing ownership is retained. */
MotionResult motion_stop(MotionOwner requester);
/* Signed mm: +forward, -reverse. Allowed magnitude1..1000mm, speed20..60mm/s.
 * speed0 selects45mm/s. Accepted!=completed; query returned command ID. */
MotionResult motion_drive_distance(MotionOwner owner, uint32_t lease, float mm, unsigned speed_mm_s,
                                   uint32_t *command_id);
/* Angle +LEFT/-RIGHT,1..180deg. Speed20..90 or0=45mm/s. Rolling rotates about
 * one rear wheel and shifts the axle; IN_PLACE counter-rotates both wheels.
 * These generic avoidance manoeuvres are new and require physical testing. */
MotionResult motion_turn_relative(MotionOwner owner, uint32_t lease, float degrees,
                                  unsigned wheel_speed_mm_s, MotionTurnStyle style,
                                  uint32_t *command_id);
/* Return only after obstacle clearance and positioning back over the stripe.
 * Requires stopped +>=400normalised on either front sensor for150ms. An
 * interrupted junction requires explicit abandon_navigation=true: it clears
 * stale turn goals; an already-decoded pending route is preserved otherwise. */
MotionResult motion_resume_line(MotionOwner owner, uint32_t lease, bool abandon_navigation);
MotionStatus motion_get_status(void);
bool motion_get_command(uint32_t id, MotionCommandStatus *status);
/* For Buddy3's VERIFIED complete barcode result, not arbitrary sensor edges.
 * A-Z accepted in AUTO in the Wi-Fi integration; its adapter resolves the active runtime map. Unassigned letters do not queue movement. Busy during avoidance; partial optical frames are
 * discarded while external control owns the robot.
 * Does not count an API-injected route as an on-board decoded barcode. */
MotionResult motion_submit_barcode(MotionOwner owner, char code);
/* Buddy3 provider input. Coordinator forwards these only in AUTO; it cannot
 * overwrite an avoidance lease. Called each control tick by the V12.2 adapter. */
MotionResult motion_submit_navigation(MotionOwner owner, int16_t left_mm_s, int16_t right_mm_s);
/* Terrain reporting itself is telemetry; bounded motor assistance is separate. */
MotionResult motion_report_terrain(MotionOwner owner, MotionTerrain terrain);
/* Bounded Buddy4 modifier, not an ownership takeover. Canonical request fields:
 * boost_milli 0..200, push_pwm_x100 0..6000, hold_milli 0..1000, ttl_ms 1..200.
 * Zero levels clear the request. Motion owns the fixed ramp control profile. */
typedef struct {
    uint16_t boost_milli, push_pwm_x100, hold_milli;
    uint8_t ttl_ms;
} MotionRampAssist;
MotionResult motion_request_ramp_assist(MotionOwner owner, const MotionRampAssist *request);
MotionRampAssist motion_get_ramp_assist(void);
/* Physical decoder approval only; Buddy3 owns its decode counters and queue. */
MotionResult motion_accept_barcode(MotionOwner owner, char code);
MotionResult motion_request_speed_assist(MotionOwner owner, unsigned percent);
MotionResult motion_report_fault(MotionOwner owner, unsigned bitmap, bool active);
const char *motion_result_name(MotionResult result);
#ifdef __cplusplus
}
#endif
#endif
