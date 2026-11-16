/**
 * @file motion_api.h
 * @brief Motion api.
 * @details Public feature contract. Requests cross coordinator ownership
 * checks; callers observe completion/status separately and must respect task
 * context.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_SHARED_MOTION_API_H
#define BUDDY_SHARED_MOTION_API_H
#include <stdbool.h>
#include <stdint.h>
#include "src/motion/drive_mode.h"
#ifdef __cplusplus
extern "C"
{
#endif
    /* One shared instance in one firmware. Requests are non-blocking and
     * callable from normal task/poll context through the supplied port lock.
     * The bundled standalone harness uses cooperative core0 hooks;
     * multicore/RTOS integration additionally needs a flash-save lockout port
     * and scheduling validation. Do not call from an interrupt, or write motor
     * GPIO/PWM from a buddy module. */
    typedef enum
    {
        MOTION_BUDDY1 = 1,
        MOTION_BUDDY2,
        MOTION_BUDDY3,
        MOTION_BUDDY4,
        MOTION_BUDDY5
    } motion_owner_t;
    typedef enum
    {
        MOTION_OK = 0,
        MOTION_INACTIVE,
        MOTION_BUSY,
        MOTION_NOT_OWNER,
        MOTION_INVALID,
        MOTION_NOT_STOPPED,
        MOTION_UNSUPPORTED,
        MOTION_NEED_LINE,
        MOTION_NEED_RECOVERY
    } motion_result_t;
    typedef enum
    {
        MOTION_AUTO = 0,
        MOTION_STOPPING,
        MOTION_PAUSED,
        MOTION_DRIVING,
        MOTION_TURNING,
        MOTION_SETTLING,
        MOTION_FAULT,
        MOTION_OFF
    } motion_mode_t;
    typedef enum
    {
        MOTION_CMD_NONE = 0,
        MOTION_CMD_RUNNING,
        MOTION_CMD_DONE,
        MOTION_CMD_CANCELLED,
        MOTION_CMD_FAILED
    } motion_command_state_t;
    typedef enum
    {
        MOTION_ROLLING = 0,
        MOTION_IN_PLACE
    } motion_turn_style_t;
    typedef enum
    {
        MOTION_TERRAIN_UNKNOWN = 0,
        MOTION_FLAT,
        MOTION_ASCENDING,
        MOTION_DESCENDING
    } motion_terrain_t;
    enum
    {
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
    typedef struct
    {
#ifdef INTERFACE_UNPACKED_REFERENCE
        unsigned session_active, stopped, navigation_interrupted, mode, owner,
            command_state, terrain, fault, stop_reason, speed_mode;
#else
    unsigned session_active : 1;
    unsigned stopped : 1;
    unsigned navigation_interrupted : 1;
    unsigned mode : 3;
    unsigned owner : 3;
    unsigned command_state : 3;
    unsigned terrain : 2;
    unsigned fault : 10;
    unsigned stop_reason : 6;
    unsigned speed_mode : 2;
#endif
        uint32_t lease;
        uint32_t generation;
        uint32_t command_id;
        uint32_t speed_mode_revision;
        float progress_mm;
        float rotation_deg;
        int16_t target_mm_s[2];
        char pending_route;
    } motion_status_t;
    typedef struct
    {
#ifdef INTERFACE_UNPACKED_REFERENCE
        unsigned mode, requested_mode, last_result, session_active, stopped,
            motion_mode;
#else
    unsigned mode : 2;
    unsigned requested_mode : 2;
    unsigned last_result : 4;
    unsigned session_active : 1;
    unsigned stopped : 1;
    unsigned motion_mode : 3;
#endif
        uint32_t revision;
    } motion_speed_mode_status_t;
    typedef struct
    {
        uint32_t id;
#ifdef INTERFACE_UNPACKED_REFERENCE
        unsigned state, fault;
#else
    unsigned state : 3;
    unsigned fault : 10;
#endif
        float progress_mm;
        float rotation_deg;
    } motion_command_status_t;

    /* Takes control and removes drive immediately; wait for PAUSED/stopped
     * before a move. Buddy5 may pre-empt another buddy; equal/lesser owners
     * cannot steal an existing lease. Returned lease must accompany every
     * movement/resume. */
    motion_result_t motion_pause (motion_owner_t owner, uint32_t * p_lease);
    /* Any buddy can request a soft stop. It cancels the active move and never
     * resumes navigation automatically. Existing ownership is retained. */
    motion_result_t motion_stop (motion_owner_t requester);
    /* Signed mm: +forward, -reverse. Allowed magnitude1..1000mm,
     * speed20..60mm/s. speed0 selects45mm/s. Accepted!=completed; query
     * returned command ID. */
    motion_result_t motion_drive_distance (motion_owner_t owner, uint32_t lease,
                                           float mm_value, unsigned speed_mm_s,
                                           uint32_t * p_command_id);
    /* Angle +LEFT/-RIGHT,1..180deg. Speed20..90 or0=45mm/s. Rolling rotates
     * about one rear wheel and shifts the axle; IN_PLACE counter-rotates both
     * wheels. These generic avoidance manoeuvres are new and require physical
     * testing. */
    motion_result_t motion_turn_relative (motion_owner_t owner, uint32_t lease,
                                          float degrees,
                                          unsigned wheel_speed_mm_s,
                                          motion_turn_style_t style,
                                          uint32_t * p_command_id);
    /* Return only after obstacle clearance and positioning back over the
     * stripe. Requires stopped +>=400normalised on either front sensor
     * for150ms. An interrupted junction requires explicit
     * abandon_navigation=true: it clears stale turn goals; an already-decoded
     * pending route is preserved otherwise. */
    motion_result_t motion_resume_line (motion_owner_t owner, uint32_t lease,
                                        bool abandon_navigation);
    motion_status_t motion_get_status (void);
    /* Runtime line cruise mode is RAM-only and may change only while the
     * complete supervisor/motion session is stopped. */
    motion_result_t motion_request_speed_mode (motion_owner_t owner,
                                               unsigned mode);
    motion_speed_mode_status_t motion_get_speed_mode_status (void);
    bool motion_get_command (uint32_t identifier,
                             motion_command_status_t * p_status);
    /* For Buddy3's VERIFIED complete barcode result, not arbitrary sensor
     * edges. A-Z accepted in AUTO in the Wi-Fi integration; its adapter
     * resolves the active runtime map. Unassigned letters do not queue
     * movement. Busy during avoidance; partial optical frames are discarded
     * while external control owns the robot. Does not count an API-injected
     * route as an on-board decoded barcode. */
    motion_result_t motion_submit_barcode (motion_owner_t owner, char code);
    /* Buddy3 provider input. Coordinator forwards these only in AUTO; it cannot
     * overwrite an avoidance lease. Called each control tick by the V12.2
     * adapter. */
    motion_result_t motion_submit_navigation (motion_owner_t owner,
                                              int16_t left_mm_s,
                                              int16_t right_mm_s);
    /* Terrain reporting itself is telemetry; bounded motor assistance is
     * separate. */
    motion_result_t motion_report_terrain (motion_owner_t owner,
                                           motion_terrain_t terrain);
    /* Bounded Buddy4 modifier, not an ownership takeover. Canonical request
     * fields: boost_milli 0..200, push_pwm_x100 0..6000, hold_milli 0..1000,
     * ttl_ms 1..200. Zero levels clear the request. Motion owns the fixed ramp
     * control profile. */
    typedef struct
    {
        uint16_t boost_milli;
        uint16_t push_pwm_x100;
        uint16_t hold_milli;
        uint8_t ttl_ms;
    } motion_ramp_assist_t;
    motion_result_t
    motion_request_ramp_assist (motion_owner_t owner,
                                const motion_ramp_assist_t * p_request);
    motion_ramp_assist_t motion_get_ramp_assist (void);
    /* Physical decoder approval only; Buddy3 owns its decode counters and
     * queue. */
    motion_result_t motion_accept_barcode (motion_owner_t owner, char code);
    motion_result_t motion_request_speed_assist (motion_owner_t owner,
                                                 unsigned percent);
    motion_result_t motion_report_fault (motion_owner_t owner, unsigned bitmap,
                                         bool active);
    const char * motion_result_name (motion_result_t result);
#ifdef __cplusplus
}
#endif
#endif

/* End of file. */
