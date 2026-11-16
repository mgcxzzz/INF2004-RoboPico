/**
 * @file motion_port.h
 * @brief Motion port.
 * @details Motion helpers run under the coordinator/control ownership contract.
 * Preserve bounded outputs, target sign, stop precedence and request lifetimes.
 * @see docs/architecture.md and this feature's README for the full logic.
 */
#ifndef BUDDY_SHARED_MOTION_PORT_H
#define BUDDY_SHARED_MOTION_PORT_H
#include "include/interfaces/motion_api.h"
/* Controller/port implementation only; buddy modules include motion_api.h. */
typedef struct
{
    uintptr_t (*lock)(void * p_user);
    void (*unlock)(void * p_user, uintptr_t state);
    void (*outputs_off)(void * p_user);
    uint64_t (*now_us)(void * p_user);
    void * user;
} motion_port_t;
typedef struct
{
    uint64_t now_us;
    double wheel_mm[2];
    float speed_mm_s[2];
    uint16_t line_level[2];
    bool navigation_busy;
    char pending_route;
} motion_feedback_t;
typedef struct
{
    bool external;
    bool turning;
    bool reset_controllers;
    bool resume_navigation;
    bool abandon_navigation;
    int16_t target_mm_s[2];
    uint32_t generation;
    char route;
} motion_output_t;
typedef struct
{
    uint64_t now_us;
    uint32_t sequence;
    uint32_t id;
    float goal;
    float progress_mm;
    float rotation_deg;
    int16_t target_mm_s[2];
#ifdef INTERFACE_UNPACKED_REFERENCE
    unsigned owner, requester, type, kind, state, fault, result, style, speed;
#else
    unsigned owner : 3;
    unsigned requester : 3;
    unsigned type : 4;
    unsigned kind : 4;
    unsigned state : 3;
    unsigned fault : 10;
    unsigned result : 4;
    unsigned style : 1;
    unsigned speed : 7;
#endif
} motion_event_t;
enum
{
    MOTION_EVENT_PAUSE = 1,
    MOTION_EVENT_STOPPED,
    MOTION_EVENT_START,
    MOTION_EVENT_DONE,
    MOTION_EVENT_CANCEL,
    MOTION_EVENT_FAULT,
    MOTION_EVENT_RESUME,
    MOTION_EVENT_ROUTE,
    MOTION_EVENT_TERRAIN,
    MOTION_EVENT_REJECT,
    MOTION_EVENT_FAULT_REPORT,
    MOTION_EVENT_TARGETS,
    MOTION_EVENT_NAV_STATE,
    MOTION_EVENT_SESSION_END
};
void motion_init (const motion_port_t * p_port);
void motion_session_begin (uint64_t now_us);
void motion_session_end (uint8_t reason);
motion_output_t motion_update (const motion_feedback_t * p_feedback);
bool motion_pop_event (motion_event_t * p_event);
void motion_assist_context (bool eligible);
void motion_note_navigation (unsigned phase, char decoded);
unsigned motion_log_count (void);
unsigned motion_log_overwritten (void);
bool motion_log_read (unsigned index, motion_event_t * p_event);
/* Execute the one hardware owner's commit only if no stop/takeover/new request
 * invalidated its calculation. Callback runs locked and must NOT call the API.
 */
bool motion_commit (uint32_t generation, void (*p_apply)(void *),
                    void * p_user);
#endif

/* End of file. */
