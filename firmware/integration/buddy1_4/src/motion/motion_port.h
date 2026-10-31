#ifndef BUDDY_SHARED_MOTION_PORT_H
#define BUDDY_SHARED_MOTION_PORT_H
#include "include/interfaces/motion_api.h"
/* Controller/port implementation only; buddy modules include motion_api.h. */
typedef struct {
    uintptr_t (*lock)(void *user);
    void (*unlock)(void *user, uintptr_t state);
    void (*outputs_off)(void *user);
    uint64_t (*now_us)(void *user);
    void *user;
} MotionPort;
typedef struct {
    uint64_t now_us;
    double wheel_mm[2];
    float speed_mm_s[2];
    uint16_t line_level[2];
    bool navigation_busy;
    char pending_route;
} MotionFeedback;
typedef struct {
    bool external, turning, reset_controllers, resume_navigation, abandon_navigation;
    int16_t target_mm_s[2];
    uint32_t generation;
    char route;
} MotionOutput;
typedef struct {
    uint64_t now_us;
    uint32_t sequence, id;
    float goal, progress_mm, rotation_deg;
    int16_t target_mm_s[2];
#ifdef INTERFACE_UNPACKED_REFERENCE
    unsigned owner, requester, type, kind, state, fault, result, style, speed;
#else
    unsigned owner : 3, requester : 3, type : 4, kind : 4, state : 3, fault : 10, result : 4,
        style : 1, speed : 7;
#endif
} MotionEvent;
enum {
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
void motion_init(const MotionPort *port);
void motion_session_begin(uint64_t now_us);
void motion_session_end(uint8_t reason);
MotionOutput motion_update(const MotionFeedback *feedback);
bool motion_pop_event(MotionEvent *event);
void motion_assist_context(bool eligible);
void motion_note_navigation(unsigned phase, char decoded);
unsigned motion_log_count(void);
unsigned motion_log_overwritten(void);
bool motion_log_read(unsigned index, MotionEvent *event);
/* Execute the one hardware owner's commit only if no stop/takeover/new request
 * invalidated its calculation. Callback runs locked and must NOT call the API. */
bool motion_commit(uint32_t generation, void (*apply)(void *), void *user);
#endif
