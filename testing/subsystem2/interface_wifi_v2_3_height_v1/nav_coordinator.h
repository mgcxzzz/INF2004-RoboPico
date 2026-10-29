#ifndef BUDDY_NAV_COORDINATOR_H
#define BUDDY_NAV_COORDINATOR_H
#include "motion_port.h"
/* Public motion_api.h is the request facade. These coordinator entry points
 * are its implementation: ownership and policy live here, not in the driver. */
void nav_coordinator_init(const MotionPort *port);
void nav_coordinator_begin(uint64_t now);
void nav_coordinator_end(uint8_t reason);
MotionResult nav_coordinator_pause(MotionOwner owner,uint32_t *lease);
MotionResult nav_coordinator_stop(MotionOwner owner);
MotionResult nav_coordinator_move(MotionOwner owner,uint32_t lease,float goal,unsigned speed,bool turn,MotionTurnStyle style,uint32_t *id);
MotionResult nav_coordinator_resume(MotionOwner owner,uint32_t lease,bool abandon);
MotionResult nav_coordinator_route(MotionOwner owner,char code);
MotionResult nav_coordinator_terrain(MotionOwner owner,MotionTerrain terrain);
MotionResult nav_coordinator_navigation(MotionOwner owner,int16_t left,int16_t right);
MotionResult nav_coordinator_fault(MotionOwner owner,unsigned bitmap,bool active);
void nav_coordinator_note(unsigned phase,char decoded);
void nav_coordinator_reject(MotionOwner owner,unsigned kind,float goal,unsigned speed,MotionResult result);
MotionStatus nav_coordinator_status(void);
bool nav_coordinator_command(uint32_t id,MotionCommandStatus *status);
MotionOutput nav_coordinator_update(const MotionFeedback *feedback);
bool nav_coordinator_commit(uint32_t epoch,void (*apply)(void *),void *user);
bool nav_coordinator_pop(MotionEvent *event);
unsigned nav_coordinator_log_count(void);
unsigned nav_coordinator_log_overwritten(void);
bool nav_coordinator_log_read(unsigned index,MotionEvent *event);
#endif
