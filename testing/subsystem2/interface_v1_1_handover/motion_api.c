#include "motion_api.h"
#include "nav_coordinator.h"
#include "nav_codec.h"
#include <math.h>
#include <stddef.h>
/* All buddy requests cross the canonical compact binary boundary before
 * coordinator policy. Convenience C arguments are not a second motor path. */
static bool valid_owner(MotionOwner o) {return o>=MOTION_BUDDY1 && o<=MOTION_BUDDY5;}
static MotionResult reject(MotionOwner o,unsigned kind,float value,unsigned speed,MotionResult result) {
    nav_coordinator_reject(o,kind,value,speed,result);return result;
}
static MotionResult submit(const NavRequest *request,uint32_t *token) {
    uint8_t packet[NAV_PACKET_BYTES];NavRequest r;
    unsigned n=nav_packet_encode(packet,request);
    if(!n || !nav_packet_decode(&r,packet,n))return reject((MotionOwner)request->owner,request->kind,request->value,request->speed,MOTION_INVALID);
    MotionOwner o=(MotionOwner)r.owner;MotionResult result;
    switch(r.kind) {
    case NAV_PAUSE:result=nav_coordinator_pause(o,token);break;
    case NAV_STOP:result=nav_coordinator_stop(o);break;
    case NAV_DRIVE:case NAV_TURN:result=nav_coordinator_move(o,r.lease,r.value,r.speed,r.kind==NAV_TURN,(MotionTurnStyle)r.style,token);break;
    case NAV_RESUME:result=nav_coordinator_resume(o,r.lease,r.flag);break;
    case NAV_ROUTE:result=nav_coordinator_route(o,r.value==0?'A':r.value==1?'F':'Z');break;
    case NAV_TERRAIN:result=nav_coordinator_terrain(o,(MotionTerrain)(unsigned)r.value);break;
    case NAV_WHEELS:result=nav_coordinator_navigation(o,r.wheels[0],r.wheels[1]);break;
    case NAV_FAULT_REPORT:result=nav_coordinator_fault(o,r.bitmap,r.flag);break;
    default:result=MOTION_UNSUPPORTED;break;
    }
    /* Per-tick Buddy3 BUSY simply means it yielded to an avoidance lease;
     * do not flood the audit log with repeated control targets. */
    if(result!=MOTION_OK && !(r.kind==NAV_WHEELS && result==MOTION_BUSY))
        nav_coordinator_reject(o,r.kind,r.value,r.speed,result);
    return result;
}
MotionResult motion_pause(MotionOwner o,uint32_t *lease) {
    if(!valid_owner(o) || !lease)return reject(o,NAV_PAUSE,0,0,MOTION_INVALID);
    NavRequest r={.kind=NAV_PAUSE,.owner=o};return submit(&r,lease);
}
MotionResult motion_stop(MotionOwner o) {
    if(!valid_owner(o))return reject(o,NAV_STOP,0,0,MOTION_INVALID);
    NavRequest r={.kind=NAV_STOP,.owner=o};return submit(&r,NULL);
}
MotionResult motion_drive_distance(MotionOwner o,uint32_t lease,float mm,unsigned speed,uint32_t *id) {
    if(id)*id=0;
    if(!valid_owner(o) || !id || speed>60)return reject(o,NAV_DRIVE,mm,speed,MOTION_INVALID);
    NavRequest r={.kind=NAV_DRIVE,.owner=o,.lease=lease,.value=mm,.speed=speed};return submit(&r,id);
}
MotionResult motion_turn_relative(MotionOwner o,uint32_t lease,float degrees,unsigned speed,MotionTurnStyle style,uint32_t *id) {
    if(id)*id=0;
    if(!valid_owner(o) || !id || speed>90 || (style!=MOTION_ROLLING && style!=MOTION_IN_PLACE))
        return reject(o,NAV_TURN,degrees,speed,MOTION_INVALID);
    NavRequest r={.kind=NAV_TURN,.owner=o,.lease=lease,.value=degrees,.speed=speed,.style=style};return submit(&r,id);
}
MotionResult motion_resume_line(MotionOwner o,uint32_t lease,bool abandon) {
    if(!valid_owner(o))return reject(o,NAV_RESUME,0,0,MOTION_INVALID);
    NavRequest r={.kind=NAV_RESUME,.owner=o,.lease=lease,.flag=abandon};return submit(&r,NULL);
}
MotionStatus motion_get_status(void) {return nav_coordinator_status();}
bool motion_get_command(uint32_t id,MotionCommandStatus *s) {return nav_coordinator_command(id,s);}
MotionResult motion_submit_barcode(MotionOwner o,char c) {
    if(!valid_owner(o) || (c!='A' && c!='F' && c!='Z'))return reject(o,NAV_ROUTE,c,0,MOTION_INVALID);
    NavRequest r={.kind=NAV_ROUTE,.owner=o,.value=c=='A'?0:c=='F'?1:2};return submit(&r,NULL);
}
MotionResult motion_submit_navigation(MotionOwner o,int16_t left,int16_t right) {
    if(!valid_owner(o))return reject(o,NAV_WHEELS,0,0,MOTION_INVALID);
    NavRequest r={.kind=NAV_WHEELS,.owner=o,.wheels={left,right}};return submit(&r,NULL);
}
MotionResult motion_report_terrain(MotionOwner o,MotionTerrain t) {
    if(!valid_owner(o) || (unsigned)t>(unsigned)MOTION_DESCENDING)return reject(o,NAV_TERRAIN,t,0,MOTION_INVALID);
    NavRequest r={.kind=NAV_TERRAIN,.owner=o,.value=(float)t};return submit(&r,NULL);
}
MotionResult motion_request_speed_assist(MotionOwner o,unsigned p) {
    return reject(o,NAV_ASSIST,(float)p,0,MOTION_UNSUPPORTED);
}
MotionResult motion_report_fault(MotionOwner o,unsigned mask,bool active) {
    if(!valid_owner(o) || !mask || (mask&~NAV_FAULT_MASK))return reject(o,NAV_FAULT_REPORT,(float)mask,0,MOTION_INVALID);
    NavRequest r={.kind=NAV_FAULT_REPORT,.owner=o,.bitmap=mask,.flag=active};return submit(&r,NULL);
}
const char *motion_result_name(MotionResult r) {
    static const char *const names[]={"OK","INACTIVE","BUSY","NOT_OWNER","INVALID","NOT_STOPPED","UNSUPPORTED","NEED_LINE","NEED_RECOVERY"};
    return (unsigned)r<sizeof(names)/sizeof(names[0])?names[r]:"UNKNOWN";
}
void motion_init(const MotionPort *p) {nav_coordinator_init(p);}
void motion_session_begin(uint64_t t) {nav_coordinator_begin(t);}
void motion_session_end(uint8_t r) {nav_coordinator_end(r);}
MotionOutput motion_update(const MotionFeedback *f) {return nav_coordinator_update(f);}
bool motion_commit(uint32_t e,void (*f)(void *),void *u) {return nav_coordinator_commit(e,f,u);}
bool motion_pop_event(MotionEvent *e) {return nav_coordinator_pop(e);}
void motion_note_navigation(unsigned phase,char decoded) {nav_coordinator_note(phase,decoded);}
unsigned motion_log_count(void) {return nav_coordinator_log_count();}
unsigned motion_log_overwritten(void) {return nav_coordinator_log_overwritten();}
bool motion_log_read(unsigned i,MotionEvent *e) {return nav_coordinator_log_read(i,e);}
