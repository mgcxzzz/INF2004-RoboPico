#ifndef CIRCUIT_RECOVERY_H
#define CIRCUIT_RECOVERY_H
#include "../analog_line_follow/symmetric_logic.h"

enum { C_LOSS_NO_DIRECTION=42,C_LOSS_TIME,C_LOSS_DISTANCE,
       C_LOSS_US=2000000,C_LOSS_MM=50,C_MEMORY_US=1500000,C_WEAK_US=30000 };
typedef struct {
    SymmetricTracker core;
    uint64_t weak_since;
    int8_t pending;
    uint8_t loss_reason;
    bool weak_timing;
} CircuitTracker;

/* Circuit-only V3 recovery. The normal steering target function is unchanged.
   Strong asymmetry retains the original immediate direction update; filtered
   weak asymmetry needs30ms of consistent sign before refreshing that memory.
   Two-white never invents a direction or renews its own recovery deadline. */
static inline AnalogDemand circuit_line_demand(CircuitTracker *t,const LineCalibration *cal,
                                                uint16_t l,uint16_t r,uint64_t now,uint32_t mm) {
    AnalogDemand d={.decision=SYM_STRAIGHT};t->loss_reason=0;
    SymmetricTracker *s=&t->core;
    if(!calibration_valid(cal)) { d.stop=true;d.decision=AL_LOST;return d; }
    d.level[0]=line_level(l,cal->white[0],cal->black[0]);
    d.level[1]=line_level(r,cal->white[1],cal->black[1]);
    if(d.level[0]<150 && d.level[1]<150) {
        t->weak_timing=false;t->pending=0;
        if(!s->white) {
            s->white=true;s->white_since=now;s->white_mm=mm;
            s->recovery=s->direction && now-s->direction_us<=C_MEMORY_US?s->direction:0;
        }
        if(!s->recovery) t->loss_reason=C_LOSS_NO_DIRECTION;
        else if(now-s->white_since>=C_LOSS_US) t->loss_reason=C_LOSS_TIME;
        else if(mm-s->white_mm>=C_LOSS_MM) t->loss_reason=C_LOSS_DISTANCE;
        if(t->loss_reason) { d.stop=true;d.decision=AL_LOST;return d; }
        d.decision=s->recovery>0?SYM_RECOVER_LEFT:SYM_RECOVER_RIGHT;
        d.target[0]=s->recovery>0?0:45;d.target[1]=s->recovery>0?45:0;
        return d;
    }
    s->white=false;s->recovery=0;
    int error=(int)d.level[0]-d.level[1];
    int magnitude=error<0?-error:error;
    unsigned strongest=d.level[0]>d.level[1]?d.level[0]:d.level[1];
    int8_t direction=error>0?1:-1;
    if(magnitude>=200 && strongest>=400) {
        s->direction=direction;s->direction_us=now;t->weak_timing=false;t->pending=0;
    } else if(magnitude>=150 && strongest>=200) {
        if(!t->weak_timing || t->pending!=direction) {
            t->weak_timing=true;t->pending=direction;t->weak_since=now;
        }
        if(now-t->weak_since>=C_WEAK_US) { s->direction=direction;s->direction_us=now; }
    } else { t->weak_timing=false;t->pending=0; }
    symmetric_targets(error,d.target);
    if(d.target[0]<d.target[1]) d.decision=SYM_LEFT;
    else if(d.target[0]>d.target[1]) d.decision=SYM_RIGHT;
    return d;
}
#endif
