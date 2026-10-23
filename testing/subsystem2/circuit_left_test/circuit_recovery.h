#ifndef CIRCUIT_RECOVERY_H
#define CIRCUIT_RECOVERY_H
#include "../analog_line_follow/symmetric_logic.h"

enum { C_LOSS_NO_DIRECTION=42,C_LOSS_TIME,C_LOSS_DISTANCE,C_LOSS_STRAIGHT,
       C_LOSS_US=2000000,C_LOSS_MM=50,C_MEMORY_US=1500000,C_WEAK_US=30000,
       C_REVERSE_US=80000,C_REACQUIRE_STRONG_US=40000,C_REACQUIRE_WEAK_US=80000,
       C_BRIDGE_US=150000,C_CENTRE_US=100000,C_CENTRE_MEMORY_US=400000,
       C_STRAIGHT_GAP_US=500000,C_STRAIGHT_GAP_MM=15 };
typedef struct {
    SymmetricTracker core;
    uint64_t weak_since,reverse_since,reacquire_since,strong_since,centre_since,centre_us;
    int16_t bridge[2];
    int8_t pending,reverse;
    uint8_t loss_reason;
    bool weak_timing,reverse_timing,reacquire_timing,strong_timing,centre_timing,centre_valid,straight_recovery;
} CircuitTracker;
/* 0 unknown/stale,1 left,2 right,3 recently centred. This is recovery
   confidence, not a delay on the ordinary optical steering targets. */
static inline unsigned circuit_memory(const CircuitTracker *t,uint64_t now) {
    const SymmetricTracker *s=&t->core;
    if(t->centre_valid && now-t->centre_us<=C_CENTRE_MEMORY_US &&
       (!s->direction || t->centre_us>=s->direction_us)) return 3;
    return s->direction && now-s->direction_us<=C_MEMORY_US?(s->direction>0?1:2):0;
}
static inline void circuit_direction(CircuitTracker *t,int error,unsigned strongest,uint64_t now) {
    SymmetricTracker *s=&t->core;
    int magnitude=error<0?-error:error;int8_t direction=error>0?1:-1;
    bool strong=magnitude>=200 && strongest>=400;
    bool weak=magnitude>=150 && strongest>=200;
    if(strong && (!s->direction || direction==s->direction)) {
        s->direction=direction;s->direction_us=now;
        t->reverse_timing=t->weak_timing=false;t->pending=0;return;
    }
    if(strong) {
        t->weak_timing=false;t->pending=0;
        if(!t->reverse_timing || t->reverse!=direction) {
            t->reverse_timing=true;t->reverse=direction;t->reverse_since=now;
        }
        if(now-t->reverse_since>=C_REVERSE_US) {
            s->direction=direction;s->direction_us=now;t->reverse_timing=false;
        }
        return;
    }
    t->reverse_timing=false;
    /* Weak observations can seed or refresh the same direction. They cannot
       reverse a direction chosen from a stronger observation. */
    if(weak && (!s->direction || direction==s->direction)) {
        if(!t->weak_timing || t->pending!=direction) {
            t->weak_timing=true;t->pending=direction;t->weak_since=now;
        }
        if(now-t->weak_since>=C_WEAK_US) { s->direction=direction;s->direction_us=now; }
    } else { t->weak_timing=false;t->pending=0; }
}
static inline AnalogDemand circuit_line_demand(CircuitTracker *t,const LineCalibration *cal,
                                                uint16_t l,uint16_t r,uint64_t now,uint32_t mm) {
    AnalogDemand d={.decision=SYM_STRAIGHT};t->loss_reason=0;
    SymmetricTracker *s=&t->core;
    if(!calibration_valid(cal)) { d.stop=true;d.decision=AL_LOST;return d; }
    d.level[0]=line_level(l,cal->white[0],cal->black[0]);
    d.level[1]=line_level(r,cal->white[1],cal->black[1]);
    int error=(int)d.level[0]-d.level[1],magnitude=error<0?-error:error;
    unsigned strongest=d.level[0]>d.level[1]?d.level[0]:d.level[1];
    bool low=d.level[0]<150 && d.level[1]<150;
    circuit_direction(t,error,strongest,now);
    bool centred=d.level[0]>=400 && d.level[1]>=400 && magnitude<=80;
    if(!centred) t->centre_timing=false;
    else {
        if(!t->centre_timing) { t->centre_timing=true;t->centre_since=now; }
        if(now-t->centre_since>=C_CENTRE_US) { t->centre_valid=true;t->centre_us=now; }
    }
    if(low && !s->white) {
        unsigned memory=circuit_memory(t,now);
        s->white=true;s->white_since=now;s->white_mm=mm;
        s->recovery=memory==1?1:memory==2?-1:0;
        t->straight_recovery=memory==3;t->reacquire_timing=t->strong_timing=false;
    }
    if(s->white) {
        /* An isolated >=150 observation no longer ends/restarts a loss. The
           same episode and search direction persist until a credible line
           is sustained, including across weak or opposite-side flashes. */
        if(strongest<300) t->reacquire_timing=t->strong_timing=false;
        else {
            if(!t->reacquire_timing) { t->reacquire_timing=true;t->reacquire_since=now; }
            if(strongest<400) t->strong_timing=false;
            else if(!t->strong_timing) { t->strong_timing=true;t->strong_since=now; }
            /* Independent bands:400-threshold flicker cannot discard an
               otherwise sustained >=300 observation. */
            if((t->strong_timing && now-t->strong_since>=C_REACQUIRE_STRONG_US) ||
               now-t->reacquire_since>=C_REACQUIRE_WEAK_US) {
                s->white=false;s->recovery=0;t->straight_recovery=false;
                t->reacquire_timing=t->strong_timing=false;
            }
        }
        if(s->white) {
            uint64_t elapsed=now-s->white_since;uint32_t travel=mm-s->white_mm;
            if(!s->recovery && !t->straight_recovery) t->loss_reason=C_LOSS_NO_DIRECTION;
            else if(t->straight_recovery && (elapsed>=C_STRAIGHT_GAP_US || travel>=C_STRAIGHT_GAP_MM))
                t->loss_reason=C_LOSS_STRAIGHT;
            else if(elapsed>=C_LOSS_US) t->loss_reason=C_LOSS_TIME;
            else if(travel>=C_LOSS_MM) t->loss_reason=C_LOSS_DISTANCE;
            if(t->loss_reason) { d.stop=true;d.decision=AL_LOST;return d; }
            if(t->straight_recovery) { d.target[0]=d.target[1]=35;return d; }
            d.decision=s->recovery>0?SYM_RECOVER_LEFT:SYM_RECOVER_RIGHT;
            d.target[0]=s->recovery>0?0:45;d.target[1]=s->recovery>0?45:0;
            /* Brief worn gaps retain the last reliable curve's curvature at
               reduced speed before beginning the usual bounded pivot search. */
            if(elapsed<C_BRIDGE_US && ((s->recovery>0 && t->bridge[0]<t->bridge[1]) ||
                                      (s->recovery<0 && t->bridge[0]>t->bridge[1]))) {
                d.target[0]=t->bridge[0];d.target[1]=t->bridge[1];
            }
            return d;
        }
    }
    symmetric_targets(error,d.target);
    if(d.target[0]<d.target[1]) d.decision=SYM_LEFT;
    else if(d.target[0]>d.target[1]) d.decision=SYM_RIGHT;
    if(strongest>=400 && ((error>80 && s->direction>0) ||
                         (error < -80 && s->direction<0))) {
        int maximum=d.target[0]>d.target[1]?d.target[0]:d.target[1];
        t->bridge[0]=(int16_t)(d.target[0]*35/maximum);
        t->bridge[1]=(int16_t)(d.target[1]*35/maximum);
    }
    return d;
}
#endif
