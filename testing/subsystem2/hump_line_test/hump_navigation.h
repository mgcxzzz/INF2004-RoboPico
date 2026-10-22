#ifndef HUMP_NAVIGATION_H
#define HUMP_NAVIGATION_H
#include <stdlib.h>
#include "hump_control.h"

/* Straight-hump experiment only. Encoders cannot establish line location or
 * true heading under slip. A recent centred observation grants one bounded
 * carry-through; weak/flickering readings cannot renew its distance or timer. */
enum { HUMP_CARRY=16,HUMP_REACQUIRE=17,
       HUMP_CARRY_US=1000000,HUMP_CARRY_MM=25,HUMP_GOOD_US=150000,
       HUMP_RECENT_US=1000000,HUMP_REACQUIRE_US=80000,
       HUMP_WHY_TIME=14,HUMP_WHY_DISTANCE=15,HUMP_WHY_HEADING=16 };
#define HUMP_HEADING_MAX 8.0f
typedef struct {
    SymmetricTracker optical;
    uint64_t good_since,last_good,carry_since,reacquire_since;
    float reference_heading,carry_path;
    bool good_timing,armed,active,reacquiring;
} HumpNavigation;
typedef struct { AnalogDemand demand;bool assist_allowed;uint8_t reason; } HumpNavigationOutput;

static inline HumpNavigationOutput hump_navigation_step(HumpNavigation *n,const LineCalibration *cal,
        uint16_t left,uint16_t right,uint64_t now,float path,float heading,
        float speed_left,float speed_right,unsigned profile) {
    HumpNavigationOutput o={0};
    AnalogDemand *d=&o.demand;
    if(!calibration_valid(cal)) { d->stop=true;d->decision=AL_LOST;return o; }
    d->level[0]=line_level(left,cal->white[0],cal->black[0]);
    d->level[1]=line_level(right,cal->white[1],cal->black[1]);
    int difference=(int)d->level[0]-d->level[1];
    bool strong=d->level[0]>=400 && d->level[1]>=400;
    bool weak=d->level[0]<400 && d->level[1]<400;
    if(!n->active) {
        bool good=profile==HUMP_ASSIST && strong && abs(difference)<=150 &&
                  speed_left>=15 && speed_right>=15;
        if(good) {
            if(!n->good_timing) { n->good_timing=true;n->good_since=now; }
            if(now-n->good_since>=HUMP_GOOD_US) {
                n->armed=true;n->last_good=now;n->reference_heading=heading;
            }
        } else n->good_timing=false;
        if(n->armed && now-n->last_good>HUMP_RECENT_US) n->armed=false;
        if(profile==HUMP_ASSIST && weak && n->armed &&
           fabsf(heading-n->reference_heading)<=HUMP_HEADING_MAX) {
            n->active=true;n->armed=false;n->good_timing=false;n->reacquiring=false;
            n->carry_since=now;n->carry_path=path;
        }
    }
    if(n->active) {
        float error=heading-n->reference_heading;
        if(now-n->carry_since>=HUMP_CARRY_US) o.reason=HUMP_WHY_TIME;
        else if(path-n->carry_path>=HUMP_CARRY_MM) o.reason=HUMP_WHY_DISTANCE;
        else if(fabsf(error)>HUMP_HEADING_MAX) o.reason=HUMP_WHY_HEADING;
        if(o.reason) { d->stop=true;d->decision=AL_LOST;return o; }
        /* Require BOTH useful levels, not one flash of black. The original
           carry deadline remains active throughout this confirmation. */
        if(strong && abs(difference)<=350) {
            if(!n->reacquiring) { n->reacquiring=true;n->reacquire_since=now; }
            if(now-n->reacquire_since>=HUMP_REACQUIRE_US) {
                n->active=false;n->reacquiring=false;n->optical=(SymmetricTracker){0};
            }
        } else n->reacquiring=false;
        if(n->active) {
            /* Positive encoder heading means rightward yaw. Slow LEFT and
               speed RIGHT to correct it. Cruise40, bounded targets32..48. */
            int correction=(int)lroundf(clampf(error*2.0f,-8.0f,8.0f));
            d->target[0]=(int16_t)(40-correction);d->target[1]=(int16_t)(40+correction);
            d->decision=n->reacquiring?HUMP_REACQUIRE:HUMP_CARRY;
            o.assist_allowed=true;return o;
        }
    }
    *d=symmetric_demand(&n->optical,cal,left,right,now,(uint32_t)path);
    o.assist_allowed=!d->stop && d->decision!=SYM_RECOVER_LEFT && d->decision!=SYM_RECOVER_RIGHT &&
                     (d->level[0]>=400 || d->level[1]>=400);
    return o;
}
#endif
