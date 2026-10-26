#ifndef BARCODE_SCAN_HOLD_H
#define BARCODE_SCAN_HOLD_H
#include "baseline/testing/subsystem2/circuit_with_barcode/barcode_analog.h"
#include "baseline/testing/subsystem2/circuit_with_barcode/smooth_follow.h"

enum { SH_DECISION=24,SH_HANDOVER=25,SH_LIMIT_MM=180,SH_LIMIT_US=5000000,
       SH_WHITE_US=150000,SH_STEADY_US=300000,SH_HANDOVER_US=200000 };
enum { SH_DECODED=1,SH_QUIET,SH_DISTANCE,SH_TIME,SH_WHITE,SH_YAW,SH_DISPATCH,SH_FAULT };
typedef struct {
    bool initialized,black,candidate,active,confirmed,blocked,quiet_timing,steady_timing,white_timing;
    float background,trim,target[2];
    uint64_t edge_since,steady_since,since,white_since,handover_until;
    double quiet_mm,wheel[2],start_mm,steady_wheel[2];
    unsigned pulses,selections;
} ScanHold;
static inline bool scan_pattern(const BarcodeAnalog *a) {
    for(unsigned b=0;b+1<AO_BANKS;++b)
        if(a->bank[b].reader.frame && a->bank[b+1].reader.frame)return true;
    return false;
}
static inline void scan_hold_event(BarcodeNav *n,const ScanHold *h,const CircuitInput *i,
                                   unsigned type,unsigned reason,const uint16_t level[2]) {
    double error=(i->wheel[1]-h->wheel[1])-(i->wheel[0]-h->wheel[0]);
    double yaw=error*180/(110*3.14159265358979323846);
    uint16_t detail[3]={level[0],level[1],(uint16_t)clampf((float)(18000+yaw*100),0,36000)};
    barcode_event(n,type,i->now,(i->wheel[0]+i->wheel[1])*.5,
        (i->wheel[0]+i->wheel[1])*.5-h->start_mm,n->last_code,'S',reason,detail);
}
static inline void scan_hold_release(ScanHold *h,Circuit *c,BarcodeNav *n,SmoothFollow *s,
                                     const CircuitInput *i,unsigned reason,const uint16_t level[2]) {
    scan_hold_event(n,h,i,reason==SH_DECODED || reason==SH_QUIET?B_HOLD_END:B_HOLD_CANCEL,reason,level);
    h->active=false;h->blocked=true;h->quiet_timing=false;h->steady_timing=false;
    n->straight_override=false;
    /* No stale optical recovery clock/curvature survives the ignored interval.
     * The next normal step evaluates current, real front readings. */
    if(!c->busy) {c->follower=(CircuitTracker){0};c->cruise_timing=false;}
    *s=(SmoothFollow){0};
    h->handover_until=(reason==SH_DECODED || reason==SH_QUIET || reason==SH_DISTANCE || reason==SH_TIME)?
        i->now+SH_HANDOVER_US:0;
}
static inline AnalogDemand scan_drive_step(Circuit *c,BarcodeNav *n,const LineCalibration *cal,
    const CircuitInput *i,unsigned profile,SmoothFollow *s,const BarcodeAnalog *a,ScanHold *h,float dt) {
    double mm=(i->wheel[0]+i->wheel[1])*.5;
    uint16_t level[2]={line_level(i->adc[0],cal->white[0],cal->black[0]),
                       line_level(i->adc[1],cal->white[1],cal->black[1])};
    dt=clampf(dt,0,.05f);
    if(!h->initialized) {
        h->initialized=true;h->background=a->baseline;h->edge_since=i->now;
        h->candidate=h->black=a->adc>=h->background+16;
        h->target[0]=h->target[1]=45;
    }
    /* Start on broad white. This detector only asks for a possible scan; it
     * never decodes, alters the threshold bank, or creates a route command. */
    bool black=a->adc>=h->background+(h->black?8:16),rising=false;
    if(black!=h->candidate) {h->candidate=black;h->edge_since=i->now;}
    if(h->candidate!=h->black && i->now-h->edge_since>=6000) {
        h->black=h->candidate;rising=h->black;
        if(h->active && rising)++h->pulses;
    }
    if(!h->black && !h->candidate) {
        if(!h->quiet_timing) {h->quiet_timing=true;h->quiet_mm=mm;}
        if(!h->active && fabsf(a->adc-h->background)<=8)
            h->background+=.02f*(a->adc-h->background);
        if(mm-h->quiet_mm>=12)h->blocked=false;
    } else if(!h->active)h->quiet_timing=false;
    bool visible=level[0]>=400 || level[1]>=400;
    bool steady=!c->busy && !c->fault && !c->follower.core.white && visible &&
        i->speed[0]>=15 && i->speed[1]>=15 && fabsf(i->speed[0]-i->speed[1])<=15 &&
        i->path>=c->clear_path;
    if(!steady)h->steady_timing=false;
    else {
        if(!h->steady_timing) {h->steady_timing=true;h->steady_since=i->now;
            memcpy(h->steady_wheel,i->wheel,sizeof(h->steady_wheel));}
        double differential=(i->wheel[1]-h->steady_wheel[1])-(i->wheel[0]-h->steady_wheel[0]);
        if(fabs(differential)>110*3.14159265358979323846*3/180) {
            h->steady_since=i->now;memcpy(h->steady_wheel,i->wheel,sizeof(h->steady_wheel));
        }
    }
    bool begin=rising && !h->active && !h->blocked && h->steady_timing &&
        i->now-h->steady_since>=SH_STEADY_US && calibration_valid(cal);
    if(begin) {
        h->active=true;h->confirmed=h->white_timing=false;h->since=i->now;
        h->start_mm=mm;memcpy(h->wheel,i->wheel,sizeof(h->wheel));
        h->selections=a->selections;h->pulses=1;h->trim=0;h->quiet_timing=false;
        /* Seed entry from the V12 requested targets, not the PWM outputs. */
        h->target[0]=s->active?s->target[0]:45;h->target[1]=s->active?s->target[1]:45;
        scan_hold_event(n,h,i,B_HOLD_BEGIN,0,level);
    }
    if(h->active) {
        if(!h->black && !h->candidate) {
            if(!h->quiet_timing) {h->quiet_timing=true;h->quiet_mm=mm;}
        } else h->quiet_timing=false;
        if(!h->confirmed && (h->pulses>=2 || scan_pattern(a))) {
            h->confirmed=true;scan_hold_event(n,h,i,B_HOLD_PATTERN,0,level);
        }
        bool white=level[0]<150 && level[1]<150;
        if(!white)h->white_timing=false;
        else if(!h->white_timing) {h->white_timing=true;h->white_since=i->now;}
        double differential=(i->wheel[1]-h->wheel[1])-(i->wheel[0]-h->wheel[0]);
        unsigned reason=c->busy?SH_DISPATCH:c->fault || a->conflict?SH_FAULT:
            a->selections>h->selections?SH_DECODED:
            h->white_timing && i->now-h->white_since>=SH_WHITE_US?SH_WHITE:
            fabs(differential)>110*3.14159265358979323846*8/180?SH_YAW:
            mm-h->start_mm>=SH_LIMIT_MM?SH_DISTANCE:i->now-h->since>=SH_LIMIT_US?SH_TIME:
            h->quiet_timing && mm-h->quiet_mm>=12?SH_QUIET:0;
        if(reason)scan_hold_release(h,c,n,s,i,reason,level);
    }
    n->straight_override=h->active && !c->busy;
    AnalogDemand d=barcode_drive_step(c,n,cal,i,profile,s,dt);
    /* Existing junction geometry/turn/acquisition always wins on this tick. */
    if(h->active && (c->busy || d.stop))scan_hold_release(h,c,n,s,i,c->busy?SH_DISPATCH:SH_FAULT,level);
    if(h->active) {
        double error=(i->wheel[1]-h->wheel[1])-(i->wheel[0]-h->wheel[0]);
        float wanted=clampf((float)(1.2*error)+.08f*(i->speed[1]-i->speed[0]),-8,8);
        h->trim+=clampf(wanted-h->trim,-40*dt,40*dt);
        float target[2]={45+h->trim,45-h->trim};
        for(unsigned w=0;w<2;++w) {h->target[w]+=clampf(target[w]-h->target[w],-200*dt,200*dt);
            d.target[w]=(int16_t)lroundf(h->target[w]);}
        d.decision=SH_DECISION;*s=(SmoothFollow){0};c->fast=false;
    } else if(h->handover_until>i->now && !c->busy && !d.stop && !c->follower.core.white) {
        for(unsigned w=0;w<2;++w) {h->target[w]+=clampf(d.target[w]-h->target[w],-200*dt,200*dt);
            d.target[w]=(int16_t)lroundf(h->target[w]);}
        d.decision=SH_HANDOVER;
    } else if(!h->active)h->handover_until=0;
    return d;
}
#endif
