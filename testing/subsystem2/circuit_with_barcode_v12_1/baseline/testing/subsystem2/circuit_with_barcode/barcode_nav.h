#ifndef BARCODE_NAV_H
#define BARCODE_NAV_H
#include "../circuit_left_test/circuit_logic.h"
#include "barcode_scan.h"
#include "uturn_logic.h"

enum { B_CROSS_MIN_MM=11,B_CROSS_MAX_MM=30,B_CROSS_WHITE_MM=4,B_SIDE_AHEAD_MM=92,B_EDGE_FAILURE=46 };
typedef struct {
    BarcodeReader reader;BarcodeEvent events[B_EVENTS];
    uint32_t event_count,event_head,event_overwritten,event_sequence;
    uint32_t decoded_a,decoded_f,unsupported,rejected,lefts,rights,straight_crosses,decoded_z,uturns;
    BarcodeUTurn uturn;
    uint64_t start_us,leading_us,cross_us;
    double leading_wheel[2],cross_wheel[2],cross_width,cross_trailing_mm;
    bool leading_gate,cross_gate,cross_ready,cross_candidate,busy,allow_cross;
    bool external_reader,external_scanning;
    bool straight_override; /* V12.1: RAM-only scan override, never a route command. */
    char pending,action,last_code;
} BarcodeNav;
static inline void barcode_event(BarcodeNav *n,unsigned type,uint64_t now,double mm,double width,
                                 char code,char action,unsigned flags,const uint16_t *pattern) {
    BarcodeEvent *e=&n->events[n->event_head];memset(e,0,sizeof(*e));
    e->ms=(now-n->start_us)/1000;e->path10=mm>0?(uint32_t)(mm*10):0;
    e->width100=(int32_t)(width*100);e->number=++n->event_sequence;
    e->type=(uint8_t)type;e->code=(uint8_t)code;e->action=(uint8_t)action;e->flags=(uint8_t)flags;
    if(pattern) memcpy(e->pattern,pattern,sizeof(e->pattern));
    n->event_head=(n->event_head+1)%B_EVENTS;
    if(n->event_count<B_EVENTS) ++n->event_count;else ++n->event_overwritten;
}
static inline void barcode_nav_begin(BarcodeNav *n,const CircuitInput *i) {
    *n=(BarcodeNav){.start_us=i->now};
    barcode_reader_reset(&n->reader,i->side_black,i->now,(i->wheel[0]+i->wheel[1])*.5);
}
static inline void barcode_recovery_events(BarcodeNav *n,uint64_t now,double position,
                                           bool was_white,bool was_waiting,bool was_search,unsigned previous_decision,
                                           const CircuitTracker *t,const AnalogDemand *d,unsigned reason) {
#ifdef CIRCUIT_SWEEP_SEARCH
    if(!was_waiting && !was_search && t->search.waiting)
        barcode_event(n,B_RECOVERY_WAIT,now,position,LS_CONFIRM_US/1000,0,'-',0,NULL);
    if(t->search.active && (!was_search || d->decision!=previous_decision) &&
       d->decision>=LS_DECISION_FIRST && d->decision<LS_DECISION_FIRST+5) {
        int speed=d->target[0]>d->target[1]?d->target[0]:d->target[1];
        barcode_event(n,B_RECOVERY_SWEEP,now,position,speed,0,d->target[1]>d->target[0]?'L':'R',
                      d->decision-LS_DECISION_FIRST+1,NULL);
    }
    if(!d->stop && was_white && !t->core.white)
        barcode_event(n,B_RECOVERY_FOUND,now,position,0,0,'-',was_search?1:0,NULL);
    if(d->stop && was_search)barcode_event(n,B_RECOVERY_FAIL,now,position,reason,0,'-',0,NULL);
#else
    (void)n;(void)now;(void)position;(void)was_white;(void)was_waiting;(void)was_search;
    (void)previous_decision;(void)t;(void)d;(void)reason;
#endif
}
static inline char barcode_action(char code) {
    return code=='A'?'L':code=='F'?'R':code=='Z'?'U':'-';
}
static inline void barcode_queue_code(BarcodeNav *n,char code) {
    n->last_code=code;
    if(code=='A') {++n->decoded_a;n->pending=code;}
    else if(code=='F') {++n->decoded_f;n->pending=code;}
    else if(code=='Z') {++n->decoded_z;n->pending=code;}
    else ++n->unsupported;
}
static inline void barcode_cross_confirm(BarcodeNav *n,uint64_t now,double mm,bool white) {
    if(n->cross_candidate && white && mm-n->cross_trailing_mm>=B_CROSS_WHITE_MM) {
        n->cross_candidate=false;n->cross_ready=true;
        barcode_event(n,B_CROSS,now,mm,n->cross_width,n->pending,barcode_action(n->pending),0,NULL);
    }
}
static inline void barcode_nav_edge(BarcodeNav *n,uint64_t now,const double wheel[2],bool black) {
    double mm=(wheel[0]+wheel[1])*.5;
    if(n->busy) { barcode_reader_reset(&n->reader,black,now,mm);return; }
    bool was_frame=n->reader.frame;
    if(black && !n->reader.black) {
        /* Confirm by encoder travel, even if this next edge arrives between
           control ticks. A new bar inside4mm cancels, never consumes pending. */
        barcode_cross_confirm(n,now,mm,true);
        if(n->cross_candidate) {
            n->cross_candidate=false;
            barcode_event(n,B_CROSS_CANCEL,now,mm,mm-n->cross_trailing_mm,n->pending,0,0,NULL);
        }
        memcpy(n->leading_wheel,wheel,sizeof(n->leading_wheel));n->leading_us=now;n->leading_gate=n->allow_cross;
    }
    BarcodeRead r=barcode_reader_edge(&n->reader,black,now,mm);
    if(r.events&(1u<<B_RUN)) barcode_event(n,B_RUN,now,mm,r.width,0,0,r.completed_black,NULL);
    if(r.events&(1u<<B_FRAME_START)) barcode_event(n,B_FRAME_START,now,mm,mm-r.start,0,0,n->reader.reverse,NULL);
    if(r.events&(1u<<B_BAD_FRAME)) { ++n->rejected;barcode_event(n,B_BAD_FRAME,now,mm,mm-r.start,'?',0,r.rejection,r.pattern); }
    if(r.events&(1u<<B_DECODE)) {
        barcode_queue_code(n,r.code);
        barcode_event(n,B_DECODE,now,mm,mm-r.start,r.code,barcode_action(r.code),r.reverse,r.pattern);
    }
    /* A complete barcode is required before queuing a route command. Frame
       context protects its bars, even when they are wider than the cross.
       Cross uses a separate isolated black run after its trailing edge. */
    if(r.completed_black && !was_frame && !(r.events&(1u<<B_FRAME_START)) &&
       n->reader.history_count<=2 && r.width>=B_CROSS_MIN_MM && r.width<=B_CROSS_MAX_MM) {
        n->cross_width=r.width;n->cross_us=n->leading_us;
        memcpy(n->cross_wheel,n->leading_wheel,sizeof(n->cross_wheel));
        n->cross_gate=n->leading_gate;n->cross_candidate=true;n->cross_trailing_mm=mm;
        barcode_event(n,B_CROSS_CANDIDATE,now,mm,r.width,n->pending,barcode_action(n->pending),0,NULL);
    }
}
static inline void barcode_mirror_tracker(CircuitTracker *t) {
    t->core.direction=-t->core.direction;t->core.recovery=-t->core.recovery;
    t->pending=-t->pending;t->reverse=-t->reverse;t->probe_direction=-t->probe_direction;
    int16_t v=t->bridge[0];t->bridge[0]=t->bridge[1];t->bridge[1]=v;
#ifdef CIRCUIT_SWEEP_SEARCH
    t->search.first=-t->search.first;
#endif
}
static inline void barcode_straight_gate(Circuit *c,BarcodeNav *n,const CircuitInput *i,const AnalogDemand *d) {
    bool visible=d->level[0]>=400 || d->level[1]>=400;
#ifdef CIRCUIT_SWEEP_SEARCH
    visible=visible && !c->follower.core.white;
#endif
    bool straight=visible && i->speed[0]>=10 && i->speed[1]>=10;
    if(!straight) c->gate_timing=false;
    else {
        if(!c->gate_timing) { c->gate_timing=true;c->gate_since=i->now;memcpy(c->gate_wheel,i->wheel,sizeof(c->gate_wheel)); }
        float yaw=heading_deg((float)(i->wheel[0]-c->gate_wheel[0]),(float)(i->wheel[1]-c->gate_wheel[1]));
        if(fabsf(yaw)>3) { c->gate_since=i->now;memcpy(c->gate_wheel,i->wheel,sizeof(c->gate_wheel)); }
    }
    n->allow_cross=c->gate_timing && i->now-c->gate_since>=C_GATE_US && i->path>=c->clear_path;
}
/* Right turns reuse the proven rolling-left geometry in a virtual mirror:
   swap front calibration, ADCs, wheel distances/speeds, then swap outputs.
   Real-world following/recovery state is restored after acquisition. */
static inline AnalogDemand barcode_circuit_step(Circuit *c,BarcodeNav *n,const LineCalibration *cal,
                                                const CircuitInput *i,unsigned profile) {
    AnalogDemand d={0};c->completed=false;c->fast=false;
    if(!circuit_cruise(profile)) circuit_fail(c,C_BAD_PROFILE);
    if(!calibration_valid(cal)) circuit_fail(c,J_LINE_LOST);
    if(c->fault) { d.stop=true;return d; }
    double mean=(i->wheel[0]+i->wheel[1])*.5;
    if(!c->busy) {
        barcode_cross_confirm(n,i->now,mean,!i->side_black);
        if(!n->external_reader && barcode_reader_expire(&n->reader,i->now,mean)) {
            ++n->rejected;barcode_event(n,B_BAD_FRAME,i->now,mean,mean-n->reader.start_mm,'?',0,B_REJECT_TIMEOUT,NULL);
        }
        /* Slow scanning and the approach with a pending command; encoding
           remains distance-based even as measured speed changes. */
        bool scan=n->external_reader?(n->external_scanning || n->pending):
            (n->reader.frame || n->reader.history_count || n->reader.black || n->pending);
        if(n->straight_override) {
            /* Keep actual optical levels for logging and cross qualification.
             * Freeze the optical follower rather than accumulating hidden loss
             * timers or steering memory while encoder guidance owns targets. */
            c->phase=C_FOLLOW;c->fast=c->cruise_timing=false;
            for(unsigned w=0;w<2;++w)d.level[w]=line_level(i->adc[w],cal->white[w],cal->black[w]);
            d.target[0]=d.target[1]=45;d.decision=24;
        } else d=circuit_follow(c,cal,i,scan?1:profile);
        if(d.stop) { circuit_fail(c,c->follower.loss_reason?c->follower.loss_reason:J_LINE_LOST);return d; }
        barcode_straight_gate(c,n,i,&d);
        if(n->cross_ready) {
            n->cross_ready=false;
            if(!n->pending) {
                ++n->straight_crosses;
                barcode_event(n,B_NO_COMMAND,i->now,mean,n->cross_width,0,'S',0,NULL);
            } else if(!n->cross_gate || i->path<c->clear_path
#ifdef CIRCUIT_SWEEP_SEARCH
                      || c->follower.core.white
#endif
                     ) {
                ++c->ignored;barcode_event(n,B_GATE_REJECT,i->now,mean,n->cross_width,n->pending,0,0,NULL);
            } else {
                n->action=n->pending;n->pending=0;n->busy=c->busy=true;
                n->uturn=(BarcodeUTurn){0};
                c->cruise_timing=c->fast=false;c->edge_us=n->cross_us;
                memcpy(c->edge_wheel,n->cross_wheel,sizeof(c->edge_wheel));
                c->edge_path=i->path-(mean-(n->cross_wheel[0]+n->cross_wheel[1])*.5);
                JunctionInput begin={.now=i->now,.side_black=false};j_begin_mode(&c->turn,3,&begin);
                c->turn.phase=J_CENTRE;c->turn.striped=true;c->turn.side_armed=true;
                c->turn.leading_mm=0;c->turn.trailing_mm=(float)n->cross_width;
                c->turn.centre_mm=(float)n->cross_width*.5f+B_SIDE_AHEAD_MM-j_turn_start_offset(&c->turn);
                c->turn.approach_heading=0;
                c->follow_ms=(uint32_t)((c->edge_us-c->leg_since)/1000);
                c->centre_ms=c->turn_ms=c->align_ms=0;
                barcode_event(n,B_DISPATCH,i->now,mean,n->cross_width,n->action,barcode_action(n->action),0,NULL);
                barcode_reader_reset(&n->reader,i->side_black,i->now,mean);
            }
        }
        if(!c->busy) return d;
    }
    bool right=n->action=='F';unsigned a=right?1:0,b=1-a;
    LineCalibration virtual_cal={{cal->white[a],cal->white[b]},{cal->black[a],cal->black[b]},cal->mask};
    JunctionInput local={.now=i->now,.path=(float)(i->path-c->edge_path),
        .wheel={(float)(i->wheel[a]-c->edge_wheel[a]),(float)(i->wheel[b]-c->edge_wheel[b])},
        .speed={i->speed[a],i->speed[b]},.adc={i->adc[a],i->adc[b]},.side_black=i->side_black};
    unsigned before=c->turn.phase;uint64_t phase_start=c->turn.phase_us;c->cruise_timing=false;
    if(before==J_ACQUIRE) d=circuit_join(c,&virtual_cal,&local);
    else {
        unsigned stage=n->uturn.stage;
        d=n->action=='Z'?barcode_uturn_step(&n->uturn,&c->turn,&virtual_cal,&local):
            junction_step(&c->turn,&virtual_cal,&local);
        if(n->action=='Z' && stage!=n->uturn.stage)
            barcode_event(n,B_UTURN_STAGE,i->now,mean,c->turn.turn_degrees,'Z','U',n->uturn.stage,NULL);
        if((before==J_TURN || (n->action=='Z' && before==J_TURN_SETTLE)) && c->turn.phase==J_ACQUIRE) {
            c->join_timing=false;c->joining=(CircuitTracker){.core=c->turn.follower};
            d=circuit_join(c,&virtual_cal,&local);
        }
    }
    c->phase=c->turn.phase;
    if(before==J_CENTRE && c->phase==J_TURN) {
        c->centre_ms=(uint16_t)((i->now-phase_start)/1000);
        if(fabsf(i->speed[0])>65 || fabsf(i->speed[1])>65) circuit_fail(c,C_APPROACH_FAST);
    }
    if((before==J_TURN || (n->action=='Z' && before==J_TURN_SETTLE)) && c->phase==J_ACQUIRE)
        c->turn_ms=(uint16_t)((i->now-(n->action=='Z'?n->uturn.since:phase_start))/1000);
    if(before==J_ACQUIRE && c->phase==J_EXIT) {
        c->align_ms=(uint16_t)((i->now-phase_start)/1000);++c->turns;c->completed=true;
        if(n->action=='Z') ++n->uturns;else if(right) ++n->rights;else ++n->lefts;
        c->busy=n->busy=false;c->phase=C_FOLLOW;c->gate_timing=c->white_timing=false;
        n->allow_cross=false;c->clear_path=i->path+C_CLEAR_MM;c->leg_since=i->now;
        c->follower=c->joining;if(right) barcode_mirror_tracker(&c->follower);
        c->follow_path=i->path;
        barcode_event(n,B_COMPLETE,i->now,mean,c->turn.turn_degrees,n->action,barcode_action(n->action),0,NULL);
        barcode_reader_reset(&n->reader,i->side_black,i->now,mean);
        return circuit_follow(c,cal,i,profile);
    }
    if(c->phase==J_CENTRE) {
        for(unsigned w=0;w<2;++w) if(d.target[w]>0) d.target[w]+=(int16_t)((profile-1)*5);
    } else if(n->action!='Z' && c->phase==J_TURN && d.target[1]>0)
        d.target[1]=(int16_t)(d.target[1]+(profile-1)*(d.target[1]>=45?10:5));
    if(right) {
        int16_t speed=d.target[0];d.target[0]=d.target[1];d.target[1]=speed;
        uint16_t level=d.level[0];d.level[0]=d.level[1];d.level[1]=level;
        if(d.decision==SYM_LEFT) d.decision=SYM_RIGHT;else if(d.decision==SYM_RIGHT) d.decision=SYM_LEFT;
        else if(d.decision==SYM_RECOVER_LEFT) d.decision=SYM_RECOVER_RIGHT;
        else if(d.decision==SYM_RECOVER_RIGHT) d.decision=SYM_RECOVER_LEFT;
        else if(d.decision==C_PROBE_LEFT) d.decision=C_PROBE_RIGHT;
        else if(d.decision==C_PROBE_RIGHT) d.decision=C_PROBE_LEFT;
    }
    if(d.stop || c->turn.phase==J_FAULT) circuit_fail(c,c->reason?c->reason:c->turn.reason);
    if(c->fault) { d.stop=true;d.target[0]=d.target[1]=0; }
    return d;
}
#endif
