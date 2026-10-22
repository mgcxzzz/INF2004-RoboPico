#ifndef CIRCUIT_LOGIC_H
#define CIRCUIT_LOGIC_H
#include <stdlib.h>
#include "../junction_left_test/junction_logic.h"

enum { C_FOLLOW=10,C_APPROACH_FAST=40,C_BAD_PROFILE=41,C_TRACE_US=250000,
       C_GATE_US=300000,C_WHITE_US=100000,C_CLEAR_MM=150 };
typedef struct {
    uint64_t now;
    double wheel[2],path;
    float speed[2];uint16_t adc[2];bool side_black;
} CircuitInput;
typedef struct {
    Junction turn;SymmetricTracker follower;
    bool busy,fault,completed,gate_timing,white_timing,side_last,candidate;
    uint8_t reason,phase;
    uint64_t gate_since,white_since,edge_us,leg_since;
    double gate_wheel[2],edge_wheel[2],edge_path,clear_path,follow_path;
    uint32_t turns,ignored,follow_ms;
    uint16_t centre_ms,turn_ms,align_ms;
} Circuit;
static inline int circuit_cruise(unsigned profile) { return profile>=1 && profile<=3?(int)profile*45:0; }
static inline AnalogDemand circuit_follow(Circuit *c,const LineCalibration *cal,const CircuitInput *i,unsigned profile) {
    AnalogDemand d=symmetric_demand(&c->follower,cal,i->adc[0],i->adc[1],i->now,(uint32_t)(i->path-c->follow_path));
    /* Blind recovery retains the tested0/45 policy at every profile. */
    if(d.decision>=SYM_STRAIGHT && d.decision<=SYM_RIGHT)
        for(unsigned w=0;w<2;++w) d.target[w]=(int16_t)(d.target[w]*(int)profile);
    return d;
}
static inline void circuit_begin(Circuit *c,const CircuitInput *i) {
    *c=(Circuit){.phase=C_FOLLOW,.side_last=i->side_black,.leg_since=i->now,.follow_path=i->path};
}
static inline void circuit_fail(Circuit *c,unsigned reason) { c->fault=true;c->phase=J_FAULT;c->reason=(uint8_t)reason; }
static inline AnalogDemand circuit_step(Circuit *c,const LineCalibration *cal,const CircuitInput *i,unsigned profile) {
    AnalogDemand d={0};c->completed=false;
    if(!circuit_cruise(profile)) circuit_fail(c,C_BAD_PROFILE);
    if(!calibration_valid(cal)) circuit_fail(c,J_LINE_LOST);
    if(c->fault) { d.stop=true;return d; }
    if(!c->busy) {
        d=circuit_follow(c,cal,i,profile);
        if(d.stop) { circuit_fail(c,J_LINE_LOST);return d; }
        bool visible=d.level[0]>=400 || d.level[1]>=400;
        bool straight=visible && i->speed[0]>=10 && i->speed[1]>=10;
        if(!straight) c->gate_timing=false;
        else {
            if(!c->gate_timing) {
                c->gate_timing=true;c->gate_since=i->now;
                c->gate_wheel[0]=i->wheel[0];c->gate_wheel[1]=i->wheel[1];
            }
            float yaw=heading_deg((float)(i->wheel[0]-c->gate_wheel[0]),(float)(i->wheel[1]-c->gate_wheel[1]));
            if(fabsf(yaw)>3) {
                c->gate_since=i->now;c->gate_wheel[0]=i->wheel[0];c->gate_wheel[1]=i->wheel[1];
            }
        }
        if(!i->side_black) {
            if(!c->white_timing) { c->white_timing=true;c->white_since=i->now; }
            c->candidate=false;
        }
        if(i->side_black && !c->side_last) {
            c->candidate=c->white_timing && i->now-c->white_since>=C_WHITE_US &&
                c->gate_timing && i->now-c->gate_since>=C_GATE_US && i->path>=c->clear_path;
            c->white_timing=false;c->edge_us=i->now;c->edge_path=i->path;
            c->edge_wheel[0]=i->wheel[0];c->edge_wheel[1]=i->wheel[1];
            if(!c->candidate) ++c->ignored;
        }
        c->side_last=i->side_black;
        if(c->candidate) {
            d.target[0]=d.target[1]=(int16_t)(30+(profile-1)*5); /* slow on first qualified edge */
            if(i->side_black && i->now-c->edge_us>=20000) {
                JunctionInput local={.now=i->now,.side_black=true};
                j_begin_mode(&c->turn,3,&local);
                c->turn.phase=J_CENTRE;c->turn.phase_us=i->now;
                c->turn.side_last=true;c->turn.side_armed=true;
                c->turn.edge_us=c->edge_us;c->turn.leading_mm=0;c->turn.edge_mm=0;
                c->turn.centre_mm=101.0f-j_turn_start_offset(&c->turn);
                c->turn.approach_heading=heading_deg((float)(i->wheel[0]-c->edge_wheel[0]),(float)(i->wheel[1]-c->edge_wheel[1]));
                c->follow_ms=(uint32_t)((c->edge_us-c->leg_since)/1000);
                c->busy=true;c->candidate=false;c->centre_ms=c->turn_ms=c->align_ms=0;
            }
        }
        if(!c->busy) return d;
    }
    JunctionInput local={.now=i->now,.path=(float)(i->path-c->edge_path),
        .wheel={(float)(i->wheel[0]-c->edge_wheel[0]),(float)(i->wheel[1]-c->edge_wheel[1])},
        .speed={i->speed[0],i->speed[1]},.adc={i->adc[0],i->adc[1]},.side_black=i->side_black};
    unsigned before=c->turn.phase;uint64_t phase_start=c->turn.phase_us;
    d=junction_step(&c->turn,cal,&local);c->phase=c->turn.phase;
    if(before==J_CENTRE && c->phase==J_TURN) {
        c->centre_ms=(uint16_t)((i->now-phase_start)/1000);
        if(fabsf(i->speed[0])>65 || fabsf(i->speed[1])>65) circuit_fail(c,C_APPROACH_FAST);
    }
    if(before==J_TURN && c->phase==J_ACQUIRE) c->turn_ms=(uint16_t)((i->now-phase_start)/1000);
    if(before==J_ACQUIRE && c->phase==J_EXIT) {
        c->align_ms=(uint16_t)((i->now-phase_start)/1000);++c->turns;c->completed=true;
        c->busy=false;c->phase=C_FOLLOW;c->gate_timing=c->white_timing=false;c->candidate=false;
        c->side_last=i->side_black;c->clear_path=i->path+C_CLEAR_MM;c->leg_since=i->now;
        c->follower=(SymmetricTracker){0};c->follow_path=i->path;
        d=circuit_follow(c,cal,i,profile); /* resume, never the old ten-second finish */
    } else if(c->phase==J_CENTRE) {
        for(unsigned w=0;w<2;++w) if(d.target[w]>0) d.target[w]+=(int16_t)((profile-1)*5);
    } else if(c->phase==J_TURN) {
        if(d.target[1]>0) d.target[1]=(int16_t)(d.target[1]+(profile-1)*(d.target[1]>=45?10:5));
    } else if(c->phase==J_ACQUIRE) {
        for(unsigned w=0;w<2;++w) if(d.target[w]>0) d.target[w]+=(int16_t)((profile-1)*5);
    }
    if(d.stop || c->turn.phase==J_FAULT) circuit_fail(c,c->reason?c->reason:c->turn.reason);
    if(c->fault) { d.stop=true;d.target[0]=d.target[1]=0; }
    return d;
}
#endif
