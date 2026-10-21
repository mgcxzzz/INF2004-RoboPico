#ifndef JUNCTION_LEFT_LOGIC_H
#define JUNCTION_LEFT_LOGIC_H
#include "../analog_line_follow/symmetric_logic.h"
#include "../dual_wheel_bench/dual_logic.h"

enum { J_APPROACH=1,J_CENTRE,J_SETTLE,J_TURN,J_TURN_SETTLE,J_EXIT,J_DONE,J_FAULT };
enum { J_OK=0,J_COMPLETE=20,J_NO_CROSS,J_BAD_STRIPE,J_CENTRE_DRIFT,J_SETTLE_TIMEOUT,
       J_TURN_TIMEOUT,J_BRANCH_MISSING,J_PATH_LIMIT,J_LINE_LOST };
enum { J_RUN_US=25000000,J_TRACE_US=400000,J_MAX_PATH_MM=900 };
typedef struct { uint8_t white,black,mask; } SideCalibration;
static inline bool side_valid(const SideCalibration *s) {
    return s->mask==3 && s->white<=1 && s->black<=1 && s->white!=s->black;
}
typedef struct {
    uint64_t now;
    float wheel[2],speed[2],path;
    uint16_t adc[2];
    bool side_black;
} JunctionInput;
typedef struct {
    uint8_t phase,reason;
    bool turn_enabled,side_last,side_armed,striped,off_line;
    bool still_timing,off_timing,line_timing;
    uint64_t phase_us,edge_us,still_us,off_us,line_us;
    float edge_mm,leading_mm,trailing_mm,centre_mm,approach_heading;
    float turn_origin[2],exit_origin,turn_degrees;
    SymmetricTracker follower;
} Junction;
static inline float j_mean(const JunctionInput *i) { return (i->wheel[0]+i->wheel[1])*0.5f; }
static inline void j_enter(Junction *j,unsigned phase,uint64_t now) {
    j->phase=(uint8_t)phase;j->phase_us=now;j->still_timing=false;
}
static inline void j_begin(Junction *j,bool turn,const JunctionInput *i) {
    *j=(Junction){.turn_enabled=turn,.edge_us=i->now,.edge_mm=j_mean(i)};
    j_enter(j,J_APPROACH,i->now);
}
static inline void j_fail(Junction *j,unsigned why) { j->phase=J_FAULT;j->reason=(uint8_t)why; }
static inline bool j_still(Junction *j,const JunctionInput *i) {
    bool still=fabsf(i->speed[0])<5 && fabsf(i->speed[1])<5;
    if(!still) { j->still_timing=false;return false; }
    if(!j->still_timing) { j->still_timing=true;j->still_us=i->now; }
    return i->now-j->phase_us>=300000 && i->now-j->still_us>=200000;
}
/* Cross detection is ONLY valid on a straight, with NO barcode/other side marks.
 * The side sensing point is approximately 92mm ahead of the wheel axle.
 * Record both cross-arm edges; their midpoint plus92mm is the axle target.
 * This is a provisional geometric value, not calibrated stopping compensation. */
static inline AnalogDemand junction_step(Junction *j,const LineCalibration *c,const JunctionInput *i) {
    AnalogDemand d={0};
    for(unsigned w=0;w<2;++w) d.level[w]=line_level(i->adc[w],c->white[w],c->black[w]);
    if(!calibration_valid(c)) j_fail(j,J_LINE_LOST);
    if(i->path>=J_MAX_PATH_MM) j_fail(j,J_PATH_LIMIT);
    float mean=j_mean(i);
    if(i->side_black!=j->side_last) {
        j->side_last=i->side_black;j->edge_us=i->now;j->edge_mm=mean;
    }
    switch(j->phase) {
    case J_APPROACH:
        d=symmetric_demand(&j->follower,c,i->adc[0],i->adc[1],i->now,(uint32_t)i->path);
        if(d.stop) { j_fail(j,J_LINE_LOST);break; }
        if(!i->side_black && i->now-j->edge_us>=100000) j->side_armed=true;
        if(i->side_black && i->now-j->edge_us>=20000) {
            if(!j->side_armed) { j_fail(j,J_NO_CROSS);break; }
            j->leading_mm=j->edge_mm;j->centre_mm=j->leading_mm+101.0f;
            j->approach_heading=heading_deg(i->wheel[0],i->wheel[1]);
            j_enter(j,J_CENTRE,i->now);
            d.target[0]=d.target[1]=30;d.stop=false;
        } else if(i->now-j->phase_us>=12000000 || mean>=450) j_fail(j,J_NO_CROSS);
        break;
    case J_CENTRE: {
        float drift=heading_deg(i->wheel[0],i->wheel[1])-j->approach_heading;
        if(fabsf(drift)>12) { j_fail(j,J_CENTRE_DRIFT);break; }
        if(!j->striped && !i->side_black && i->now-j->edge_us>=20000) {
            j->trailing_mm=j->edge_mm;
            float width=j->trailing_mm-j->leading_mm;
            if(width<6 || width>40) { j_fail(j,J_BAD_STRIPE);break; }
            j->striped=true;
            j->centre_mm=(j->leading_mm+j->trailing_mm)*0.5f+92.0f;
        }
        if((!j->striped && mean-j->leading_mm>50) || i->now-j->phase_us>=6000000) {
            j_fail(j,J_BAD_STRIPE);break;
        }
        float remaining=j->centre_mm-mean;
        if(j->striped && remaining<=1.0f) {
            j_enter(j,J_SETTLE,i->now);break;
        }
        float base=remaining<15?25.0f:30.0f;
        float correction=clampf(drift*1.5f,-6,6);
        d.target[0]=(int16_t)(base-correction);d.target[1]=(int16_t)(base+correction);
        break;
    }
    case J_SETTLE:
        if(j_still(j,i)) {
            if(!j->turn_enabled) { j->phase=J_DONE;j->reason=J_COMPLETE;break; }
            for(unsigned w=0;w<2;++w) j->turn_origin[w]=i->wheel[w];
            j->off_line=false;j->off_timing=j->line_timing=false;
            j_enter(j,J_TURN,i->now);
        } else if(i->now-j->phase_us>=2000000) j_fail(j,J_SETTLE_TIMEOUT);
        break;
    case J_TURN: {
        float left=-(i->wheel[0]-j->turn_origin[0]);
        float right=i->wheel[1]-j->turn_origin[1];
        j->turn_degrees=(left+right)/TRACK_MM*(180.0f/3.14159265358979323846f);
        if(left < -5 || right < -5 || fabsf(left-right)>20) { j_fail(j,J_CENTRE_DRIFT);break; }
        bool white=d.level[0]<200 && d.level[1]<200;
        if(white && j->turn_degrees>10) {
            if(!j->off_timing) { j->off_timing=true;j->off_us=i->now; }
            if(i->now-j->off_us>=30000) j->off_line=true;
        } else j->off_timing=false;
        bool found=j->off_line && j->turn_degrees>=75 && d.level[0]>=650 && d.level[1]>=650;
        if(found) {
            if(!j->line_timing) { j->line_timing=true;j->line_us=i->now; }
            if(i->now-j->line_us>=30000) { j_enter(j,J_TURN_SETTLE,i->now);break; }
        } else j->line_timing=false;
        if(j->turn_degrees>=110 || left>=120 || right>=120) { j_fail(j,J_BRANCH_MISSING);break; }
        if(i->now-j->phase_us>=6000000) { j_fail(j,J_TURN_TIMEOUT);break; }
        int16_t speed=j->turn_degrees>=65?25:35;
        /* Balance wheel arcs so this remains an approximate in-place turn. */
        float sync=clampf((left-right)*1.0f,-7,7);
        d.target[0]=-(int16_t)(speed-sync);d.target[1]=(int16_t)(speed+sync);
        break;
    }
    case J_TURN_SETTLE:
        if(j_still(j,i)) {
            if(d.level[0]<400 || d.level[1]<400) { j_fail(j,J_BRANCH_MISSING);break; }
            j->exit_origin=mean;j->follower=(SymmetricTracker){0};j_enter(j,J_EXIT,i->now);
        } else if(i->now-j->phase_us>=2000000) j_fail(j,J_SETTLE_TIMEOUT);
        break;
    case J_EXIT:
        if(mean-j->exit_origin>=20) { j->phase=J_DONE;j->reason=J_COMPLETE;break; }
        if(i->now-j->phase_us>=3000000) { j_fail(j,J_LINE_LOST);break; }
        d=symmetric_demand(&j->follower,c,i->adc[0],i->adc[1],i->now,(uint32_t)i->path);
        if(d.stop) j_fail(j,J_LINE_LOST);
        break;
    default:break;
    }
    if(j->phase==J_DONE || j->phase==J_FAULT) { d.stop=true;d.target[0]=d.target[1]=0; }
    return d;
}
#endif
