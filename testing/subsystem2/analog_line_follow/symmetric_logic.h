#ifndef SYMMETRIC_LINE_LOGIC_H
#define SYMMETRIC_LINE_LOGIC_H
#include "line_logic.h"

/* For the NEW placement: physical left/right spots span the black strip.
 * Two-black is normal, never evidence of a crossing in this profile.
 * This cannot locate the exact centre inside a saturated two-black plateau. */
enum { SYM_STRAIGHT=11,SYM_LEFT,SYM_RIGHT,SYM_RECOVER_LEFT,SYM_RECOVER_RIGHT };
enum { SYM_RUN_US=10000000,SYM_RUN_MM=500,SYM_TRACE_US=125000,SYM_LOSS_US=650000,SYM_LOSS_MM=25,
       SYM_MEMORY_US=500000 };
typedef struct {
    uint64_t direction_us,white_since;
    uint32_t white_mm;
    int8_t direction,recovery;
    bool white;
} SymmetricTracker;
/* V11 adaptive forward speed. Level difference is reflectance error, not mm.
 * Piecewise interpolation keeps target changes <=1mm/s per error count:
 * |error|80:45/45,250:40/50,500:25/50,1000:0/45 (inner/outer).
 * Targets stay forward-only. Zero is coast/off through the unchanged wheel PI. */
static inline void symmetric_targets(int32_t error,int16_t target[2]) {
    if(error>1000) error=1000;
    if(error < -1000) error=-1000;
    int32_t magnitude=error<0?-error:error;
    int32_t inner=45,outer=45;
    if(magnitude>80 && magnitude<=250) {
        int32_t turn=((magnitude-80)*5+169)/170;
        inner=45-turn;outer=45+turn;
    } else if(magnitude>250 && magnitude<=500) {
        inner=40-(magnitude-250)*15/250;outer=50;
    } else if(magnitude>500) {
        inner=(1000-magnitude)*25/500;
        /* Quantize the outer target with the inner target, so rounding cannot
           weaken curvature on a step where only the outer speed decreases. */
        outer=45+inner/5;
    }
    target[0]=(int16_t)(error>=0?inner:outer);
    target[1]=(int16_t)(error>=0?outer:inner);
}
static inline AnalogDemand symmetric_demand(SymmetricTracker *t,const LineCalibration *c,
                                             uint16_t l,uint16_t r,uint64_t now,uint32_t mm) {
    AnalogDemand d={.decision=SYM_STRAIGHT};
    if(!calibration_valid(c)) { d.stop=true;d.decision=AL_LOST;return d; }
    d.level[0]=line_level(l,c->white[0],c->black[0]);
    d.level[1]=line_level(r,c->white[1],c->black[1]);
    if(d.level[0]<150 && d.level[1]<150) {
        if(!t->white) {
            t->white=true;t->white_since=now;t->white_mm=mm;
            t->recovery=t->direction && now-t->direction_us<=SYM_MEMORY_US?t->direction:0;
        }
        if(!t->recovery || now-t->white_since>=SYM_LOSS_US || mm-t->white_mm>=SYM_LOSS_MM) {
            d.stop=true;d.decision=AL_LOST;return d;
        }
        d.decision=t->recovery>0?SYM_RECOVER_LEFT:SYM_RECOVER_RIGHT;
        d.target[0]=t->recovery>0?0:45;d.target[1]=t->recovery>0?45:0;
        return d;
    }
    t->white=false;t->recovery=0;
    int32_t error=(int32_t)d.level[0]-d.level[1];
    /* Only a clear, recent asymmetric observation may choose loss recovery. */
    if((error>=200 || error<=-200) && (d.level[0]>=400 || d.level[1]>=400)) {
        t->direction=error>0?1:-1;t->direction_us=now;
    }
    symmetric_targets(error,d.target);
    if(d.target[0]<d.target[1]) d.decision=SYM_LEFT;
    else if(d.target[0]>d.target[1]) d.decision=SYM_RIGHT;
    return d;
}
#endif
