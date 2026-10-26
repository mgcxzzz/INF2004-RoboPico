#ifndef STRAIGHT_DRIVE_PROBE_LOGIC_H
#define STRAIGHT_DRIVE_PROBE_LOGIC_H
#include "../circuit_with_barcode/smooth_follow.h"
enum { PROBE_RUN_US=4000000,PROBE_SPEED=45,PROBE_DECISION=25 };
static inline uint64_t probe_deadline(uint64_t now) {return now+PROBE_RUN_US;}
/* The normal wheel controller remains in the adapter. Optical readings are
 * logged but cannot change targets, invoke search or dispatch a junction. */
static inline AnalogDemand probe_drive_step(Circuit *c,BarcodeNav *n,const LineCalibration *cal,
                                            const CircuitInput *i,unsigned profile,SmoothFollow *s,float dt) {
    (void)profile;(void)dt;
    *s=(SmoothFollow){0};c->phase=C_FOLLOW;c->busy=true;c->fast=false;
    c->follower=(CircuitTracker){0};n->allow_cross=false;n->pending=0;n->busy=false;
    AnalogDemand d={.decision=PROBE_DECISION,.target={PROBE_SPEED,PROBE_SPEED}};
    for(unsigned w=0;w<2;++w)d.level[w]=line_level(i->adc[w],cal->white[w],cal->black[w]);
    if(!calibration_valid(cal)) {circuit_fail(c,J_LINE_LOST);d.stop=true;d.target[0]=d.target[1]=0;}
    return d;
}
#endif
