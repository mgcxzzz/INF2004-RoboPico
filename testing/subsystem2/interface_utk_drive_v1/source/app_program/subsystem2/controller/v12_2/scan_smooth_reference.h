#ifndef BARCODE_SCAN_SMOOTH_H
#define BARCODE_SCAN_SMOOTH_H
#include "baseline/testing/subsystem2/circuit_with_barcode/barcode_analog.h"
#include "baseline/testing/subsystem2/circuit_with_barcode/smooth_follow.h"
typedef struct {
    float mean,half;
    int16_t emitted[2];
    bool active,scanning;
    uint64_t activity_us;
    double start_mm;
} ScanSmooth;
/* Optical state, recovery, crossing and turns run on every real ADC reading.
 * This output adapter never captures a heading or freezes the follower. */
static inline AnalogDemand scan_smooth_step(Circuit *c,BarcodeNav *n,const LineCalibration *cal,
    const CircuitInput *i,unsigned profile,SmoothFollow *legacy,const BarcodeAnalog *a,ScanSmooth *s,float dt) {
    AnalogDemand d=barcode_circuit_step(c,n,cal,i,profile);
    dt=clampf(dt,0,.05f);
    bool activity=a->scanning || n->pending || a->adc>=(unsigned)a->baseline+16;
    if(activity)s->activity_us=i->now;
    bool scanning=!c->busy && !d.stop &&
        (activity || (s->scanning && i->now-s->activity_us<600000));
    if(scanning!=s->scanning) {
        double mm=(i->wheel[0]+i->wheel[1])*.5;
        if(scanning)s->start_mm=mm;
        barcode_event(n,scanning?B_SOFT_BEGIN:B_SOFT_END,i->now,mm,mm-s->start_mm,
            n->last_code,'-',c->busy?1:0,NULL);
    }
    s->scanning=scanning;
    /* The original optical tracker decides all loss/search/stop conditions.
     * No synthetic ADCs, extra centre calibration or record of approach yaw. */
    bool following=c->phase==C_FOLLOW && !c->busy && !d.stop && !c->follower.core.white;
    bool ordinary=d.decision==SYM_STRAIGHT || d.decision==SYM_LEFT ||
        d.decision==SYM_RIGHT || d.decision==C_FAST;
    unsigned strongest=d.level[0]>d.level[1]?d.level[0]:d.level[1];
    unsigned weakest=d.level[0]<d.level[1]?d.level[0]:d.level[1];
    unsigned magnitude=(unsigned)abs((int)d.level[0]-d.level[1]);
    *legacy=(SmoothFollow){0};
    if(!following || !ordinary || strongest<200) {s->active=false;return d;}
    /* Retain the V12 strong-bend bypass. Smoothing a clear one-sided departure
     * was a previous regression: it must never hold a crooked entry heading. */
    bool urgent=magnitude>=650 || ((d.target[0]<=10 || d.target[1]<=10) && strongest>=400);
    if(urgent) {
        s->mean=(d.target[0]+d.target[1])*.5f;
        s->half=(d.target[1]-d.target[0])*.5f;s->active=true;
        s->emitted[0]=d.target[0];s->emitted[1]=d.target[1];c->fast=false;return d;
    }
    float desired_mean=(d.target[0]+d.target[1])*.5f;
    float desired_half=(d.target[1]-d.target[0])*.5f;
    if(weakest>=200 && magnitude<350) {
        /* Soften only the small balance response, restoring full V12 demand
         * continuously by350. Unequal brightness still guides the robot. */
        desired_half*=.45f+.55f*magnitude/350.0f;
    }
    if(!scanning && profile==1 && weakest>=200 && magnitude<350)
        desired_mean+=15*clampf((350-(float)magnitude)/200,0,1);
    if(profile==2 && c->fast && !scanning)desired_mean=RAMP2_SPEED;
    if(scanning && desired_mean>45)desired_mean=45;
    if(!s->active) {
        /* Seed from measured forward motion; the unchanged wheel controller
         * additionally ramps its reference and owns breakaway/PWM. */
        float speed=fmaxf(0,(i->speed[0]+i->speed[1])*.5f);
        s->mean=fminf(desired_mean,speed);s->half=0;s->active=true;
        s->emitted[0]=s->emitted[1]=(int16_t)lroundf(s->mean);
    }
    float urgency=clampf(((float)magnitude-200)/450,0,1);
    float tau=(scanning?.180f:.120f)*(1-urgency)+.025f*urgency;
    float desired_change=(desired_half-s->half)*dt/(tau+dt);
    float steer_rate=(scanning?100:140)+460*urgency;
    s->half+=clampf(desired_change,-steer_rate*dt,steer_rate*dt);
    /* Filter common speed as well as differential steering: alternating ADC
     * balance used to modulate total forward speed even on a straight. */
    float mean_tau=.160f*(1-urgency)+.025f*urgency;
    float mean_change=(desired_mean-s->mean)*dt/(mean_tau+dt);
    s->mean+=clampf(mean_change,-240*dt,80*dt);
    /* Faster cruising must not leak through a departure or scan transition. */
    float ceiling=scanning?50:(magnitude>350 || weakest<200)?50:60;
    if(profile>1 && !scanning && c->fast)ceiling=desired_mean;
    s->mean=fminf(s->mean,ceiling-fabsf(s->half));
    for(unsigned w=0;w<2;++w) {
        float target=s->mean+(w?s->half:-s->half);
        target=fmaxf(0,target);
        /* A1mm/s Schmitt band prevents sub-count rounding chatter without
         * adding delay to the urgent/stop/search paths above. */
        float difference=target-s->emitted[w];
        if(difference>=1)s->emitted[w]+=(int16_t)floorf(difference);
        else if(difference<=-1)s->emitted[w]-=(int16_t)floorf(-difference);
        if(s->emitted[w]>ceiling)s->emitted[w]=(int16_t)ceiling;
        d.target[w]=s->emitted[w];
    }
    c->fast=!scanning && (d.target[0]>55 || d.target[1]>55);
    d.decision=scanning?24:25;
    return d;
}
#endif
