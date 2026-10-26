#ifndef BUDDY_NORMAL_DRIVE_SMOOTH_H
#define BUDDY_NORMAL_DRIVE_SMOOTH_H
#include "v12_2/baseline/testing/subsystem2/circuit_left_test/circuit_wheel.h"
/* V1.2 ordinary profile1/3 forward drive only. The optical targets remain V12.2.
 * Keep its reference ramp/PI but replace relay-like breakaway kicks with
 * bounded slew and slowly retained load effort. Caller bypasses for all
 * planned turns, search, ramp, loaded capture and external API manoeuvres. */
typedef struct {
    float duty,extra;
    unsigned deficit_us:16,previous_target:8,active:1;
} DriveSmooth;
enum { DRIVE_PWM_UP_PER_S=900,DRIVE_PWM_DOWN_PER_S=600,
       DRIVE_EXTRA_LIMIT=90,DRIVE_DEFICIT_US=60000 };
static inline int32_t drive_smooth_step(DriveSmooth *s,SignedWheel *p,unsigned w,
    int16_t wanted,float speed,float dt,uint64_t now,bool turning,unsigned profile,
    bool enabled,int32_t previous_output) {
    int32_t legacy=circuit_wheel_step(p,w,wanted,speed,dt,now,turning,profile);
    if(!enabled || turning || (profile!=1 && profile!=3) || wanted<0 || wanted>180) {*s=(DriveSmooth){0};return legacy;}
    if(!wanted) {*s=(DriveSmooth){0};return 0;}
    dt=clampf(dt,0,.05f);
    int cap=(int)circuit_forward_cap(profile);
    if(!s->active) {*s=(DriveSmooth){.active=true,.duty=clampf(previous_output,0,(float)cap),.previous_target=(unsigned)wanted};}
    /* The legacy helper still maintains its diagnostic state. Do not apply
     * its instantaneous250*1.20 PWM pulse to ordinary forward movement. */
    int32_t base=legacy;
    if(p->kick.active) {
        float error=p->pi.ramp-speed;
        float raw=clampf(feedforward(w,1,p->pi.ramp)+LINE_SPEED_KP*error+p->pi.integral,0,profile==3?350:PWM_LIMIT);
        base=effort_scale20((int32_t)(raw+.5f));
    }
    bool sharp_cut=wanted+8<=(int)s->previous_target;
    if(legacy==0 || wanted<=15 || sharp_cut) {
        /* Genuine off/coast/strong steering reductions bypass output slew.
         * Never carry retained load support through an intentionally slow wheel. */
        s->extra=0;s->deficit_us=0;
        int32_t out=base;
        if(sharp_cut && out>s->duty)out=(int32_t)s->duty;
        s->duty=(float)out;s->previous_target=(unsigned)wanted;return out;
    }
    float error=p->pi.ramp-speed;
    bool deficit=wanted>=20 && p->pi.ramp>=20 && speed>=-8 &&
        error>fmaxf(8,.20f*p->pi.ramp) && wanted>=(int)s->previous_target-2;
    if(deficit) {
        unsigned us=s->deficit_us+(unsigned)(dt*1000000+.5f);
        s->deficit_us=us>DRIVE_DEFICIT_US?DRIVE_DEFICIT_US:us;
        if(s->deficit_us>=DRIVE_DEFICIT_US)s->extra+=fminf(180,4*error)*dt;
    } else {
        s->deficit_us=0;
        /* Preserve useful load effort while under target; unwind faster for
         * actual overspeed. This is extra effort, not a forced PWM minimum. */
        float release=error < -4?300:error<=3?10:0;
        s->extra-=release*dt;
    }
    if(wanted<(int)s->previous_target && s->previous_target)
        s->extra*=wanted/(float)s->previous_target;
    s->extra=clampf(s->extra,0,DRIVE_EXTRA_LIMIT);
    float desired=clampf(base+s->extra,0,(float)cap);
    s->duty+=clampf(desired-s->duty,-DRIVE_PWM_DOWN_PER_S*dt,DRIVE_PWM_UP_PER_S*dt);
    s->duty=clampf(s->duty,0,(float)cap);s->previous_target=(unsigned)wanted;
    return (int32_t)(s->duty+.5f);
}
#endif
