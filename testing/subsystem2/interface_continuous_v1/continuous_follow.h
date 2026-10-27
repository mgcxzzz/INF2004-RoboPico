#ifndef BUDDY_CONTINUOUS_FOLLOW_H
#define BUDDY_CONTINUOUS_FOLLOW_H
#include "continuous_drive.h"
#include "v12_2/scan_smooth.h"
#include "continuous_steering.h"
/* Barcode state and original optical tracker run exactly once per tick.
 * Only the ordinary visible-line output adapter is replaced. */
static inline AnalogDemand continuous_follow_step(Circuit *c,BarcodeNav *n,
 const LineCalibration *cal,const CircuitInput *i,unsigned profile,
 SmoothFollow *legacy,const BarcodeAnalog *a,ScanSmooth *s,ContinuousSteering *t,float dt) {
 if(profile==2) {
  continuous_steering_reset(t);
  return scan_smooth_step(c,n,cal,i,profile,legacy,a,s,dt);
 }
 AnalogDemand d=barcode_circuit_step(c,n,cal,i,profile);DRIVE_DIAG_RAW(d);
 bool activity=a->scanning || n->pending || a->adc>=(unsigned)a->baseline+16;
 if(activity)s->activity_us=i->now;
 bool scanning=!c->busy && !d.stop && (activity || (s->scanning && i->now-s->activity_us<600000));
 if(scanning!=s->scanning) {
  double mm=(i->wheel[0]+i->wheel[1])*.5;if(scanning)s->start_mm=mm;
  barcode_event(n,scanning?B_SOFT_BEGIN:B_SOFT_END,i->now,mm,mm-s->start_mm,
                n->last_code,'-',c->busy?1:0,NULL);
 }
 s->scanning=scanning;*legacy=(SmoothFollow){0};
 d=continuous_steering_step(t,d,c,i,scanning,profile,dt);
 s->mean=t->mean;s->half=t->half;s->active=t->active;
 for(unsigned w=0;w<2;++w)s->emitted[w]=t->emitted[w];
 DRIVE_DIAG_SOFT(d,t->active?1:0,scanning,t->active);return d;
}
static inline bool continuous_drive_selected(unsigned profile,unsigned phase,bool busy,
 bool external,bool ramp,bool maneuver,bool white,const AnalogDemand *d) {
 return (profile==1 || profile==3) && phase==C_FOLLOW && !busy && !external &&
        !ramp && !maneuver && !white && !d->stop && (d->decision==24 || d->decision==25);
}
static inline int32_t continuous_output_cap(int32_t pwm) {
 return pwm<0?0:pwm>CONTINUOUS_PWM_MAX?CONTINUOUS_PWM_MAX:pwm;
}
static inline DualLevels continuous_output_levels(unsigned w,int32_t pwm) {
 DualLevels v={0};if(w>1)return v;
 if(w==0)v.a=(uint16_t)continuous_output_cap(pwm);else v.b=(uint16_t)continuous_output_cap(pwm);
 return v;
}
/* Drop private mode latches when leaving ordinary drive. Seed the existing
 * legacy PI in its native units; its special manoeuvre rules still take over. */
static inline void continuous_leave(ContinuousDrive *s,SignedWheel *p,unsigned w,
 int16_t wanted,float speed,int32_t previous) {
 if(!s->active)return;
 if(wanted>0 && p->sign==1) {
  float ff=feedforward(w,1,p->pi.ramp);
  p->pi.integral=clampf((float)previous/1.2f-ff-LINE_SPEED_KP*(p->pi.ramp-speed),
                       -LINE_SPEED_I_LIMIT,LINE_SPEED_I_LIMIT);
 }
 *s=(ContinuousDrive){0};
}
#endif
