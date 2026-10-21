#ifndef ANALOG_LINE_LOGIC_H
#define ANALOG_LINE_LOGIC_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint16_t white[2],black[2]; uint8_t mask; } LineCalibration;
enum { AL_EDGE, AL_RIGHT_GUARD, AL_BOTH_BLACK, AL_SEEK_LEFT, AL_LOST };
typedef struct { uint64_t white_since,both_since; uint32_t white_mm; bool white,both; } AnalogTracker;
typedef struct { int16_t target[2]; uint16_t level[2]; uint8_t decision; bool stop; } AnalogDemand;
enum { LINE_USB_HOLD, LINE_BANK, LINE_USB_FREE };
enum { START_OK,START_ACTIVE,START_SETTLING,START_CAL,START_GP20,START_REARM,
       START_LEFT_WHITE,START_RIGHT_DARK,START_HOLD_GP21,START_RELEASE_GP21,START_GUARD };
typedef struct { uint64_t since; bool timing,ready; } LineRearm;
/* Readiness is permission for a NEW explicit request, never auto-start.
 * Holding GP21 after readiness is allowed for the original USB hold mode. */
static inline void line_rearm_step(LineRearm *r,uint64_t now,bool released,bool stationary,bool busy) {
    if(busy || !stationary) { *r=(LineRearm){0};return; }
    if(r->ready) return;
    if(!released) { r->timing=false;return; }
    if(!r->timing) { r->timing=true;r->since=now; }
    if(now-r->since>=1000000) r->ready=true;
}
static inline unsigned line_start_block(bool active,bool settling,bool valid,bool stop,
                                        bool ready,uint16_t left,uint16_t right,bool hold,unsigned mode) {
    if(active) return START_ACTIVE;
    if(settling) return START_SETTLING;
    if(!valid) return START_CAL;
    if(stop) return START_GP20;
    if(!ready) return START_REARM;
    /* Full left black is now permitted. All-white / right-dark still reject. */
    if(left<150) return START_LEFT_WHITE;
    if(right>450) return START_RIGHT_DARK;
    if(mode==LINE_USB_HOLD && !hold) return START_HOLD_GP21;
    if(mode!=LINE_USB_HOLD && hold) return START_RELEASE_GP21;
    return START_OK;
}
static inline bool line_countdown_cancel(bool connected,bool held,bool stop) {
    return !connected || held || stop;
}

/* Profile3 isolates the existing wheel controllers from optical steering.
 * Deliberately forward-only; two-second stages include the normal speed ramp.
 * Caller retains button, feedback, direction, distance and deadline guards. */
enum { RESPONSE_US=6000000, RESPONSE_MM=450 };
static inline AnalogDemand response_demand(uint64_t elapsed) {
    AnalogDemand d={0};
    unsigned phase=(unsigned)(elapsed/2000000u);
    d.decision=(uint8_t)(5+(phase>2?2:phase));
    if(elapsed>=RESPONSE_US) { d.stop=true;return d; }
    d.target[0]=phase==1?35:phase==2?95:65;
    d.target[1]=phase==1?95:phase==2?35:65;
    return d;
}
static inline unsigned run_start_block(bool response,bool active,bool settling,bool valid,bool stop,
                                       bool ready,uint16_t left,uint16_t right,bool hold,unsigned mode) {
    /* Only optical alignment is bypassed. Calibration and deliberate rearm
       remain required, including for a diagnostic on a plain white surface. */
    return line_start_block(active,settling,valid,stop,ready,response?500:left,response?0:right,hold,mode);
}
enum { AL_KICK_PWM=250, AL_KICK_US=120000, AL_KICK_COOLDOWN_US=100000 };
typedef struct { uint64_t end_us,cooldown_until; bool active; } LineBreakaway;
/* Forward-only floor breakaway. Independent drive-off guards still own the
 * outputs. Pulse ends on the first control update at120ms or speed>=20mm/s;
 * it cannot trigger again until100ms cooldown and near-stationary feedback. */
static inline int32_t line_breakaway(LineBreakaway *b,uint64_t now,int16_t wanted,
                                     float ramp,float speed,int32_t pwm) {
    if(wanted<=0) { *b=(LineBreakaway){0};return 0; }
    if(b->active && (now>=b->end_us || speed>=20 || speed < -8)) {
        b->active=false;b->cooldown_until=now+AL_KICK_COOLDOWN_US;
    }
    if(!b->active && now>=b->cooldown_until && wanted>=20 && ramp>=20 && speed>=-8 && speed<8) {
        b->active=true;b->end_us=now+AL_KICK_US;
    }
    if(b->active && pwm<AL_KICK_PWM) pwm=AL_KICK_PWM;
    if(pwm<0) pwm=0;
    if(pwm>AL_KICK_PWM) pwm=AL_KICK_PWM;
    return pwm;
}
static inline bool calibration_valid(const LineCalibration *c) {
    return c->mask==7 && c->white[0]<=4095 && c->white[1]<=4095 &&
        c->black[0]<=4095 && c->black[1]<=4095 &&
        c->black[0]>=c->white[0]+100 && c->black[1]>=c->white[1]+100;
}
static inline uint16_t line_level(uint16_t raw,uint16_t white,uint16_t black) {
    if(black<=white || raw<=white) return 0;
    if(raw>=black) return 1000;
    return (uint16_t)(((uint32_t)(raw-white)*1000u)/(black-white));
}
/* Follow the RIGHT edge of the strip with the physical LEFT sensor:
 * blackness above midpoint -> steer right, below midpoint -> steer left.
 * Positive differential means left wheel faster, which turns chassis right.
 * Right sensor assists after larger rightward line displacement. No barcode. */
static inline AnalogDemand analog_demand(AnalogTracker *t,const LineCalibration *c,
                                        uint16_t l,uint16_t r,uint64_t now,uint32_t mm) {
    AnalogDemand d={0};
    if(!calibration_valid(c)) { d.stop=true;return d; }
    d.level[0]=line_level(l,c->white[0],c->black[0]);
    d.level[1]=line_level(r,c->white[1],c->black[1]);
    int32_t error=(int32_t)d.level[0]-500;
    if(error>-60 && error<60) error=0;
    int32_t differential=error*60/1000; /* -30..30 mm/s, base65 */
    if(d.level[1]>=650) { differential=30;d.decision=AL_RIGHT_GUARD; }
    bool both=d.level[0]>=750 && d.level[1]>=750;
    if(both) {
        if(!t->both) t->both_since=now;
        t->both=true;d.decision=AL_BOTH_BLACK;
        d.target[0]=d.target[1]=35;
        if(now-t->both_since>=120000) d.stop=true;
    } else {
        t->both=false;d.target[0]=(int16_t)(65+differential);d.target[1]=(int16_t)(65-differential);
    }
    bool white=d.level[0]<150 && d.level[1]<150;
    if(white) {
        if(!t->white) { t->white_since=now;t->white_mm=mm; }
        t->white=true;d.decision=AL_SEEK_LEFT;
        if(now-t->white_since>=650000 || mm-t->white_mm>=40) {
            d.stop=true;d.decision=AL_LOST;
        }
    } else t->white=false;
    if(d.stop) d.target[0]=d.target[1]=0;
    return d;
}
#endif
