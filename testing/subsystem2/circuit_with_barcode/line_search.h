#ifndef BARCODE_LINE_SEARCH_H
#define BARCODE_LINE_SEARCH_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

enum { LS_CONFIRM_US=1500000,LS_BRIDGE_US=150000,LS_ALIGN_US=15000000,LS_ALIGN_MM=1100,
       LS_TOTAL_US=12000000,LS_MAX_MM=1000,LS_SPEED=100,LS_CONFIRM_SPEED=25,
       LS_TIMEOUT=47,LS_DISTANCE=48,LS_DRIFT=49,LS_DECISION_FIRST=19 };
typedef struct {
    uint64_t since,wait_since,still_since;uint32_t mm;int8_t first;
    double angle,origin_angle,centre,origin_centre;float speed;
    uint8_t stage;bool active,waiting,settling,still_timing;
} LineSearch;
typedef struct { int16_t target[2];uint8_t stage,reason; } LineSearchDemand;
static inline int line_search_outer(bool active,int left,int right) {
    return !active?-1:left<=0 && right>0?1:right<=0 && left>0?0:-1;
}
static inline void line_search_observe(LineSearch *s,double left,double right,float vl,float vr) {
    s->angle=(right-left)*180.0/(110.0*3.14159265358979323846);
    s->centre=(left+right)*.5;s->speed=fmaxf(fabsf(vl),fabsf(vr));
}
/* Count a continuous both-white interval, with a150..199 hysteresis band.
 * A credible return resets this pre-search timer; once sweeping, flashes
 * cannot renew its separate time/distance budget. */
static inline bool line_search_ready(LineSearch *s,uint64_t now,uint16_t left,uint16_t right) {
    if(s->active)return true;
    if(left>=200 || right>=200) {s->waiting=false;return false;}
    if(!s->waiting) {
        if(left>=150 || right>=150)return false;
        s->waiting=true;s->wait_since=now;
    }
    return now-s->wait_since>=LS_CONFIRM_US;
}
static inline void line_search_begin(LineSearch *s,uint64_t now,uint32_t mm,int direction) {
    double angle=s->angle,centre=s->centre;float speed=s->speed;
    *s=(LineSearch){.since=now,.mm=mm,.first=direction<0?-1:1,.active=true,
        .angle=angle,.origin_angle=angle,.centre=centre,.origin_centre=centre,.speed=speed};
}
/* Counter-rotation about the rear-wheel midpoint. Encoder-relative endpoints
 * are +35,-60,+90,-120,+150 degrees, mirrored toward the last reliable side.
 * Stop briefly at each endpoint before reversing. Time bounds still expire
 * with no ticks; encoder yaw/centre are estimates and cannot detect slipping. */
static inline LineSearchDemand line_search_step(LineSearch *s,uint64_t now,uint32_t mm,bool confirming) {
    LineSearchDemand d={0};uint64_t elapsed=now-s->since;
    if(elapsed>=LS_TOTAL_US) { d.reason=LS_TIMEOUT;return d; }
    if(mm-s->mm>=LS_MAX_MM) { d.reason=LS_DISTANCE;return d; }
    if(fabs(s->centre-s->origin_centre)>=30) {d.reason=LS_DRIFT;return d;}
    static const int angle[5]={35,60,90,120,150};
    int direction=s->stage%2?-s->first:s->first;
    double remaining=angle[s->stage]-direction*(s->angle-s->origin_angle);
    if(remaining<=0)s->settling=true;
    if(s->settling) {
        if(s->speed>=5)s->still_timing=false;
        else if(!s->still_timing) {s->still_timing=true;s->still_since=now;}
        if(s->still_timing && now-s->still_since>=80000) {
            if(s->stage==4) {d.reason=LS_TIMEOUT;return d;}
            ++s->stage;s->settling=s->still_timing=false;direction=-direction;
        } else {d.stage=s->stage;return d;}
    }
    d.stage=s->stage;
    /* A possible line stops rotation for the existing40/80/120ms optical
     * dwell. A flash cannot restart the independent search budget. */
    if(confirming)return d;
    double goal=angle[s->stage]-direction*(s->angle-s->origin_angle);
    int speed=goal<12?LS_CONFIRM_SPEED:LS_SPEED;
    float centre_error=(float)(s->centre-s->origin_centre);
    int trim=(int)fmaxf(-20,fminf(20,-4*centre_error));
    d.target[0]=(int16_t)(-direction*speed+trim);
    d.target[1]=(int16_t)(direction*speed+trim);
    return d;
}
#endif
