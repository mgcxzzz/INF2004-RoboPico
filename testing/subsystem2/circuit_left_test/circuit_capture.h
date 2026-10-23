#ifndef CIRCUIT_CAPTURE_H
#define CIRCUIT_CAPTURE_H
#include "circuit_logic.h"
enum { C_CAPTURE_US=3000000,C_CAPTURE_MM=45,C_ROLLING_US=100000 };
typedef struct {
    uint64_t since,rolling_since;
    double path;
    int8_t outer;
    bool window,rolling_timing;
} CircuitCapture;
static inline void circuit_capture_begin(CircuitCapture *s,uint64_t now,double path) {
    *s=(CircuitCapture){.since=now,.path=path,.outer=-1,.window=true};
}
/* Return outer wheel0/1, or-1 for ordinary effort. No target changes.
   Assistance is possible only just after optical confirmation of a branch,
   with a visible line and a near-pivot demand. It never assists blind recovery. */
static inline int circuit_capture_outer(CircuitCapture *s,const CircuitInput *i,unsigned phase,const AnalogDemand *d) {
    if(!s->window) return -1;
    if(d->stop || (phase!=J_ACQUIRE && phase!=C_FOLLOW) ||
       i->now-s->since>=C_CAPTURE_US || i->path-s->path>=C_CAPTURE_MM) {
        s->window=false;s->outer=-1;return -1;
    }
    bool rolling=d->target[0]>=15 && d->target[1]>=15 && i->speed[0]>=10 && i->speed[1]>=10;
    if(!rolling) s->rolling_timing=false;
    else {
        if(!s->rolling_timing) { s->rolling_timing=true;s->rolling_since=i->now; }
        if(i->now-s->rolling_since>=C_ROLLING_US) { s->window=false;s->outer=-1;return -1; }
    }
    unsigned outer=d->target[0]>d->target[1]?0:1,inner=1-outer;
    unsigned limit=s->outer==(int)outer?20:12; /* effort hysteresis, not steering thresholds */
    bool visible=d->level[0]>=400 || d->level[1]>=400;
    if(visible && d->target[outer]>=30 && d->target[inner]<=(int)limit) {
        s->outer=(int8_t)outer;return (int)outer;
    }
    s->outer=-1;return -1;
}
#endif
