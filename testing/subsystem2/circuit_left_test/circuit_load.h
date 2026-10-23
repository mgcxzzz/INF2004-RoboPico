#ifndef CIRCUIT_LOAD_H
#define CIRCUIT_LOAD_H
#include "circuit_logic.h"

enum { C_LOAD_QUALIFY_US=120000,C_LOAD_US=2000000,C_LOAD_MM=30,C_LOAD_COOLDOWN_US=300000 };
typedef struct {
    uint64_t since,cooldown_until;
    double path;
    unsigned outer;
    bool timing,active;
} CircuitLoad;
/* Ordinary visible sharp corrections can need the same effort as branch
   capture. Qualify measured underspeed; never change the optical targets. */
static inline int circuit_load_outer(CircuitLoad *s,const CircuitInput *i,unsigned phase,
                                     const AnalogDemand *d,bool capture_active) {
    unsigned outer=d->target[0]>d->target[1]?0:1,inner=1-outer;
    bool eligible=!d->stop && !capture_active && phase==C_FOLLOW &&
        (d->level[0]>=400 || d->level[1]>=400) && d->target[outer]>=30 &&
        d->target[inner]<=(s->active?20:12);
    if(s->active) {
        if(!eligible || outer!=s->outer || i->now-s->since>=C_LOAD_US ||
           i->path-s->path>=C_LOAD_MM) {
            s->active=s->timing=false;s->cooldown_until=i->now+C_LOAD_COOLDOWN_US;
            return -1;
        }
        return (int)outer;
    }
    if(!eligible || i->now<s->cooldown_until || i->speed[outer]>=15) {
        s->timing=false;return -1;
    }
    if(!s->timing || outer!=s->outer) {
        s->timing=true;s->outer=outer;s->since=i->now;
    }
    if(i->now-s->since<C_LOAD_QUALIFY_US) return -1;
    s->active=true;s->timing=false;s->since=i->now;s->path=i->path;
    return (int)outer;
}
#endif
