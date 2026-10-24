#ifndef BARCODE_LINE_SEARCH_H
#define BARCODE_LINE_SEARCH_H
#include <stdint.h>
#include <stdbool.h>

enum { LS_CONFIRM_US=150000,LS_TOTAL_US=5250000,LS_MAX_MM=150,
       LS_TIMEOUT=47,LS_DISTANCE=48,LS_DECISION_FIRST=19 };
typedef struct { uint64_t since;uint32_t mm;int8_t first;bool active; } LineSearch;
typedef struct { int16_t target[2];uint8_t stage,reason; } LineSearchDemand;
static inline int line_search_outer(bool active,int left,int right) {
    return !active?-1:left==0 && right>0?1:right==0 && left>0?0:-1;
}
static inline void line_search_begin(LineSearch *s,uint64_t now,uint32_t mm,int direction) {
    *s=(LineSearch){.since=now,.mm=mm,.first=direction<0?-1:1,.active=true};
}
/* Forward-only arcs: coast one wheel and drive the other. Their directions
 * alternate;350/700/1050/1400/1750ms durations widen the angular sweep if
 * motion is comparable in both directions. This does not claim a stationary
 * pivot or an encoder-guaranteed angle. Time also expires with zero ticks. */
static inline LineSearchDemand line_search_step(const LineSearch *s,uint64_t now,uint32_t mm,bool confirming) {
    LineSearchDemand d={0};uint64_t elapsed=now-s->since;
    if(elapsed>=LS_TOTAL_US) { d.reason=LS_TIMEOUT;return d; }
    if(mm-s->mm>=LS_MAX_MM) { d.reason=LS_DISTANCE;return d; }
    uint64_t end=350000;
    while(d.stage<4 && elapsed>=end) { ++d.stage;end+=(uint64_t)(d.stage+1)*350000; }
    int direction=d.stage%2?-s->first:s->first;
    d.target[direction>0?1:0]=confirming?30:45;
    return d;
}
#endif
