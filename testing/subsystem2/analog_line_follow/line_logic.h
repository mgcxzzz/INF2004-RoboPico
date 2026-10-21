#ifndef ANALOG_LINE_LOGIC_H
#define ANALOG_LINE_LOGIC_H
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint16_t white[2],black[2]; uint8_t mask; } LineCalibration;
enum { AL_EDGE, AL_RIGHT_GUARD, AL_BOTH_BLACK, AL_SEEK_LEFT, AL_LOST };
typedef struct { uint64_t white_since,both_since; uint32_t white_mm; bool white,both; } AnalogTracker;
typedef struct { int16_t target[2]; uint16_t level[2]; uint8_t decision; bool stop; } AnalogDemand;
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
