#ifndef BARCODE_COVERAGE_STATE_H
#define BARCODE_COVERAGE_STATE_H
#include <stdint.h>
#include <stdbool.h>
/* Normalised to each sensor's stored white/black references, not raw ADC.
 * With the school calibration:60~375 ADC,120~575,180~775. */
enum { CV_WHITE=60,CV_CLEAR=120,CV_PRESENT=180,
       CV_WHITE_US=30000,CV_RETURN_US=20000,CV_HARD_US=120000,CV_AMBIGUOUS_US=80000,
       CV_CENTRE=24,CV_SOFT_LEFT=26,CV_SOFT_RIGHT=27,
       CV_FIRM_LEFT=28,CV_FIRM_RIGHT=29,CV_UNCERTAIN=30 };
typedef struct {
    uint64_t white_since[2],clear_since[2],low_since;
    bool white[2],white_timing[2],clear_timing[2],low_timing,paired;
} CoverageState;
static inline void coverage_observe(CoverageState *s,const uint16_t level[2],uint64_t now) {
    for(unsigned w=0;w<2;++w) {
        if(!s->white[w]) {
            if(level[w]>CV_WHITE)s->white_timing[w]=false;
            else {
                if(!s->white_timing[w]) {s->white_timing[w]=true;s->white_since[w]=now;}
                if(now-s->white_since[w]>=CV_WHITE_US)s->white[w]=true;
            }
        } else {
            if(level[w]<CV_CLEAR)s->clear_timing[w]=false;
            else {
                if(!s->clear_timing[w]) {s->clear_timing[w]=true;s->clear_since[w]=now;}
                if(now-s->clear_since[w]>=CV_RETURN_US) {
                    s->white[w]=s->white_timing[w]=s->clear_timing[w]=false;
                }
            }
        }
    }
    unsigned minimum=level[0]<level[1]?level[0]:level[1];
    if(minimum<CV_CLEAR)s->paired=false;
    else if(minimum>=CV_PRESENT)s->paired=true;
    if(level[0]>=CV_CLEAR || level[1]>=CV_CLEAR)s->low_timing=false;
    else if(!s->low_timing) {s->low_timing=true;s->low_since=now;}
}
static inline void coverage_mirror(CoverageState *s) {
    uint64_t v=s->white_since[0];s->white_since[0]=s->white_since[1];s->white_since[1]=v;
    v=s->clear_since[0];s->clear_since[0]=s->clear_since[1];s->clear_since[1]=v;
    bool b=s->white[0];s->white[0]=s->white[1];s->white[1]=b;
    b=s->white_timing[0];s->white_timing[0]=s->white_timing[1];s->white_timing[1]=b;
    b=s->clear_timing[0];s->clear_timing[0]=s->clear_timing[1];s->clear_timing[1]=b;
}
#endif
