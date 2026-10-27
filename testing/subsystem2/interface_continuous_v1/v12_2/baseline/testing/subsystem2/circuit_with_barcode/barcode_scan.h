#ifndef BARCODE_SCAN_H
#define BARCODE_SCAN_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

enum { B_RUN=1,B_FRAME_START,B_DECODE,B_BAD_FRAME,B_CROSS,B_NO_COMMAND,
       B_DISPATCH,B_COMPLETE,B_GATE_REJECT,B_QUEUE_OVERFLOW,B_CROSS_CANDIDATE,B_CROSS_CANCEL,
       B_ANALOG_CANDIDATE,B_ANALOG_SELECT,B_ANALOG_CONFLICT,B_ANALOG_GAP,B_UTURN_STAGE,B_RECOVERY_WAIT,B_RECOVERY_SWEEP,B_RECOVERY_FOUND,B_RECOVERY_FAIL,
       B_HOLD_BEGIN,B_HOLD_PATTERN,B_HOLD_END,B_HOLD_CANCEL,B_SOFT_BEGIN,B_SOFT_END,
       B_EVENTS=192,B_ROWS=1000,B_EDGE_QUEUE=256 };
enum { B_REJECT_WIDTH=1,B_REJECT_SEPARATION=2,B_REJECT_COUNT=4,
       B_REJECT_GAP=8,B_REJECT_PATTERN=16,B_REJECT_TIMEOUT=32,B_REJECT_RUN=64 };
typedef struct {
    uint64_t ms;uint32_t path10;int32_t width100;uint32_t number;
    uint16_t pattern[3];uint8_t type,code,action,flags;
} BarcodeEvent;
_Static_assert(sizeof(BarcodeEvent)==32,"Barcode event ABI");
typedef struct {
    /* Index0=black bars,1=white gaps. D0 can expand black and shrink white,
       so comparing their widths in a single sorted list is not reliable. */
    double narrow[2],wide[2],threshold[2];uint16_t pattern;bool valid;
} BarcodeGroup;
/* Standard Code39 encodings, checked against the supplied A/Z PDF and
   ZXing's published Code39Reader table. Decoder/classifier below is local. */
static inline char barcode_character(uint16_t pattern) {
    static const char alphabet[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%";
    static const uint16_t table[]={
        0x034,0x121,0x061,0x160,0x031,0x130,0x070,0x025,0x124,0x064,
        0x109,0x049,0x148,0x019,0x118,0x058,0x00d,0x10c,0x04c,0x01c,
        0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,
        0x181,0x0c1,0x1c0,0x091,0x190,0x0d0,0x085,0x184,0x0c4,0x0a8,
        0x0a2,0x08a,0x02a};
    for(unsigned n=0;n<43;++n) if(table[n]==pattern) return alphabet[n];
    return pattern==0x094?'*':'?';
}
static inline uint16_t barcode_reverse9(uint16_t x) {
    uint16_t y=0;for(unsigned n=0;n<9;++n) { y=(uint16_t)((y<<1)|(x&1));x>>=1; }return y;
}
static inline BarcodeGroup barcode_group(const double *w) {
    BarcodeGroup g={0};
    /* Sentinel * (either direction) has two wide black bars and one wide
       white gap. Learn each colour independently from that structure. */
    for(unsigned colour=0;colour<2;++colour) {
        double sorted[5];unsigned count=0;
        for(unsigned n=colour;n<9;n+=2) sorted[count++]=w[n];
        for(unsigned n=1;n<count;++n) { double v=sorted[n];unsigned j=n;while(j && sorted[j-1]>v) { sorted[j]=sorted[j-1];--j; }sorted[j]=v; }
        unsigned wide_count=colour?1:2,narrow_count=count-wide_count;
        if(sorted[0]<0.35 || sorted[count-1]>40 ||
           sorted[narrow_count]<sorted[narrow_count-1]*1.55 ||
           sorted[count-1]>sorted[narrow_count]*1.75) return g;
        for(unsigned n=0;n<narrow_count;++n) g.narrow[colour]+=sorted[n]/narrow_count;
        for(unsigned n=narrow_count;n<count;++n) g.wide[colour]+=sorted[n]/wide_count;
        g.threshold[colour]=(sorted[narrow_count-1]+sorted[narrow_count])*0.5;
        if(g.narrow[colour]>14 || g.wide[colour]<g.narrow[colour]*1.8) return g;
    }
    for(unsigned n=0;n<9;++n)
        g.pattern=(uint16_t)((g.pattern<<1)|(w[n]>g.threshold[n%2]));
    g.valid=true;return g;
}
static inline bool barcode_classify(const double *w,const BarcodeGroup *reference,uint16_t *pattern,unsigned *rejection) {
    unsigned wide_count=0;*pattern=0;
    double max_narrow[2]={0},min_wide[2]={1e9,1e9};
    for(unsigned n=0;n<9;++n) {
        unsigned colour=n%2;bool wide=w[n]>reference->threshold[colour];
        *pattern=(uint16_t)((*pattern<<1)|wide);
        if(w[n]<0.35 || w[n]>40) *rejection|=B_REJECT_WIDTH;
        if(wide) {
            ++wide_count;min_wide[colour]=fmin(min_wide[colour],w[n]);
            if(w[n]<reference->wide[colour]*.55 || w[n]>reference->wide[colour]*1.8) *rejection|=B_REJECT_WIDTH;
        } else {
            max_narrow[colour]=fmax(max_narrow[colour],w[n]);
            /* Learned colour threshold already defines the narrow class.
               Do not impose a second limit based on the sentinel's mean:
               partly shortened edge bars can bias that mean substantially. */
        }
    }
    for(unsigned colour=0;colour<2;++colour)
        if(max_narrow[colour]>0 && min_wide[colour]<max_narrow[colour]*1.5) *rejection|=B_REJECT_SEPARATION;
    if(wide_count!=3) *rejection|=B_REJECT_COUNT;
    return !*rejection;
}
typedef struct {
    double widths[29],history[9],edge_mm,start_mm,max_frame_mm;
    BarcodeGroup reference;
    uint64_t edge_us,frame_us;
    uint8_t count,history_count;bool initialized,black,frame,reverse;
} BarcodeReader;
typedef struct {
    unsigned events,rejection;bool completed_black,reverse;double width,start,end;
    char code;uint16_t pattern[3];
} BarcodeRead;
static inline void barcode_reader_reset(BarcodeReader *r,bool black,uint64_t now,double mm) {
    *r=(BarcodeReader){.initialized=true,.black=black,.edge_us=now,.edge_mm=mm};
}
/* Completed runs use encoder distance, never a fixed bar time or fixed print
   length. Single-character frames have29 alternating elements: * + gap + X
   + gap + *. Both physical scan directions are accepted. */
static inline BarcodeRead barcode_reader_edge(BarcodeReader *r,bool black,uint64_t now,double mm) {
    BarcodeRead result={0};
    if(!r->initialized) { barcode_reader_reset(r,black,now,mm);return result; }
    if(black==r->black) return result;
    bool previous=r->black;double width=mm-r->edge_mm;
    result.completed_black=previous;result.width=width;result.start=r->edge_mm;result.end=mm;
    r->black=black;r->edge_mm=mm;r->edge_us=now;result.events=1u<<B_RUN;
    if(width<0.35 || (width>40 && r->frame)) {
        if(r->frame) { result.events|=1u<<B_BAD_FRAME;result.rejection=B_REJECT_RUN; }
        r->frame=false;r->count=r->history_count=0;return result;
    }
    if(r->frame) {
        r->widths[r->count++]=width;
        if(r->count<29) return result;
        bool good=true;
        for(unsigned n=0;n<3;++n) {
            uint16_t pattern=0;unsigned rejection=0;
            if(!barcode_classify(r->widths+n*10,&r->reference,&pattern,&rejection)) good=false;
            result.rejection|=rejection;
            result.pattern[n]=r->reverse?barcode_reverse9(pattern):pattern;
        }
        for(unsigned n=9;n<=19;n+=10)
            if(r->widths[n]<fmax(.35,r->reference.narrow[1]*.25) ||
               r->widths[n]>r->reference.threshold[1]) { good=false;result.rejection|=B_REJECT_GAP; }
        char code=barcode_character(result.pattern[1]);
        if(result.pattern[0]!=0x094 || result.pattern[2]!=0x094 || code=='?' || code=='*') {
            good=false;result.rejection|=B_REJECT_PATTERN;
        }
        result.start=r->start_mm;result.reverse=r->reverse;result.code=good?code:'?';
        result.events|=1u<<(good?B_DECODE:B_BAD_FRAME);
        r->frame=false;r->count=r->history_count=0;return result;
    }
    if(width>40) { r->history_count=0;return result; }
    if(r->history_count<9) r->history[r->history_count++]=width;
    else { memmove(r->history,r->history+1,8*sizeof(double));r->history[8]=width; }
    if(previous && r->history_count==9) {
        BarcodeGroup g=barcode_group(r->history);
        if(g.valid && (g.pattern==0x094 || barcode_reverse9(g.pattern)==0x094)) {
            memcpy(r->widths,r->history,sizeof(r->history));r->count=9;r->frame=true;
            r->reverse=g.pattern!=0x094;r->frame_us=now;r->reference=g;
            double length=0;for(unsigned n=0;n<9;++n) length+=r->history[n];
            r->max_frame_mm=fmin(600,length*4.5+2*g.threshold[1]);
            r->start_mm=mm-length;result.start=r->start_mm;result.events|=1u<<B_FRAME_START;
        }
    }
    return result;
}
static inline bool barcode_reader_expire(BarcodeReader *r,uint64_t now,double mm) {
    if(r->frame && (now-r->frame_us>=20000000 || mm-r->start_mm>r->max_frame_mm)) {
        r->frame=false;r->count=r->history_count=0;return true;
    }
    if(!r->frame && !r->black && mm-r->edge_mm>40) r->history_count=0;
    return false;
}
#endif
