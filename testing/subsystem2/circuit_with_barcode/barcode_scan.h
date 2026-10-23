#ifndef BARCODE_SCAN_H
#define BARCODE_SCAN_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

enum { B_RUN=1,B_FRAME_START,B_DECODE,B_BAD_FRAME,B_CROSS,B_NO_COMMAND,
       B_DISPATCH,B_COMPLETE,B_GATE_REJECT,B_QUEUE_OVERFLOW,
       B_EVENTS=192,B_ROWS=1000,B_EDGE_QUEUE=256 };
typedef struct {
    uint64_t ms;uint32_t path10;int32_t width100;uint32_t number;
    uint16_t pattern[3];uint8_t type,code,action,flags;
} BarcodeEvent;
_Static_assert(sizeof(BarcodeEvent)==32,"Barcode event ABI");
typedef struct {
    double narrow,wide;uint16_t pattern;bool valid;
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
    BarcodeGroup g={0};double sorted[9];memcpy(sorted,w,sizeof(sorted));
    for(unsigned n=1;n<9;++n) { double v=sorted[n];unsigned j=n;while(j && sorted[j-1]>v) { sorted[j]=sorted[j-1];--j; }sorted[j]=v; }
    if(sorted[0]<0.50 || sorted[5]>5.5 || sorted[8]>11.5 ||
       sorted[6]<sorted[5]*1.60 || sorted[8]>sorted[6]*1.75 || sorted[5]>sorted[0]*2.6) return g;
    double threshold=(sorted[5]+sorted[6])*0.5;
    for(unsigned n=0;n<9;++n) {
        g.pattern=(uint16_t)((g.pattern<<1)|(w[n]>threshold));
        if(w[n]>threshold) g.wide+=w[n]/3.0;else g.narrow+=w[n]/6.0;
    }
    g.valid=g.wide>=g.narrow*1.8 && g.wide<=g.narrow*4.2;return g;
}
typedef struct {
    double widths[29],history[9],edge_mm,start_mm,narrow,wide;
    uint64_t edge_us,frame_us;
    uint8_t count,history_count;bool initialized,black,frame,reverse;
} BarcodeReader;
typedef struct {
    unsigned events;bool completed_black,reverse;double width,start,end;
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
    if(width<0.35 || (width>12 && r->frame)) {
        if(r->frame) result.events|=1u<<B_BAD_FRAME;
        r->frame=false;r->count=r->history_count=0;return result;
    }
    if(r->frame) {
        r->widths[r->count++]=width;
        if(r->count<29) return result;
        BarcodeGroup group[3]={barcode_group(r->widths),barcode_group(r->widths+10),barcode_group(r->widths+20)};
        bool good=true;
        for(unsigned n=0;n<3;++n) {
            result.pattern[n]=r->reverse?barcode_reverse9(group[n].pattern):group[n].pattern;
            if(!group[n].valid || group[n].narrow<r->narrow*0.60 || group[n].narrow>r->narrow*1.65) good=false;
        }
        if(r->widths[9]<r->narrow*.45 || r->widths[9]>r->narrow*1.9 ||
           r->widths[19]<r->narrow*.45 || r->widths[19]>r->narrow*1.9) good=false;
        char code=barcode_character(result.pattern[1]);
        if(result.pattern[0]!=0x094 || result.pattern[2]!=0x094 || code=='?' || code=='*') good=false;
        result.start=r->start_mm;result.reverse=r->reverse;result.code=good?code:'?';
        result.events|=1u<<(good?B_DECODE:B_BAD_FRAME);
        r->frame=false;r->count=r->history_count=0;return result;
    }
    if(width>12) { r->history_count=0;return result; }
    if(r->history_count<9) r->history[r->history_count++]=width;
    else { memmove(r->history,r->history+1,8*sizeof(double));r->history[8]=width; }
    if(previous && r->history_count==9) {
        BarcodeGroup g=barcode_group(r->history);
        if(g.valid && (g.pattern==0x094 || barcode_reverse9(g.pattern)==0x094)) {
            memcpy(r->widths,r->history,sizeof(r->history));r->count=9;r->frame=true;
            r->reverse=g.pattern!=0x094;r->frame_us=now;r->narrow=g.narrow;r->wide=g.wide;
            double length=0;for(unsigned n=0;n<9;++n) length+=r->history[n];
            r->start_mm=mm-length;result.start=r->start_mm;result.events|=1u<<B_FRAME_START;
        }
    }
    return result;
}
static inline bool barcode_reader_expire(BarcodeReader *r,uint64_t now,double mm) {
    if(r->frame && (now-r->frame_us>=20000000 || mm-r->start_mm>210)) {
        r->frame=false;r->count=r->history_count=0;return true;
    }
    if(!r->frame && !r->black && mm-r->edge_mm>15) r->history_count=0;
    return false;
}
#endif
