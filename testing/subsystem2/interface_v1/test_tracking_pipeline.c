#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tracking_cruise.h"
static const LineCalibration cal={{164,168},{3514,3389},7};
typedef struct {Circuit c;BarcodeNav n;BarcodeAnalog a;SmoothFollow old;ScanSmooth optical;TrackingCruise cruise;CircuitInput i;AnalogDemand d;} Rig;
static void init(Rig *r) {
    memset(r,0,sizeof(*r));r->i=(CircuitInput){.now=1000,.adc={3011,1778},.speed={45,45}}; /* normalised850/500 */
    circuit_begin(&r->c,&r->i);barcode_nav_begin(&r->n,&r->i);barcode_analog_begin(&r->a,&r->n,180,r->i.now,r->i.wheel);
}
static void tick(Rig *r,unsigned adc,unsigned profile) {
    r->i.now+=10000;for(unsigned w=0;w<2;++w)r->i.wheel[w]+=.45;r->i.path+=.45;
    for(unsigned j=0;j<5;++j)barcode_analog_sample(&r->a,&r->n,adc,r->i.now-8000+j*2000,r->i.wheel);
    r->i.side_black=r->a.black;
    r->d=scan_smooth_step(&r->c,&r->n,&cal,&r->i,profile,&r->old,&r->a,&r->optical,.01f);
    r->d=tracking_cruise_step(&r->cruise,&r->c,&r->i,profile,&r->optical,r->d,.01f);
}
static unsigned pattern(char c) {for(unsigned p=0;p<512;++p)if(barcode_character(p)==c)return p;assert(false);return 0;}
int main(void) {
    Rig r;
    for(unsigned profile=1;profile<=3;profile+=2) {
        init(&r);for(unsigned n=0;n<400;++n)tick(&r,180,profile);
        assert(r.cruise.fast && r.d.target[1]==(profile==3?135:60));
        /* Same pipeline must instantly retain real optical urgent departures. */
        r.i.adc[1]=168;tick(&r,180,profile);assert(r.d.target[0]<=15 && r.d.target[1]<=50 && !r.cruise.active);
    }
    for(unsigned letter=0;letter<3;++letter)for(unsigned reverse=0;reverse<2;++reverse) {
        char code="AFZ"[letter];unsigned p[3]={0x094,pattern(code),0x094};double widths[29];
        for(unsigned g=0;g<3;++g)for(unsigned j=0;j<9;++j)widths[g*10+j]=p[g]&(1u<<(8-j))?9:3;
        widths[9]=widths[19]=3;
        if(reverse)for(unsigned j=0;j<14;++j) {double t=widths[j];widths[j]=widths[28-j];widths[28-j]=t;}
        init(&r);bool decoded=false,dispatched=false;
        for(unsigned n=0;n<1000;++n) {
            double x=r.i.path+.45-45;unsigned adc=180;
            if(x>=0)for(unsigned j=0;j<29;++j) {if(x<widths[j]) {adc=j%2?203:widths[j]<4?220:3000;break;}x-=widths[j];}
            if(r.i.path+.45>=216 && r.i.path+.45<234)adc=3000;
            tick(&r,adc,1);decoded|=r.n.pending==code;
            assert(!r.d.stop);
            if(r.c.busy) {assert(r.n.action==code && !r.n.pending && !r.cruise.active);dispatched=true;break;}
        }
        assert(decoded && dispatched);
    }
    puts("PASS composed V1.2 optical/cruise pipeline: steady unequal850/500 qualifies both profiles; urgent bend immediate; analogue A/F/Z both directions still decode and dispatch separate cross.");
}
