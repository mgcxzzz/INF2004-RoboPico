#include <assert.h>
#include <stdio.h>
#include "scan_hold.h"
static const LineCalibration cal={{164,168},{3514,3389},7};
typedef struct {Circuit c;BarcodeNav n;BarcodeAnalog a;SmoothFollow s;ScanHold h;CircuitInput i;AnalogDemand d;} Rig;
static void init(Rig *r) {
    memset(r,0,sizeof(*r));r->i=(CircuitInput){.now=1000,.adc={3514,3389},.speed={45,45}};
    circuit_begin(&r->c,&r->i);barcode_nav_begin(&r->n,&r->i);
    barcode_analog_begin(&r->a,&r->n,180,r->i.now,r->i.wheel);
}
static void tick(Rig *r,unsigned adc,bool moving) {
    r->i.now+=10000;
    if(moving)for(unsigned w=0;w<2;++w)r->i.wheel[w]+=r->i.speed[w]*.01;
    r->i.path=(r->i.wheel[0]+r->i.wheel[1])*.5;
    /* Real acquisition is2ms; execute five actual decoder samples/control tick. */
    for(unsigned j=0;j<5;++j)barcode_analog_sample(&r->a,&r->n,adc,r->i.now-8000+j*2000,r->i.wheel);
    r->i.side_black=r->a.black;
    r->d=scan_drive_step(&r->c,&r->n,&cal,&r->i,1,&r->s,&r->a,&r->h,.01f);
}
static void ready(Rig *r) {for(unsigned k=0;k<60;++k)tick(r,180,true);assert(r->h.steady_timing);}
static unsigned exit_reason(const Rig *r) {
    for(unsigned k=0;k<r->n.event_count;++k) {
        const BarcodeEvent *e=&r->n.events[(r->n.event_head+B_EVENTS-1-k)%B_EVENTS];
        if(e->type==B_HOLD_END || e->type==B_HOLD_CANCEL)return e->flags;
    }
    return 0;
}
static void start(Rig *r) {ready(r);tick(r,220,true);tick(r,220,true);assert(r->h.active);}
static void parity(void) {
    Rig r;init(&r);Circuit c=r.c;BarcodeNav n=r.n;SmoothFollow s={0};
    for(unsigned k=0;k<900;++k) {
        unsigned part=k/150;r.i.adc[0]=part==1?164:part==3?1100:3514;
        r.i.adc[1]=part==2?168:part==4?1200:3389;
        tick(&r,180,true);
        AnalogDemand d=barcode_drive_step(&c,&n,&cal,&r.i,1,&s,.01f);
        assert(!r.h.active && d.target[0]==r.d.target[0] && d.target[1]==r.d.target[1]);
        assert(d.stop==r.d.stop && d.decision==r.d.decision && c.phase==r.c.phase);
    }
}
static void limits(void) {
    Rig r;init(&r);start(&r);
    for(unsigned k=0;k<100;++k) {
        r.i.adc[0]=k%2?164:3514;r.i.adc[1]=k%2?3389:168;
        tick(&r,220,true);assert(r.h.active && r.d.target[0]==45 && r.d.target[1]==45);
    }
    r.i.wheel[1]+=4;tick(&r,220,true);tick(&r,220,true);
    assert(r.d.target[0]>r.d.target[1] && r.d.target[0]<=53 && r.d.target[1]>=37);
    r.i.wheel[1]+=20;tick(&r,220,true);assert(!r.h.active && exit_reason(&r)==SH_YAW);
    init(&r);start(&r);r.i.adc[0]=164;r.i.adc[1]=168;
    for(unsigned k=0;k<16;++k)tick(&r,220,true);
    assert(!r.h.active && exit_reason(&r)==SH_WHITE);
    init(&r);start(&r);r.i.speed[0]=r.i.speed[1]=0;
    for(unsigned k=0;k<501;++k)tick(&r,220,false);
    assert(!r.h.active && exit_reason(&r)==SH_TIME);
    init(&r);start(&r);r.i.wheel[0]+=181;r.i.wheel[1]+=181;tick(&r,220,false);
    assert(!r.h.active && exit_reason(&r)==SH_DISTANCE);
    init(&r);start(&r);for(unsigned k=0;k<10;++k)tick(&r,220,true);
    for(unsigned k=0;k<30;++k)tick(&r,180,true);
    assert(!r.h.active && exit_reason(&r)==SH_QUIET && !r.n.pending && !r.c.busy);
    /* Brief/noisy marks cannot latch, and a bend cannot arm a hold. */
    init(&r);ready(&r);tick(&r,220,true);tick(&r,180,true);tick(&r,180,true);assert(!r.h.active);
    init(&r);ready(&r);r.i.speed[0]=0;tick(&r,220,true);tick(&r,220,true);assert(!r.h.active);
}
static void heading_model(void) {
    for(unsigned side=0;side<2;++side) {
        Rig r;init(&r);start(&r);double hold_wheel[2]={r.h.wheel[0],r.h.wheel[1]};
        for(unsigned k=0;k<250;++k) {
            for(unsigned w=0;w<2;++w)
                r.i.speed[w]+=.12f*(r.d.target[w]+(w==side?-3:0)-r.i.speed[w]);
            tick(&r,220,true);assert(r.h.active && !r.d.stop);
        }
        double error=(r.i.wheel[1]-hold_wheel[1])-(r.i.wheel[0]-hold_wheel[0]);
        assert(fabs(error)<2); /* compare7.5mm open-loop drift from3mm/s bias */
    }
}
static unsigned pattern(char c) {
    for(unsigned x=0;x<512;++x)if(barcode_character(x)==c)return x;
    assert(false);return 0;
}
static void barcode_case(char code,bool reverse,bool missing) {
    Rig r;init(&r);double widths[29];unsigned patterns[3]={0x094,pattern(code),0x094};
    for(unsigned g=0;g<3;++g)for(unsigned j=0;j<9;++j)widths[g*10+j]=patterns[g]&(1u<<(8-j))?9:3;
    widths[9]=widths[19]=3;
    if(reverse)for(unsigned j=0;j<14;++j) {double v=widths[j];widths[j]=widths[28-j];widths[28-j]=v;}
    unsigned holds=0;bool before=false,released=false,dispatched=false;
    for(unsigned k=0;k<1000;++k) {
        double x=r.i.path+.45-45;unsigned adc=180;
        if(x>=0)for(unsigned j=0;j<29;++j) {
            if(x<widths[j]) {adc=j%2?203:widths[j]<4?220:3000;
                if(missing && j==12)adc=203;break;}
            x-=widths[j];
        }
        double cross=45+141+30;
        if(r.i.path+.45>=cross && r.i.path+.45<cross+18)adc=3000;
        tick(&r,adc,true);
        if(r.h.active && !before)++holds;
        if(before && !r.h.active && exit_reason(&r)==SH_DECODED)released=true;
        before=r.h.active;
        if(r.c.busy) {dispatched=true;assert(r.n.action==code && !r.n.pending && !r.h.active);break;}
        assert(!r.d.stop);
    }
    if(missing)assert(!r.n.decoded_a && !r.n.pending && !dispatched);
    else {assert(holds>=1 && released && dispatched);
        assert(r.n.decoded_a+r.n.decoded_f+r.n.decoded_z==1);}
}
int main(void) {
    parity();limits();heading_model();
    for(unsigned r=0;r<2;++r) {barcode_case('A',r,false);barcode_case('F',r,false);barcode_case('Z',r,false);}
    barcode_case('A',false,true);
    puts("PASS V12.1: non-scan V12 target parity; tentative entry/noise/bend gate; optical suppression; trim sign/bounds; white/yaw/distance/time/quiet exits; A/F/Z complete decode both directions; pending/dispatch priority; missing bar rejection.");
}
