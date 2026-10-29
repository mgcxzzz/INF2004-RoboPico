#include <assert.h>
#include <stdio.h>
#include "barcode_analog.h"
static double make_frame(char c,bool reverse,double widths[29]) {
    uint16_t patterns[3]={0x094,0,0x094};
    for(unsigned p=0;p<512;++p) if(barcode_character((uint16_t)p)==c) patterns[1]=(uint16_t)p;
    unsigned count=0;double total=0;
    for(unsigned ch=0;ch<3;++ch) {
        for(unsigned bit=0;bit<9;++bit) widths[count++]=patterns[ch]&(1u<<(8-bit))?9:3;
        if(ch<2) widths[count++]=3;
    }
    if(reverse) for(unsigned n=0;n<14;++n) { double v=widths[n];widths[n]=widths[28-n];widths[28-n]=v; }
    for(unsigned n=0;n<29;++n) total+=widths[n];return total;
}
static void measured_frame(char code,bool reverse,bool missing) {
    BarcodeAnalog a;BarcodeNav n;CircuitInput i={.now=1000};barcode_nav_begin(&n,&i);
    barcode_analog_begin(&a,&n,181,i.now,i.wheel);double w[29],total=make_frame(code,reverse,w);
    const unsigned bar[]={221,220,215,218,219},gap[]={204,206,203,205,206};
    for(unsigned step=0;i.wheel[0]<total+40;++step) {
        i.now+=2000;i.wheel[0]+=.06;i.wheel[1]=i.wheel[0];double x=i.wheel[0]-20;uint16_t adc=181;
        if(x>=0 && x<total) for(unsigned run=0;run<29;++run) {
            if(x<w[run]) { adc=run%2?(w[run]<4?gap[step%5]:181):
                w[run]<4?(missing?181:bar[step%5]):1800;break; }
            x-=w[run];
        }
        barcode_analog_sample(&a,&n,adc,i.now,i.wheel);
    }
    if(missing) assert(!n.decoded_a && !n.decoded_f && !n.pending && !a.selections);
    else {
        assert(n.last_code==code && a.candidates>=2 && a.selections==1);
        assert(a.selected>=1 && a.selected<=32);
        if(code=='A' || code=='F' || code=='Z') assert(n.pending==code && n.decoded_a+n.decoded_f+n.decoded_z==1);
        else assert(!n.pending && n.unsupported==1);
    }
}
int main(void) {
    assert(AO_LEGACY_BANKS==63 && AO_BANKS==65 && barcode_analog_threshold(AO_DEFAULT_BANK)==213);
    for(unsigned b=1;b<AO_LEGACY_BANKS;++b) assert(barcode_analog_threshold(b)>barcode_analog_threshold(b-1));
    assert(barcode_analog_threshold(63)==204 && barcode_analog_threshold(64)==205);
    assert(barcode_analog_hysteresis(63)==1 && barcode_analog_hysteresis(64)==1);
    for(unsigned t=128;t<=3968;t+=128) {
        bool found=false;for(unsigned b=0;b<AO_BANKS;++b) if(barcode_analog_threshold(b)==t)
            found=barcode_analog_hysteresis(b)==32;
        assert(found);
    }
    assert(barcode_analog_hysteresis(11)==2);
    assert(barcode_analog_log_threshold(7,15)==2048 && barcode_analog_log_threshold(8,11)==212 && barcode_analog_log_threshold(30,11)==212 && barcode_analog_log_threshold(31,11)==213);
    for(unsigned rev=0;rev<2;++rev) {
        measured_frame('A',rev,false);measured_frame('F',rev,false);measured_frame('Z',rev,false);measured_frame('Q',rev,false);
        measured_frame('A',rev,true);
    }
    puts("PASS: measured215..221 bars/203..206 gaps decode complete A/F/Z both ways; missing narrow bars reject; original thresholds retained; fine grid sorted; legacy log threshold mapping.");
}
