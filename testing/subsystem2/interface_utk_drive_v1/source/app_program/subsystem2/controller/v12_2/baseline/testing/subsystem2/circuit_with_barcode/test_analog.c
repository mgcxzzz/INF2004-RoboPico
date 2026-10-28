#include <assert.h>
#include <stdio.h>
#include "barcode_analog.h"
#include "barcode_record.h"
static const LineCalibration cal={{184,184},{3464,3692},7};
static unsigned code_pattern(char c) {
    for(unsigned p=0;p<512;++p) if(barcode_character(p)==c) return p;
    assert(false);return 0;
}
static double frame(char code,double narrow,double ratio,bool reverse,double width[29]) {
    unsigned pat[3]={0x094,code_pattern(code),0x094};double length=0;
    for(unsigned k=0;k<3;++k) {
        for(unsigned j=0;j<9;++j) width[k*10+j]=narrow*((pat[k]&(1u<<(8-j)))?ratio:1);
        if(k<2) width[k*10+9]=narrow;
    }
    if(reverse) for(unsigned j=0;j<14;++j) { double t=width[j];width[j]=width[28-j];width[28-j]=t; }
    for(unsigned j=0;j<29;++j) length+=width[j];return length;
}
/* Piecewise reflectance fixture: sensor voltages, not already classified edges. */
static uint16_t adc_at(double position,double start,const double width[29],unsigned fault) {
    double x=position-start;if(x<0) return 220;
    for(unsigned j=0;j<29;++j) {
        if(x<width[j]) {
            if(j%2) return 220;
            if(fault==1 && width[j]<4) return 220; /* narrow black bars physically unresolved */
            if(fault==2 && j==28) return 220; /* incomplete stop */
            return width[j]<4?1800:3600;
        }
        x-=width[j];
    }
    return 220;
}
static void capture_case(char code,double narrow,double ratio,bool reverse,unsigned fault) {
    BarcodeNav n;BarcodeAnalog a;CircuitInput i={.now=1000};barcode_nav_begin(&n,&i);
    barcode_analog_begin(&a,&n,220,i.now,i.wheel);n.allow_cross=true;
    double width[29],length=frame(code,narrow,ratio,reverse,width),start=20;
    for(unsigned step=0;i.wheel[0]<start+length+25;++step) {
        i.now+=2000;i.wheel[0]+=step%100<50?.06:.12;i.wheel[1]=i.wheel[0];
        uint16_t adc=adc_at(i.wheel[0],start,width,fault);
        adc=(uint16_t)(adc+(step%7)-3);
        barcode_analog_sample(&a,&n,adc,i.now,i.wheel);
        assert(!n.cross_ready); /* stop bars never become junctions */
    }
    if(fault) { assert(!n.pending && !n.decoded_a && !n.decoded_f);return; }
    if(code=='A' || code=='F' || code=='Z') {
        assert(n.pending==code && n.decoded_a+n.decoded_f+n.decoded_z==1 && a.selections==1);
        assert(a.candidates>=2);
    } else { assert(!n.pending && n.unsupported==1);return; }
    /* Cross is separate, independent of barcode-to-cross distance. */
    double cross=i.wheel[0]+40;
    while(i.wheel[0]<cross+23) {
        i.now+=2000;i.wheel[0]+=.06;i.wheel[1]=i.wheel[0];
        barcode_analog_sample(&a,&n,i.wheel[0]>=cross && i.wheel[0]<cross+18?3600:220,i.now,i.wheel);
    }
    assert(n.cross_ready && n.cross_gate && n.pending==code);
    Circuit c;i.path=i.wheel[0];i.adc[0]=3464;i.adc[1]=3692;i.speed[0]=i.speed[1]=30;
    circuit_begin(&c,&i);(void)barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(c.busy && n.action==code && !n.pending && c.phase==J_CENTRE);
}
static void conflict_and_signal_tests(void) {
    BarcodeNav n;BarcodeAnalog a;CircuitInput i={.now=1000};barcode_nav_begin(&n,&i);
    barcode_analog_begin(&a,&n,220,i.now,i.wheel);
    for(unsigned step=1;step<2000;++step) {
        i.now+=2000;i.wheel[0]=i.wheel[1]=step*.1;
        barcode_analog_sample(&a,&n,(uint16_t)(220+step%9),i.now,i.wheel);
    }
    assert(!n.pending && !n.decoded_a && !n.decoded_f && !n.cross_ready);
    /* One isolated stripe cannot create a command. */
    for(unsigned step=0;step<500;++step) {
        i.now+=2000;i.wheel[0]+=.1;i.wheel[1]=i.wheel[0];
        barcode_analog_sample(&a,&n,step<180?3600:220,i.now,i.wheel);
    }
    assert(!n.pending && !n.decoded_a && !n.decoded_f);
    /* Contradicting adjacent-bank groups for one frame are rejected. */
    for(unsigned k=0;k<4;++k) {
        a.bank[k].vote=k<2?'A':'F';a.bank[k].vote_start=100;a.bank[k].vote_end=240;a.bank[k].vote_us=i.now;
    }
    n.pending='A';assert(!barcode_analog_select(&a,&n,i.now,240));assert(!n.pending && a.conflict);
}
static void driving_simulation(unsigned profile) {
    Circuit c;BarcodeNav n;BarcodeAnalog a;
    CircuitInput i={.now=1000,.adc={3464,3692}};
    circuit_begin(&c,&i);barcode_nav_begin(&n,&i);barcode_analog_begin(&a,&n,220,i.now,i.wheel);
    double start=100,w[29],length=frame('A',3,3,false,w),gap=80;
    AnalogDemand d={.target={45,45}};unsigned complete=0;
    for(unsigned step=0;step<900000 && complete<4;++step) {
        for(unsigned k=0;k<2;++k) { i.speed[k]=d.target[k];i.wheel[k]+=i.speed[k]*.002;i.path+=fabs(i.speed[k])*.001; }
        i.now+=2000;
        uint16_t adc=adc_at(i.path,start,w,0);
        if(i.path>=start+length+gap && i.path<start+length+gap+18) adc=3600;
        barcode_analog_sample(&a,&n,adc,i.now,i.wheel);i.side_black=a.black;
        if(step%5) continue;
        i.adc[0]=3464;i.adc[1]=3692;
        if(c.busy && c.turn.phase==J_TURN) {
            unsigned left=n.action=='F'?1:0,right=1-left;
            double angle=((i.wheel[right]-c.edge_wheel[right]-c.turn.turn_origin[1])-
                          (i.wheel[left]-c.edge_wheel[left]-c.turn.turn_origin[0]))/TRACK_MM*180/3.141592653589793;
            if(angle>3 && angle<88) i.adc[0]=i.adc[1]=184;
            if(angle>=88) i.adc[right]=184;
        }
        if(c.busy && c.turn.phase==J_ACQUIRE) i.adc[n.action=='F'?0:1]=184;
        d=barcode_circuit_step(&c,&n,&cal,&i,profile);
        if(d.stop) fprintf(stderr,"AO SIM fault profile%u phase%u reason%u at%.1f pending%c\n",profile,c.phase,c.reason,i.path,n.pending);
        assert(!d.stop && !c.fault);
        if(c.completed) {
            ++complete;assert(c.turns==complete && n.lefts+n.rights==complete);
            start=i.path+250;gap=complete%2?30:200;
            length=frame(complete%2?'F':'A',complete%2?1.5:3,3,complete%2,w);
        }
    }
    assert(complete==4 && n.decoded_a==2 && n.decoded_f==2 && n.lefts==2 && n.rights==2);
}
int main(void) {
    for(unsigned size=0;size<2;++size) for(unsigned reverse=0;reverse<2;++reverse)
        for(unsigned letter=0;letter<2;++letter) for(unsigned ratio=2;ratio<=3;++ratio)
            capture_case(letter?'F':'A',size?3:1.5,ratio,reverse,0);
    capture_case('A',3,3,false,1);capture_case('A',3,3,false,2);capture_case('Z',3,3,false,0);capture_case('Q',3,3,false,0);
    conflict_and_signal_tests();
    for(unsigned profile=1;profile<=3;++profile) driving_simulation(profile);
    puts("PASS: raw ADC A/F both sizes/directions/ratios; changing speed; reduced narrow-bar amplitude; missing-bars/malformed-stop/noise rejection; unsupported code; conflicting votes; distance-qualified cross;4 alternating turns/profile.");
}
