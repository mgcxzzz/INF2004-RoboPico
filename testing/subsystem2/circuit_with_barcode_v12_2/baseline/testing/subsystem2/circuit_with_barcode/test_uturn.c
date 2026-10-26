#include <assert.h>
#include <stdio.h>
#include "smooth_follow.h"
#include "barcode_analog.h"
#include "barcode_record.h"
static const LineCalibration cal={{184,184},{3464,3692},7};
static double frame_widths(char code,double narrow,bool reverse,double w[29]) {
    uint16_t pat[3]={0x094,0,0x094};
    for(unsigned p=0;p<512;++p)if(barcode_character((uint16_t)p)==code)pat[1]=(uint16_t)p;
    double length=0;
    for(unsigned k=0;k<3;++k) {
        for(unsigned n=0;n<9;++n)w[k*10+n]=narrow*((pat[k]&(1u<<(8-n)))?3:1);
        if(k<2)w[k*10+9]=narrow;
    }
    if(reverse)for(unsigned n=0;n<14;++n) {double t=w[n];w[n]=w[28-n];w[28-n]=t;}
    for(unsigned n=0;n<29;++n)length+=w[n];return length;
}
static void decode_z(BarcodeNav *nav,BarcodeAnalog *a,CircuitInput *i,unsigned size,bool reverse) {
    double w[29],length=frame_widths('Z',size?6:3,reverse,w),start=i->path+20;
    barcode_analog_begin(a,nav,180,i->now,i->wheel);
    while(i->path<start+length+15) {
        i->now+=2000;i->path+=.08;i->wheel[0]+=.08;i->wheel[1]+=.08;
        double x=i->path-start;uint16_t adc=180;
        if(x>=0 && x<length)for(unsigned n=0;n<29;++n) {
            if(x<w[n]) {adc=n%2?180:w[n]<(size?9:4.5)?220:2600;break;}
            x-=w[n];
        }
        barcode_analog_sample(a,nav,adc,i->now,i->wheel);
    }
    assert(nav->pending=='Z' && nav->decoded_z==1 && !nav->unsupported && a->selections==1);
    assert(!nav->cross_ready && !nav->cross_candidate);
}
static bool black_at(double x,double y) {return fabs(y)<=9 || fabs(x)<=9;}
static void kinematic_trial(unsigned size,bool reverse,bool hide_return,bool lag) {
    CircuitInput i={.now=1000};Circuit c;BarcodeNav nav;BarcodeAnalog a;SmoothFollow smooth={0};
    barcode_nav_begin(&nav,&i);decode_z(&nav,&a,&i,size,reverse);circuit_begin(&c,&i);
    double x=-250,y=0,theta=0,motor[2]={45,45};AnalogDemand d={.target={45,45}};
    bool first_branch_seen=false,reverse_seen=false,completed=false;unsigned stages=0,last_stage=0;
    for(unsigned step=0;step<15000 && !completed;++step) {
        for(unsigned w=0;w<2;++w) {
            double rate=d.target[w]==0?350:500;
            motor[w]=lag?motor[w]+clampf((float)(d.target[w]-motor[w]),(float)(-rate*.01),(float)(rate*.01)):d.target[w];
        }
        double dl=motor[0]*.01,dr=motor[1]*.01;
        double angle=(dr-dl)/TRACK_MM,mid=theta+angle*.5,travel=(dl+dr)*.5;
        x+=travel*cos(mid);y+=travel*sin(mid);theta+=angle;
        i.wheel[0]+=dl;i.wheel[1]+=dr;i.path+=(fabs(dl)+fabs(dr))*.5;
        i.speed[0]=(float)motor[0];i.speed[1]=(float)motor[1];i.now+=10000;
        for(unsigned w=0;w<2;++w) {
            double lateral=w?-5:5;
            bool black=black_at(x+127*cos(theta)-lateral*sin(theta),y+127*sin(theta)+lateral*cos(theta));
            if(hide_return && theta>2.4)black=false;
            i.adc[w]=black?cal.black[w]:cal.white[w];
        }
        double sx=x+92*cos(theta)-35*sin(theta),sy=y+92*sin(theta)+35*cos(theta);
        barcode_analog_sample(&a,&nav,black_at(sx,sy)?2600:180,i.now,i.wheel);i.side_black=a.black;
        d=barcode_drive_step(&c,&nav,&cal,&i,1,&smooth,.01f);
        if(c.busy && nav.uturn.stage==0 && theta>1.4 && (i.adc[0]>1000 || i.adc[1]>1000)) {
            first_branch_seen=true;assert(c.phase!=J_ACQUIRE && !c.completed);
        }
        if(nav.uturn.stage!=last_stage) {stages|=1u<<nav.uturn.stage;last_stage=nav.uturn.stage;}
        if(d.target[0]<0) {reverse_seen=true;assert(!d.target[1] && nav.action=='Z' && c.phase==J_TURN);}
        if(c.busy && c.phase==J_CENTRE)assert(c.edge_path<=i.path && i.path-c.edge_path<100);
        if(d.stop) {
            if(!hide_return)fprintf(stderr,"U fault=%u stage=%u angle=%.1f x=%.1f y=%.1f\n",c.reason,nav.uturn.stage,theta*180/3.141592653589793,x,y);
            assert(hide_return && c.fault && !nav.uturns && !nav.pending);break;
        }
        if(c.completed) {
            completed=true;assert(nav.uturns==1 && !nav.lefts && !nav.rights && c.turns==1 && !nav.busy && !nav.pending);
            assert(c.turn.turn_degrees>=165 && c.turn.turn_degrees<195 && c.align_ms>=150);
            assert(c.turn_ms<6500); /* Ideal/lag fixtures, not a physical timing guarantee. */
            assert(fabs(y)<15 && x>0 && theta>2.8 && theta<3.4);
            assert(d.target[0]>=0 && d.target[1]>=0);
        }
    }
    assert(reverse_seen && first_branch_seen);
    if(hide_return)assert(!completed && c.fault);
    else {
        assert(completed && stages==30);
        unsigned complete_events=0;
        for(unsigned n=0;n<nav.event_count;++n) {
            BarcodeEvent *e=&nav.events[n];
            if(e->type==B_COMPLETE) {++complete_events;assert(e->code=='Z' && e->action=='U');}
        }
        assert(complete_events==1);
        printf("U geometry: x=%.1f y=%.1f angle=%.1f travel=%.1f turn_ms=%u\n",x,y,theta*180/3.141592653589793,i.path,c.turn_ms);
        /* Following the next crossing after reverse travel uses cumulative
         * path for the local manoeuvre and signed positions for its geometry. */
        i.path+=200;i.wheel[0]+=200;i.wheel[1]+=200;i.now+=5000000;
        nav.pending='A';nav.cross_ready=nav.cross_gate=true;nav.cross_width=18;
        nav.cross_us=i.now-100000;nav.cross_wheel[0]=i.wheel[0]-22;nav.cross_wheel[1]=i.wheel[1]-22;
        i.adc[0]=cal.black[0];i.adc[1]=cal.black[1];
        d=barcode_drive_step(&c,&nav,&cal,&i,1,&smooth,.01f);
        assert(c.busy && nav.action=='A' && fabs(c.edge_path-(i.path-22))<.01 && !d.stop);
    }
}
static void limits_and_saved_record(void) {
    assert(barcode_travel_ticks(INT32_MIN)==2147483648ull && barcode_travel_ticks(0)==0);
    assert(barcode_travel_ticks(100)+barcode_travel_ticks(-100)==200);
    Junction j={.phase=J_TURN};BarcodeUTurn u={0};JunctionInput i={.now=1000};
    AnalogDemand d=barcode_uturn_step(&u,&j,&cal,&i);assert(d.target[1]==U_FORWARD_FAST);
    i.now+=U_SEGMENT_US;d=barcode_uturn_step(&u,&j,&cal,&i);assert(d.stop && j.reason==J_TURN_TIMEOUT);
    j=(Junction){.phase=J_TURN};u=(BarcodeUTurn){0};i=(JunctionInput){.now=1000,.wheel={13,20}};
    d=barcode_uturn_step(&u,&j,&cal,&i);assert(d.stop && j.reason==J_CENTRE_DRIFT);
    j=(Junction){.phase=J_TURN_SETTLE,.phase_us=1000};u=(BarcodeUTurn){.stage=1};
    i=(JunctionInput){.now=1000+U_SETTLE_US,.speed={20,20}};
    d=barcode_uturn_step(&u,&j,&cal,&i);assert(d.stop && j.reason==J_SETTLE_TIMEOUT);
    SignedWheel wheel={0};int pwm=boosted_junction_step(&wheel,0,-35,0,.01f,1000,true);
    assert(pwm<0 && pwm>=-480);DualLevels levels=boosted_junction_levels(0,pwm,true);
    assert(!levels.a && levels.b==(unsigned)-pwm);
    static BarcodeSaved saved;saved.magic=B_MAGIC;saved.profile=saved.recorded_profile=1;
    saved.calibration=cal;saved.side=(SideCalibration){0,1,3};
    for(unsigned version=1;version<=18;++version) {
        saved.version=version;saved.decoded_z=saved.uturns=version>=11?3:0;
        saved.crc=log_crc((const uint8_t *)&saved+offsetof(BarcodeSaved,count),barcode_record_crc_size(version));
        assert(barcode_record_valid(&saved));
    }
    saved.decoded_z^=1;assert(!barcode_record_valid(&saved));
    assert(barcode_record_crc_size(10)+8==barcode_record_crc_size(11));
}
int main(void) {
    for(unsigned size=0;size<2;++size)for(unsigned rev=0;rev<2;++rev)kinematic_trial(size,rev,false,false);
    kinematic_trial(0,false,true,false);kinematic_trial(0,false,false,true);limits_and_saved_record();
    puts("PASS: complete analogue Z both sizes/directions; queued crossing; 90deg arm ignored; reverse quarter and settled forward rejoin; original-line geometry; missing-return/time/drift rejection; signed PWM; next junction after reverse; V1..V12 record integrity.");
}
