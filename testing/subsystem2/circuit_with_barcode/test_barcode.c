#include <assert.h>
#include <stdio.h>
#include "barcode_record.h"
#include "smooth_follow.h"
#include "barcode_capture.h"
#include "../circuit_left_test/circuit_wheel.h"
#include "fixtures/pdf_runs.h"
#include "fixtures/captured_a.h"
#include "fixtures/captured_a_v2.h"
static const LineCalibration cal={{184,184},{3464,3692},7};
static uint16_t encoding(char code) {
    for(unsigned p=0;p<512;++p) if(barcode_character((uint16_t)p)==code) return (uint16_t)p;
    assert(false);return 0;
}
static double make_frame(char code,double narrow,double ratio,bool reverse,double w[29]) {
    uint16_t pattern[3]={0x094,encoding(code),0x094};double total=0;
    for(unsigned c=0;c<3;++c) {
        for(unsigned j=0;j<9;++j) w[c*10+j]=narrow*((pattern[c]&(1u<<(8-j)))?ratio:1);
        if(c<2) w[c*10+9]=narrow;
    }
    if(reverse) for(unsigned j=0;j<14;++j) { double v=w[j];w[j]=w[28-j];w[28-j]=v; }
    for(unsigned j=0;j<29;++j) total+=w[j];
    return total;
}
static BarcodeRead read_frame(const double w[29],char expected) {
    BarcodeReader r;uint64_t now=1000;double mm=0;
    barcode_reader_reset(&r,false,now,mm);
    now+=100000;mm=20;(void)barcode_reader_edge(&r,true,now,mm);
    BarcodeRead result={0};
    for(unsigned j=0;j<29;++j) {
        mm+=w[j];now+=(uint64_t)(w[j]/(j%3?37:80)*1e6); /* changing speed */
        result=barcode_reader_edge(&r,j%2==1,now,mm);
        if(j<28) assert(!(result.events&(1u<<B_DECODE)));
    }
    assert((result.events&(1u<<B_DECODE)) && result.code==expected && !r.frame);
    return result;
}
static void decoder_tests(void) {
    assert(encoding('A')==0x109 && encoding('F')==0x058 && encoding('Z')==0x0d0);
    for(unsigned size=0;size<2;++size) for(unsigned reverse=0;reverse<2;++reverse)
        for(unsigned letter=0;letter<2;++letter) {
            double w[29];make_frame(letter?'F':'A',size?3:1.5,3,reverse,w);
            BarcodeRead r=read_frame(w,letter?'F':'A');assert(r.reverse==(bool)reverse);
            for(unsigned j=0;j<29;++j) w[j]*=j%3==0?1.08:j%3==1?.94:1.02;
            (void)read_frame(w,letter?'F':'A');
        }
    for(unsigned page=0;page<4;++page) {
        (void)read_frame(pdf_widths[page],pdf_codes[page]);double reverse[29];
        for(unsigned j=0;j<29;++j) reverse[j]=pdf_widths[page][28-j];
        assert(read_frame(reverse,pdf_codes[page]).reverse);
    }
    /* Actual large A has8mm narrow black vs2mm narrow white. The preceding
       V1 global-width classifier rejects it; independent colour learning
       must decode the measured data, including its uneven final gaps. */
    (void)read_frame(captured_a_widths,'A');double reversed[29];
    for(unsigned n=0;n<29;++n) reversed[n]=captured_a_widths[28-n];
    assert(read_frame(reversed,'A').reverse);
    (void)read_frame(captured_a_v2_widths,'A');
    for(unsigned n=0;n<29;++n) reversed[n]=captured_a_v2_widths[28-n];
    assert(read_frame(reversed,'A').reverse);
    for(unsigned size=0;size<2;++size) for(unsigned reverse=0;reverse<2;++reverse)
        for(unsigned letter=0;letter<2;++letter) {
            double biased[29],base=size?6:3;make_frame(letter?'F':'A',base,3,reverse,biased);
            for(unsigned n=0;n<29;++n) biased[n]+=n%2?-base*2/3:base/3;
            (void)read_frame(biased,letter?'F':'A');
        }
    /* No symbol is decoded from one wide stripe, a malformed stop or noise. */
    BarcodeReader r;barcode_reader_reset(&r,false,1000,0);
    (void)barcode_reader_edge(&r,true,2000,20);
    BarcodeRead result=barcode_reader_edge(&r,false,400000,38);
    assert(!(result.events&(1u<<B_DECODE)) && !r.frame);
    double w[29];make_frame('F',3,3,false,w);w[28]=9;
    barcode_reader_reset(&r,false,1000,0);double mm=20;
    (void)barcode_reader_edge(&r,true,2000,mm);
    for(unsigned j=0;j<29;++j) { mm+=w[j];result=barcode_reader_edge(&r,j%2==1,3000+j*100000,mm); }
    assert(!(result.events&(1u<<B_DECODE)) && (result.events&(1u<<B_BAD_FRAME)));
    make_frame('A',3,3,false,w);barcode_reader_reset(&r,false,1000,0);mm=20;
    (void)barcode_reader_edge(&r,true,2000,mm);
    for(unsigned j=0;j<9;++j) { mm+=w[j];(void)barcode_reader_edge(&r,j%2==1,3000+j*100000,mm); }
    assert(r.frame && barcode_reader_expire(&r,30000000,mm) && !r.frame);
    /* A valid-looking pattern still needs plausible widths and gaps.
       Diagnostics must retain complete094/109/094 even on rejection. */
    for(unsigned fault=0;fault<2;++fault) {
        make_frame('A',3,3,false,w);if(fault) w[9]=9;else w[10]=32;
        barcode_reader_reset(&r,false,1000,0);mm=20;
        (void)barcode_reader_edge(&r,true,2000,mm);
        for(unsigned j=0;j<29;++j) { mm+=w[j];result=barcode_reader_edge(&r,j%2==1,3000+j*100000,mm); }
        assert((result.events&(1u<<B_BAD_FRAME)) && !(result.events&(1u<<B_DECODE)));
        assert(result.rejection&(fault?B_REJECT_GAP:B_REJECT_WIDTH));
        assert(result.pattern[0]==0x094 && result.pattern[1]==0x109 && result.pattern[2]==0x094);
    }
}
static void capture_tests(void) {
    BarcodeCapture c={0};BarcodeEdge e;barcode_capture_reset(&c,false,1000);
    barcode_capture_tick(&c,2000,true,12,14);barcode_capture_tick(&c,3000,false,13,15);
    assert(!barcode_capture_pop(&c,&e));
    barcode_capture_tick(&c,5000,true,20,21);barcode_capture_tick(&c,6000,true,22,23);
    assert(!barcode_capture_pop(&c,&e));barcode_capture_tick(&c,7000,true,24,25);
    assert(barcode_capture_pop(&c,&e) && e.us==5000 && e.ticks[0]==20 && e.ticks[1]==21 && e.black);
    assert(!barcode_capture_pop(&c,&e));
    barcode_capture_reset(&c,false,1000);
    for(unsigned n=0;n<256;++n) {
        bool black=(n%2)==0;uint64_t now=5000+(uint64_t)n*4000;
        barcode_capture_tick(&c,now,black,n,n);barcode_capture_tick(&c,now+2000,black,n+1,n+1);
    }
    assert(c.overflow==1);unsigned count=0;while(barcode_capture_pop(&c,&e)) ++count;assert(count==255);
}
static void feed_nav_frame(BarcodeNav *n,char code,double narrow,bool reverse,double *mm,uint64_t *now) {
    double w[29];make_frame(code,narrow,3,reverse,w);
    double wheel[2]={*mm,*mm};barcode_nav_edge(n,*now,wheel,true);
    for(unsigned j=0;j<29;++j) {
        *mm+=w[j];*now+=(uint64_t)(w[j]/45*1e6);wheel[0]=wheel[1]=*mm;
        barcode_nav_edge(n,*now,wheel,j%2==1);
        if(j<28) assert(!n->cross_ready);
    }
}
static void queue_tests(void) {
    BarcodeNav n;CircuitInput i={.now=1000};barcode_nav_begin(&n,&i);
    double mm=20;uint64_t now=100000;
    feed_nav_frame(&n,'A',3,false,&mm,&now);assert(n.pending=='A' && n.decoded_a==1 && !n.cross_ready);
    mm+=1;now+=30000;feed_nav_frame(&n,'F',1.5,true,&mm,&now);
    assert(n.pending=='F' && n.decoded_f==1 && !n.cross_ready); /* latest supported command */
    mm+=20;now+=500000;feed_nav_frame(&n,'Z',3,false,&mm,&now);
    assert(n.unsupported==1 && n.last_code=='Z' && n.pending=='F');
    n.allow_cross=true;mm+=200;now+=5000000;
    double wheel[2]={mm,mm};barcode_nav_edge(&n,now,wheel,true);
    mm+=18;now+=400000;wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,false);
    assert(n.cross_candidate && !n.cross_ready && n.cross_gate && n.cross_width==18 && n.pending=='F');
    barcode_cross_confirm(&n,now+100000,mm+3.99,true);assert(!n.cross_ready);
    barcode_cross_confirm(&n,now+110000,mm+4,true);assert(n.cross_ready && !n.cross_candidate);
    /* An incomplete frame's oversized run is rejected rather than dispatched. */
    barcode_nav_begin(&n,&i);double w[29];make_frame('A',3,3,false,w);mm=20;now=100000;
    wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,true);
    for(unsigned j=0;j<9;++j) { mm+=w[j];now+=100000;wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,j%2==1); }
    assert(n.reader.frame);mm+=3;now+=100000;wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,true);
    mm+=45;now+=1000000;wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,false);
    assert(!n.cross_ready && n.rejected==1);
}
static void captured_a_replay(unsigned source) {
    BarcodeNav n;CircuitInput i={.now=1000};barcode_nav_begin(&n,&i);n.allow_cross=true;
    const double *widths=source==1?captured_a_widths:captured_a_v2_widths;
    const uint32_t *timestamps=source==1?captured_a_ms:captured_a_v2_ms;
    double mm=source==1?793.09:664.98,wheel[2]={mm,mm};uint64_t now=source==1?29038000:25915000;
    barcode_nav_edge(&n,now,wheel,true);
    for(unsigned run=0;run<29;++run) {
        double next=mm+widths[run];uint64_t end=(uint64_t)timestamps[run]*1000;
        /* Replay10ms control-time expiry checks as well as the raw edges.
           This catches the former210mm total-frame limit. */
        for(uint64_t t=now+10000;t<end;t+=10000) {
            double pos=mm+(next-mm)*(double)(t-now)/(double)(end-now);
            assert(!barcode_reader_expire(&n.reader,t,pos));
        }
        now=end;mm=next;wheel[0]=wheel[1]=mm;
        barcode_nav_edge(&n,now,wheel,run%2==1);
        if(run<28) assert(!n.pending);
        assert(!n.cross_ready && !n.cross_candidate);
    }
    assert(n.pending=='A' && n.decoded_a==1 && !n.rejected);
    unsigned starts=0,decodes=0,crosses=0;
    for(unsigned j=0;j<n.event_count;++j) {
        starts+=n.events[j].type==B_FRAME_START;decodes+=n.events[j].type==B_DECODE;
        crosses+=n.events[j].type==B_CROSS || n.events[j].type==B_CROSS_CANDIDATE;
        if(n.events[j].type==B_DECODE) {
            assert(n.events[j].pattern[0]==0x094 && n.events[j].pattern[1]==0x109 && n.events[j].pattern[2]==0x094);
            printf("CAPTURE V%u REPLAY: DECODE A/LEFT at%.3fs; patterns094,109,094; span%.2fmm\n",source,(double)n.events[j].ms/1000,(double)n.events[j].width100/100);
        }
    }
    assert(starts==1 && decodes==1 && !crosses);
    /* Both actual crosses are smaller than some captured barcode bars. */
    double gap=source==1?113.29:113.80,cross=source==1?16.27:15.75;
    mm+=gap;now=source==1?39228000:35601000;wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,true);
    mm+=cross;now=source==1?39880000:35952000;wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,false);
    assert(n.cross_candidate && !n.cross_ready && n.pending=='A');
    barcode_cross_confirm(&n,now+100000,mm+4,true);assert(n.cross_ready);
    Circuit c;i=(CircuitInput){.now=now+100000,.path=mm+4,.wheel={mm+4,mm+4},.adc={3464,3692},.speed={30,30}};
    circuit_begin(&c,&i);(void)barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(c.busy && n.action=='A' && !n.pending && c.phase==J_CENTRE);
    printf("CAPTURE V%u REPLAY: following%.2fmm cross confirmed after4mm white; dispatched A/LEFT once.\n",source,cross);
    /* A second black bar inside the white confirmation cancels a candidate
       and retains the instruction, rather than forcing a turn mid-barcode. */
    barcode_nav_begin(&n,&i);n.pending='A';mm+=200;now+=5000000;
    wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,true);
    mm+=18;now+=400000;wheel[0]=wheel[1]=mm;barcode_nav_edge(&n,now,wheel,false);
    assert(n.cross_candidate);mm+=3;now+=100000;wheel[0]=wheel[1]=mm;
    barcode_nav_edge(&n,now,wheel,true);
    assert(!n.cross_ready && !n.cross_candidate && n.pending=='A');
}
static void navigation_simulation(unsigned profile) {
    SmoothFollow smoothing={0};Circuit c;BarcodeNav nav;CircuitInput i={.now=1000,.adc={3464,3692}};
    circuit_begin(&c,&i);barcode_nav_begin(&nav,&i);
    double start=100,w[29],length=make_frame('A',3,3,false,w),gap=100;
    bool old=false,acquired=false;unsigned completed=0;
    AnalogDemand d={.target={45,45}};
    for(unsigned step=0;step<1500000 && completed<12;++step) {
        for(unsigned k=0;k<2;++k) { i.speed[k]=d.target[k];i.wheel[k]+=i.speed[k]*.001;i.path+=fabs(i.speed[k])*.0005; }
        i.now+=1000;double pos=i.path;
        bool black=false;
        if(pos>=start && pos<start+length) {
            double x=pos-start;unsigned run=0;while(run<28 && x>=w[run]) { x-=w[run];++run; }
            black=run%2==0;
        } else if(pos>=start+length+gap && pos<start+length+gap+18) black=true;
        i.side_black=black;
        if(black!=old) { barcode_nav_edge(&nav,i.now,i.wheel,black);old=black; }
        if(step%10) continue;
        i.adc[0]=3464;i.adc[1]=3692;
        if(c.busy && c.turn.phase==J_TURN) {
            unsigned a=nav.action=='F'?1:0,b=1-a;
            double angle=((i.wheel[b]-c.edge_wheel[b]-c.turn.turn_origin[1])-
                          (i.wheel[a]-c.edge_wheel[a]-c.turn.turn_origin[0]))/TRACK_MM*180/3.141592653589793;
            if(angle>3 && angle<88) i.adc[0]=i.adc[1]=184;
            if(angle>=88) i.adc[b]=184;
        }
        if(c.busy && c.turn.phase==J_ACQUIRE) {
            acquired=true;unsigned b=nav.action=='F'?0:1;i.adc[b]=184;
        }
        d=barcode_drive_step(&c,&nav,&cal,&i,profile,&smoothing,.01f);
        if(d.stop) fprintf(stderr,"profile%u phase%u reason%u pos%.2f time%.2f pending%c action%c\n",profile,c.phase,c.reason,i.path,(double)i.now/1e6,nav.pending,nav.action);
        assert(!d.stop && !c.fault);
        if(c.phase==J_TURN) {
            if(nav.action=='A') assert(!d.target[0] && d.target[1]>0);
            else assert(!d.target[1] && d.target[0]>0);
        }
        if(c.completed) {
            ++completed;assert(acquired && !nav.pending && c.turns==completed && c.align_ms>=150);
            assert(nav.lefts+nav.rights==completed);
            /* Returned tracker and command must be in the real robot frame. */
            if(nav.action=='A') assert(d.target[0]<d.target[1]);else assert(d.target[0]>d.target[1]);
            acquired=false;start=i.path+250;
            gap=completed%3==0?.5:completed%3==1?300:30;
            length=make_frame(completed%2?'F':'A',completed%3?1.5:3,3,completed%2,w);
        }
    }
    assert(completed==12 && nav.lefts==6 && nav.rights==6 && nav.decoded_a==6 && nav.decoded_f==6);
}
static void no_command_and_record(void) {
    Circuit c;BarcodeNav n;CircuitInput i={.now=1000,.adc={3464,3692}};
    circuit_begin(&c,&i);barcode_nav_begin(&n,&i);
    n.cross_ready=n.cross_gate=true;n.cross_width=18;
    assert(!barcode_circuit_step(&c,&n,&cal,&i,1).stop && !c.busy && n.straight_crosses==1);
    /* Rejected gates do not consume a command; a qualified crossing does. */
    n.pending='F';n.cross_gate=false;n.cross_ready=true;n.cross_width=18;
    (void)barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(!c.busy && n.pending=='F' && c.ignored==1);
    n.cross_gate=true;n.cross_ready=true;c.clear_path=150;
    (void)barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(!c.busy && n.pending=='F' && c.ignored==2);
    i.now=1000000;i.path=i.wheel[0]=i.wheel[1]=250;i.speed[0]=i.speed[1]=45;
    n.cross_gate=n.cross_ready=true;n.cross_wheel[0]=n.cross_wheel[1]=232;n.cross_us=500000;
    (void)barcode_circuit_step(&c,&n,&cal,&i,1);
    assert(c.busy && n.busy && n.action=='F' && !n.pending);
    static BarcodeSaved s;s.magic=B_MAGIC;s.version=1;s.profile=s.recorded_profile=1;s.calibration=cal;s.side=(SideCalibration){0,1,3};
    s.count=B_ROWS;s.event_count=B_EVENTS;s.summary_count=C_SUMMARIES;
    s.crc=log_crc((uint8_t *)&s+offsetof(BarcodeSaved,count),sizeof(s)-offsetof(BarcodeSaved,count));
    assert(barcode_record_valid(&s));s.version=2;assert(barcode_record_valid(&s));
    s.version=3;assert(barcode_record_valid(&s));s.version=4;assert(barcode_record_valid(&s));s.version=5;assert(barcode_record_valid(&s));s.version=6;assert(barcode_record_valid(&s));s.version=7;assert(barcode_record_valid(&s));s.version=8;assert(barcode_record_valid(&s));s.version=9;assert(barcode_record_valid(&s));s.version=10;assert(!barcode_record_valid(&s));s.version=3;
    s.events[10].code^=1;assert(!barcode_record_valid(&s));
    printf("record_bytes=%zu trace=%zu summary=%zu\n",sizeof(s),sizeof(BarcodeTrace),sizeof(BarcodeSummary));
}
int main(void) {
    decoder_tests();capture_tests();queue_tests();captured_a_replay(1);captured_a_replay(2);no_command_and_record();
    for(unsigned p=1;p<=3;++p) navigation_simulation(p);
    puts("PASS: PDF A/Z measured fixtures; A/F both sizes and directions; speed/jitter/framing rejection; D0 debounce/overflow; queued A/F;12 alternating turns/profile; mirrored acquisition; no-command straight; CRC/flash ABI.");
    return 0;
}
