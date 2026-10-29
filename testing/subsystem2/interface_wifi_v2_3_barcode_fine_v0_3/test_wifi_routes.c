#define BUDDY_RUNTIME_MAPPING 1
#include <assert.h>
#include <stdio.h>
#include "v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_nav.h"
#include "nav_api.h"
#include "motion_port.h"

static uint32_t map_for(char letter,unsigned slot) {
    uint32_t word=0;unsigned index=(unsigned)(letter-'A');
    for(unsigned k=0;k<4;++k)word|=((index+k+26-slot)%26)<<(k*5);
    assert(wp_map_valid(word));return wp_map_replace(0,word);
}
static unsigned pattern_for(char c) {
    for(unsigned p=0;p<512;++p)if(barcode_character((uint16_t)p)==c)return p;
    assert(0);return 0;
}
static void reader_roundtrip(char c,bool reverse) {
    double widths[29];unsigned at=0;
    for(unsigned group=0;group<3;++group) {
        unsigned pattern=pattern_for(group==1?c:'*');
        for(unsigned k=0;k<9;++k)widths[at++]=(pattern&(1u<<(8-k)))?7.5:2.5;
        if(group<2)widths[at++]=2.5;
    }
    BarcodeReader r={0};uint64_t now=1000;double mm=20;
    barcode_reader_reset(&r,false,now,0);(void)barcode_reader_edge(&r,true,now,mm);
    BarcodeRead read={0};
    for(unsigned k=0;k<29;++k) {
        mm+=widths[reverse?28-k:k];now+=100000;
        read=barcode_reader_edge(&r,(k%2)!=0,now,mm);
    }
    assert((read.events&(1u<<B_DECODE)) && read.code==c && read.reverse==reverse);
}
static void capture_and_dispatch(char c,unsigned slot) {
    static const char actions[]="SULR";
    BarcodeNav n={0};n.mapping=map_for(c,slot);
    barcode_queue_code(&n,c);assert(n.pending==c && n.pending_action==actions[slot]);
    unsigned revision=n.pending_revision;
    n.mapping=wp_map_replace(n.mapping,map_for(c,(slot+1)%4)&WP_MAP_MASK);
    assert(n.pending_revision==revision && barcode_nav_action(&n,c)==actions[slot]);
    LineCalibration cal={{160,160},{3457,3700},7};
    CircuitInput i={.now=1000000,.path=100,.wheel={100,100},.speed={45,45},.adc={3400,3500}};
    Circuit circuit={0};circuit_begin(&circuit,&i);
    n.cross_ready=n.cross_gate=true;n.cross_us=900000;n.cross_width=18;
    n.cross_wheel[0]=n.cross_wheel[1]=76;
    AnalogDemand d=barcode_circuit_step(&circuit,&n,&cal,&i,1);
    assert(!d.stop && !n.pending);
    if(slot==0) {
        assert(!circuit.busy && !n.busy && n.straight_crosses==1 && !circuit.turns);
        assert(n.events[0].type==B_DISPATCH && n.events[1].type==B_COMPLETE);
        assert(n.events[0].code==c && n.events[0].action=='S' && n.events[0].map_revision==revision);
    } else {
        assert(circuit.busy && n.busy && n.action==c && n.executing_action==actions[slot]);
        assert(n.executing_revision==revision);
        n.mapping=map_for(c,(slot+2)%4);
        assert(barcode_nav_action(&n,c)==actions[slot]);
    }
}
static uint64_t clock_us;
static uint64_t now(void *p){(void)p;return clock_us;}
int main(void) {
    for(char c='A';c<='Z';++c) {
        reader_roundtrip(c,false);reader_roundtrip(c,true);
        for(unsigned slot=0;slot<4;++slot)capture_and_dispatch(c,slot);
    }
    BarcodeNav n={0};barcode_queue_code(&n,'A');assert(n.last_code=='A' && !n.pending && n.unsupported==1);
    n=(BarcodeNav){0};n.mapping=map_for('A',2);barcode_queue_code(&n,'A');
    unsigned old_revision=n.pending_revision;
    n.mapping=wp_map_replace(n.mapping,map_for('H',2)&WP_MAP_MASK);
    barcode_queue_code(&n,'A'); /* Unassigned new decode must not falsely log the old queued action. */
    assert(n.pending=='A' && n.pending_revision==old_revision && barcode_nav_action(&n,'A')=='L');
    assert(barcode_decode_action(&n,'A')=='-');
    barcode_event(&n,B_DECODE,0,0,0,'A',barcode_decode_action(&n,'A'),0,NULL);
    assert(n.events[0].action=='-' && n.events[0].map_revision==(uint8_t)(n.mapping>>21));
    MotionPort p={.now_us=now};motion_init(&p);motion_session_begin(1000);
    for(char c='A';c<='Z';++c)assert(nav_submit_barcode(MOTION_BUDDY3,c)==MOTION_OK);
    assert(nav_submit_barcode(MOTION_BUDDY1,'B')==MOTION_INVALID);
    assert(nav_submit_barcode(MOTION_BUDDY3,'*')==MOTION_INVALID);
    puts("PASS all26 Code39 letters both directions; four mapped actions; queued revision stable; straight consumed once; A-Z coordinator API");
    return 0;
}
