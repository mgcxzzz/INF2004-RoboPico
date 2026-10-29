#ifndef BARCODE_ANALOG_H
#define BARCODE_ANALOG_H
#include "barcode_nav.h"

/* No operator white/black calibration: search raw ADC thresholds, and select
 * only after two neighbouring thresholds decode the same complete Code39
 * frame. Existing measured module polarity is BLACK_HIGH. No narrow bars
 * are removed or inferred. Original decoder validates every element. */
/* Keep the original 63 banks byte-for-byte in behaviour. A separate pair
 * tests the weak 204..205 ADC boundary found in the captured Z scan.
 * Only bank63 can win this pair, which still fits the 6-bit log bank ID. */
enum { AO_LEGACY_BANKS=63,AO_BANKS=65,AO_STEP=128,AO_HYSTERESIS=32,AO_FINE_HYSTERESIS=3,
       AO_DEFAULT_BANK=11,AO_DEBOUNCE_US=2000 }; /*212 ADC diagnostic preview */
typedef struct {
    BarcodeReader reader;
    bool stable,candidate,changed,was_frame;
    uint64_t candidate_us,vote_us;
    double candidate_wheel[2],vote_start,vote_end;
    char vote;bool reverse;uint16_t pattern[3];
} AnalogBank;
typedef struct {
    AnalogBank bank[AO_BANKS];
    uint16_t adc,baseline,minimum,maximum;uint8_t selected;bool black,busy,scanning,accepted,conflict;
    uint64_t last_us;uint32_t max_gap_us,gap_count,candidates,selections;
    double accepted_start,accepted_end,conflict_end;
} BarcodeAnalog;
/* Sorted union: original128,256,...3968 plus192,194,...254. Fine banks
 * target the user's measured narrow gap203..206 and narrow bar215..221.
 * These are sensitivity hypotheses, not a hardcoded barcode or calibration. */
static inline unsigned barcode_analog_threshold(unsigned bank) {
    if(bank>=AO_LEGACY_BANKS)return 204+bank-AO_LEGACY_BANKS;
    return bank==0?128:bank<=32?190+2*bank:(bank-31)*AO_STEP;
}
static inline unsigned barcode_analog_hysteresis(unsigned bank) {
    if(bank>=AO_LEGACY_BANKS)return 1;
    return bank>=1 && bank<=32?AO_FINE_HYSTERESIS:AO_HYSTERESIS;
}
static inline unsigned barcode_analog_log_threshold(unsigned version,unsigned bank) {
    return version<8?(bank+1)*AO_STEP:barcode_analog_threshold(bank);
}
static inline void barcode_analog_begin(BarcodeAnalog *a,BarcodeNav *n,uint16_t adc,uint64_t now,const double wheel[2]) {
    *a=(BarcodeAnalog){.adc=adc,.baseline=adc,.minimum=adc,.maximum=adc,.selected=AO_DEFAULT_BANK,.last_us=now};
    for(unsigned b=0;b<AO_BANKS;++b) {
        AnalogBank *v=&a->bank[b];v->stable=v->candidate=adc>=barcode_analog_threshold(b);
        v->candidate_us=now;memcpy(v->candidate_wheel,wheel,sizeof(v->candidate_wheel));
        barcode_reader_reset(&v->reader,v->stable,now,(wheel[0]+wheel[1])*.5);
    }
    a->black=a->bank[a->selected].stable;n->external_reader=true;n->external_scanning=false;
}
static inline bool barcode_analog_same_frame(const AnalogBank *v,const AnalogBank *w) {
    return v->vote && v->vote==w->vote && v->reverse==w->reverse &&
        fabs(v->vote_start-w->vote_start)<=6 && fabs(v->vote_end-w->vote_end)<=6 &&
        (v->vote_us>w->vote_us?v->vote_us-w->vote_us:w->vote_us-v->vote_us)<=500000;
}
static inline bool barcode_analog_select(BarcodeAnalog *a,BarcodeNav *n,uint64_t now,double mm) {
    int winner=-1;char code=0;bool conflict=false;double best=1e9,centre=(a->minimum+a->maximum)*.5;
    for(unsigned b=0;b+1<AO_BANKS;++b) {
        if(b+1==AO_LEGACY_BANKS)continue; /* never pair across the two families */
        const AnalogBank *v=&a->bank[b],*w=&a->bank[b+1];
        if(!barcode_analog_same_frame(v,w) || now<v->vote_us || now-v->vote_us>500000 ||
           mm-v->vote_end>10 || mm<v->vote_end) continue;
        if(a->accepted && fabs(v->vote_start-a->accepted_start)<=12 && fabs(v->vote_end-a->accepted_end)<=12) continue;
        if(winner>=0 && code!=v->vote) conflict=true;
        else if(winner<0 || fabs(barcode_analog_threshold(b)-centre)<best) {
            winner=(int)b;code=v->vote;best=fabs(barcode_analog_threshold(b)-centre);
        }
    }
    if(winner<0) return false;
    if(conflict) {
        if(!a->conflict || mm-a->conflict_end>12) {
            barcode_event(n,B_ANALOG_CONFLICT,now,mm,0,'?',0,0,NULL);
            a->conflict=true;a->conflict_end=mm;
        }
        n->pending=0;n->cross_candidate=n->cross_ready=false;return false;
    }
    if(a->conflict && mm-a->conflict_end<=12) return false;
    a->selected=(uint8_t)winner;const AnalogBank *v=&a->bank[a->selected];
    a->black=v->stable;a->accepted=true;a->accepted_start=v->vote_start;a->accepted_end=v->vote_end;++a->selections;
    n->cross_candidate=n->cross_ready=false;barcode_queue_code(n,code);
    barcode_event(n,B_ANALOG_SELECT,now,mm,barcode_analog_threshold(a->selected),code,
        barcode_decode_action(n,code),a->selected+1,v->pattern);
    barcode_event(n,B_DECODE,now,mm,v->vote_end-v->vote_start,code,
        barcode_decode_action(n,code),0x80u|(a->selected<<1)|v->reverse,v->pattern);
    return true;
}
static inline void barcode_analog_sample(BarcodeAnalog *a,BarcodeNav *n,uint16_t adc,uint64_t now,const double wheel[2]) {
    double mm=(wheel[0]+wheel[1])*.5;a->adc=adc;
    if(adc<a->minimum) a->minimum=adc;
    if(adc>a->maximum) a->maximum=adc;
    if(n->busy) {
        if(!a->busy) { barcode_analog_begin(a,n,adc,now,wheel);a->busy=true; }
        a->last_us=now;return;
    }
    if(a->busy) barcode_analog_begin(a,n,adc,now,wheel);
    if(now>a->last_us) {
        uint64_t gap=now-a->last_us;
        if(gap>a->max_gap_us) a->max_gap_us=(uint32_t)gap;
        if(gap>10000) {
            ++a->gap_count;
            if(a->gap_count<=4) barcode_event(n,B_ANALOG_GAP,now,mm,(double)gap/1000,0,0,0,NULL);
        }
    }
    a->last_us=now;a->scanning=false;
    for(unsigned b=0;b<AO_BANKS;++b) {
        AnalogBank *v=&a->bank[b];unsigned t=barcode_analog_threshold(b);v->changed=false;
        unsigned hysteresis=barcode_analog_hysteresis(b);
        bool black=v->stable?adc>t-hysteresis:adc>=t+hysteresis;
        if(black!=v->candidate) {
            v->candidate=black;v->candidate_us=now;memcpy(v->candidate_wheel,wheel,sizeof(v->candidate_wheel));
        }
        if(v->candidate!=v->stable && now-v->candidate_us>=AO_DEBOUNCE_US) {
            v->stable=v->candidate;v->changed=true;v->was_frame=v->reader.frame;
            double edge_mm=(v->candidate_wheel[0]+v->candidate_wheel[1])*.5;
            BarcodeRead r=barcode_reader_edge(&v->reader,v->stable,v->candidate_us,edge_mm);
            if(b==a->selected) {
                if(r.events&(1u<<B_RUN)) barcode_event(n,B_RUN,v->candidate_us,edge_mm,r.width,0,0,r.completed_black,NULL);
                if(r.events&(1u<<B_FRAME_START)) barcode_event(n,B_FRAME_START,v->candidate_us,edge_mm,edge_mm-r.start,0,0,v->reader.reverse,NULL);
                if(r.events&(1u<<B_BAD_FRAME)) { ++n->rejected;barcode_event(n,B_BAD_FRAME,v->candidate_us,edge_mm,edge_mm-r.start,'?',0,r.rejection,r.pattern); }
            }
            if(r.events&(1u<<B_DECODE)) {
                v->vote=r.code;v->vote_us=v->candidate_us;v->vote_start=r.start;v->vote_end=edge_mm;
                v->reverse=r.reverse;memcpy(v->pattern,r.pattern,sizeof(v->pattern));++a->candidates;
                barcode_event(n,B_ANALOG_CANDIDATE,v->candidate_us,edge_mm,t,r.code,0,b+1,r.pattern);
                if(a->accepted && fabs(r.start-a->accepted_start)<=12 && fabs(edge_mm-a->accepted_end)<=12 && r.code!=n->last_code) {
                    a->conflict=true;a->conflict_end=mm;n->pending=0;n->cross_candidate=n->cross_ready=false;
                    barcode_event(n,B_ANALOG_CONFLICT,now,mm,t,r.code,0,b+1,r.pattern);
                }
            }
        }
        if(barcode_reader_expire(&v->reader,now,mm) && b==a->selected) {
            ++n->rejected;barcode_event(n,B_BAD_FRAME,now,mm,mm-v->reader.start_mm,'?',0,B_REJECT_TIMEOUT,NULL);
        }
        if(v->reader.frame || v->reader.history_count || (t>a->baseline+100u && v->stable)) a->scanning=true;
    }
    bool selected=barcode_analog_select(a,n,now,mm);
    AnalogBank *v=&a->bank[a->selected];a->black=v->stable;
    n->external_scanning=a->scanning;
    /* A cross is searched only after a verified command. The selecting edge
       is the final stop bar, and is never reused as the next junction. */
    if(selected || !n->pending) return;
    if(v->reader.frame) { n->cross_candidate=false;return; }
    if(v->changed && !v->was_frame) {
        double edge_mm=(v->candidate_wheel[0]+v->candidate_wheel[1])*.5;
        if(v->stable) {
            barcode_cross_confirm(n,v->candidate_us,edge_mm,true);
            if(n->cross_candidate) {
                n->cross_candidate=false;barcode_event(n,B_CROSS_CANCEL,v->candidate_us,edge_mm,edge_mm-n->cross_trailing_mm,n->pending,0,0,NULL);
            }
            n->leading_us=v->candidate_us;memcpy(n->leading_wheel,v->candidate_wheel,sizeof(n->leading_wheel));
            n->leading_gate=n->allow_cross;
        } else if(n->leading_us>=n->start_us && now>=n->leading_us && now-n->leading_us<20000000) {
            double leading=(n->leading_wheel[0]+n->leading_wheel[1])*.5,width=edge_mm-leading;
            if(width>=B_CROSS_MIN_MM && width<=B_CROSS_MAX_MM) {
                n->cross_width=width;n->cross_us=n->leading_us;memcpy(n->cross_wheel,n->leading_wheel,sizeof(n->cross_wheel));
                n->cross_gate=n->leading_gate;n->cross_candidate=true;n->cross_trailing_mm=edge_mm;
                barcode_event(n,B_CROSS_CANDIDATE,v->candidate_us,edge_mm,width,n->pending,barcode_nav_action(n,n->pending),0,NULL);
            }
        }
    }
    barcode_cross_confirm(n,now,mm,!a->black);
}
#endif
