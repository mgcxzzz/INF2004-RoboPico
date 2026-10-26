#ifndef BARCODE_COVERAGE_FOLLOW_H
#define BARCODE_COVERAGE_FOLLOW_H
/* Included after CircuitTracker and the shared confidence helpers. */
static inline AnalogDemand coverage_line_demand(CircuitTracker *t,AnalogDemand d,uint64_t now,uint32_t mm) {
    SymmetricTracker *s=&t->core;CoverageState *cv=&t->coverage;
    coverage_observe(cv,d.level,now);
    unsigned maximum=d.level[0]>d.level[1]?d.level[0]:d.level[1];
    unsigned minimum=d.level[0]<d.level[1]?d.level[0]:d.level[1];
    int direction=d.level[0]>d.level[1]?1:-1;
    bool both_white=cv->white[0] && cv->white[1] && maximum<CV_CLEAR;
    bool ambiguous=cv->low_timing && now-cv->low_since>=CV_AMBIGUOUS_US;
    bool centred=cv->paired && !both_white;
    if(!centred)t->centre_timing=false;
    else {
        if(!t->centre_timing) {t->centre_timing=true;t->centre_since=now;}
        if(now-t->centre_since>=C_CENTRE_US) {t->centre_valid=true;t->centre_us=now;}
    }
    if(!s->white && (both_white || ambiguous)) {
        unsigned memory=circuit_memory(t,now);
        s->white=true;s->white_since=now;s->white_mm=mm;
        s->recovery=memory==1?1:memory==2?-1:0;t->straight_recovery=memory==3;
        t->reacquire_timing=t->medium_timing=t->strong_timing=false;
    }
    if(s->white) {
        if(maximum>=CV_CLEAR) {
            if(!t->reacquire_timing) {t->reacquire_timing=true;t->reacquire_since=now;}
            if(now-t->reacquire_since>=CV_RETURN_US) {
                s->white=false;s->recovery=0;t->straight_recovery=false;
                t->reacquire_timing=t->medium_timing=t->strong_timing=false;
                t->search=(LineSearch){0};
            }
        } else t->reacquire_timing=false;
        if(s->white) {
            /* Stop on a possible return so the front spots do not sweep past
             * it while the20ms confidence check completes. */
            if(t->reacquire_timing) {d.decision=CV_UNCERTAIN;return d;}
            if(line_search_ready(&t->search,now,d.level[0],d.level[1])) {
                if(!t->search.active)line_search_begin(&t->search,now,mm,s->recovery);
                LineSearchDemand seek=line_search_step(&t->search,now,mm,false);
                if(seek.reason) {t->loss_reason=seek.reason;d.stop=true;d.decision=AL_LOST;return d;}
                d.target[0]=seek.target[0];d.target[1]=seek.target[1];
                d.decision=(uint8_t)(LS_DECISION_FIRST+seek.stage);return d;
            }
            if(now-s->white_since<LS_BRIDGE_US && (s->recovery || t->straight_recovery)) {
                if(t->straight_recovery)d.target[0]=d.target[1]=35;
                else {d.target[0]=t->bridge[0];d.target[1]=t->bridge[1];}
            }
            d.decision=CV_UNCERTAIN;return d;
        }
    }
    if(centred) {
        d.target[0]=d.target[1]=45;d.decision=CV_CENTRE;
        circuit_direction(t,0,0,now); /* Black-strength imbalance is not yaw. */
    } else if(minimum>=CV_CLEAR) {
        d.target[0]=d.target[1]=30;d.decision=CV_UNCERTAIN;
        circuit_direction(t,0,0,now);
    } else if(maximum>=CV_CLEAR) {
        unsigned weak=d.level[0]<d.level[1]?0:1;
        bool confirmed=cv->white[weak];
        bool hard=confirmed && now-cv->white_since[weak]>=CV_HARD_US;
        int inner=hard?0:confirmed?25:35,outer=hard?45:50;
        d.target[direction>0?0:1]=(int16_t)inner;d.target[direction>0?1:0]=(int16_t)outer;
        d.decision=hard?(direction>0?SYM_LEFT:SYM_RIGHT):confirmed?
            (direction>0?CV_FIRM_LEFT:CV_FIRM_RIGHT):(direction>0?CV_SOFT_LEFT:CV_SOFT_RIGHT);
        circuit_direction(t,direction*(confirmed?1000:160),confirmed?1000:maximum,now);
    } else {
        /* Neither has a reliable line return. Allow only a short known
         * curvature bridge; after80ms the same bounded loss episode begins. */
        if(cv->low_timing && now-cv->low_since<CV_AMBIGUOUS_US && circuit_memory(t,now)) {
            d.target[0]=t->bridge[0];d.target[1]=t->bridge[1];
        }
        d.decision=CV_UNCERTAIN;return d;
    }
    int outer=d.target[0]>d.target[1]?d.target[0]:d.target[1];
    if(outer>0)for(unsigned w=0;w<2;++w)t->bridge[w]=(int16_t)(d.target[w]*35/outer);
    return d;
}
#endif
