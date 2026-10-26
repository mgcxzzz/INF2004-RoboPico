#include <assert.h>
#include <stdio.h>
#include "barcode_start.h"
#include "barcode_record.h"

int main(void) {
    /* All combinations retain the old priority for non-optical conditions. */
    for(unsigned mode=0;mode<3;++mode) for(unsigned bits=0;bits<64;++bits) {
        bool active=bits&1,settling=bits&2,valid=bits&4;
        bool stop=bits&8,ready=bits&16,hold=bits&32;
        assert(barcode_start_block(active,settling,valid,stop,ready,hold,mode)==
            profile_start_block(5,active,settling,valid,stop,ready,1000,1000,hold,mode));
    }
    /* Latest saved references plus previously reported normal black readings.
       Reproduce reason11 without assuming this represents the current mount. */
    LineCalibration cal={{137,153},{303,1461},7};
    unsigned l=line_level(197,137,303),r=line_level(230,153,1461);
    assert(calibration_valid(&cal) && l<650 && r<650);
    assert(profile_start_block(5,false,false,true,false,true,l,r,false,LINE_USB_FREE)==START_LINE_ABSENT);
    assert(barcode_start_block(false,false,true,false,true,false,LINE_USB_FREE)==START_OK);
    assert(barcode_start_block(false,false,true,false,true,false,LINE_BANK)==START_OK);
    assert(barcode_start_block(false,false,true,false,true,true,LINE_USB_HOLD)==START_OK);
    /* No-history starts still require an explicit request. The opt-in search
       waits150ms then alternates, with a deadline even when no ticks arrive. */
    Circuit c;CircuitInput i={.now=1000,.adc={137,153}};
    circuit_begin(&c,&i);AnalogDemand d=circuit_follow(&c,&cal,&i,1);
#ifdef CIRCUIT_SWEEP_SEARCH
    assert(!d.stop && !d.target[0] && !d.target[1]);
#ifdef CIRCUIT_COVERAGE_FOLLOW
    i.now+=CV_WHITE_US;d=circuit_follow(&c,&cal,&i,1);
    assert(c.follower.core.white && !d.stop);
#endif
    i.now+=LS_CONFIRM_US;
    d=circuit_follow(&c,&cal,&i,1);assert(!d.stop && c.follower.search.active);
    i.now+=LS_TOTAL_US;d=circuit_follow(&c,&cal,&i,1);
    assert(d.stop && c.follower.loss_reason==LS_TIMEOUT);
#else
    assert(d.stop && c.follower.loss_reason==C_LOSS_NO_DIRECTION);
#endif
    puts("PASS:192 non-optical start combinations; reason11 removed; all start modes; no-history recovery remains bounded.");
}
