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
    /* The start change grants no blind-search budget. All-white without a
       previous line direction still invokes the existing running stop. */
    Circuit c;CircuitInput i={.now=1000,.adc={137,153}};
    circuit_begin(&c,&i);AnalogDemand d=circuit_follow(&c,&cal,&i,1);
    assert(d.stop && c.follower.loss_reason==C_LOSS_NO_DIRECTION);
    puts("PASS:192 non-optical start combinations; reason11 removed; all start modes; confirmed no-direction line-loss stop retained.");
}
