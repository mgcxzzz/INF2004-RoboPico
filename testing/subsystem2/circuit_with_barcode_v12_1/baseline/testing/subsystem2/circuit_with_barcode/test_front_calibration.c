#include <assert.h>
#include <stdio.h>
#include "barcode_record.h"
#include "../line_motion_integration/logic.h"

int main(void) {
    /* User's captured references and independently reported held-surface ranges. */
    LineCalibration c={{138,152},{199,229},7};
    assert(calibration_valid(&c)==(LINE_CAL_MIN_SPAN<=61));
    if(LINE_CAL_MIN_SPAN<=61) {
        unsigned white[5][2]={{138,153},{137,153},{137,153},{136,154},{139,156}};
        unsigned black[5][2]={{197,232},{197,230},{200,231},{199,230},{197,231}};
        for(unsigned k=0;k<5;++k) {
            uint16_t wl=line_level(white[k][0],c.white[0],c.black[0]);
            uint16_t wr=line_level(white[k][1],c.white[1],c.black[1]);
            uint16_t bl=line_level(black[k][0],c.white[0],c.black[0]);
            uint16_t br=line_level(black[k][1],c.white[1],c.black[1]);
            assert(wl<=17 && wr<=54 && bl>=950 && br>=1000);
            assert(profile_start_block(5,false,false,true,false,true,wl,wr,false,LINE_USB_FREE)==START_LINE_ABSENT);
            assert(profile_start_block(5,false,false,true,false,true,bl,br,false,LINE_USB_FREE)==START_OK);
            Circuit circuit;CircuitInput i={.now=1000,.adc={black[k][0],black[k][1]}};
            circuit_begin(&circuit,&i);AnalogDemand d=circuit_follow(&circuit,&c,&i,1);
            assert(!d.stop && d.target[0]==45 && d.target[1]==45);
        }
        static BarcodeSaved s;s.magic=B_MAGIC;s.version=5;s.profile=s.recorded_profile=1;
        s.calibration=c;s.side=(SideCalibration){0,1,3};
        s.crc=log_crc((uint8_t *)&s+offsetof(BarcodeSaved,count),barcode_record_crc_size(s.version));
        assert(barcode_record_valid(&s));s.calibration.black[0]--;
        assert(!barcode_record_valid(&s));
    }
    LineCalibration edge={{100,100},{100+LINE_CAL_MIN_SPAN,100+LINE_CAL_MIN_SPAN},7};
    assert(calibration_valid(&edge));--edge.black[1];assert(!calibration_valid(&edge));
    edge.black[1]=100+LINE_CAL_MIN_SPAN;edge.mask=3;assert(!calibration_valid(&edge));
    edge.mask=7;edge.black[0]=90;assert(!calibration_valid(&edge));
    printf("PASS: minimum=%u; actual references; observed white/black noise; normalized start gate; balanced follower; saved-record CRC; below-limit/incomplete/reversed rejection.\n",(unsigned)LINE_CAL_MIN_SPAN);
}
