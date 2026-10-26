#include <assert.h>
#include <stdio.h>
#include "smooth_follow.h"
#include "../circuit_left_test/circuit_wheel.h"
static const LineCalibration school={{176,175},{3483,3498},7};
int main(void) {
    static const unsigned black[][2]={{3244,2752},{3238,2763},{3238,2766},{3234,2774},{3231,2787}};
    assert(calibration_valid(&school));
    Circuit c;BarcodeNav n;SmoothFollow s={0};CircuitInput i={.now=1000};
    circuit_begin(&c,&i);barcode_nav_begin(&n,&i);
    for(unsigned k=0;k<5;++k) {
        i.adc[0]=black[k][0];i.adc[1]=black[k][1];i.now+=10000;
        AnalogDemand d=barcode_drive_step(&c,&n,&school,&i,1,&s,.01f);
        assert(!d.stop && !c.follower.core.white && !c.follower.search.active);
        assert(d.level[0]>=920 && d.level[1]>=770);
        assert(d.target[0]>=40 && d.target[1]>=40);
    }
    /* The reported weak/white episodes must not permanently lock out a
     * clearly visible return. A single good front sensor remains sufficient. */
    i.now+=10000;i.adc[0]=151;i.adc[1]=154;
    (void)barcode_drive_step(&c,&n,&school,&i,1,&s,.01f);assert(c.follower.core.white);
    i.now+=10000;i.adc[0]=3238;i.adc[1]=175;
    (void)barcode_drive_step(&c,&n,&school,&i,1,&s,.01f);
    i.now+=40000;AnalogDemand d=barcode_drive_step(&c,&n,&school,&i,1,&s,.01f);
    assert(!d.stop && !c.follower.core.white && !c.follower.search.active);
    /* Retain V12's immediate strong-bend targets and outer-wheel breakaway,
     * rather than V13's slower visible-wheel controller. */
    for(unsigned w=0;w<2;++w) {
        SignedWheel p={.sign=1,.pi={.ramp=45}};
        assert(circuit_wheel_step(&p,w,45,0,.01f,1000000,false,1)==300);
        assert(!circuit_wheel_step(&p,w,0,20,.01f,1010000,false,1));
    }
    puts("PASS: school worn-black samples recognized; one-sensor return clears loss; V12 breakaway restored; zero-off retained.");
}
