#include <assert.h>
#include <stdio.h>
#include "app_program/line_navigation/drive_diagnostics.h"
static unsigned bits(const uint8_t *p, unsigned at, unsigned n) {
    unsigned v=0;for(unsigned k=0;k<n;k++)v|=((p[(at+k)/8]>>((at+k)%8))&1U)<<k;return v;
}
int main(void) {
    uint32_t meta=0x12345678,aux=0x23456789;
    for(unsigned phase=0;phase<4;phase++) {
        rl_row_bend_phase(phase,&meta,&aux);
        assert((meta&0x7fffffffU)==0x12345678 && (aux&0x7fffffffU)==0x23456789);
        assert(((meta>>31)|((aux>>31)<<1))==phase);
    }
    uint8_t f[RL_DRIVE_DIAG_BYTES];
    RlDriveDiagnostic d=rl_drive_diag_make(2,1,100,3,4,-123,-35,35);
    rl_drive_diag_pack(&d,123456,7,9,f);
    assert(f[0]==0xf2 && f[1]==15 && f[2]==9 && f[3]==0xc8 && f[4]==0x22);
    const uint8_t *p=f+3;
    assert(bits(p,16,32)==123456 && bits(p,48,16)==7);
    assert(bits(p,64,2)==2 && bits(p,66,2)==1 && bits(p,68,7)==100);
    assert(bits(p,75,3)==3 && bits(p,80,3)==4);
    assert(bits(p,83,13)==((unsigned)-123&8191U));
    assert(bits(p,96,9)==((unsigned)-35&511U) && bits(p,105,9)==35);
    assert(bits(p,78,2)==0 && bits(p,114,6)==0);
    FILE *file=fopen("testing/rtos_migration/build-host/drive_diagnostics.bin","wb");
    assert(file);assert(fwrite(f,1,sizeof f,file)==sizeof f);fclose(file);
    d=rl_drive_diag_make(9,9,200,9,9,-9999,-999,999);
    assert(d.saturated && d.bend_phase==3 && d.bend_side==2 && d.bend_percent==100);
    assert(d.yaw_x10==-4096 && d.target_left==-180 && d.target_right==180);
    rl_drive_diag_pack(&d,0,0,0,f);assert(bits(f+3,114,1)==1);
    puts("PASS drive diagnostics packed bounds/reserved bits/C fixture");
    return 0;
}
