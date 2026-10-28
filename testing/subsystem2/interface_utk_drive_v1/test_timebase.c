/* Executes actual sys_hal_init/validate with a register/RTOS-clock model.
 * No claim of physical clock accuracy or hardware validation. */
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include "source/app_program/system/sys_hal.h"
#include "source/include/sys/sysdepend/pico_rp2040/timebase.h"
static uint32_t tick,selected,divider,kernel_ms,timer_us,writes,reads;
static bool ignore_write,never_running,frozen,fast_clock;
static void reset(uint32_t inherited) {
    tick=inherited;selected=4;divider=256;kernel_ms=timer_us=writes=reads=0;
    ignore_write=never_running=frozen=fast_clock=false;
}
uint32_t test_read(uint32_t addr) {
    ++reads;
    if(addr==UTK_TB_TICK_ADDR)return tick;
    if(addr==UTK_TB_REF_SELECTED)return selected;
    if(addr==UTK_TB_REF_DIV_ADDR)return divider;
    return 0;
}
void test_write(uint32_t addr,uint32_t value) {
    assert(addr==UTK_TB_TICK_ADDR);++writes;
    if(!ignore_write)tick=value|(never_running?0:UTK_TB_RUNNING);
}
uint32_t in_w(uint32_t addr){return addr==TIMER_TIMERAWL?timer_us:0;}
void out_w(uint32_t addr,uint32_t value){(void)addr;(void)value;}
ER tk_get_otm(SYSTIM *out){out->hi=0;out->lo=kernel_ms;return E_OK;}
ER tk_dly_tsk(uint32_t ms) {
    kernel_ms+=ms;
    uint32_t cycles=tick&UTK_TB_CYCLES_MASK;
    if(cycles==0)cycles=512;
    if(!frozen)timer_us+=(uint32_t)((uint64_t)ms*1000u*12u/cycles)*(fast_clock?2u:1u);
    return E_OK;
}
ER tk_def_int(UINT irq,const T_DINT *def){(void)irq;(void)def;return E_OK;}
void ClearInt(UINT irq){(void)irq;}
void EnableInt(UINT irq,INT level){(void)irq;(void)level;}
int tm_printf(const UB *fmt,...){(void)fmt;return 0;}
int main(void) {
    /* The old RUNNING-only conditional would preserve this wrong divider. */
    reset(0x600u);assert(tick&UTK_TB_RUNNING);
    assert(sys_hal_init()==E_OK && writes==1u);
    assert((tick&UTK_TB_CYCLES_MASK)==12u);
    assert(sys_timebase_validate()==E_OK && timer_us==250000u && kernel_ms==250u);
    reset(0x60cu);assert(sys_hal_init()==E_OK && writes==1u); /* idempotent */
    reset(0u);assert(sys_hal_init()==E_OK && sys_timebase_validate()==E_OK);
    reset(0x601u);assert(sys_hal_init()==E_OK); /* any running wrong divider */
    reset(0x600u);ignore_write=true;
    assert(sys_hal_init()==E_SYS && writes==1u && reads<UTK_TB_POLL_LIMIT*4u);
    assert(sys_timebase_validate()==E_SYS && timer_us==5859u); /* 512/12 slow */
    reset(0u);never_running=true;assert(sys_hal_init()==E_SYS);
    reset(0x60cu);selected=1;assert(sys_hal_init()==E_SYS && writes==0u);
    reset(0x60cu);divider=512;assert(sys_hal_init()==E_SYS && writes==0u);
    reset(0x60cu);assert(sys_hal_init()==E_OK);frozen=true;assert(sys_timebase_validate()==E_SYS);
    reset(0x60cu);assert(sys_hal_init()==E_OK);fast_clock=true;assert(sys_timebase_validate()==E_SYS);
    reset(0x60cu);assert(sys_hal_init()==E_OK);timer_us=UINT32_MAX-100;kernel_ms=UINT32_MAX-100;
    assert(sys_timebase_validate()==E_OK); /* modular elapsed arithmetic */
    assert(utk_timebase_elapsed_ok(251000u,250u));
    assert(!utk_timebase_elapsed_ok(250000u,0u));
    puts("PASS actual timer initialization and startup cross-check: inherited wrong divider, readback failure, wrong reference, slow/frozen/fast clocks, wraparound.");
}
