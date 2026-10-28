/* Includes the actual application startup: a failed clock check must never
 * create/start the drive task. Kernel scheduling itself is not simulated. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <tk/tkernel.h>
#define APP_H
#define EXPORT
#define GPIO_MODE_OUT 1
#define TA_RNG0 0
#define TA_ASSPRC 0x200
#define TP_PRC2 2
#define TMO_FEVR (-1)
typedef int ID;
typedef void (*FP)(int,void *);
typedef struct { FP task;int itskpri;uint32_t stksz,tskatr,assprc; } T_CTSK;
static jmp_buf done;
static int clock_result,init_result,bridge_result,checks,bridges,created,started,off_pins;
int gpio_set_val(UINT pin,UINT value){assert(pin>=8&&pin<=11&&value==0);off_pins|=1<<(pin-8);return 0;}
int gpio_set_pin(UINT pin,UINT mode){assert(pin>=8&&pin<=11&&mode==GPIO_MODE_OUT);return 0;}
ER sys_hal_init(void){return init_result;}
ER sys_timebase_validate(void){++checks;return clock_result;}
void sys_timebase_report(void){}
int drive_bridge_init(void){++bridges;return bridge_result;}
void buddy_drive_task(int a,void *b){(void)a;(void)b;assert(0);}
ID tk_cre_tsk(const T_CTSK *task){assert(checks==1&&clock_result==E_OK);assert(task->stksz==12288&&task->assprc==TP_PRC2);++created;return 1;}
ER tk_sta_tsk(ID id,int arg){assert(id==1&&arg==0);++started;return E_OK;}
ER tk_slp_tsk(int timeout){assert(timeout==TMO_FEVR);longjmp(done,1);}
ER tk_dly_tsk(uint32_t delay){assert(delay==2000);longjmp(done,1);}
int tm_printf(const UB *fmt,...){(void)fmt;return 0;}
#include "source/app_program/app_main.c"
static void run(int init,int check,int bridge) {
    init_result=init;clock_result=check;bridge_result=bridge;
    checks=bridges=created=started=off_pins=0;
    if(!setjmp(done))(void)usermain();
    assert(off_pins==15);
}
int main(void) {
    run(E_SYS,E_OK,E_OK);assert(checks==0&&bridges==0&&created==0&&started==0);
    run(E_OK,E_SYS,E_OK);assert(checks==1&&bridges==0&&created==0&&started==0);
    run(E_OK,E_OK,E_SYS);assert(checks==1&&bridges==1&&created==0&&started==0);
    run(E_OK,E_OK,E_OK);assert(checks==1&&bridges==1&&created==1&&started==1);
    puts("PASS actual app startup: motors held off; no drive task on timer/config/check/bridge failure; valid clock admits exactly one drive task.");
}
