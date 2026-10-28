/* Kernel driving integration. Networking remains in cyw43_utk_task.
 * The validated V2 controller, logical coordinator and single PWM writer
 * execute together in one core-1 task; no template nav/motion tasks run. */
#include "app.h"
#include "system/sys_hal.h"
extern int drive_bridge_init(void);
extern void buddy_drive_task(int stacd,void *exinf);

EXPORT INT usermain(void)
{
    for(UINT pin=8;pin<=11;++pin) {
        (void)gpio_set_val(pin,0);(void)gpio_set_pin(pin,GPIO_MODE_OUT);
    }
    tm_printf((UB*)"\nBUDDY123 UTK DRIVE V1.1 | verified 1MHz timer | Continuous V2 steering + kernel MQTT\n");
    tm_printf((UB*)"Motors OFF on boot. Await explicit start. Buddy5/IMU hooks only.\n");
    ER er=sys_hal_init();
    if(er<E_OK)goto failed;
    er=sys_timebase_validate();
    sys_timebase_report();
    if(er<E_OK)goto failed; /* no drive task, no ADC/PWM start on a bad clock */
    er=drive_bridge_init();
    if(er<E_OK)goto failed;
    T_CTSK task={0};
    task.task=(FP)buddy_drive_task;task.itskpri=14;task.stksz=12288;
    task.tskatr=TA_HLNG|TA_RNG0|TA_ASSPRC;task.assprc=TP_PRC2;
    ID id=tk_cre_tsk(&task);
    if(id<E_OK){er=id;goto failed;}
    er=tk_sta_tsk(id,0);
    if(er<E_OK)goto failed;
    for(;;)(void)tk_slp_tsk(TMO_FEVR);
failed:
    for(;;) {
        tm_printf((UB*)"UTK DRIVE V1.1 init failed (%d); driving disabled; motors remain OFF.\n",(int)er);
        sys_timebase_report();
        (void)tk_dly_tsk(2000);
    }
}
