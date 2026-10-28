/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 *
 *	How micro T-Kernel starts your code
 *	-----------------------------------
 *	  reset -> kernel initialises itself -> creates the "initial task"
 *	        -> initial task calls usermain()
 *
 *	usermain() runs inside that initial task at priority 1 (the highest).
 *	On a single core nothing you create runs until usermain() blocks, but
 *	on the dual-core build the OTHER core may start a new task at once.
 *	So usermain() works in two phases:
 *	  1. every subsystem creates its kernel objects  (subsystemN_init)
 *	  2. every subsystem starts its tasks             (subsystemN_start)
 *	and then puts the initial task to sleep forever.
 *	If usermain() RETURNS, the kernel shuts the whole system down.
 *
 *	The example system: one subsystem per folder, one task each
 *	------------------------------------------------------------
 *	  subsystem1/  WiFi and MQTT (Buddy 1): supervisory commands from the
 *	               dashboard, heartbeat and telemetry (needs NET=lwip MQTT=1)
 *	  subsystem2/  Producer  makes a "sample" every 300 ms     (core 1)
 *	  subsystem3/  Consumer  collects samples into statistics  (core 0)
 *	  subsystem4/  Monitor   prints reports, pauses/resumes the LED
 *	  subsystem5/  Obstacle Scan and Recovery (Buddy 5): HC-SR04+ on an
 *	               SG90, profiles obstacles, plans the bypass
 *	  system/      shared: Safety Supervisor, Navigation Coordinator
 *
 *	How they talk to each other (the interfaces, see TEAM.md section 5):
 *	  IF-01  S2 -> S3  mailbox mbx_samples      (owned by S3)
 *	  IF-02  S3 -> S2  semaphore sem_slots + memory pool mpf_samples
 *	                   (owned by S2: S3 hands slots and blocks back)
 *	  IF-03  S3 -> S4  message buffer mbf_summary + event flag flg_events
 *	                   (owned by S4)
 *	  IF-04  S4 -> S3  reads app_stats under mutex mtx_stats (owned by S3)
 *	  IF-05  S4 -> S1  suspends/resumes tid_led (owned by S1; on the buddy1
 *	                   branch an idle stand-in task)
 *	The subsystem that RECEIVES or OWNS the data creates the object and
 *	publishes its ID in its header; the other side just uses it.
 *
 *	micro T-Kernel APIs used, and where to find them:
 *	  tasks       tk_cre_tsk tk_sta_tsk tk_slp_tsk tk_wup_tsk tk_can_wup
 *	              tk_dly_tsk tk_sus_tsk tk_rsm_tsk tk_chg_pri tk_ref_tsk
 *	              tk_get_tid tk_get_prc                         app_main.c, S1-S4
 *	  semaphore   tk_cre_sem tk_wai_sem tk_sig_sem tk_ref_sem       S1-S4
 *	  mem. pool   tk_cre_mpf tk_get_mpf tk_rel_mpf tk_ref_mpf       S2, S3, S4
 *	  mailbox     tk_cre_mbx tk_snd_mbx tk_rcv_mbx                  S2, S3
 *	  mutex       tk_cre_mtx tk_loc_mtx tk_unl_mtx                  S3, S4
 *	  msg buffer  tk_cre_mbf tk_snd_mbf tk_rcv_mbf                  S1, S3, S4
 *	  event flag  tk_cre_flg tk_set_flg tk_wai_flg                  S3, S4
 *	  cyclic      tk_cre_cyc tk_ref_cyc                             (template S1 only)
 *	  alarm       tk_cre_alm tk_sta_alm tk_stp_alm                  S4
 *	  time        tk_get_tim                                        S2, S4
 *	  system      tk_ref_ver                                        app_main.c
 *
 *	Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

/* Network proof only: real kernel Wi-Fi/MQTT + message-buffer bridge.
 * Physical motion/line/IMU/obstacle tasks are deliberately not started. */
#include "system/system.h"
#include "cyw43_utk.h"
extern bool utk_network_settings_available(void);
extern const char *utk_network_broker(void);

static void network_status_task(INT stacd, void *exinf)
{
    (void)stacd;(void)exinf;
    for(;;) {
        T_CYW43_UTK_STATUS status;
        cyw43_utk_get_status(&status);
        const unsigned char *ip=(const unsigned char *)&status.ip_addr;
        const char *broker=utk_network_broker();
        tm_printf((UB*)"[UTK_NETWORK_V1] motors=OFF cfg=%d owner_prc=%d polls=%u link=%d ip=%u.%u.%u.%u broker=%s\n",
            utk_network_settings_available()?1:0,(int)status.owner_prc,(unsigned)status.poll_count,
            (int)status.link_status,(unsigned)ip[0],(unsigned)ip[1],(unsigned)ip[2],(unsigned)ip[3],
            broker?broker:"UNCONFIGURED");
        (void)tk_dly_tsk(2000);
    }
}

EXPORT INT usermain(void)
{
    /* Force Robo Pico motor inputs low before any app task starts.
     * No physical motion provider is linked in this proof build. */
    for(UINT pin=8;pin<=11;++pin) {
        (void)gpio_set_val(pin,0);(void)gpio_set_pin(pin,GPIO_MODE_OUT);
    }
    tm_printf((UB*)"\nBUDDY123 UTK NETWORK V1 | professor kernel Wi-Fi service + mqtt_utk.c\n");
    tm_printf((UB*)"MOTOR OUTPUTS DISABLED. Network/mapping/state-only bench. No line tracing or avoidance.\n");
    tm_printf((UB*)"Status prints automatically every2s; n/g/f/w/l/r/k from standalone firmware are not used.\n");
    ER er=system_init();CHECK(er,"system_init");
    if(er<E_OK)goto failed;
    er=subsystem1_init();CHECK(er,"subsystem1_init");
    if(er<E_OK)goto failed;
    er=system_start();CHECK(er,"system_start");
    if(er<E_OK)goto failed;
    er=subsystem1_start();CHECK(er,"subsystem1_start");
    if(er<E_OK)goto failed;
    if(app_start_task((FP)network_status_task,14,TP_PRC2,"network_status")<E_OK)goto failed;
    for(;;)(void)tk_slp_tsk(TMO_FEVR);
failed:
    tm_printf((UB*)"UTK NETWORK startup failed; motors remain disabled.\n");
    for(;;)(void)tk_slp_tsk(TMO_FEVR);
}
