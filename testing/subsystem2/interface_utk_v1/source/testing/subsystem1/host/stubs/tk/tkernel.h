/* Minimal host-only kernel declarations for the Safety regression test. */
#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H
#include <stdint.h>
typedef int ER;
typedef int ID;
typedef int INT;
typedef unsigned int UINT;
typedef struct { uint32_t hi; uint32_t lo; } SYSTIM;
typedef struct { UINT flgatr; UINT iflgptn; } T_CFLG;
typedef struct
{
    UINT tskatr;
    void (*task)(INT, void *);
    INT itskpri;
    INT stksz;
} T_CTSK;
#define E_OK 0
#define E_PAR (-17)
#define E_NOEXS (-42)
#define E_TMOUT (-50)
#define TA_TFIFO 0U
#define TA_WMUL 8U
#define TA_HLNG 1U
#define TA_RNG3 0x300U
#define TWF_ORW 1U
#define TWF_CLR 16U
ER tk_get_otm(SYSTIM * t);
ID tk_cre_flg(T_CFLG const * c);
ID tk_cre_tsk(T_CTSK const * c);
ER tk_set_flg(ID id, UINT bits);
ER tk_sta_tsk(ID id, INT code);
ER tk_wai_flg(ID id, UINT bits, UINT mode, UINT * out, INT timeout);
#endif
