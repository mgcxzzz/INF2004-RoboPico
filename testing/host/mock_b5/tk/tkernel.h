/*
 * Minimal host stand-in for <tk/tkernel.h>, just enough for b5_task.c.
 * Used only by test_b5_task.c; the real kernel header is used on target.
 */
#ifndef MOCK_TKERNEL_H
#define MOCK_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

typedef int32_t  INT;
typedef uint32_t UINT;
typedef int32_t  ER;
typedef int32_t  ID;
typedef int32_t  TMO;
typedef int32_t  PRI;
typedef int32_t  SZ;
typedef uint32_t ATR;
typedef int      BOOL;
typedef uint8_t  UB;
typedef uint32_t UW;
typedef void (*FP)(INT, void *);

#define TRUE        1
#define FALSE       0
#define EXPORT
#define IMPORT      extern
#define LOCAL       static

#define E_OK        (0)
#define E_PAR       (-17)
#define E_IO        (-57)
#define E_NOEXS     (-42)
#define E_TMOUT     (-50)

#define TMO_POL     (0)
#define TMO_FEVR    (-1)

#define TA_HLNG     0x00000001U
#define TA_RNG3     0x00000300U
#define TA_TFIFO    0x00000000U
#define TA_WSGL     0x00000000U
#define TWF_ORW     0x00000001U
#define TWF_CLR     0x00000010U
#define TWF_BITCLR  0x00000020U
#define TA_INHERIT  0x00000002U
#define TA_WMUL     0x00000008U

typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ATR tskatr; FP task; PRI itskpri; SZ stksz; } T_CTSK;
typedef struct { void *exinf; ATR mtxatr; PRI ceilpri; } T_CMTX;

ID tk_cre_flg(T_CFLG const *p);
ER tk_set_flg(ID id, UINT ptn);
ER tk_clr_flg(ID id, UINT ptn);
ER tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *p_ptn, TMO tmo);
ID tk_cre_tsk(T_CTSK const *p);
ER tk_sta_tsk(ID id, INT stacd);
ER tk_dly_tsk(TMO ms);
ID tk_cre_mtx(T_CMTX const *p);
ER tk_loc_mtx(ID id, TMO tmo);
ER tk_unl_mtx(ID id);

#endif
