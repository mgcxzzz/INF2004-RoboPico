/*
 * Minimal host stand-in for <tk/tkernel.h>, just enough for the
 * Safety Supervisor host test. Not used on the target.
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
typedef int32_t  W;
typedef uint32_t UW;
typedef uint32_t ATR;
typedef int      BOOL;
typedef uint8_t  UB;
typedef void (*FP)(INT, void *);

#define TRUE        1
#define FALSE       0
#define EXPORT
#define IMPORT      extern
#define LOCAL       static

#define E_OK        (0)
#define E_PAR       (-17)
#define E_NOEXS     (-42)
#define E_TMOUT     (-50)
#define TMO_POL     (0)
#define TMO_FEVR    (-1)

#define TA_HLNG     0x00000001U
#define TA_RNG3     0x00000300U
#define TA_TFIFO    0x00000000U
#define TA_MFIFO    0x00000000U
#define TA_USERBUF  0x00000020U
#define TA_WMUL     0x00000008U
#define TA_WSGL     0x00000000U
#define TWF_ORW     0x00000001U
#define TWF_CLR     0x00000010U
#define TWF_BITCLR  0x00000020U

typedef struct { W hi; UW lo; } SYSTIM;
typedef struct { void *exinf; ATR tskatr; FP task; PRI itskpri; SZ stksz; } T_CTSK;
typedef struct { void *exinf; ATR flgatr; UINT iflgptn; } T_CFLG;
typedef struct { void *exinf; ATR mbfatr; SZ bufsz; INT maxmsz; void *bufptr; } T_CMBF;

ID  tk_cre_tsk(T_CTSK const *p);
ER  tk_sta_tsk(ID id, INT stacd);
ID  tk_cre_mbf(T_CMBF const *p);
ER  tk_snd_mbf(ID id, void const *msg, INT size, TMO tmo);
INT tk_rcv_mbf(ID id, void *msg, TMO tmo);
ER  tk_get_otm(SYSTIM *p);
ID  tk_cre_flg(T_CFLG const *p);
ER  tk_set_flg(ID id, UINT ptn);
ER  tk_clr_flg(ID id, UINT ptn);
ER  tk_wai_flg(ID id, UINT ptn, UINT mode, UINT *p_ptn, TMO tmo);

#endif
