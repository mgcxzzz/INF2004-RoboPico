#ifndef DRIVE_BRIDGE_TEST_TKERNEL_H
#define DRIVE_BRIDGE_TEST_TKERNEL_H
#include <stdint.h>
#include <stddef.h>
typedef int ER;
typedef int ID;
typedef int INT;
typedef int TMO;
typedef unsigned int UINT;
typedef unsigned int ATR;
typedef struct { ATR mbfatr; int bufsz; int maxmsz; } T_CMBF;
typedef struct { ATR mtxatr; } T_CMTX;
typedef struct { int frbufsz; } T_RMBF;
#define E_OK 0
#define E_TMOUT (-50)
#define TA_TFIFO 0u
#define TA_INHERIT 0u
#define TMO_POL 0
ID tk_cre_mbf(const T_CMBF *spec);
ER tk_del_mbf(ID id);
ER tk_snd_mbf(ID id,const void *data,INT size,TMO timeout);
INT tk_rcv_mbf(ID id,void *data,TMO timeout);
ER tk_ref_mbf(ID id,T_RMBF *status);
ID tk_cre_mtx(const T_CMTX *spec);
ER tk_loc_mtx(ID id,TMO timeout);
ER tk_unl_mtx(ID id);
#endif
