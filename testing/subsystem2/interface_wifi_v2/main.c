#include "drive_diag.h"
static DriveDiag diag_capture;
static DiagValues diag_tick;
static bool diag_observing;
#define DRIVE_DIAG_RAW(d) do { if(diag_observing) { \
    diag_tick.v[D_OPT_L]=(d).target[0];diag_tick.v[D_OPT_R]=(d).target[1]; \
    diag_tick.v[D_LEVEL_L]=(d).level[0];diag_tick.v[D_LEVEL_R]=(d).level[1]; \
    diag_tick.v[D_OPT_DECISION]=(d).decision;diag_tick.v[D_FLAGS]|=DF_OPTICAL; \
} } while(0)
#define DRIVE_DIAG_SOFT(d,kind,scan,active) do { if(diag_observing) { \
    diag_tick.v[D_SOFT_L]=(d).target[0];diag_tick.v[D_SOFT_R]=(d).target[1]; \
    diag_tick.v[D_FLAGS]|=(kind)|((scan)?DF_SCAN:0)|((active)?DF_ACTIVE:0); \
} } while(0)
#include "v12_2/main.c"
