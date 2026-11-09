#ifndef RTOS_DRIVE_DIAGNOSTICS_H
#define RTOS_DRIVE_DIAGNOSTICS_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Copied by the ADC/control owner. Canonical wire bits never depend on native
 * compiler bitfield layout. C8 v0.2.2: 18-byte frame, 15-byte payload.
 * p0 subtype, p1 codec version, p2 uptime32, p6 run16,
 * p8 bend phase2/side2/blend-percent7/recovery phase3 (two reserved bits),
 * p10 sweep stage3/yaw signed13 in 0.1deg,
 * p12 targets signed9+signed9/saturation1 (five reserved bits).
 */
enum { RL_DRIVE_DIAG_BYTES = 18 };
/* Two previously unused bits retain the 56-byte controller-row ABI. */
static inline void rl_row_bend_phase(unsigned phase, uint32_t *meta, uint32_t *aux) {
    *meta = (*meta & 0x7fffffffU) | ((uint32_t)(phase & 1U) << 31);
    *aux = (*aux & 0x7fffffffU) | ((uint32_t)((phase >> 1) & 1U) << 31);
}
typedef struct {
    unsigned bend_phase : 2, bend_side : 2, bend_percent : 7, recovery_phase : 3;
    unsigned sweep_stage : 3;
    signed yaw_x10 : 13, target_left : 9, target_right : 9;
    unsigned saturated : 1;
} RlDriveDiagnostic;

static inline int rl_diag_clamp(int value, int lo, int hi, bool *clipped) {
    if (value < lo) { *clipped = true; return lo; }
    if (value > hi) { *clipped = true; return hi; }
    return value;
}
static inline RlDriveDiagnostic rl_drive_diag_make(unsigned phase, unsigned side,
                                                  unsigned percent, unsigned recovery,
                                                  unsigned stage, int yaw_x10,
                                                  int left, int right) {
    bool clipped = false;
    RlDriveDiagnostic d = {0};
    d.bend_phase = (unsigned)rl_diag_clamp((int)phase, 0, 3, &clipped);
    d.bend_side = (unsigned)rl_diag_clamp((int)side, 0, 2, &clipped);
    d.bend_percent = (unsigned)rl_diag_clamp((int)percent, 0, 100, &clipped);
    d.recovery_phase = (unsigned)rl_diag_clamp((int)recovery, 0, 7, &clipped);
    d.sweep_stage = (unsigned)rl_diag_clamp((int)stage, 0, 4, &clipped);
    d.yaw_x10 = rl_diag_clamp(yaw_x10, -4096, 4095, &clipped);
    d.target_left = rl_diag_clamp(left, -180, 180, &clipped);
    d.target_right = rl_diag_clamp(right, -180, 180, &clipped);
    d.saturated = clipped;
    return d;
}
static inline void rl_diag_put(uint8_t *p, unsigned bit, unsigned width, uint32_t value) {
    for (unsigned k = 0; k < width; ++k)
        if ((value >> k) & 1U) p[(bit+k)/8] |= (uint8_t)(1U << ((bit+k)%8));
}
static inline void rl_drive_diag_pack(const RlDriveDiagnostic *d, uint32_t uptime,
                                      uint16_t run, uint8_t seq,
                                      uint8_t out[RL_DRIVE_DIAG_BYTES]) {
    memset(out, 0, RL_DRIVE_DIAG_BYTES);
    out[0]=0xf2; out[1]=15; out[2]=seq;
    uint8_t *p=out+3; p[0]=0xc8; p[1]=0x22;
    rl_diag_put(p,16,32,uptime); rl_diag_put(p,48,16,run);
    rl_diag_put(p,64,2,d->bend_phase); rl_diag_put(p,66,2,d->bend_side);
    rl_diag_put(p,68,7,d->bend_percent); rl_diag_put(p,75,3,d->recovery_phase);
    rl_diag_put(p,80,3,d->sweep_stage); rl_diag_put(p,83,13,(uint32_t)d->yaw_x10);
    rl_diag_put(p,96,9,(uint32_t)d->target_left);
    rl_diag_put(p,105,9,(uint32_t)d->target_right);
    rl_diag_put(p,114,1,d->saturated);
}
#endif
