"""Static model of the micro T-Kernel allocation pool for the RTOS images.

usage: python3 testing/rtos_migration/buddy5/kernel_pool_budget.py <app.elf> <rtos_buddy1_4|rtos_buddy1_5>
       [--nm arm-none-eabi-nm] [--buddy5-stack 3072]

The kernel takes every task stack (stksz + 128 B system stack, 8-byte rounded)
and every message buffer from one pool, knl_Imalloc, which spans __bss_end ..
INTERNAL_RAM_END - CNF_EXC_STACK_SIZE (kernel/sysdepend/cpu/core/armv6m/
reset_hdl.c, kernel/tkernel/memory.c). Each block carries an 8-byte AreaQue
header; the IMACB and two boundary queues come first. Nothing frees or
allocates after init in these images, so the pool either fits at boot (and
stays fitted) or a tk_cre_* call at boot returns E_NOMEM and app_main prints
"INIT FAILED; motors disabled".

The allocation list below was taken from every tk_cre_tsk / tk_cre_mbf /
Kmalloc reference in the linked objects of the v0.2.12 build (app_main.c,
sysinit.c inittask, usb_console.c, cyw43_utk_kernel.c, control.c,
connected_control.c, network_bridge.c x2, line_service.c; msdrvif Kmalloc
is unused with USE_SDEV_DRV=0). It is a model, not a measurement: the first
idle boot confirms it (every TASK line result=0, no INIT FAILED).
"""
import argparse
import subprocess
import sys

RAM_END = 0x20040000
EXC_STACK = 2048
SYS_STACK = 128       # DEFAULT_SYS_STKSZ (armv6m)
HDR = 8               # AreaQue per allocated block
IMACB = 24            # rounded IMACB + alignment
BOUNDARY = 16         # top/end AreaQue


def r8(n):
    return (n + 7) // 8 * 8


def demand(app, b5_stack):
    tasks = [('inittask', 1024), ('usb-console', 4096), ('cyw43 (lwIP)', 8192),
             ('network-bridge', 4096), ('flash-park', 4096), ('motion', 4096), ('imu-terrain', 4096),
             ('safety', 4096), ('nav-bench', 4096), ('usb-command', 4096), ('diagnostics', 4096)]
    if app == 'rtos_buddy1_5':
        tasks.insert(8, ('buddy5-obstacle', b5_stack))
    mbfs = [('control commands', 256), ('connected requests', 256), ('network TX', 1024), ('network RX', 256),
            ('line commands', 256)]
    rows = [(n, r8(s + SYS_STACK) + HDR) for n, s in tasks] + [(n, r8(s) + HDR) for n, s in mbfs]
    return rows, sum(v for _, v in rows) + IMACB + BOUNDARY


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('elf')
    ap.add_argument('app', choices=('rtos_buddy1_4', 'rtos_buddy1_5'))
    ap.add_argument('--nm', default='arm-none-eabi-nm')
    ap.add_argument('--buddy5-stack', type=int, default=3072)
    a = ap.parse_args(argv)
    syms = {}
    for line in subprocess.run([a.nm, a.elf], capture_output=True, text=True, check=True).stdout.splitlines():
        p = line.split()
        if len(p) == 3:
            syms[p[2]] = int(p[0], 16)
    end = syms['__bss_end']
    pool = RAM_END - EXC_STACK - end
    rows, need = demand(a.app, a.buddy5_stack)
    for n, v in rows:
        print(f'  {n:<20} {v:6d} B')
    print(f'{a.app}: __bss_end 0x{end:08x}; static RAM left {RAM_END - end} B; kernel pool {pool} B; '
          f'boot allocations {need} B; margin {pool - need} B')
    return 0 if pool - need >= 0 else 1


if __name__ == '__main__':
    sys.exit(main())
