"""Exercise the actual SDK allocator with this RTOS image's lwipopts.h.

Adapted from interface_wifi_v2 at 10d7886. This tests timer capacity,
not radio operation or an actual MQTT handshake.
"""
from pathlib import Path
import os,re,shlex,shutil,subprocess

root=Path(__file__).resolve().parent
workspace=root.parents[3]
sdk=Path(os.environ.get('PICO_SDK_PATH',workspace/'.buddy2-tools/pico-sdk'))
lwip=sdk/'lib/lwip/src'
compiler=shlex.split(os.environ['CC']) if os.environ.get('CC') else [shutil.which('gcc') or shutil.which('clang') or 'cc']
config=root/'source/lib/liblwip/sysdepend/pico_rp2040'
if not (lwip/'core/timeouts.c').is_file():
    raise SystemExit('Set PICO_SDK_PATH to the SDK used for this firmware.')
build=root/'build/lwip-timeout-test';build.mkdir(parents=True,exist_ok=True)
original=build/'default-options';original.mkdir(exist_ok=True)
(original/'lwipopts.h').write_text(re.sub(r'^#define MEMP_NUM_SYS_TIMEOUT[^\n]*\n','',
    (config/'lwipopts.h').read_text(),flags=re.M))
for name,options,args in [('default',original,['--expect-exhaustion']),('fixed',config,[])]:
    exe=build/(name+'.exe')
    subprocess.run([*compiler,'-std=c11','-Wall','-Wextra','-Werror','-O0','-UNDEBUG',
        '-I'+str(root/'lwip_test_port'),'-I'+str(options),'-I'+str(lwip/'include'),
        str(root/'test_lwip_timeouts.c'),str(lwip/'core/timeouts.c'),str(lwip/'core/memp.c'),
        '-o',str(exe)],check=True)
    subprocess.run([str(exe),*args],check=True)
print('PASS actual RTOS options: default pool reproduces panic; reserved pool survives rescheduling. No physical networking tested.')
