"""Reproduce the old timeout panic and test the fix using the real SDK lwIP."""
from pathlib import Path
import os,re,shlex,shutil,subprocess
root=Path(__file__).resolve().parent
sdk=Path(os.environ.get('PICO_SDK_PATH',root.parents[3]/'pico-sdk'))
lwip=sdk/'lib/lwip/src'
if not (lwip/'core/timeouts.c').is_file():
    raise SystemExit('Set PICO_SDK_PATH to the Pico SDK checkout used for this firmware.')
compiler=shlex.split(os.environ.get('CC',shutil.which('clang') or shutil.which('gcc') or 'cc'))
build=root/'build/lwip-timeout-test';build.mkdir(parents=True,exist_ok=True)
original=build/'default-options';original.mkdir(exist_ok=True)
# Removing only the override reinstates lwIP's original core-only pool formula.
(original/'lwipopts.h').write_text(re.sub(r'^#define MEMP_NUM_SYS_TIMEOUT[^\n]*\n','',(root/'lwipopts.h').read_text(),flags=re.M))
for name,config,args in [('default',original,['--expect-exhaustion']),('fixed',root,[])]:
    exe=build/(name+'.exe')
    subprocess.run([*compiler,'-std=c11','-Wall','-Wextra','-Werror',
        '-I'+str(root/'lwip_test_port'),'-I'+str(config),'-I'+str(lwip/'include'),
        str(root/'test_lwip_timeouts.c'),str(lwip/'core/timeouts.c'),str(lwip/'core/memp.c'),
        '-o',str(exe)],check=True)
    subprocess.run([str(exe),*args],check=True)
print('PASS real lwIP default-failure/fixed-capacity regression; no physical networking tested.')
