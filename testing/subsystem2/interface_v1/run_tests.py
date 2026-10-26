"""Local C tests; optional ZIG environment variable overrides the compiler."""
import pathlib, subprocess, os, json, hashlib
root=pathlib.Path(__file__).resolve().parent
workspace=root.parents[3]
compiler=pathlib.Path(os.environ.get('ZIG',workspace/'.buddy2-tools/host-test-compiler/ziglang/zig.exe'))
os.environ.setdefault('ZIG_GLOBAL_CACHE_DIR',str(workspace/'.buddy2-tools/zig-cache'))
os.environ.setdefault('ZIG_LOCAL_CACHE_DIR',str(workspace/'.buddy2-tools/zig-local'))
build=root/'build';build.mkdir(exist_ok=True)
base=root/'v12_2/baseline/testing/subsystem2/circuit_with_barcode'
core=[root/n for n in ('motion_api.c','nav_coordinator.c','motion_executor.c','nav_codec.c')]
def run(source,extra=(),defines=(),suffix=''):
    exe=build/(source.stem+suffix+'.exe')
    subprocess.run([str(compiler),'cc','-std=c11','-Wall','-Wextra','-Werror',
        '-DLINE_CAL_MIN_SPAN=50','-DCIRCUIT_SWEEP_SEARCH=1',*defines,str(source),*[str(s) for s in extra],'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
for source in [root/'v12_2/test_scan_smooth.c']+sorted(base.glob('test_*.c')):run(source)
run(root/'test_interface.c',core)
run(root/'test_clock.c',core)
run(root/'test_clock.c',core,['-DINTERFACE_UNPACKED_REFERENCE=1'],'_unpacked')
run(root/'test_interface.c',core,['-DINTERFACE_UNPACKED_REFERENCE=1'],'_unpacked')
run(root/'test_storage.c',[root/'nav_codec.c'])
run(root/'test_buddy5.c',core+[root/'examples/buddy5_extension.c'])
if (root/'test_parity.c').exists():run(root/'test_parity.c',core)
manifest=json.loads((root/'V12_2_BASELINE.json').read_text())['sha256']
modified={n for n,h in manifest.items() if hashlib.sha256((root/'v12_2'/n).read_bytes()).hexdigest()!=h}
allowed={
 'baseline/testing/subsystem2/circuit_with_barcode/main.c',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_scan.h',
 'baseline/testing/subsystem2/circuit_with_barcode/test_barcode.c',
 'baseline/testing/subsystem2/circuit_with_barcode/test_uturn.c'}
assert modified<=allowed,modified-allowed
print('PASS V12.2 frozen algorithms: only explicit harness/record-version adaptations differ:',sorted(modified))
print('ALL INTERFACE AND RETAINED V12.2 TESTS PASSED')
