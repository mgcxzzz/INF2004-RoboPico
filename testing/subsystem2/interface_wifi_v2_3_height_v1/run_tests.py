"""Local C tests; optional ZIG environment variable overrides the compiler."""
import pathlib, subprocess, os, json, hashlib, shutil, shlex
root=pathlib.Path(__file__).resolve().parent
workspace=root.parents[3]
zig=os.environ.get('ZIG')
default_zig=workspace/'.buddy2-tools/host-test-compiler/ziglang/zig.exe'
if os.environ.get('CC'):
    compiler=shlex.split(os.environ['CC'])
elif zig or default_zig.exists():
    compiler=[str(zig or default_zig),'cc']
else:
    compiler=shlex.split(os.environ.get('CC',shutil.which('clang') or shutil.which('gcc') or 'cc'))
os.environ.setdefault('ZIG_GLOBAL_CACHE_DIR',str(workspace/'.buddy2-tools/zig-cache'))
os.environ.setdefault('ZIG_LOCAL_CACHE_DIR',str(workspace/'.buddy2-tools/zig-local'))
build=root/'build';build.mkdir(exist_ok=True)
base=root/'v12_2/baseline/testing/subsystem2/circuit_with_barcode'
core=[root/n for n in ('motion_api.c','nav_coordinator.c','motion_executor.c','nav_codec.c')]
executions=0
def run(source,extra=(),defines=(),suffix='',arguments=()):
    global executions
    executions+=1
    exe=build/(source.stem+suffix+'.exe')
    subprocess.run([*compiler,'-std=c11','-Wall','-Wextra','-Werror','-O0','-UNDEBUG',*shlex.split(os.environ.get('CFLAGS','')),
        '-DLINE_CAL_MIN_SPAN=50','-DCIRCUIT_SWEEP_SEARCH=1',*defines,str(source),*[str(s) for s in extra],'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe),*arguments],check=True)
for source in [root/'v12_2/test_scan_smooth.c']+sorted(base.glob('test_*.c')):run(source)
run(root/'test_interface.c',core)
run(root/'test_clock.c',core)
run(root/'test_clock.c',core,['-DINTERFACE_UNPACKED_REFERENCE=1'],'_unpacked')
run(root/'test_interface.c',core,['-DINTERFACE_UNPACKED_REFERENCE=1'],'_unpacked')
run(root/'test_storage.c',[root/'nav_codec.c'])
run(root/'test_drive_diag.c')
run(root/'test_diag_parity.c')
run(root/'test_cruise_handover.c')
run(root/'test_continuous_drive.c')
run(root/'test_continuous_steering.c')
run(root/'test_continuous_steering_reference.c')
run(root/'test_height_replay.c',arguments=[str(root/'fixtures/raised_height_trace.csv')])
run(root/'test_continuous_follow.c')
run(root/'test_continuous_integration.c',core)
run(root/'test_buddy5.c',core+[root/'examples/buddy5_extension.c'])
run(root/'test_wifi_protocol.c',[root/'wifi_protocol.c'])
run(root/'test_wifi_packets.c',[root/'wifi_packets.c'])
run(root/'test_wifi_settings.c')
run(root/'test_wifi_link_state.c')
run(root/'test_wifi_transport.c',defines=['-I'+str(root/'transport_test_stubs'),'-I'+str(root)])
run(root/'test_wifi_harness.c',core+[root/'wifi_protocol.c',root/'wifi_packets.c'],['-DBUDDY_RUNTIME_MAPPING=1'])
run(root/'test_wifi_storage.c')
run(root/'test_successful_calibration.c',defines=['-DBUDDY_WIFI_LOGGING=1','-DBUDDY_RUNTIME_MAPPING=1'])
run(root/'test_wifi_routes.c',core+[root/'wifi_protocol.c'],['-DBUDDY_RUNTIME_MAPPING=1'])
if (root/'test_parity.c').exists():run(root/'test_parity.c',core)
manifest=json.loads((root/'V12_2_BASELINE.json').read_text())['sha256']
modified={n for n,h in manifest.items() if hashlib.sha256((root/'v12_2'/n).read_bytes()).hexdigest()!=h}
allowed={
 'scan_smooth.h',
 'baseline/testing/subsystem2/circuit_with_barcode/main.c',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_scan.h',
 'baseline/testing/subsystem2/circuit_with_barcode/test_barcode.c',
 'baseline/testing/subsystem2/circuit_with_barcode/test_uturn.c',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_nav.h',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_analog.h',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_raw.h'}
assert modified<=allowed,modified-allowed
print('PASS V12.2 frozen algorithms: only explicit harness/record-version adaptations differ:',sorted(modified))
vector_exe=build/'wifi_vectors.exe'
subprocess.run([*compiler,'-std=c11','-Wall','-Wextra','-Werror',str(root/'wifi_vectors.c'),str(root/'wifi_protocol.c'),str(root/'wifi_packets.c'),'-o',str(vector_exe)],check=True)
vector_json=build/'wifi_vectors.json'
vector_json.write_bytes(subprocess.check_output([str(vector_exe)]))
subprocess.run(['node',str(root/'test_wifi_vectors.mjs'),str(vector_json)],check=True)
subprocess.run(['node','--test',str(root/'dashboard/test_schema.js')],check=True)
subprocess.run([__import__('sys').executable,'-m','unittest','test_setup_helpers.py','test_network_diagnostics.py'],cwd=root,check=True)
print('ALL INTERFACE AND WIFI TESTS PASSED; C test executables=',executions)
