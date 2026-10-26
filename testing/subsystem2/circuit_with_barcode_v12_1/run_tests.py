import pathlib,subprocess,os
root=pathlib.Path(__file__).resolve().parent
compiler=root.parents[3]/'.buddy2-tools/host-test-compiler/ziglang/zig.exe'
os.environ['ZIG_GLOBAL_CACHE_DIR']=str(root.parents[3]/'.buddy2-tools/zig-cache')
os.environ['ZIG_LOCAL_CACHE_DIR']=str(root.parents[3]/'.buddy2-tools/zig-local')
build=root/'build';build.mkdir(exist_ok=True)
base=root/'baseline/testing/subsystem2/circuit_with_barcode'
for source in [root/'test_scan_hold.c']+sorted(base.glob('test_*.c')):
 exe=build/(source.stem+'.exe')
 subprocess.run([str(compiler),'cc','-std=c11','-Wall','-Wextra','-Werror','-DLINE_CAL_MIN_SPAN=50','-DCIRCUIT_SWEEP_SEARCH=1',str(source),'-lm','-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)
