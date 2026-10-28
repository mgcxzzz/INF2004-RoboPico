"""Host checks for the actual UTK snapshot and desktop setup tools."""
from pathlib import Path
import json,os,subprocess,sys
from provision_utk_wifi import settings_page
root=Path(__file__).resolve().parent;build=root/'build';build.mkdir(exist_ok=True)
workspace=root.parents[3]
zig=Path(os.environ.get('ZIG',workspace/'.buddy2-tools/host-test-compiler/ziglang/zig.exe'))
os.environ.setdefault('ZIG_GLOBAL_CACHE_DIR',str(workspace/'.buddy2-tools/zig-cache'))
os.environ.setdefault('ZIG_LOCAL_CACHE_DIR',str(workspace/'.buddy2-tools/zig-local'))
host=root/'source/testing/subsystem1/host';s1=root/'source/app_program/subsystem1';system=root/'source/app_program/system'
def run(source,extra=(),include=()):
    exe=build/(source.stem+'.exe')
    subprocess.run([str(zig),'cc','-std=c11','-Wall','-Wextra','-Werror','-O0','-UNDEBUG',
        '-I'+str(s1),'-I'+str(system),*include,str(source),*[str(p) for p in extra],'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True);return exe
common=[s1/'b1_proto.c',s1/'b1_mapping.c',system/'sys_tlm.c']
proto=run(host/'test_b1.c',common)
run(host/'test_mapping.c',common)
run(host/'test_safety_mask.c',[system/'sys_tlm.c'],['-I'+str(host/'stubs')])
decoder=run(root/'test_network_settings.c')
page=build/'wcf1_cross.bin';page.write_bytes(settings_page('UTK-TEST','dummy-test-password','192.168.1.2'))
subprocess.run([str(decoder),str(page)],check=True)
(host/'vectors.json').write_bytes(subprocess.check_output([str(proto),'--vectors']))
subprocess.run(['node',str(host/'check_dashboard.mjs')],check=True)
subprocess.run(['node','--test',str(root/'dashboard/test_schema.js')],check=True)
subprocess.run([sys.executable,'-m','unittest','-v','test_provision_utk.py'],cwd=root,check=True)
print('PASS four C host executables, C-to-dashboard vectors, ten schema tests, four setup tests and Python-to-C settings interoperability.')
