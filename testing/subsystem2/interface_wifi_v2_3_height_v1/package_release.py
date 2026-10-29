"""Package the height release after running host, ARM and stack checks.
Requires git and arm-none-eabi-{gcc,size,nm} on PATH. Does not build or publish.
"""
from pathlib import Path
import argparse,difflib,hashlib,json,struct,subprocess,zipfile
root=Path(__file__).resolve().parent
repo=root.parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def write_json(path,value):path.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8',newline='\n')
def command(*args):return subprocess.check_output(args,cwd=repo,text=True).strip()
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--build',type=Path,default=root/'build-arm')
args=parser.parse_args();build=args.build.resolve()
base=json.loads((root/'HEIGHT_BASELINE.json').read_text())
main='v12_2/baseline/testing/subsystem2/circuit_with_barcode/main.c'
record='v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h'
runtime=['CMakeLists.txt','continuous_steering.h','successful_calibration.h',
         'drive_diag.h','drive_diag_storage.h','drive_diag_pico.h',main,record]
changes={};diff=[]
for rel in runtime:
    old=subprocess.check_output(['git','show',base['base_commit']+':'+base['source_folder']+'/'+rel],cwd=repo)
    new=(root/rel).read_bytes()
    assert hashlib.sha256(old).hexdigest()==base['files'][rel],rel
    oldlines=old.decode().splitlines(True);newlines=new.decode().splitlines(True)
    hunks=[]
    for tag,a,b,c,d in difflib.SequenceMatcher(None,oldlines,newlines,autojunk=False).get_opcodes():
        if tag!='equal':hunks.append({'old_start':a,'new_start':c,'old':oldlines[a:b],'new':newlines[c:d]})
    assert hunks,rel
    changes[rel]={'base_sha256':base['files'][rel],'modified_sha256':sha(root/rel),'hunks':hunks}
    diff.extend(difflib.unified_diff(oldlines,newlines,fromfile='v2.3/'+rel,tofile='height_v1/'+rel))
write_json(root/'HEIGHT_PATCH.json',{'base_commit':base['base_commit'],'changes':changes})
(root/'HEIGHT_CHANGES.patch').write_text(''.join(diff),encoding='utf-8',newline='\n')
artifact='firmware/buddy123_interface_wifi_v2_3_height_v1.uf2'
bundle='firmware/buddy123_interface_wifi_v2_3_height_v1_bundle.zip'
elf=build/'buddy123_interface_wifi_v2_3_height_v1.elf'
assert elf.is_file() and (root/artifact).is_file()
assert not (root/'wifi_config.local.h').exists(),'Remove private credentials header and rebuild first'
for log in ('host_tests.txt','lwip_timeouts.txt','stack_layout.txt'):
    assert (root/'logs'/log).is_file() and 'PASS' in (root/'logs'/log).read_text(encoding='utf-8-sig'),log
size=[int(v) for v in command('arm-none-eabi-size',str(elf)).splitlines()[-1].split()[:3]]
symbols={}
for line in command('arm-none-eabi-nm','--defined-only',str(elf)).splitlines():
    parts=line.split()
    if len(parts)==3 and parts[2] in ('__flash_binary_end','__bss_end__','__HeapLimit','__StackLimit','__StackOneBottom','core1_stack','__StackBottom','__StackOneTop','__StackTop'):
        symbols[parts[2]]=hex(int(parts[0],16))
meta={'firmware_version':'v2.3_height_v1','architecture':'standalone Pico SDK; core0 controller, core1 network owner; no micro T-Kernel',
      'date':'2026-10-30','base_commit':base['base_commit'],'base_folder':base['source_folder'],
      'pico_sdk_commit':'98a542c1a62fb549ffb5d66a3e5892b06276b670',
      'arm_compiler':command('arm-none-eabi-gcc','--version').splitlines()[0],
      'artifact':artifact,'bundle':bundle,'sha256':sha(root/artifact),'bytes':(root/artifact).stat().st_size,
      'blocks':(root/artifact).stat().st_size//512,'arm_builds':{'network_on':dict(zip(('text','data','bss'),size))},
      'symbols':symbols,'calibration':{'white':[191,154],'black':[1940,2306],'record_version':29},
      'host_c_executables':37,'javascript_schema_tests':10,'python_helper_tests':20,
      'diagnostic_capture_version':5,'embedded_credentials':False,'default_buddy_extension':None,
      'timeout_pool':{'core_slots':7,'total_slots':11,'additional_bss_bytes':64,
      'real_allocator_regression':'core-only pool exhausts; fixed 100 callback reschedules and 1000 add/remove cycles pass'},
      'stack_layout':{'core0_bytes':8192,'core1_bytes':8192,'known_driving_chain_frame_bytes':4424,'complete_worst_case_bound':False},
      'recorded_replay':{'rows':256,'old_target_total_variation':883,'height_target_total_variation':780,
      'old_mean_target':31.04,'height_mean_target':35.14,'old_minimum_wheel_target':6,'height_minimum_wheel_target':15,
      'closed_loop_prediction':False},'physical_height_validated':False}
write_json(root/'BUILD_METRICS.json',meta)
files={}
for p in sorted(root.rglob('*')):
    if not p.is_file():continue
    rel=p.relative_to(root)
    if any(d.lower().startswith('build') or d in ('__pycache__','dashboard_session','firmware') for d in rel.parts[:-1]):continue
    if p.name in ('BUILD_METRICS.json','SOURCE_SHA256.json','SHA256SUMS.txt','wifi_config.local.h','wifi_settings.local.json'):continue
    if p.suffix=='.pyc':continue
    files[rel.as_posix()]=sha(p)
write_json(root/'SOURCE_SHA256.json',{'base_commit':base['base_commit'],'files':files})
with zipfile.ZipFile(root/bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=9) as z:
    for rel in [*files,'BUILD_METRICS.json','SOURCE_SHA256.json',artifact]:
        z.write(root/rel,root.name+'/'+rel)
(root/'firmware/SHA256SUMS.txt').write_text(''.join(sha(root/rel)+'  '+Path(rel).name+'\n' for rel in (artifact,bundle)),encoding='ascii',newline='\n')
print('Packaged',len(files),'source/evidence files;',meta['bytes'],'UF2 bytes;',meta['sha256'])
