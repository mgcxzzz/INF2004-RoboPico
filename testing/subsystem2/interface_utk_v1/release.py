"""Package/verify the isolated UTK network proof; never includes local credentials."""
from pathlib import Path
import argparse,hashlib,io,json,re,shutil,struct,subprocess,tarfile,zipfile

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[2]
WORKSPACE=ROOT.parents[3]
BASE='b09403074939f45cf4d7c11b7606148f6c2a4e73'
SDK='a1438dff1d38bd9c65dbd693f0e5db4b9ae91779'
NAME='buddy123_utk_network_v1'
VENDOR_SHA='b088a7f9045df4e478dbc378f41125066e43d9c602755ee4c5cda0f3e9380ba0'

def digest(data):return hashlib.sha256(data).hexdigest()
def save(name,value):
    (ROOT/name).write_bytes((json.dumps(value,indent=2)+'\n').encode('utf-8'))
def command(*args):return subprocess.check_output(args,cwd=REPO,text=True)
def files():
    paths=command('git','ls-files','--cached','--others','--exclude-standard','--',ROOT.relative_to(REPO).as_posix()).splitlines()
    return sorted({(REPO/p).relative_to(ROOT).as_posix() for p in paths if (REPO/p).is_file()})

def package():
    firmware=ROOT/'firmware';firmware.mkdir(exist_ok=True)
    logs=ROOT/'logs';logs.mkdir(exist_ok=True)
    output=ROOT/'source/build_make'/('mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt')
    shutil.copyfile(output.with_suffix('.uf2'),firmware/(NAME+'.uf2'))
    for source,target in [('final_build.log','arm_build.txt'),('host_tests.log','host_tests.txt'),
                          ('lwip_timeout_pool_regression.log','lwip_timeout_pool_regression.txt'),
                          ('ui_smoke.log','ui_smoke.txt')]:
        shutil.copyfile(ROOT/'build'/source,logs/target)

    # Compare the copied kernel/app snapshot to the exact Buddy 1 commit.
    selected=['.gitignore','README.md','TEAM.md','app_program','build_make','config',
              'device','etc','include','kernel','lib','tools/elf2uf2','testing/subsystem1/host']
    archive=subprocess.check_output(['git','archive','--format=tar',BASE,*selected],cwd=REPO)
    with tarfile.open(fileobj=io.BytesIO(archive)) as tar:
        upstream={m.name:digest(tar.extractfile(m).read()) for m in tar.getmembers() if m.isfile()}
    local={p.removeprefix('source/'):digest((ROOT/p).read_bytes()) for p in files() if p.startswith('source/')}
    missing=sorted(set(upstream)-set(local))
    if missing:raise RuntimeError('Missing upstream source files: '+str(missing))
    changed={p:{'before':upstream[p],'after':local[p]} for p in upstream if upstream[p]!=local[p]}
    added={p:local[p] for p in local if p not in upstream}
    save('UPSTREAM_BASELINE.json',{'repository':'https://github.com/mgcxzzz/INF2004-RoboPico',
         'branch':'buddy1','commit':BASE,'selected_paths':selected,'files':upstream})
    save('PATCHES.json',{'base_commit':BASE,'modified':changed,'added':added})

    tools=WORKSPACE/'.buddy2-tools/arm-gcc/bin'
    nm=command(str(tools/'arm-none-eabi-nm.exe'),'-S',str(output.with_suffix('.elf')))
    (logs/'release_symbols.txt').write_bytes(('\n'.join(line for line in nm.splitlines() if any(
        name in line for name in ('b1_net_ready','cyw43_utk_task','mqtt_utk_start','mc_',
        'memp_memory_SYS_TIMEOUT','__data_org')))+'\n').encode('utf-8'))
    size=command(str(tools/'arm-none-eabi-size.exe'),str(output.with_suffix('.elf')))
    text,data,bss=map(int,size.splitlines()[-1].split()[:3])
    app=(firmware/(NAME+'.uf2')).read_bytes()
    save('BUILD_METRICS.json',{'date':'2026-10-28','scope':'motors-off kernel network/map/state proof',
        'upstream_commit':BASE,'teammate_comparison_commit':'10d7886','pico_sdk_commit':SDK,
        'compiler':command(str(tools/'arm-none-eabi-gcc.exe'),'--version').splitlines()[0],
        'profile':'WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0',
        'elf':{'text':text,'data':data,'bss':bss},
        'artifact':'firmware/'+NAME+'.uf2','sha256':digest(app),'bytes':len(app),
        'timeout_pool':{'core_slots':6,'total_slots':10,'application_allowance':4,
            'real_allocator_regression':'default assertion reproduced; fixed 100 callbacks and 1000 add/remove cycles passed'},
        'physical_network_validated':False,'physical_motion_enabled':False,'embedded_credentials':False})
    hashes={p:digest((ROOT/p).read_bytes()) for p in files() if not p.startswith('firmware/') and p!='SOURCE_SHA256.json'}
    save('SOURCE_SHA256.json',hashes)
    members=[p for p in files() if not p.startswith('firmware/')]+['firmware/'+NAME+'.uf2']
    with zipfile.ZipFile(firmware/(NAME+'_bundle.zip'),'w',zipfile.ZIP_DEFLATED,compresslevel=9) as bundle:
        for p in sorted(set(members)):
            info=zipfile.ZipInfo('interface_utk_v1/'+p,date_time=(2026,10,28,0,0,0))
            info.compress_type=zipfile.ZIP_DEFLATED
            bundle.writestr(info,(ROOT/p).read_bytes())
    names=[NAME+'.uf2',NAME+'_bundle.zip']
    (firmware/'SHA256SUMS.txt').write_bytes(''.join(digest((firmware/n).read_bytes())+'  '+n+'\n' for n in names).encode('ascii'))

def verify():
    hashes=json.loads((ROOT/'SOURCE_SHA256.json').read_text())
    for p,expected in hashes.items():assert digest((ROOT/p).read_bytes())==expected,p
    assert digest((ROOT/'dashboard/vendor/mqtt.min.js').read_bytes())==VENDOR_SHA
    for line in (ROOT/'firmware/SHA256SUMS.txt').read_text().splitlines():
        expected,name=line.split('  ');assert digest((ROOT/'firmware'/name).read_bytes())==expected,name
    data=(ROOT/'firmware'/(NAME+'.uf2')).read_bytes();assert len(data)%512==0
    payload=bytearray();addresses=[];count=len(data)//512
    for i in range(count):
        block=data[i*512:(i+1)*512]
        a,b,flags,address,size,number,total,family=struct.unpack_from('<8I',block)
        assert (a,b,family)==(0x0a324655,0x9e5d5157,0xe48bff56)
        assert flags==0x2000 and size==256 and number==i and total==count
        assert struct.unpack_from('<I',block,508)[0]==0x0ab16f30
        assert 0x10000000<=address and address+size<=0x101aa000
        addresses.append(address);payload.extend(block[32:32+size])
    assert addresses==list(range(0x10000000,0x10000000+count*256,256))
    for marker in [b'BUDDY123 UTK NETWORK V1',b'MOTOR OUTPUTS DISABLED',b'mqtt: connecting to']:
        assert marker in payload,marker
    symbols=(ROOT/'logs/release_symbols.txt').read_text()
    for required in [' T b1_net_ready',' T cyw43_utk_task',' T mqtt_utk_start',' W mc_set_wheel_speeds']:
        assert required in symbols,required
    app=(ROOT/'source/app_program/app_main.c').read_text()
    for n in [2,3,4,5]:assert f'subsystem{n}_start()' not in app
    assert 'for(UINT pin=8;pin<=11;++pin)' in app
    assert not list((ROOT/'source').rglob('wifi_secrets.h'))
    opts=(ROOT/'source/lib/liblwip/sysdepend/pico_rp2040/lwipopts.h').read_text()
    mqtt=(ROOT/'source/lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c').read_text()
    assert '#define MEMP_NUM_SYS_TIMEOUT        (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)' in opts
    assert '#if MEMP_NUM_SYS_TIMEOUT < (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)' in mqtt
    assert 'if(!b1_net_ready()) return;' in mqtt
    with zipfile.ZipFile(ROOT/'firmware'/(NAME+'_bundle.zip')) as bundle:
        assert bundle.testzip() is None
        actual=set(bundle.namelist())
        expected={'interface_utk_v1/'+p for p in files() if not p.startswith('firmware/')}
        expected.add('interface_utk_v1/firmware/'+NAME+'.uf2')
        assert actual==expected,(actual^expected)
        for p in expected:assert bundle.read(p)==(ROOT/p.removeprefix('interface_utk_v1/')).read_bytes(),p
    print(f'PASS release: {count} RP2040 blocks; flash end 0x{addresses[-1]+256:08x}; source, bundle and vendor hashes match; motors disabled; timeout guard present.')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--package',action='store_true')
    if parser.parse_args().package:package()
    verify()
