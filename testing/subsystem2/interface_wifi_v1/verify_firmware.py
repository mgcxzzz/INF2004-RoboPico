"""Verify the published WiFi artifact, frozen V2 controls and source bundle."""
from pathlib import Path
import hashlib,json,struct,subprocess,zipfile
root=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
meta=json.loads((root/'BUILD_METRICS.json').read_text(encoding='utf-8'))
ledger=json.loads((root/'SOURCE_SHA256.json').read_text(encoding='utf-8'))['files']
for rel,h in ledger.items():assert sha(root/rel)==h,'Source differs: '+rel
baseline=json.loads((root/'WIFI_BASELINE.json').read_text(encoding='utf-8'))
patches={p['path']:p for p in json.loads((root/'WIFI_PATCHES.json').read_text(encoding='utf-8'))}
for rel,h in baseline['sha256'].items():
    expected=patches[rel]['wifi_sha256'] if rel in patches else h
    assert sha(root/rel)==expected,rel
    if rel in patches:assert patches[rel]['v2_sha256']==h
protected=['continuous_drive.h','continuous_steering.h','continuous_follow.h','motion_executor.c','motion_executor.h',
    'buddy_hooks.c','buddy_hooks.h','examples/buddy5_extension.c','examples/buddy5_extension.h']
for rel in baseline['sha256']:
    if rel.endswith(('/circuit_logic.h','/circuit_recovery.h','/symmetric_logic.h','/circuit_wheel.h',
        '/junction_logic.h','/line_search.h','/uturn_logic.h','/ramp_profile.h','/maneuver_power.h')):protected.append(rel)
for rel in protected:assert rel not in patches and sha(root/rel)==baseline['sha256'][rel],rel
artifact=root/meta['artifact'];data=artifact.read_bytes()
assert sha(artifact)==meta['sha256'] and len(data)==meta['bytes']
assert len(data)%512==0 and len(data)//512==meta['blocks']
payload=bytearray()
for k in range(meta['blocks']):
    block=data[k*512:(k+1)*512]
    a,b,flags,address,size,index,total,family=struct.unpack_from('<8I',block)
    assert (a,b,flags,size,index,total,family)==(0x0a324655,0x9e5d5157,0x2000,256,k,meta['blocks'],0xe48bff56)
    assert address==0x10000000+256*k and address+size<=0x101aa000
    assert struct.unpack_from('<I',block,508)[0]==0x0ab16f30
    payload.extend(block[32:288])
for marker in [b'BUDDY123 INTERFACE WIFI v1.1',b'WIFI_LINK:',b'no boot assignments',b'WIFI_PROVISIONED:',b'WIFI_SETUP_REJECTED:',
    b'interface-wifi1',b'DRIVE_DIAG_BEGIN:',b'NAV_LOG_BEGIN:',b'is28/cmd',b'is28/tlm',b'is28/hb']:
    assert marker in payload,marker
assert b'*A* LEFT / *F* RIGHT' not in payload
assert meta['default_buddy_extension'] is None and meta['embedded_credentials'] is False
assert meta['packed_shared_ram_saving']==4960
assert int(meta['symbols']['__flash_binary_end'],16)<0x101aa000
assert int(meta['symbols']['__bss_end__'],16)<int(meta['symbols']['__StackLimit'],16)
assert sha(root/'dashboard/vendor/mqtt.min.js')=='b088a7f9045df4e478dbc378f41125066e43d9c602755ee4c5cda0f3e9380ba0'
assert not (root/'wifi_config.local.h').exists(),'Published artifact must be built without a local credentials header.'
assert 'return cyw43_tcpip_link_status(' in (root/'wifi_link_state.h').read_text()
assert 'cyw43_wifi_link_status(' not in (root/'wifi_transport.c').read_text()
build=root/'build-wifi-on'
if build.exists():
    assert artifact.read_bytes()==(build/'buddy123_interface_wifi_v1.uf2').read_bytes()
    cache=(build/'CMakeCache.txt').read_text(encoding='utf-8')
    assert 'BUDDY_WIFI_ENABLED:BOOL=ON' in cache and 'BUDDY_EXTENSION_SOURCE:FILEPATH=\n' in cache
    assert 'INTERFACE_UNPACKED_REFERENCE:BOOL=OFF' in cache
repo=root.parents[2]
if (repo/'.git').exists():
    for path in (artifact,root/meta['bundle']):
        result=subprocess.run(['git','ls-files','--error-unmatch','--',path.relative_to(repo).as_posix()],cwd=repo,capture_output=True)
        assert result.returncode==0,'Artifact is ignored/untracked: '+str(path)
    old=repo/'testing/subsystem2/interface_continuous_v2/firmware/buddy23_interface_continuous_v2.uf2'
    if old.exists():assert sha(old)=='591fa3c93e3e3cc33b900ec43b71f738964d87c17747b8db5df0f824cdc0e4a0'
bundle=root/meta['bundle']
with zipfile.ZipFile(bundle) as z:
    names=set(z.namelist());prefix=root.name+'/'
    assert all(name.startswith(prefix) and '..' not in Path(name).parts for name in names)
    assert not any('wifi_config.local.h' in n or 'wifi_settings.local.json' in n or '/build' in n or '__pycache__' in n for n in names)
    assert hashlib.sha256(z.read(prefix+meta['artifact'])).hexdigest()==meta['sha256']
    for rel,h in ledger.items():assert hashlib.sha256(z.read(prefix+rel)).hexdigest()==h,rel
print('PASS source provenance, frozen V2 controls, empty Buddy5 hooks, RP2040 UF2/flash bounds, default network configuration, tracked artifacts and complete source/dashboard bundle.')
print('UF2',meta['bytes'],'bytes',meta['sha256'])
