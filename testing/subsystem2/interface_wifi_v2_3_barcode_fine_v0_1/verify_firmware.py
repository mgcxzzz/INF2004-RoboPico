"""Verify the V2.3 release against its working SDK V2.2 base and package."""
from pathlib import Path
import hashlib,json,struct,zipfile
root=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
meta=json.loads((root/'BUILD_METRICS.json').read_text())
ledger=json.loads((root/'SOURCE_SHA256.json').read_text())['files']
for rel,h in ledger.items(): assert sha(root/rel)==h,'Source differs: '+rel
base=json.loads((root/'WORKING_V2_2_BASELINE.json').read_text())
patches=json.loads((root/'CALIBRATION_PATCH.json').read_text())['changes']
assert set(patches)=={'CMakeLists.txt',
    'v12_2/baseline/testing/subsystem2/circuit_with_barcode/main.c',
    'v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h'}
documentation={'.gitattributes','README.md','BUILD.md','TEST_LOG.md','STACK_LAYOUT.md',
    'BUILD_METRICS.json','SOURCE_SHA256.json','run_tests.py','verify_firmware.py'}
for rel,h in base['files'].items():
    if rel in documentation:continue
    if rel in patches:
        patch=patches[rel]
        assert patch['base_sha256']==h and sha(root/rel)==patch['modified_sha256'],rel
        lines=(root/rel).read_bytes().decode().splitlines(True)
        for part in reversed(patch['hunks']):
            pos=part['new_start'];replacement=part['new']
            assert lines[pos:pos+len(replacement)]==replacement,rel
            lines[pos:pos+len(replacement)]=part['old']
        assert hashlib.sha256(''.join(lines).encode()).hexdigest()==h,'Non-approved change: '+rel
    else:assert sha(root/rel)==h,'Working V2.2 file changed: '+rel
assert not (root/'wifi_config.local.h').exists(),'Local credentials header present'
assert not meta['embedded_credentials'] and meta['default_buddy_extension'] is None
assert meta['pico_sdk_commit']=='98a542c1a62fb549ffb5d66a3e5892b06276b670'
assert meta['calibration']=={'white':[163,160],'black':[3457,3700],'record_version':28}
main=(root/'v12_2/baseline/testing/subsystem2/circuit_with_barcode/main.c').read_text()
assert "else if(ch=='c')" in main and 'wifi_start_pending=false;arm.state=ARM_IDLE;' in main
assert 'MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)' in (root/'lwipopts.h').read_text()
ss={k:int(v,16) for k,v in meta['symbols'].items()}
assert ss['__flash_binary_end']<0x101aa000
assert ss['__StackTop']-ss['__StackBottom']==8192
assert ss['__StackOneTop']-ss['__StackOneBottom']==8192
assert ss['__bss_end__']<=ss['__HeapLimit']==ss['__StackLimit']<=ss['__StackOneBottom']
assert ss['__StackOneTop']<=ss['__StackBottom']<ss['__StackTop']<=0x20040000
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
assert struct.unpack_from('<I',payload,256)[0]==ss['__StackTop'],'Boot vector SP differs'
for marker in (b'BUDDY123 INTERFACE WIFI v2.3',b'CAL RESTORED:',b'WIFI_MQTT:',
    b'tcp_connect',b'await_connack',b'await_suback',b'WIFI_PROGRESS:',b'WIFI_SETUP:',
    b'WIFI_PROVISIONED:',b'WIFI_SETUP_REJECTED:',b'is28/cmd',b'is28/tlm',b'is28/hb'):
    assert marker in payload,marker
assert b'TIMEBASE: FAIL' not in payload and b'UTK DRIVE V1.1 init failed' not in payload
with zipfile.ZipFile(root/meta['bundle']) as z:
    names=set(z.namelist());prefix=root.name+'/'
    assert len(names)==len(z.namelist()),'Duplicate ZIP members'
    for name in names:
        assert name.startswith(prefix) and '..' not in Path(name).parts
        parts=Path(name).parts[1:]
        assert not any(p.startswith('build') or p in ('__pycache__','dashboard_session') for p in parts)
        assert Path(name).name not in ('wifi_config.local.h','wifi_settings.local.json')
    assert hashlib.sha256(z.read(prefix+meta['artifact'])).hexdigest()==meta['sha256']
    for rel,h in ledger.items():assert hashlib.sha256(z.read(prefix+rel)).hexdigest()==h,rel
    for rel in ('BUILD_METRICS.json','SOURCE_SHA256.json'):
        assert z.read(prefix+rel)==(root/rel).read_bytes(),rel
print('PASS V2.3 source ledger, working V2.2 preservation, exact approved calibration patch, UF2/flash/stack bounds and complete package.')
print('No embedded credentials or default Buddy5 extension. Physical V2.3 test pending.')
print('UF2:',meta['bytes'],'bytes;',meta['sha256'])
