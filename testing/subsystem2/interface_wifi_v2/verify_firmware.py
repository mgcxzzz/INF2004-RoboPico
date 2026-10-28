"""Verify the recorded V2 source, preserved controller and firmware package."""
from pathlib import Path
import hashlib,json,struct,zipfile
root=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
meta=json.loads((root/'BUILD_METRICS.json').read_text())
ledger=json.loads((root/'SOURCE_SHA256.json').read_text())['files']
for rel,h in ledger.items(): assert sha(root/rel)==h,'Source differs: '+rel
base=json.loads((root/'WIFI_V2_BASELINE.json').read_text())
for rel in base['protected']:
    if rel=='lwipopts.h':
        patch="""/* lwIP's default only budgets core timers. The MQTT cyclic timer is an
 * application timer, started after TCP connects; reserve bounded headroom. */
#define MEMP_NUM_SYS_TIMEOUT            (LWIP_NUM_SYS_TIMEOUT_INTERNAL + 4)
"""
        text=(root/rel).read_bytes();assert patch.encode() in text
        assert hashlib.sha256(text.replace(patch.encode(),b'')).hexdigest()==base['sha256'][rel]
        continue
    assert sha(root/rel)==base['sha256'][rel],'Controller differs: '+rel
main=base['banner_only_main']
restored=(root/main).read_bytes().replace(b'BUDDY123 INTERFACE WIFI v2.2',b'BUDDY123 INTERFACE WIFI v1.2')
assert hashlib.sha256(restored).hexdigest()==base['sha256'][main],'Nested main changed beyond banner'
assert not (root/'wifi_config.local.h').exists(),'Remove local credentials header for release verification'
assert not meta['embedded_credentials'] and meta['default_buddy_extension'] is None
assert 'return cyw43_tcpip_link_status(' in (root/'wifi_link_state.h').read_text()
assert int(meta['symbols']['__flash_binary_end'],16)<0x101aa000
assert int(meta['symbols']['__bss_end__'],16)<int(meta['symbols']['__StackLimit'],16)
ss={k:int(v,16) for k,v in meta['symbols'].items()}
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
assert struct.unpack_from('<I',payload,256)[0]==ss['__StackTop'],'Boot vector stack pointer differs'
for marker in (b'BUDDY123 INTERFACE WIFI v2.2',b'WIFI_MQTT:',b'tcp_connect',b'await_connack',b'await_suback',
               b'WIFI_PROGRESS:',b'WIFI_SETUP:',b'WIFI_PROVISIONED:',b'WIFI_SETUP_REJECTED:',b'is28/cmd',b'is28/tlm',b'is28/hb'):
    assert marker in payload,marker
assert sha(root/'dashboard/vendor/mqtt.min.js')=='b088a7f9045df4e478dbc378f41125066e43d9c602755ee4c5cda0f3e9380ba0'
with zipfile.ZipFile(root/meta['bundle']) as z:
    names=set(z.namelist());prefix=root.name+'/'
    assert len(names)==len(z.namelist()),'Duplicate ZIP members'
    for name in names:
        assert name.startswith(prefix) and '..' not in Path(name).parts
        parts=Path(name).parts[1:]
        assert not any(p.startswith('build') or p in ('__pycache__','dashboard_session') for p in parts)
        assert Path(name).name not in ('wifi_config.local.h','wifi_settings.local.json')
    assert hashlib.sha256(z.read(prefix+meta['artifact'])).hexdigest()==meta['sha256']
    for rel,h in ledger.items(): assert hashlib.sha256(z.read(prefix+rel)).hexdigest()==h,rel
print('PASS V2 source ledger, preserved V1 controller, banner-only main, UF2 RP2040 blocks/flash bounds, RAM bounds and source/dashboard package.')
print('No embedded credentials or default Buddy5 extension. Physical networking/driving NOT TESTED.')
print('UF2:',meta['bytes'],'bytes;',meta['sha256'])
