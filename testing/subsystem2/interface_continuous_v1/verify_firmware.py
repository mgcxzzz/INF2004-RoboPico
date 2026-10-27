"""Verify packaged continuous-drive test identity, source scope and UF2 bounds."""
from pathlib import Path
import hashlib,json,struct
root=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
base=json.loads((root/'CONTINUOUS_BASELINE.json').read_text())['sha256']
patch={p['path']:p for p in json.loads((root/'CONTINUOUS_PATCHES.json').read_text())}
for rel,h in base.items():
    assert sha(root/rel)==(patch[rel]['continuous_sha256'] if rel in patch else h),rel
    if rel in patch:assert patch[rel]['handover_sha256']==h
for rel,p in patch.items():assert sha(root/rel)==p['continuous_sha256'],rel
protected=['nav_api.h','motion_api.h','motion_api.c','nav_coordinator.h','nav_coordinator.c',
 'motion_port.h','motion_executor.h','motion_executor.c','nav_codec.h','nav_codec.c','buddy_hooks.h','buddy_hooks.c',
 'examples/buddy5_extension.c','examples/buddy5_extension.h']
for rel in protected:assert rel not in patch and sha(root/rel)==base[rel],rel
core=['barcode_scan.h','barcode_nav.h','barcode_analog.h','uturn_logic.h','maneuver_power.h',
      'ramp_profile.h','barcode_raw.h','barcode_start.h']
for name in core:
    rel='v12_2/baseline/testing/subsystem2/circuit_with_barcode/'+name
    assert rel not in patch and sha(root/rel)==base[rel],rel
for rel in base:
    if rel.endswith(('/circuit_logic.h','/circuit_recovery.h','/junction_logic.h','/symmetric_logic.h','/line_search.h')):
        assert rel not in patch and sha(root/rel)==base[rel],rel
meta=json.loads((root/'BUILD_METRICS.json').read_text());data=(root/meta['artifact']).read_bytes()
assert len(data)==meta['bytes'] and sha(root/meta['artifact'])==meta['sha256']
assert len(data)%512==0 and len(data)//512==meta['blocks']
payload=bytearray()
for k in range(meta['blocks']):
    b=data[k*512:(k+1)*512]
    a,c,flags,address,size,index,total,family=struct.unpack_from('<8I',b)
    assert (a,c,flags,size,index,total,family)==(0x0a324655,0x9e5d5157,0x2000,256,k,meta['blocks'],0xe48bff56)
    assert address==0x10000000+256*k and address+size<=0x101ab000
    assert struct.unpack_from('<I',b,508)[0]==0x0ab16f30
    payload.extend(b[32:288])
assert data==(root/'build/buddy23_interface_v1.uf2').read_bytes()
for marker in [b'BUDDY23 INTERFACE CONTINUOUS v1',b'DRIVE_DIAG_BEGIN:',b'control=continuous1 capture=3',
 b'DIAG_TIMING:',b'DRIVER0=legacy_ordinary_or_loaded,1=maneuver,2=ramp,3=continuous',b'NAV_LOG_BEGIN:',b'ANALOG_SELECT']:
    assert marker in payload,marker
for old,h in [('interface_v1_1_handover/firmware/buddy23_interface_v1_1_handover_v1.uf2','ca15cf9d0f680e00b313d7d6189d3b540098ec54c47030bc47f64b4dd709524e'),
 ('interface_v1/firmware/buddy23_interface_v1_1.uf2','42a81aeca01fffba960a5e508f3b1c6f3e00d5a03cf53fff0a2c5d445b73f775')]:
    assert sha(root.parent/old)==h
cache=(root/'build/CMakeCache.txt').read_text()
assert 'BUDDY_EXTENSION_SOURCE:FILEPATH=\n' in cache
assert 'INTERFACE_UNPACKED_REFERENCE:BOOL=OFF' in cache
assert meta['default_buddy_extension'] is None
assert meta['packed_shared_ram_saving']==4960
assert meta['diagnostic_bits']==459 and meta['diagnostic_row_bytes']==58
assert int(meta['symbols']['__flash_binary_end'],16)<0x101ab000
assert int(meta['symbols']['__bss_end__'],16)<int(meta['symbols']['__StackLimit'],16)
print('PASS API/coordinator/public data layout and empty default Buddy hooks unchanged; legacy optical/barcode/manoeuvre sources protected.')
print('PASS explicit continuous-drive source manifest, RP2040 UF2 block integrity, build equality, checksum and flash bank bounds.')
print('UF2',meta['artifact'],meta['bytes'],'bytes;',meta['sha256'])
