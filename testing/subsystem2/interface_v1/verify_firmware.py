from pathlib import Path
import hashlib,json,struct
root=Path(__file__).resolve().parent
baseline=json.loads((root/'V12_2_BASELINE.json').read_text())['sha256']
patches={p['path']:p for p in json.loads((root/'V12_2_PATCHES.json').read_text())}
assert set(patches)=={
 'baseline/testing/subsystem2/circuit_with_barcode/main.c',
 'baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h',
 'baseline/testing/subsystem2/circuit_with_barcode/test_barcode.c',
 'baseline/testing/subsystem2/circuit_with_barcode/test_uturn.c'}
for path,h in baseline.items():
    actual=hashlib.sha256((root/'v12_2'/path).read_bytes()).hexdigest()
    assert actual==(patches[path]['interface_sha256'] if path in patches else h),path
    if path in patches:assert patches[path]['v12_2_sha256']==h
meta=json.loads((root/'BUILD_METRICS.json').read_text())
data=(root/meta['artifact']).read_bytes()
assert len(data)==meta['bytes'] and hashlib.sha256(data).hexdigest()==meta['sha256']
assert len(data)%512==0 and len(data)//512==meta['blocks']
payload=bytearray()
for k in range(meta['blocks']):
    b=data[k*512:(k+1)*512]
    a,c,flags,address,size,index,total,family=struct.unpack_from('<8I',b)
    assert (a,c,flags,size,index,total,family)==(0x0a324655,0x9e5d5157,0x2000,256,k,meta['blocks'],0xe48bff56)
    assert address==0x10000000+256*k and address+size<=0x101af000
    assert struct.unpack_from('<I',b,508)[0]==0x0ab16f30
    payload.extend(b[32:288])
for marker in [b'BUDDY23 INTERFACE v1.1',b'V12.2 BASELINE',b'NAV_LOG_BEGIN:',b'TARGETS_ACCEPTED',b'SESSION_END',
               b'ANALOG_SELECT',b'V12 faster Z:',b'NO heading lock',b'RAW_ADC_BEGIN:',b'NAV_COORD:']:
    assert marker in payload,marker
if (root/'build/buddy23_interface_v1.uf2').exists():
    assert data==(root/'build/buddy23_interface_v1.uf2').read_bytes()
old=root.parent/'circuit_with_barcode_v12_2/firmware/buddy23_circuit_with_barcode_v12_2.uf2'
if old.exists():assert hashlib.sha256(old.read_bytes()).hexdigest()=='76941390c3779295ff750596a76d84e575a92c2b26bc78a3203771efa052641c'
print('PASS frozen V12.2 algorithms/manifest, four explicit harness/record adaptations, original artifact unchanged when present.')
print('PASS RP2040 UF2, contiguous256-byte payloads, image below independent log sectors, expected interface/legacy strings and SHA256:',meta['sha256'])
print('UF2 bytes=',len(data),'blocks=',meta['blocks'],'flash_end=',meta['padded_flash_end'])

original=root/'firmware/buddy23_interface_v1.uf2'
assert hashlib.sha256(original.read_bytes()).hexdigest()=='abef641045462f2e452265e352e32de996fd2b3d63c70551f0c3e0186b1b52dc'
print('PASS original V1 artifact unchanged; current release is V1.1 (supersedes V1 startup bug).')
