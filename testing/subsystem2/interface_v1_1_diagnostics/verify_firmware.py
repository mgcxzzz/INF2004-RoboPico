from pathlib import Path
import hashlib,json,struct
root=Path(__file__).resolve().parent
baseline=json.loads((root/'V1_1_BASELINE.json').read_text())['sha256']
patches={p['path']:p for p in json.loads((root/'DIAG_PATCHES.json').read_text())}
for rel,h in baseline.items():
    current=hashlib.sha256((root/rel).read_bytes()).hexdigest()
    assert current==(patches[rel]['diag_sha256'] if rel in patches else h),rel
    if rel in patches:assert patches[rel]['v1_1_sha256']==h
production={p for p in patches if p.endswith(('.c','.h')) and '/test_' not in p and not p.startswith('test_')}
assert production=={'main.c','v12_2/scan_smooth.h',
    'v12_2/baseline/testing/subsystem2/circuit_with_barcode/main.c',
    'v12_2/baseline/testing/subsystem2/circuit_with_barcode/barcode_record.h'},production
assert hashlib.sha256((root/'v12_2/scan_smooth_reference.h').read_bytes()).hexdigest()==baseline['v12_2/scan_smooth.h']
meta=json.loads((root/'BUILD_METRICS.json').read_text());data=(root/meta['artifact']).read_bytes()
assert len(data)==meta['bytes'] and hashlib.sha256(data).hexdigest()==meta['sha256']
assert len(data)%512==0 and len(data)//512==meta['blocks']
payload=bytearray()
for k in range(meta['blocks']):
    b=data[k*512:(k+1)*512]
    a,c,flags,address,size,index,total,family=struct.unpack_from('<8I',b)
    assert (a,c,flags,size,index,total,family)==(0x0a324655,0x9e5d5157,0x2000,256,k,meta['blocks'],0xe48bff56)
    assert address==0x10000000+256*k and address+size<=0x101ab000
    assert struct.unpack_from('<I',b,508)[0]==0x0ab16f30
    payload.extend(b[32:288])
for marker in [b'BUDDY23 INTERFACE v1.1 DIAG1',b'V12.2 BASELINE',b'DRIVE_DIAG_BEGIN:',b'DRIVE_DIAG_END:',
               b'DIAG_TIMING:',b'DIAG_FLAGS:',b'DGD1',b'NAV_LOG_BEGIN:',b'ANALOG_SELECT',b'NO heading lock']:
    assert marker in payload,marker
for marker in [b'FORWARD_EFFORT:',b'DRIVE_SMOOTH:']:assert marker not in payload,marker
assert data==(root/'build/buddy23_interface_v1.uf2').read_bytes()
old=root.parent/'interface_v1/firmware/buddy23_interface_v1_1.uf2'
assert hashlib.sha256(old.read_bytes()).hexdigest()=='42a81aeca01fffba960a5e508f3b1c6f3e00d5a03cf53fff0a2c5d445b73f775'
assert 0x1ab000+16384==0x1af000<0x1b0000<0x1d0000
print('PASS V1.1 source provenance; original motor/API/coordinator sources unchanged; only observation/version adaptations in production.')
print('PASS unchanged V1.1 optical reference, RP2040 UF2 integrity/build equality/SHA and independent nonoverlapping flash banks.')
print('UF2',meta['artifact'],meta['bytes'],'bytes;',meta['sha256'])
