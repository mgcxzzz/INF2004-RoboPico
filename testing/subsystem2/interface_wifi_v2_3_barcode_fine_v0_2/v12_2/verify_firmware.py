from pathlib import Path
import hashlib,json,struct
root=Path(__file__).resolve().parent
manifest=json.loads((root/'BASELINE.json').read_text())
allowed={'main.c','barcode_record.h','barcode_scan.h','test_barcode.c','test_uturn.c'}
changed=[]
for name,sha in manifest['sha256'].items():
 f=root/'baseline'/name
 if hashlib.sha256(f.read_bytes()).hexdigest()!=sha:
  assert name.startswith('testing/subsystem2/circuit_with_barcode/') and f.name in allowed,name
  changed.append(f.name)
assert set(changed)==allowed,changed
artifact=root/'firmware/buddy23_circuit_with_barcode_v12_2.uf2'
data=artifact.read_bytes();assert data and len(data)%512==0
payload=bytearray();count=len(data)//512
for k in range(count):
 block=data[k*512:(k+1)*512];a,b,flags,address,size,index,total,family=struct.unpack_from('<8I',block)
 assert (a,b,flags,size,index,total,family)==(0x0a324655,0x9e5d5157,0x2000,256,k,count,0xe48bff56)
 assert struct.unpack_from('<I',block,508)[0]==0x0ab16f30
 assert address==0x10000000+256*k and address+size<=0x101b0000
 payload.extend(block[32:32+size])
for marker in [b'BUDDY23 CIRCUIT WITH BARCODE v12.2',b'SCAN_SOFT_BEGIN',b'SCAN_SOFT_END',b'V12 faster Z:',b'Rolling line search:',b'ANALOG_SELECT',b'optical_start_gate=OFF',b'RAW_ADC_BEGIN:']:
 assert marker in payload,marker
assert b'V16 coverage steering:' not in payload
assert data==(root/'build/buddy23_circuit_with_barcode_v12_2.uf2').read_bytes()
old=root.parent/'circuit_with_barcode/firmware/buddy23_circuit_with_barcode_v12.uf2'
assert hashlib.sha256(old.read_bytes()).hexdigest()=='c47fe618365ec17640b5ba3b10d803b38f7c8949a885483170872ab8337f66d6'
assert b'NO heading lock' in payload
print('PASS isolated V12 baseline manifest; explicit5file patch set; original V12 artifact unchanged')
print(f'PASS RP2040 UF2 blocks={count} end=0x{0x10000000+len(payload):08x} bytes={len(data)} sha256={hashlib.sha256(data).hexdigest()}')
