from pathlib import Path
import hashlib
import struct
root=Path(__file__).resolve().parent
artifact=root/'firmware/buddy23_circuit_with_barcode_v2.uf2'
data=artifact.read_bytes();assert data and len(data)%512==0
count=len(data)//512;payload=bytearray()
for n in range(count):
    block=data[n*512:(n+1)*512]
    a,b,flags,address,size,index,total,family=struct.unpack_from('<8I',block)
    assert (a,b,flags,size,index,total,family)==(0x0A324655,0x9E5D5157,0x2000,256,n,count,0xE48BFF56)
    assert struct.unpack_from('<I',block,508)[0]==0x0AB16F30
    assert address==0x10000000+n*256 and address+size<=0x101d0000
    payload.extend(block[32:32+size])
for marker in [b'BUDDY23 CIRCUIT WITH BARCODE v2',b'BARCODE_EVENTS:',b'CIRCUIT WITH BARCODE v%u:',b'decoded_A=',b'Code39']:
    assert marker in payload,marker
built=root/'build/buddy23_circuit_with_barcode_v2.uf2'
assert not built.exists() or built.read_bytes()==data
print(f'PASS: {count} RP2040 blocks; end=0x{0x10000000+len(payload):08x}; barcode/baseline records preserved')
print(f'bytes={len(data)} sha256={hashlib.sha256(data).hexdigest()}')
