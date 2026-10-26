from pathlib import Path
import hashlib
import struct
root=Path(__file__).resolve().parent
p=root/'firmware/buddy23_straight_drive_probe_v1.uf2'
data=p.read_bytes()
assert data and len(data)%512==0
count=len(data)//512
payload=bytearray()
for n in range(count):
    block=data[n*512:(n+1)*512]
    a,b,flags,address,size,index,total,family=struct.unpack_from('<8I',block)
    assert (a,b,flags,size,index,total,family)==(0x0A324655,0x9E5D5157,0x2000,256,n,count,0xE48BFF56)
    assert struct.unpack_from('<I',block,508)[0]==0x0AB16F30
    assert address==0x10000000+n*256 and address+size<=0x101b0000
    payload.extend(block[32:32+size])
for marker in (b'BUDDY23 STRAIGHT DRIVE PROBE v1',b'fixed45/45mm/s for4s',b'4-second fixed-speed diagnostic complete',b'profile always1',b'MOTION_DUMP_END:',b'key=%c',b'CRC-verified'):
    assert marker in payload,marker
assert b'BUDDY23 CIRCUIT WITH BARCODE v15' not in payload
assert (root/'build/buddy23_straight_drive_probe_v1.uf2').read_bytes()==data
normal=root.parent/'circuit_with_barcode/firmware/buddy23_circuit_with_barcode_v15.uf2'
assert hashlib.sha256(normal.read_bytes()).hexdigest()=='42143adba24e31670ae145cb53d3145ad2b4991e7d2c0834ab66b416b4fbf1b2'
print(f'PASS: {count} RP2040 blocks; end=0x{0x10000000+len(payload):08x}; normal V15 unchanged')
print(f'bytes={len(data)} sha256={hashlib.sha256(data).hexdigest()}')
