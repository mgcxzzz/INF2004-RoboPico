from pathlib import Path
import hashlib
import struct
root=Path(__file__).resolve().parent
artifact=root/'firmware/buddy23_circuit_with_barcode_v14.uf2'
data=artifact.read_bytes();assert data and len(data)%512==0
count=len(data)//512;payload=bytearray()
for n in range(count):
    block=data[n*512:(n+1)*512]
    a,b,flags,address,size,index,total,family=struct.unpack_from('<8I',block)
    assert (a,b,flags,size,index,total,family)==(0x0A324655,0x9E5D5157,0x2000,256,n,count,0xE48BFF56)
    assert struct.unpack_from('<I',block,508)[0]==0x0AB16F30
    assert address==0x10000000+n*256 and address+size<=0x101b0000
    payload.extend(block[32:32+size])
for marker in [b'BUDDY23 CIRCUIT WITH BARCODE v14',b'BARCODE_EVENTS:',b'CIRCUIT WITH BARCODE v%u:',b'decoded_A=',b'Code39',b'BAD_FRAME flags:',b'AUTO ANALOG GP27',b'ANALOG_CANDIDATE',b'ANALOG_SELECT',b'MOTION_DUMP_END:',b'min_span=',b'V13 stationary line search:',b'stationary sweep angle/time bound',b'V14 restores V12 visible-line steering and motor controller',b'optical_start_gate=OFF',b'V8 barcode sensitivity:63 thresholds',b'RAW_ADC_BEGIN:',b'RAW_ADC_END:',b'V9 smooth visible-line steering:',b'V10 PROFILE2 RAMP:',b'RAMP2: straight_target_mm_s=180',b'V11 Z queues U-turn',b'UTURN_STAGE',b'decoded_Z=',b'V12 faster Z:',b'RECOVERY_SWEEP',b'RECOVERY_WAIT',b'RECOVERY_FOUND']:
    if marker==b'V9 smooth visible-line steering:':marker=b'V9 smooth visible-line steering restored as V12:'
    assert marker in payload,marker
assert b'smooth proportional line steering' not in payload
built=root/'build/buddy23_circuit_with_barcode_v14.uf2'
assert not built.exists() or built.read_bytes()==data
print(f'PASS: {count} RP2040 blocks; end=0x{0x10000000+len(payload):08x}; barcode/baseline records preserved')
print(f'bytes={len(data)} sha256={hashlib.sha256(data).hexdigest()}')

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v3.uf2").read_bytes()).hexdigest()=="9946f4d3aaafea3769803a8ad6a406c3ce20be4adb0344a4f2ae9721356ad6bd"
print("PASS: V3 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v4.uf2").read_bytes()).hexdigest()=="7719f10c2b1f7c451bc104be6ac3e7b6a10c0c75864fd1a2db7a108d5022f2c5"
print("PASS: V4 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v5.uf2").read_bytes()).hexdigest()=="0ae311b7a76960d21e2735d5764f984a43a8679e968bba9b4f2f24b4b02793b0"
print("PASS: V5 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v6.uf2").read_bytes()).hexdigest()=="67e7e459ea00077d30093e9530f89a35fa00d58258338f8eac30ee0151130773"
print("PASS: V6 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v7.uf2").read_bytes()).hexdigest()=="3033548bffcdda8954f5758bf372b885f23542ffa738c79603f7666a28edf804"
print("PASS: V7 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v8.uf2").read_bytes()).hexdigest()=="ffecf42fa8cdd61caeb8ccee7b5e2a2e63b10135e08583096566381b3f6fd570"
print("PASS: V8 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v9.uf2").read_bytes()).hexdigest()=="1089a1feeec64c8d89ae86d57f6458749345a189b7e2daeee5138bb6a6da5238"
print("PASS: V9 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v10.uf2").read_bytes()).hexdigest()=="18aba75df836187437c3cd7adff2ab01e682d679173f14900ce88986b603cf05"
print("PASS: V10 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v11.uf2").read_bytes()).hexdigest()=="c01b659635489fa5e503c4317f3ecb23fcb781bd7e4af651f883d3b02dfc70c7"
print("PASS: V11 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v12.uf2").read_bytes()).hexdigest()=="c47fe618365ec17640b5ba3b10d803b38f7c8949a885483170872ab8337f66d6"
print("PASS: V12 UF2 unchanged")

assert hashlib.sha256((root/"firmware/buddy23_circuit_with_barcode_v13.uf2").read_bytes()).hexdigest()=="2437fa97483a029d2e497145ad4c8d04f7c5295696c6c95d3b51037c87e1ca96"
print("PASS: V13 UF2 unchanged")
