"""Check this IMU-only UF2, reserved calibration sectors and absence of drive paths."""
from pathlib import Path
import hashlib
import struct

root = Path(__file__).resolve().parent
artifact = root / 'firmware/buddy4_imu_isolated_v2.uf2'
data = artifact.read_bytes()
assert len(data) % 512 == 0
count = len(data) // 512
payload = bytearray()
for i in range(count):
    block = data[i*512:(i+1)*512]
    magic1, magic2, flags, address, size, number, total, family = struct.unpack_from('<8I', block)
    assert (magic1, magic2, flags, size, number, total, family) == (
        0x0A324655, 0x9E5D5157, 0x2000, 256, i, count, 0xE48BFF56)
    assert struct.unpack_from('<I', block, 508)[0] == 0x0AB16F30
    assert address == 0x10000000+i*256 and address+size <= 0x101FE000
    payload.extend(block[32:32+size])
assert b'BUDDY4 IMU ISOLATED v2' in payload
assert b'NO_CONFIGURED_DEVICE' in payload
assert b'motors=OFF always' in payload
assert (root/'build/buddy4_imu_isolated_v2.uf2').read_bytes() == data
print(f'PASS: RP2040 blocks={count}; bytes={len(data)}; end=0x{0x10000000+len(payload):08x}; reserved hump/junction sectors excluded')
print('sha256='+hashlib.sha256(data).hexdigest())
