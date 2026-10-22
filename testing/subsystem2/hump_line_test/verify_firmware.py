"""Verify delivered RP2040 UF2 identity, contiguity, and reserved flash sectors."""
from pathlib import Path
import hashlib
import struct

root = Path(__file__).resolve().parent
artifact = root / "firmware/buddy23_hump_line_v3.uf2"
data = artifact.read_bytes()
assert len(data) % 512 == 0
count = len(data) // 512
payload = bytearray()
for index in range(count):
    block = data[index * 512:(index + 1) * 512]
    start, magic, flags, address, size, number, total, family = struct.unpack_from("<8I", block)
    assert (start, magic, flags, size, number, total, family) == (
        0x0A324655, 0x9E5D5157, 0x2000, 256, index, count, 0xE48BFF56)
    assert struct.unpack_from("<I", block, 508)[0] == 0x0AB16F30
    assert address == 0x10000000 + index * 256
    assert address + size <= 0x101FE000, "Reserved hump or junction sector would be overwritten"
    payload.extend(block[32:32 + size])
assert b"BUDDY23 HUMP LINE v3" in payload
assert b"HUMP LINE v%u: key=" in payload
built = root / "build/buddy23_hump_line_v3.uf2"
if built.exists():
    assert built.read_bytes() == data
print(f"PASS: {count} sequential RP2040 blocks; end=0x{0x10000000+len(payload):08x}; reserved sectors excluded")
print(f"bytes={len(data)} sha256={hashlib.sha256(data).hexdigest()}")
