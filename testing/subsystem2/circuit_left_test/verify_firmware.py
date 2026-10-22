"""Check delivered UF2 structure, identity and separation from saved logs."""
from pathlib import Path
import hashlib
import struct

root = Path(__file__).resolve().parent
artifact = root / "firmware/buddy23_circuit_left_v1.uf2"
data = artifact.read_bytes()
assert data and len(data) % 512 == 0
count = len(data) // 512
payload = bytearray()
for index in range(count):
    block = data[index * 512:(index + 1) * 512]
    start, magic, flags, address, size, number, total, family = struct.unpack_from("<8I", block)
    assert (start, magic, flags, size, number, total, family) == (
        0x0A324655, 0x9E5D5157, 0x2000, 256, index, count, 0xE48BFF56)
    assert struct.unpack_from("<I", block, 508)[0] == 0x0AB16F30
    assert address == 0x10000000 + index * 256
    assert address + size <= 0x101E0000, "Circuit/legacy saved records would be overwritten"
    payload.extend(block[32:32 + size])
for marker in (b"BUDDY23 CIRCUIT LEFT v1", b"SUMMARY: total_time_s=", b"CIRCUIT LEFT v1: key="):
    assert marker in payload, marker
built = root / "build/buddy23_circuit_left_v1.uf2"
if built.exists():
    assert built.read_bytes() == data
print(f"PASS: {count} sequential RP2040 blocks; end=0x{0x10000000+len(payload):08x}; saved records excluded")
print(f"bytes={len(data)} sha256={hashlib.sha256(data).hexdigest()}")
