from pathlib import Path
import hashlib
import struct

root = Path(__file__).resolve().parent
artifact = root / 'firmware/buddy23_barcode_analog_probe_v5.uf2'
data = artifact.read_bytes()
assert data and len(data) % 512 == 0
count = len(data) // 512
payload = bytearray()
for n in range(count):
    block = data[n*512:(n+1)*512]
    a, b, flags, address, size, index, total, family = struct.unpack_from('<8I', block)
    assert (a, b, flags, size, index, total, family) == (
        0x0A324655, 0x9E5D5157, 0x2000, 256, n, count, 0xE48BFF56)
    assert struct.unpack_from('<I', block, 508)[0] == 0x0AB16F30
    assert address == 0x10000000+n*256 and address+size <= 0x101c0000
    payload.extend(block[32:288])
for marker in (b'BUDDY23 BARCODE ANALOG PROBE v5', b'GP27/ADC1',
               b'ANALOG_SWEEP v5', b'CAL SAVED: CRC/readback verified',
               b'SWEEP_CAL:', b'missed_slots=', b'motors=OFF', b'SWEEP_DUMP_END:'):
    assert marker in payload, marker
assert (root/'build'/artifact.name).read_bytes() == data
for v, sha in (
    (1, '031a6276a25ce3f0b3bafa34ddf7816a03c0f29726254835e9935e5ed434fccf'),
    (2, '800d727f8ef512ade68506252c3a0e433b8ea102d0ccf5d359ab909f2f0f4133'),
    (3, 'd063f755bbc5e495609d82216f13cda5ad8f7ef49b92f4821a816b737575951c'),
    (4, 'e3e52ffe7a9cb2692c563556f16b1df21e474b384bc9d3857a79d5ed163db044')):
    assert hashlib.sha256((root/f'firmware/buddy23_barcode_analog_probe_v{v}.uf2').read_bytes()).hexdigest() == sha
print(f'PASS: {count} RP2040 blocks; end=0x{0x10000000+count*256:08x}; V1-V4 unchanged')
print(f'bytes={len(data)} sha256={hashlib.sha256(data).hexdigest()}')
