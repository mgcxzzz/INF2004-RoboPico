"""Validate V8 a-dumps, including complete-row count and binary-record CRC."""
from decimal import Decimal
from pathlib import Path
import re
import struct
import sys
import zlib


def parse_capture(text):
    # A user can include help, live status and other dumps in the same file.
    blocks = text.split('RAW_ADC_BEGIN:')[1:]
    if not blocks:
        raise ValueError('No RAW_ADC_BEGIN found')
    block = blocks[-1]
    header, *lines = block.splitlines()
    fields = dict(re.findall(r'(\w+)=(\S+)', header))
    expected = int(fields['rows'])
    if not 0 <= expected <= 4096:
        raise ValueError('Invalid capture row count')
    crc_expected = int(fields['data_crc'], 16)
    rows, payload, end = [], bytearray(), None
    for line in lines:
        if line.startswith('RAW_ADC_END:'):
            end = dict(re.findall(r'(\w+)=(\S+)', line))
            break
        if line == 'sample,us,path_mm,adc' or not line.strip():
            continue
        parts = line.split(',')
        if len(parts) != 4:
            raise ValueError(f'Unexpected/truncated raw line: {line[:70]}')
        index, us, path, adc = int(parts[0]), int(parts[1]), Decimal(parts[2])*100, int(parts[3])
        if index != len(rows) or path != int(path) or not 0 <= adc <= 4095:
            raise ValueError('Invalid/missing sample index, path or ADC')
        if rows and us <= rows[-1][0]:
            raise ValueError('Non-increasing sample time')
        payload.extend(struct.pack('<IiHH', us, int(path), adc, 0))
        rows.append((us, int(path), adc))
    if end is None or len(rows) != expected or int(end['rows']) != expected:
        raise ValueError(f'Incomplete capture: got{len(rows)}, expected{expected}, footer={end is not None}')
    crc = zlib.crc32(payload)
    if crc != crc_expected or int(end['data_crc'], 16) != crc_expected:
        raise ValueError('Raw capture CRC mismatch')
    return fields, rows


if __name__ == '__main__':
    fields, rows = parse_capture(Path(sys.argv[1]).read_text(errors='replace'))
    print(f'PASS: {len(rows)} rows; CRC={fields["data_crc"]}; stored={fields.get("stored")}; matches_motion={fields.get("matches_motion")}')
    if rows:
        print(f'ADC range={min(r[2] for r in rows)}..{max(r[2] for r in rows)}; duration_s={(rows[-1][0]-rows[0][0])/1e6:.3f}; path_mm={rows[0][1]/100:.2f}..{rows[-1][1]/100:.2f}')
