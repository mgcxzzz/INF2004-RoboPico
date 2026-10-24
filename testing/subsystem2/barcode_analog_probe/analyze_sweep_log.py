"""Audit mixed PuTTY sessions without treating missing CSV rows as sensor failures."""
from dataclasses import dataclass, field
from pathlib import Path
import math
import re
import sys
import zlib

@dataclass
class Sweep:
    version: int
    expected: int
    rows: list = field(default_factory=list)
    malformed: int = 0
    end: bool = False
    footer_rows: int = -1
    footer_crc: int = -1
    crc: int = 0

    def summary(self):
        offset = 1 if self.version >= 5 else 0
        times = [r[offset] for r in self.rows]
        monotonic = all(b > a for a, b in zip(times, times[1:]))
        indexes = self.version < 5 or [int(r[0]) for r in self.rows] == list(range(len(self.rows)))
        complete = self.end and len(self.rows) == self.expected and not self.malformed and monotonic and indexes
        verified = complete and self.version >= 5 and self.footer_rows == self.expected and self.footer_crc == self.crc
        return dict(version=self.version, expected=self.expected, received=len(self.rows),
                    malformed=self.malformed, end=self.end, complete=complete,
                    checksum_verified=verified, indexes_ok=indexes,
                    largest_time_gap_s=max((b-a for a,b in zip(times,times[1:])), default=0)/1e6,
                    final_time_s=times[-1]/1e6 if times else None,
                    last_distance_mm=self.rows[-1][-1] if self.rows else None,
                    ADC_range=[min(r[offset+1] for r in self.rows), max(r[offset+1] for r in self.rows)] if self.rows else None)

def parse(text):
    result = []
    current = None
    for line in text.splitlines():
        start = re.match(r'^ANALOG_SWEEP v(\d+): samples=(\d+)\b', line)
        if start:
            if current is not None:
                result.append(current)
            current = Sweep(int(start[1]), int(start[2]))
            continue
        if current is None:
            continue
        footer = re.match(r'^SWEEP_DUMP_END: rows=(\d+) crc32=([0-9a-fA-F]{8})\b', line)
        if footer:
            current.footer_rows, current.footer_crc = int(footer[1]), int(footer[2], 16)
        elif line.startswith('END ANALOG_SWEEP'):
            current.end = True
            result.append(current)
            current = None
        elif re.match(r'^\d+,', line):
            fields = line.split(',')
            try:
                expected_fields = 9 if current.version >= 5 else 8
                if len(fields) != expected_fields:
                    raise ValueError('column count')
                values = [float(v) for v in fields]
                if not all(math.isfinite(v) for v in values):
                    raise ValueError('nonfinite sample')
                offset = 1 if current.version >= 5 else 0
                for j in range(offset+5):
                    int(fields[j])
                if not 0 <= values[offset+1] <= 4095 or not -1 <= values[offset+2] <= 1000:
                    raise ValueError('ADC/level range')
                current.rows.append(values)
                current.crc = zlib.crc32((line+'\n').encode('ascii'), current.crc)
            except (ValueError, UnicodeError):
                current.malformed += 1
    if current is not None:
        result.append(current)
    return result

if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise SystemExit('Usage: python analyze_sweep_log.py putty.txt')
    import json
    for n, sweep in enumerate(parse(Path(sys.argv[1]).read_text(errors='replace')), 1):
        print(json.dumps(dict(sweep=n, **sweep.summary())))
