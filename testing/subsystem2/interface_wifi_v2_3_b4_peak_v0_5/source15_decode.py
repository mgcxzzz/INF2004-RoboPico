#!/usr/bin/env python3
"""Decode BOTH source-15 layouts of the combined build from one native mosquitto_sub capture
(lines: ISO-time topic hex) or bare hex lines, in capture order:
  - ramp assist v0.2 (+ Buddy4 estimator marker): payload 28 bytes (31-byte frame)
  - barcode peak v0.1: payload 24 bytes (27-byte frame), payload[4] = 0xB1
Dispatch is by validated header, length and subtype, never by source number alone. Any
other source-15 frame is reported as 'unknown' (not guessed).
Usage: source15_decode.py CAPTURE [FROM_LINE [TO_LINE]]   -> kind,time,key=value...
"""
import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import ramp_assist_decode as ra   # noqa: E402
import barcode_peak_decode as bp  # noqa: E402

def classify(b):
    """Return ('ramp'|'peak'|'unknown'|None, decoded dict or None). None = not source 15."""
    if len(b) < 3 or b[0] != 0xF2:
        return None, None
    if len(b) != 3 + b[1]:
        return 'unknown', None
    if b[1] == 28:
        d = ra.decode(b); return ('ramp', d) if d else ('unknown', None)
    if b[1] == 24 and b[7] == 0xB1:
        d = bp.decode(b); return ('peak', d) if d else ('unknown', None)
    return 'unknown', None

def main():
    path = sys.argv[1]; a = int(sys.argv[2]) if len(sys.argv) > 2 else 1; z = int(sys.argv[3]) if len(sys.argv) > 3 else 1 << 62
    with open(path) as f:
        for n, line in enumerate(f, 1):
            if n < a or n > z: continue
            p = line.split()
            hx, t = (p[2], p[0]) if len(p) >= 3 else (p[0] if p else '', '')
            if not hx.lower().startswith('f2'): continue
            try: b = bytes.fromhex(hx)
            except ValueError: continue
            kind, d = classify(b)
            if kind is None: continue
            if d is None: print(f'unknown,{t},len={len(b)},hex={hx}'); continue
            keys = ra.FIELDS if kind == 'ramp' else bp.FIELDS
            print(kind + ',' + t + ',' + ','.join(f'{k}={d[k]}' for k in keys if k != 'time'))
if __name__ == '__main__': main()
