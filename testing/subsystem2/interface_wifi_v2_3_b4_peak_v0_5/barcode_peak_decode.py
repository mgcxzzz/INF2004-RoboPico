#!/usr/bin/env python3
"""Decode barcode-peak v0.1 source-15 frames (0xF2, len 24, subtype 0xB1; combined build sets flags bit7) from a native
mosquitto_sub capture (lines: ISO-time topic hex) or bare hex lines. Prints CSV.
Usage: barcode_peak_decode.py CAPTURE [FROM_LINE [TO_LINE]]"""
import struct, sys
REASON = {0: 'ok', 1: 'too_few_bars', 2: 'bar_height_unclear', 3: 'wide_bar_count', 4: 'unit_out_of_range',
          5: 'space_unclear', 6: 'wide_char_gap', 7: 'not_a_*X*_frame', 8: 'ambiguous', 9: 'no_wide_bars',
          10: 'run_start_announce'}
FIELDS = ['time', 'seq', 'uptime_ms', 'version', 'build', 'decoded', 'applied', 'orig_same_frame', 'orig_conflict',
          'direction', 'overflow', 'code', 'reason', 'bars', 'window', 'removed', 'base_adc', 'min_narrow_peak',
          'max_wide_peak', 'min_narrow_rise', 'max_narrow_space_u', 'min_wide_space_u', 'unit_mm', 'span_mm', 'groups',
          'combined_build']
def decode(b):
    if len(b) != 27 or b[0] != 0xF2 or b[1] != 24 or (b[7] & 0xF0) != 0xB0:
        return None
    up, sub, fl, code, why, bars, win, base, npk, wpk, nrise, nsp, wsp, unit, span, groups = \
        struct.unpack('<IBBBBBBHHHBBBBHH', bytes(b[3:]))
    return {'seq': b[2], 'uptime_ms': up, 'version': f'v0.{sub & 15}', 'build': 'active' if fl & 1 else 'shadow',
            'decoded': int(bool(fl & 2)), 'applied': int(bool(fl & 4)), 'orig_same_frame': int(bool(fl & 8)),
            'orig_conflict': int(bool(fl & 32)), 'direction': 'rev' if fl & 16 else 'fwd', 'overflow': int(bool(fl & 64)),
            'code': chr(code), 'reason': REASON.get(why, why), 'bars': bars, 'window': win & 15, 'removed': (win >> 4) & 3,
            'base_adc': base, 'min_narrow_peak': npk, 'max_wide_peak': wpk, 'min_narrow_rise': nrise,
            'max_narrow_space_u': nsp / 20, 'min_wide_space_u': wsp / 20, 'unit_mm': unit / 50, 'span_mm': span / 10,
            'groups': groups, 'combined_build': int(bool(fl & 0x80))}
def main():
    path = sys.argv[1]; a = int(sys.argv[2]) if len(sys.argv) > 2 else 1; z = int(sys.argv[3]) if len(sys.argv) > 3 else 1 << 62
    print(','.join(FIELDS))
    with open(path) as f:
        for n, line in enumerate(f, 1):
            if n < a or n > z: continue
            p = line.split()
            hx, t = (p[2], p[0]) if len(p) >= 3 else (p[0] if p else '', '')
            if not hx.lower().startswith('f218'): continue
            try: d = decode(bytes.fromhex(hx))
            except ValueError: continue
            if d: d['time'] = t; print(','.join(str(d[k]) for k in FIELDS))
if __name__ == '__main__': main()
