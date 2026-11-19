"""Offline Buddy5 (APP=rtos_buddy1_5) telemetry review for a passive capture.

Input: capture_wireless.py *_mqtt.csv (wall_s,topic,hex,retained) or text lines
"is28/tlm <hex>". Reports the E5 mission-status timeline, and per width /
direction run the records RECEIVED versus CLAIMED by the car's summary, listing
missing indices. Never fills a gap. --sdk-text writes the is28/tlm frames as
"topic hex" lines so the frozen SDK tools (decode_width_calibrated.py for the
width estimator parity, decode_edge_probe_dir.py for direction v3) can replay
the same capture unchanged. Width is a display estimate; nothing here affects
motion."""
import argparse, csv, json, sys
from pathlib import Path

STAGE = ['idle', 'guard', 'halting', 'width probe', 'line alignment', 'approach', 'direction probe',
         'bypass', 'hold', 'cancelled', 'fresh range']
REASON = ['none', 'cancelled', 'sonar fault', 'lease lost', 'move refused/interrupted', 'stage timeout',
          'stale observations/coordinator fault', 'no encoder progress', 'width probe aborted',
          'width probe could not start', 'no line evidence after alignment search', 'line lost during approach',
          'approach range not reached', 'direction probe aborted', 'direction unknown/contradictory',
          'direction probe could not start', 'fresh range unusable', 'bypass blocked (range/clearance)',
          'bypass/rejoin fault']
WPOLICY = ['none', 'measured', 'SKIPPED (clean range outside 270-330 mm, no probe)', 'FAILED (attempted, HOLD)',
           'running']
BASIS = ['none', 'probe evidence', 'DEFAULT side (healthy, no preference)', 'held']
SIDE = ['none', 'right', 'left']
WBASIS = ['NONE', 'CALIBRATED_RANGE_RISE', 'EMPIRICAL_REACH_COMPLETION', 'PROJECT_RANGE_ASSUMPTION']
WEND = ['NONE', 'COMPLETE', 'CANCELLED', 'MOVE', 'SENSOR', 'LEASE', 'TIMEOUT', 'REFERENCE', 'ALIGNMENT',
        'TRAVEL', 'CLEARANCE']
DEC = ['HOLD', 'LEFT', 'RIGHT']
EPS = ['UNKNOWN', 'OPEN', 'CROSSED', 'REMAINS']
EV = ['NONE', 'TAKEOVER', 'PLAN', 'HOLD_CLEARANCE', 'LEG_END', 'RETURN_START', 'CANDIDATE', 'REJECT',
      'STRIPE_FOUND', 'POSITIONED', 'RESTORED', 'SEARCH_TURN', 'CONFIRMED', 'HANDBACK', 'STOPPED',
      'GEOMETRY_HOLD', 'ACQUIRE', 'ACQUIRED', 'RETURN_FALLBACK']


def u16(p, o): return p[o] | p[o + 1] << 8
def i16(p, o): v = u16(p, o); return v - 65536 if v & 0x8000 else v
def u32(p, o): return u16(p, o) | u16(p, o + 2) << 16
def opt(v, scale): return None if v == 0xFFFF else v / scale
def name(t, i): return t[i] if 0 <= i < len(t) else f'?{i}'
def i8(p, o): return p[o] - 256 if p[o] & 0x80 else p[o]


def decode(frame):
    if len(frame) < 9 or frame[0] != 0xF2 or len(frame) != 3 + frame[1]:
        return None
    p, n, m = frame[3:], frame[1], frame[3 + 4]
    if (n == 54 and m == 0xE5 and p[5] == 1) or (n == 59 and m == 0xE5 and p[5] == 2):
        v2 = p[5] == 2
        return {'kind': 'status', 'version': p[5],
                'width_policy': name(WPOLICY, p[19]) if v2 else None,
                'steer_deg': i8(p, 54) / 10 if v2 else None, 'steer_max_deg': p[55] / 10 if v2 else None,
                'steer_saturated': p[56] if v2 else None, 'width_gate_mm': (u16(p, 57) or None) if v2 else None, 'ms': u32(p, 0), 'run': u16(p, 6), 'stage': name(STAGE, p[8]),
                'reason': name(REASON, p[9]), 'module_reason': p[10], 'owner': p[11] & 7, 'mode': (p[11] >> 3) & 7,
                'flags': u16(p, 12), 'ping_mm': None if u16(p, 14) == 0xFFFF else u16(p, 14),
                'side': name(SIDE, p[28]), 'basis': name(BASIS, p[25]), 'decision': name(DEC, p[24]),
                'width_mm': opt(u16(p, 34), 100), 'width_basis': name(WBASIS, p[30]), 'width_end': name(WEND, p[29]),
                'width_produced': p[48], 'width_admitted': p[49], 'direction_produced': p[50],
                'direction_admitted': p[51], 'trigger_mm': u16(p, 42), 'd0': u16(p, 40), 'start_mm': u16(p, 44)}
    if n == 56 and m == 0xD1:
        return {'kind': 'width_record', 'run': u16(p, 6), 'index': u16(p, 8)}
    if n == 58 and m == 0xD2:
        return {'kind': 'width_summary', 'run': u16(p, 6), 'end': name(WEND, p[8]), 'records': u16(p, 14),
                'lost': u16(p, 16), 'd0': u16(p, 18)}
    if n == 58 and m == 0xD3:
        return {'kind': 'width_result', 'run': u16(p, 6), 'end': name(WEND, p[8]), 'basis': name(WBASIS, p[9]),
                'display_mm': opt(u16(p, 12), 100), 'raw_mm': opt(u16(p, 14), 100), 'records': u16(p, 44)}
    if n == 32 and m == 0xD4:
        return {'kind': 'width_retry', 'run': u16(p, 6), 'phase': p[8]}
    if n == 48 and m == 0xBE:
        return {'kind': 'direction_record', 'run': u16(p, 6), 'index': u16(p, 8), 'version': p[5]}
    if n == 44 and m == 0xBF:
        return {'kind': 'direction_summary', 'run': u16(p, 6), 'version': p[5], 'near_mm': 25 if p[5] == 3 else 15,
                'end': name(WEND[:9], p[8]), 'decision': name(DEC, p[9]), 'left': name(EPS, p[10]),
                'right': name(EPS, p[11]), 'records': u16(p, 12)}
    if n == 32 and m == 0xB5:
        return {'kind': 'event', 'code': name(EV, p[6]), 'seq': u16(p, 28), 'overwritten': u16(p, 30)}
    return None


def frames(path):
    text = Path(path).read_text(encoding='utf-8', errors='replace')
    if text.startswith('wall_s'):
        for row in csv.DictReader(text.splitlines()):
            if row.get('topic') == 'is28/tlm' and row.get('retained', 'False') in ('False', 'false', '0', ''):
                yield row['topic'], bytes.fromhex(row['hex'])
    else:
        for line in text.splitlines():
            parts = line.split()
            if len(parts) == 2 and parts[0] == 'is28/tlm':
                try:
                    yield parts[0], bytes.fromhex(parts[1])
                except ValueError:
                    pass


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('capture')
    ap.add_argument('--json', type=Path)
    ap.add_argument('--sdk-text', type=Path, help='write "is28/tlm hex" lines for the frozen SDK decoders')
    a = ap.parse_args(argv)
    width, direction, statuses, events, raw = {}, {}, [], [], []
    for topic, f in frames(a.capture):
        raw.append(f'{topic} {f.hex()}')
        d = decode(f)
        if not d:
            continue
        k = d['kind']
        if k == 'status':
            if not statuses or (statuses[-1]['stage'], statuses[-1]['reason']) != (d['stage'], d['reason']):
                print(f"E5 {d['ms']:>9} ms run {d['run']}: {d['stage']}{'' if d['reason'] == 'none' else ' (' + d['reason'] + ')'}")
            statuses.append(d)
        elif k.startswith('width'):
            r = width.setdefault(d['run'], {'indices': set(), 'dup': 0, 'summary': None, 'result': None, 'retries': 0})
            if k == 'width_record':
                r['dup'] += d['index'] in r['indices']; r['indices'].add(d['index'])
            elif k == 'width_summary': r['summary'] = d
            elif k == 'width_result': r['result'] = d
            else: r['retries'] += 1
        elif k.startswith('direction'):
            r = direction.setdefault(d['run'], {'indices': set(), 'dup': 0, 'summary': None})
            if k == 'direction_record':
                r['dup'] += d['index'] in r['indices']; r['indices'].add(d['index'])
            else: r['summary'] = d
        else:
            events.append(d)
    report = {'width': {}, 'direction': {}}
    for label, runs in (('width', width), ('direction', direction)):
        for run, r in runs.items():
            claimed = r['summary']['records'] if r['summary'] else (r.get('result') or {}).get('records')
            missing = sorted(set(range(claimed)) - r['indices']) if claimed is not None else None
            entry = {'received': len(r['indices']), 'claimed': claimed, 'missing': missing, 'duplicates': r['dup'],
                     'summary': r['summary'], 'result': r.get('result'), 'retries': r.get('retries')}
            report[label][run] = entry
            state = 'COMPLETE' if missing == [] and r['summary'] else 'INCOMPLETE (telemetry gap; not filled)'
            print(f"{label} run {run}: {entry['received']}/{claimed} records, missing {missing}, {state}")
            if r.get('result'):
                x = r['result']
                print(f"  width {x['display_mm']} mm basis {x['basis']} end {x['end']}"
                      f"{' (ASSUMED, not measured)' if x['basis'] == 'PROJECT_RANGE_ASSUMPTION' else ''}")
            if label == 'direction' and r['summary']:
                x = r['summary']
                print(f"  direction {x['decision']} L {x['left']} / R {x['right']} end {x['end']} near {x['near_mm']} mm")
    if statuses:
        s = statuses[-1]
        print(f"last E5 v{s['version']}: stage {s['stage']}, side {s['side']} ({s['basis']}), width {s['width_mm']} mm "
              f"{s['width_basis']}, car produced/admitted width {s['width_produced']}/{s['width_admitted']}, "
              f"direction {s['direction_produced']}/{s['direction_admitted']}")
        if s['version'] == 2:
            print(f"  width policy {s['width_policy']} (gate {s['width_gate_mm']} mm); approach steering last "
                  f"{s['steer_deg']} deg, largest {s['steer_max_deg']} deg of 8.0, at bound {s['steer_saturated']} frame(s)")
    seqs = [e['seq'] for e in events]
    gaps = [(a, b) for a, b in zip(seqs, seqs[1:]) if (b - a) & 0xFFFF != 1]
    print(f"bypass events {len(events)}, event-seq gaps {gaps}, overwritten on car {events[-1]['overwritten'] if events else 0}")
    if a.sdk_text:
        a.sdk_text.write_text('\n'.join(raw) + '\n', encoding='utf-8')
    if a.json:
        a.json.write_text(json.dumps({'report': report, 'statuses': statuses[-1:], 'events': events}, indent=1,
                                     default=list) + '\n', encoding='utf-8')
    return 0


if __name__ == '__main__':
    sys.exit(main())
