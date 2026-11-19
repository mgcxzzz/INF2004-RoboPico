"""Offline Buddy5 (APP=rtos_buddy1_5) telemetry review for a passive capture.

Input: capture_wireless.py *_mqtt.csv (wall_s,topic,hex,retained), text lines
"is28/tlm <hex>", "TLM <hex>" (host simulation) or JSON lines {"topic","hex"}.
Reports the E5 mission-status timeline and, per width / direction run, the
records RECEIVED versus CLAIMED by the car, listing missing indices. A gap is
never filled.

Width protocols (v0.2.12 long-width candidate):
  current  D1/D2 wire 10, D3 v3, D4 v3, D5 v2 (220 mm clearance-limited probe,
           estimator rule 2, basis 4 CORROBORATED_EDGE_ESTIMATE)
  legacy   D1/D2 wire 4, D3 v1, D4 v1 (rtos_buddy1_5 v0.1/v0.2 short probe):
           decoded under the original v0.2 tables; replay with the frozen
           decode_width_calibrated.py (--sdk-text output)
A run that mixes the two is rejected.

Width verification labels (current protocol):
  VERIFIED BY REPLAY  the capture holds every record the car claims, and the
                      unchanged SDK decode_width_long.py v0.2 (--sdk-width-decoder)
                      independently reproduces the D3 estimate, the clearance
                      policy and the D5 limits with no mismatch.
  PICO-REPORTED       the car's own D3/E5 estimate, shown as reported and NOT
                      independently verified (records missing, D2/D3/D5
                      missing, or no replay decoder given). A replay of an
                      incomplete capture is not used to confirm or refute it.
  NO RECORDS          the probe ended before its REF record (e.g. Stop).
  MISMATCH            replay disagrees with the car, or a policy check failed.
  INSUFFICIENT        replay could not check a limit from the received poses.
Width is a display estimate only; nothing here affects motion or clearance."""
import argparse
import csv
import importlib.util
import json
import sys
import tempfile
from pathlib import Path

STAGE = ['idle', 'guard', 'halting', 'width probe', 'line alignment', 'approach', 'direction probe',
         'bypass', 'hold', 'cancelled', 'fresh range']
REASON = ['none', 'cancelled', 'sonar fault', 'lease lost', 'move refused/interrupted', 'stage timeout',
          'stale observations/coordinator fault', 'no encoder progress', 'width probe aborted',
          'width probe could not start', 'no line evidence after alignment search', 'line lost during approach',
          'approach range not reached', 'direction probe aborted', 'direction unknown/contradictory',
          'direction probe could not start', 'fresh range unusable', 'bypass blocked (range/clearance)',
          'bypass/rejoin fault', 'close obstacle while takeover blocked (not a straight)']
# E5 v4 takeover gate (flags b12-15); sonar still treats any close reflector as an obstacle
GATE = ['unknown', 'STRAIGHT (open)', 'settling on a straight', 'stale Buddy3 state', 'Buddy3 state unavailable',
        'bend entry', 'bend', 'bend exit', 'curve (encoder heading)', 'junction', 'turn', 'line lost / recovery',
        'other']
WPOLICY = ['none', 'measured', 'SKIPPED (clean range outside 270-330 mm, no probe)', 'FAILED (attempted, HOLD)',
           'running']
BASIS = ['none', 'probe evidence', 'DEFAULT side (healthy, no preference)', 'held']
SIDE = ['none', 'right', 'left']
WBASIS_V1 = ['NONE', 'CALIBRATED_RANGE_RISE', 'EMPIRICAL_REACH_COMPLETION', 'PROJECT_RANGE_ASSUMPTION']
WBASIS = WBASIS_V1 + ['CORROBORATED_EDGE_ESTIMATE']
WSTATUS = ['NONE', 'ESTIMATE', 'REACH_LIMIT', 'INVALID']
WREASON = ['NONE', 'NO_RECORDS', 'INCONSISTENT', 'RUN_OR_REFERENCE', 'EDGES_UNRESOLVED']
WEDGE = ['NO_DATA', 'ONSET', 'HIGH_FROM_START', 'WALK_FREE']
WEND = ['NONE', 'COMPLETE', 'CANCELLED', 'MOVE', 'SENSOR', 'LEASE', 'TIMEOUT', 'REFERENCE', 'ALIGNMENT',
        'TRAVEL', 'CLEARANCE']
WSIDE = ['UNKNOWN', 'NO_INNER_IN', 'BOUNDARY', 'NOT_REACHED', 'CONTRADICTORY', 'UNCORROBORATED']
WPSTATE = ['IDLE', 'CLAIM', 'REF', 'TURN', 'PRE', 'STEP', 'OUT', 'RETRACE', 'RET', 'TURN_BACK', 'FINAL', 'DONE',
           'HOLD', 'TRIM', 'LADDER', 'MID']
WPHASE = ['REF', 'PRE', 'OUT', 'RET', 'FINAL', 'LAD_OUT', 'LAD_RET', 'MID']
LIMIT = ['NONE', 'FAR', 'CEILING', 'CLEARANCE', 'STEPS', 'ABORT']
DEC = ['HOLD', 'LEFT', 'RIGHT']
EPS = ['UNKNOWN', 'OPEN', 'CROSSED', 'REMAINS']
EV = ['NONE', 'TAKEOVER', 'PLAN', 'HOLD_CLEARANCE', 'LEG_END', 'RETURN_START', 'CANDIDATE', 'REJECT',
      'STRIPE_FOUND', 'POSITIONED', 'RESTORED', 'SEARCH_TURN', 'CONFIRMED', 'HANDBACK', 'STOPPED',
      'GEOMETRY_HOLD', 'ACQUIRE', 'ACQUIRED', 'RETURN_FALLBACK']
# (payload length, marker) -> {version: protocol}
WIDTH_VERSIONS = {(56, 0xD1): {10: 'long', 4: 'legacy'}, (58, 0xD2): {10: 'long', 4: 'legacy'},
                  (58, 0xD3): {3: 'long', 1: 'legacy'}, (32, 0xD4): {3: 'long', 1: 'legacy'},
                  (36, 0xD5): {2: 'long'}}
LONG_CONSTANTS = {'ceiling_mm': 220.0, 'overshoot_mm': 5.0, 'heading_reserve_deg': 5.0, 'margin_mm': 50.0}
PROTO_TEXT = {'long': 'long probe D1/D2 v10, D3 v3, D4 v3, D5 v2',
              'legacy': 'legacy short probe D1/D2 v4, D3 v1 (original rules)'}


def u16(p, o): return p[o] | p[o + 1] << 8
def i16(p, o): v = u16(p, o); return v - 65536 if v & 0x8000 else v
def u32(p, o): return u16(p, o) | u16(p, o + 2) << 16
def opt(v, scale): return None if v == 0xFFFF else v / scale
def name(t, i): return t[i] if 0 <= i < len(t) else f'?{i}'
def i8(p, o): return p[o] - 256 if p[o] & 0x80 else p[o]
def mm(v): return 'n/a' if v is None else f'{v:.2f}'


def width_protocol(frame):
    """'long', 'legacy', 'unknown' (width marker, unknown version) or None."""
    p = frame[3:]
    known = WIDTH_VERSIONS.get((len(p), p[4]))
    if known is None:
        return None
    return known.get(p[5], 'unknown')


def decode_status(p):
    v = p[5]
    v2 = v >= 2
    d = {'kind': 'status', 'version': v, 'width_policy': name(WPOLICY, p[19]) if v2 else None,
         'steer_deg': i8(p, 54) / 10 if v2 else None, 'steer_max_deg': p[55] / 10 if v2 else None,
         'steer_saturated': p[56] if v2 else None, 'width_gate_mm': (u16(p, 57) or None) if v2 else None,
         'ms': u32(p, 0), 'run': u16(p, 6), 'stage': name(STAGE, p[8]),
         'reason': name(REASON, p[9]), 'module_reason': p[10], 'owner': p[11] & 7, 'mode': (p[11] >> 3) & 7,
         'flags': u16(p, 12), 'ping_mm': None if u16(p, 14) == 0xFFFF else u16(p, 14),
         'side': name(SIDE, p[28]), 'basis': name(BASIS, p[25]), 'decision': name(DEC, p[24]),
         'width_end': name(WEND, p[29]), 'width_basis': name(WBASIS if v >= 3 else WBASIS_V1, p[30]),
         'width_status': name(WSTATUS, p[31]), 'width_reason': name(WREASON, p[32]), 'width_retries': p[33],
         'width_mm': opt(u16(p, 34), 100), 'width_raw_mm': opt(u16(p, 36), 100),
         'width_corrected_mm': opt(u16(p, 38), 100),
         'width_produced': p[48], 'width_admitted': p[49], 'direction_produced': p[50],
         'direction_admitted': p[51], 'trigger_mm': u16(p, 42), 'd0': u16(p, 40), 'start_mm': u16(p, 44)}
    if v >= 3:
        ws, wp = p[59], p[60]
        d.update(probe_state=name(WPSTATE, ws & 15), probe_side=['none', 'L', 'R', '?'][(ws >> 4) & 3],
                 sample_phase=name(WPHASE, wp & 7), sample_step=wp >> 3)
    d.update(gate=None, gate_open=None, gate_echoes=None, blocked_gate=None)
    if v >= 4:
        g = u16(p, 12) >> 12
        d.update(gate=name(GATE, g), gate_open=g == 1, gate_echoes=p[59] >> 6,
                 blocked_gate=name(GATE, p[10]) if p[9] == 19 else None)
    return d


def decode_width(p, proto):
    m = p[4]
    base = {'run': u16(p, 6), 'protocol': proto, 'version': p[5], 'ms': u32(p, 0)}
    if m == 0xD1:
        return dict(base, kind='width_record', index=u16(p, 8), phase=name(WPHASE, p[10]),
                    side=['C', 'L', 'R'][p[11]] if p[11] < 3 else '?', step=p[12])
    if m == 0xD2:
        return dict(base, kind='width_summary', end=name(WEND, p[8]), width_result=p[9],
                    state_l=name(WSIDE, p[10]), state_r=name(WSIDE, p[11]), steps_l=p[12], steps_r=p[13],
                    records=u16(p, 14), lost=u16(p, 16), d0=u16(p, 18), out_l_mm=i16(p, 20) / 10,
                    out_r_mm=i16(p, 22) / 10, flags=u16(p, 38),
                    l_in_mm=i16(p, 44) / 10, l_far_mm=i16(p, 46) / 10, r_in_mm=i16(p, 48) / 10,
                    r_far_mm=i16(p, 50) / 10, width_lo_mm=u16(p, 52) / 10,
                    width_hi_mm=u16(p, 54) / 10, min_front_gap_mm=i16(p, 56) / 10)
    if m == 0xD3:
        bases = WBASIS if proto == 'long' else WBASIS_V1
        sides = {}
        for k, s in enumerate('LR'):
            o = 26 + 8 * k
            sides[s] = {'status': name(WEDGE, p[o]), 'n': p[o + 1], 'a_mm': opt(u16(p, o + 2), 10),
                        'b_mm': opt(u16(p, o + 4), 10), 'base_mm': i16(p, o + 6) / 4}
        return dict(base, kind='width_result', end=name(WEND, p[8]), basis=name(bases, p[9]),
                    status=name(WSTATUS, p[10]), reason=name(WREASON, p[11]),
                    display_mm=opt(u16(p, 12), 100), raw_mm=opt(u16(p, 14), 100),
                    corrected_mm=opt(u16(p, 16), 100), sampled_mm=opt(u16(p, 18), 10),
                    span_lo_mm=opt(u16(p, 20), 10), span_hi_mm=opt(u16(p, 22), 10), flags=u16(p, 24),
                    sides=sides, d0=u16(p, 42), records=u16(p, 44), lost=u16(p, 46),
                    offset_mm=u16(p, 48) / 10, range_mm=[u16(p, 50), u16(p, 52)], rule=p[56],
                    width_result=p[57])
    if m == 0xD4:
        return dict(base, kind='width_retry', phase=name(WPHASE, p[8]), side=p[9], step=p[10], attempt=p[11])
    sides = {}
    for k, s in enumerate('LR'):
        o = 18 + 8 * k
        ct, mg = i16(p, o + 4), i16(p, o + 6)
        sides[s] = {'limit': name(LIMIT, p[o]), 'steps': p[o + 1], 'out_mm': i16(p, o + 2) / 10,
                    'clear_total_mm': None if ct == 0x7FFF else ct / 10,
                    'min_turn_gap_mm': None if mg == 0x7FFF else mg / 10}
    return dict(base, kind='width_limits', end=name(WEND, p[8]), ceiling_mm=u16(p, 10) / 10,
                overshoot_mm=u16(p, 12) / 10, heading_reserve_deg=u16(p, 14) / 100,
                margin_mm=u16(p, 16) / 10, sides=sides, d0=u16(p, 34))


def decode(frame):
    if len(frame) < 9 or frame[0] != 0xF2 or len(frame) != 3 + frame[1]:
        return None
    p, n, m = frame[3:], frame[1], frame[3 + 4]
    if m == 0xE5 and (n, p[5]) in ((54, 1), (59, 2), (61, 3), (61, 4)):
        return decode_status(p)
    proto = width_protocol(frame)
    if proto == 'unknown':
        return {'kind': 'width_unknown', 'run': u16(p, 6), 'marker': m, 'version': p[5]}
    if proto:
        return decode_width(p, proto)
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
        return
    for line in text.splitlines():
        line = line.strip()
        try:
            if line.startswith('{'):
                obj = json.loads(line)
                if obj.get('topic', 'is28/tlm') == 'is28/tlm' and 'hex' in obj:
                    yield 'is28/tlm', bytes.fromhex(obj['hex'])
                continue
            parts = line.split()
            if len(parts) == 2 and parts[0] in ('is28/tlm', 'TLM'):
                yield 'is28/tlm', bytes.fromhex(parts[1])
        except ValueError:
            pass


def load_sdk(path):
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location('sdk_decode_width_long_v02', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    if getattr(mod, 'VERSION', '') != 'decode_width_long v0.2':
        raise SystemExit(f'{path}: expected decode_width_long v0.2, found {getattr(mod, "VERSION", "?")}')
    return mod


def replay(sdk, long_frames):
    """Run the unchanged SDK v0.2 decoder on the current-protocol width frames."""
    if not long_frames:
        return {}
    with tempfile.TemporaryDirectory() as tmp:
        cap = Path(tmp) / 'rtos_width_long.txt'
        cap.write_text(''.join(f'is28/tlm {f.hex()}\n' for f in long_frames), encoding='utf-8')
        rows = sdk.report(cap)
    return {r['run']: r for r in rows if r.get('wire_version') == sdk.WIRE_LONG}


def width_verdict(entry, rep, sdk_given):
    """(label, reasons) for one current-protocol width run. Never upgrades a gap."""
    gaps = []
    if entry['missing']:
        gaps.append(f"{len(entry['missing'])} D1 record(s) missing from the capture {entry['missing']}")
    if entry['claimed'] is None:
        gaps.append('no D2 summary or D3 result received: record count unknown')
    for key, label in (('summary', 'D2 summary'), ('result', 'D3 result'), ('limits', 'D5 limits')):
        if not entry[key]:
            gaps.append(f'{label} missing')
    if entry['duplicates']:
        gaps.append(f"{entry['duplicates']} duplicate D1 index(es)")
    if gaps:
        # A replay of an incomplete capture cannot confirm or refute the car's
        # estimate; it is reported as the car's value with the gap named.
        return 'PICO-REPORTED (not independently verified)', gaps
    if entry['claimed'] == 0:
        return 'NO RECORDS (probe ended before REF; nothing to verify)', []
    if rep is not None:
        bad = (rep.get('parity') or []) + (rep.get('long_probe_checks') or [])
        if bad:
            return 'MISMATCH', ['replay: ' + x for x in bad]
    if rep is None:
        return ('PICO-REPORTED (not independently verified)',
                ['replay produced no row for this run' if sdk_given else
                 'replay not run: pass --sdk-width-decoder <width_long_probe_v0_2/decode_width_long.py>'])
    if rep.get('decoder_errors') or not rep.get('records_consistent', False):
        return 'MISMATCH', ['replay: ' + x for x in rep.get('decoder_errors') or ['records inconsistent']]
    if rep.get('long_probe_insufficient') or rep.get('incomplete'):
        return 'INSUFFICIENT', (rep.get('long_probe_insufficient') or []) + \
            ([rep['incomplete']] if rep.get('incomplete') else [])
    return 'VERIFIED BY REPLAY', []


def print_width(run, entry, r):
    proto = entry['protocol']
    claimed, missing = entry['claimed'], entry['missing']
    state = 'COMPLETE' if missing == [] and r['summary'] else 'INCOMPLETE (telemetry gap; not filled)'
    print(f"width run {run} [{PROTO_TEXT.get(proto, proto)}]: {entry['received']}/{claimed} records, "
          f"missing {missing}, {state}, D4 retries {r['retries']}")
    s = r['summary']
    if s:
        print(f"  D2 end {s['end']}, L {s['state_l']} {s['steps_l']} steps out {s['out_l_mm']:.1f} mm, "
              f"R {s['state_r']} {s['steps_r']} steps out {s['out_r_mm']:.1f} mm, d0 {s['d0']} mm, lost {s['lost']}")
        if s['width_result']:
            print(f"  D2 IN/FAR edge brackets (axis x mm): L IN {s['l_in_mm']:.1f} / FAR {s['l_far_mm']:.1f}; "
                  f"R IN {s['r_in_mm']:.1f} / FAR {s['r_far_mm']:.1f}; probe width bracket "
                  f"{s['width_lo_mm']:.1f}-{s['width_hi_mm']:.1f}; min modelled front gap {s['min_front_gap_mm']:.1f}")
        else:
            print(f"  D2 IN/FAR edge brackets: n/a (both sides not bounded); min modelled front gap "
                  f"{s['min_front_gap_mm']:.1f} mm")
    lim = r['limits']
    if lim:
        for n in 'LR':
            x = lim['sides'][n]
            ct = 'n/a' if x['clear_total_mm'] is None else f"{x['clear_total_mm']:.1f}"
            mg = 'n/a' if x['min_turn_gap_mm'] is None else f"{x['min_turn_gap_mm']:.1f}"
            print(f"  D5 {n}: stop {x['limit']}, reach {x['out_mm']:.1f} mm in {x['steps']} steps, "
                  f"clearance allowed {ct} mm, min modelled turn gap {mg} mm")
        consts = {k: lim[k] for k in LONG_CONSTANTS}
        if consts != LONG_CONSTANTS:
            print(f'  D5 policy constants differ from this decoder: {consts}')
    x = r['result']
    if x:
        note = {'PROJECT_RANGE_ASSUMPTION': ' (ASSUMED mid-range, not measured)',
                'EMPIRICAL_REACH_COMPLETION': ' (ASSUMED completion of an unresolved edge)',
                'CORROBORATED_EDGE_ESTIMATE': ' (estimate, corroborated edge evidence)',
                'CALIBRATED_RANGE_RISE': ' (estimate, calibrated range rise)'}.get(x['basis'], '')
        print(f"  width DISPLAY {mm(x['display_mm'])} mm | raw {mm(x['raw_mm'])} | corrected before clamp "
              f"{mm(x['corrected_mm'])} (offset -{x['offset_mm']:g} mm, display range "
              f"{x['range_mm'][0]}-{x['range_mm'][1]} mm)")
        print(f"  basis {x['basis']}{note}, status {x['status']}, reason {x['reason']}, end {x['end']}"
              + (f", rule {x['rule']}" if proto == 'long' else ''))
        print('  edges: ' + '; '.join(
            f"{n} {x['sides'][n]['status']} a {x['sides'][n]['a_mm']} b {x['sides'][n]['b_mm']} mm (axis x)"
            for n in 'LR'))
    print(f"  WIDTH {entry['verdict']}" + (': ' + '; '.join(entry['why']) if entry['why'] else ''))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('capture')
    ap.add_argument('--json', type=Path)
    ap.add_argument('--sdk-text', type=Path, help='write "is28/tlm hex" lines for the frozen SDK decoders')
    ap.add_argument('--sdk-width-decoder', type=Path,
                    help='bench_tools/buddy5_sdk/width_long_probe_v0_2/decode_width_long.py (read only)')
    a = ap.parse_args(argv)
    sdk = load_sdk(a.sdk_width_decoder) if a.sdk_width_decoder else None
    width, direction, statuses, events, raw, wframes = {}, {}, [], [], [], {}
    for topic, f in frames(a.capture):
        raw.append(f'{topic} {f.hex()}')
        d = decode(f)
        if not d:
            continue
        k = d['kind']
        if k == 'status':
            if not statuses or (statuses[-1]['stage'], statuses[-1]['reason']) != (d['stage'], d['reason']):
                print(f"E5 {d['ms']:>9} ms run {d['run']}: {d['stage']}"
                      f"{'' if d['reason'] == 'none' else ' (' + d['reason'] + ')'}"
                      f"{' [gate: ' + d['blocked_gate'] + ']' if d.get('blocked_gate') else ''}")
            if d.get('gate') and (not statuses or statuses[-1].get('gate') != d['gate']) and d['stage'] == 'guard':
                print(f"E5 {d['ms']:>9} ms   takeover gate: {d['gate']}")
            statuses.append(d)
        elif k.startswith('width'):
            r = width.setdefault(d['run'], {'indices': set(), 'dup': 0, 'summary': None, 'result': None,
                                            'limits': None, 'retries': 0, 'protocols': set(), 'unknown': 0})
            wframes.setdefault(d['run'], []).append(f)
            if k == 'width_unknown':
                r['unknown'] += 1
                continue
            r['protocols'].add(d['protocol'])
            if k == 'width_record':
                r['dup'] += d['index'] in r['indices']
                r['indices'].add(d['index'])
            elif k == 'width_summary':
                r['summary'] = d
            elif k == 'width_result':
                r['result'] = d
            elif k == 'width_limits':
                r['limits'] = d
            else:
                r['retries'] += 1
        elif k.startswith('direction'):
            r = direction.setdefault(d['run'], {'indices': set(), 'dup': 0, 'summary': None})
            if k == 'direction_record':
                r['dup'] += d['index'] in r['indices']
                r['indices'].add(d['index'])
            else:
                r['summary'] = d
        else:
            events.append(d)
    long_runs = {run for run, r in width.items() if r['protocols'] == {'long'} and not r['unknown']}
    replayed = replay(sdk, [f for run in sorted(long_runs) for f in wframes[run]]) if sdk else {}
    report = {'width': {}, 'direction': {}}
    failures = 0
    for run, r in sorted(width.items()):
        claimed = r['summary']['records'] if r['summary'] else (r['result'] or {}).get('records')
        missing = sorted(set(range(claimed)) - r['indices']) if claimed is not None else None
        entry = {'received': len(r['indices']), 'claimed': claimed, 'missing': missing, 'duplicates': r['dup'],
                 'summary': r['summary'], 'result': r['result'], 'limits': r['limits'], 'retries': r['retries'],
                 'protocol': '/'.join(sorted(r['protocols'])) or 'unknown'}
        report['width'][run] = entry
        if len(r['protocols']) != 1 or r['unknown']:
            entry['verdict'] = 'REJECTED'
            entry['why'] = ['mixed legacy/current width protocol versions in one run' if len(r['protocols']) > 1
                            else f"{r['unknown']} width frame(s) with an unknown version"]
            failures += 1
            print(f"width run {run}: REJECTED ({entry['why'][0]})")
            continue
        if entry['protocol'] == 'long':
            entry['verdict'], entry['why'] = width_verdict(entry, replayed.get(run), sdk is not None)
            if run in replayed:
                rep = replayed[run]
                entry['replay'] = {k: rep.get(k) for k in ('source', 'parity', 'long_probe_checks',
                                                           'long_probe_insufficient', 'incomplete',
                                                           'reference_exact', 'records')}
            failures += entry['verdict'] == 'MISMATCH'
        else:
            entry['verdict'] = 'PICO-REPORTED (legacy protocol, original rules)'
            entry['why'] = ['replay with the frozen decode_width_calibrated.py using --sdk-text']
        print_width(run, entry, r)
    for run, r in sorted(direction.items()):
        claimed = r['summary']['records'] if r['summary'] else None
        missing = sorted(set(range(claimed)) - r['indices']) if claimed is not None else None
        entry = {'received': len(r['indices']), 'claimed': claimed, 'missing': missing, 'duplicates': r['dup'],
                 'summary': r['summary']}
        report['direction'][run] = entry
        state = 'COMPLETE' if missing == [] and r['summary'] else 'INCOMPLETE (telemetry gap; not filled)'
        print(f"direction run {run}: {entry['received']}/{claimed} records, missing {missing}, {state}")
        if r['summary']:
            x = r['summary']
            print(f"  direction {x['decision']} L {x['left']} / R {x['right']} end {x['end']} near {x['near_mm']} mm")
    if statuses:
        s = statuses[-1]
        print(f"last E5 v{s['version']}: stage {s['stage']}, side {s['side']} ({s['basis']}), width display "
              f"{s['width_mm']} mm (raw {s['width_raw_mm']}, corrected before clamp {s['width_corrected_mm']}) "
              f"{s['width_basis']}, car produced/admitted width {s['width_produced']}/{s['width_admitted']}, "
              f"direction {s['direction_produced']}/{s['direction_admitted']}")
        if s['version'] >= 2:
            print(f"  width policy {s['width_policy']} (gate {s['width_gate_mm']} mm); approach steering last "
                  f"{s['steer_deg']} deg, largest {s['steer_max_deg']} deg of 8.0, at bound {s['steer_saturated']} "
                  'frame(s)')
        if s['version'] >= 3:
            print(f"  width probe {s['probe_state']} side {s['probe_side']} sample {s['sample_phase']} "
                  f"step {s['sample_step']}")
        if s['version'] >= 4:
            print(f"  takeover gate {'ALLOWED (straight)' if s['gate_open'] else 'BLOCKED: ' + s['gate']}, "
                  f"close echoes {s['gate_echoes']}/3"
                  + (f"; HELD: close obstacle where the manoeuvre may not start ({s['blocked_gate']}); "
                     'width, direction and bypass not run' if s['blocked_gate'] else ''))
        else:
            print('  takeover gate: n/a (status before v4)')
    seqs = [e['seq'] for e in events]
    gaps = [(x, y) for x, y in zip(seqs, seqs[1:]) if (y - x) & 0xFFFF != 1]
    print(f"bypass events {len(events)}, event-seq gaps {gaps}, overwritten on car "
          f"{events[-1]['overwritten'] if events else 0}")
    if a.sdk_text:
        a.sdk_text.write_text('\n'.join(raw) + '\n', encoding='utf-8')
    if a.json:
        a.json.write_text(json.dumps({'report': report, 'statuses': statuses[-1:], 'events': events}, indent=1,
                                     default=lambda o: sorted(o) if isinstance(o, set) else str(o)) + '\n',
                          encoding='utf-8')
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
