"""Read-only Step 2 ADC/clock check. Sends no commands and never starts motors."""
from __future__ import annotations
import argparse
import datetime as dt
import json
from pathlib import Path
import time
from capture_bringup import analyze as analyze_timing, parse_line


def analyze(rows: list[dict]) -> dict:
    timing = analyze_timing(rows)
    adcs = [r for r in rows if r['kind'] == 'ADC']
    lines = [r for r in rows if r['kind'] == 'LINE']
    issues = []
    if timing.get('result') != 'PASS':
        issues.append('Clock/motor-readiness observation did not pass; inspect timing details.')
    if len(adcs) < 2 or len(lines) < 2:
        return {'result': 'INCONCLUSIVE', 'issues': issues + ['Missing ADC/LINE status; check firmware and port.'],
                'timing': timing}
    span = adcs[-1]['wall_s'] - adcs[0]['wall_s']
    if span < 60:
        issues.append('Less than 60 seconds of independent ADC status; capture 70 seconds.')
    if any(r.get('state') != 2 or r.get('good') != 3 for r in adcs):
        issues.append('ADC was unavailable or still qualifying during the captured interval.')
    if any((r.get('flags', 0) & 3) != 3 for r in lines):
        issues.append('Line acquisition/calibration readiness was lost.')
    for key in ('conversions', 'frames', 'ch0', 'ch1', 'ch2'):
        if adcs[-1].get(key, 0) <= adcs[0].get(key, 0):
            issues.append(f'{key} did not advance: acquisition may be frozen.')
    fault_keys = ('timeouts', 'errors', 'init_errors', 'bad_arguments', 'fail0', 'fail1', 'fail2')
    if any(r.get(k, 0) for r in adcs for k in fault_keys):
        issues.append('A genuine ADC failure is recorded, even if it recovered; send the failure diagnostics.')
    if any(r.get('active', 0) for r in rows if r['kind'] == 'CONTROL'):
        issues.append('Motors were active; this check must be repeated stopped.')
    raw = []
    for r in lines:
        try:
            raw.append([int(v) for v in r['raw'].split(',')])
        except (KeyError, ValueError, AttributeError):
            pass
    return {'result': 'PASS' if not issues else 'REVIEW', 'issues': issues,
            'adc_observation_seconds': round(span, 3), 'timing': timing,
            'adc_first': adcs[0], 'adc_last': adcs[-1],
            'first_failure': next((r for r in rows if r['kind'] == 'ADC_FIRST'), None),
            'last_failure': next((r for r in reversed(rows) if r['kind'] == 'ADC_LAST'), None),
            'calibration': next((r for r in reversed(rows) if r['kind'] == 'CAL_FRONT'), None),
            'raw_range_left_right_side': [[min(a[j] for a in raw), max(a[j] for a in raw)]
                                          for j in range(3)] if raw and all(len(a) == 3 for a in raw) else [],
            'scope': 'Idle ADC freshness and clock/readiness only; not driving, contrast or physical-stop acceptance.'}


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port')
    p.add_argument('--seconds', type=float, default=70)
    p.add_argument('--output', type=Path, default=Path('rtos_adc_session'))
    args = p.parse_args()
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        print('Install pyserial with your Python: python3 -m pip install pyserial')
        return 2
    if not args.port:
        for item in serial.tools.list_ports.comports():
            print(f'{item.device}: {item.description}')
        args.port = input('Pico port (COM4 or /dev/cu.usbmodem...): ').strip()
    if args.port.isdecimal():
        args.port = 'COM' + args.port
    args.output.mkdir(parents=True, exist_ok=True)
    stem = dt.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    rows = []
    print('Keep the robot stopped. Close miniterm/PuTTY first. No commands will be sent.')
    print('Capture takes about 70s. During seconds 20-40, move white/black beneath each sensor.')
    try:
        with serial.Serial(args.port, 115200, timeout=.25) as port, \
                (args.output / (stem + '_usb.txt')).open('w', encoding='utf-8') as out:
            start = time.monotonic()
            while time.monotonic() - start < args.seconds + 2:
                line = port.readline().decode('utf-8', errors='replace').strip()
                if not line:
                    continue
                wall = time.monotonic() - start
                out.write(f'{wall:.6f}\t{line}\n')
                out.flush()
                kind, values = parse_line(line)
                if wall >= 2 and kind in ('CLOCK', 'CONTROL', 'WHEELS', 'LINE', 'ADC',
                                           'ADC_FIRST', 'ADC_LAST', 'CAL_FRONT'):
                    rows.append({'wall_s': wall, 'kind': kind, **values})
                print(line)
    except serial.SerialException as error:
        print(f'Capture interrupted: {error}; partial evidence retained.')
    info = analyze(rows)
    (args.output / (stem + '_summary.json')).write_text(json.dumps(info, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(info, indent=2))
    print(f'Send both files in {args.output.resolve()} for review.')
    return 0 if info['result'] == 'PASS' else 2


if __name__ == '__main__':
    raise SystemExit(main())
