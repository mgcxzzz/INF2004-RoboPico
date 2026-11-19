"""Counterfactual replay of the accepted direction-checkpoint captures with the
integration's 25 mm near allowance (edge_scan v0.1.1 decoder NEAR_OVERRIDE).
Read-only: frozen captures and decoders are imported, never modified."""
import importlib.util, json, sys
from pathlib import Path
sys.dont_write_bytecode = True
SDK = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location('dirdec', SDK / 'edge_scan_v0_1_1/decode_edge_probe_dir.py')
D = importlib.util.module_from_spec(spec); spec.loader.exec_module(D)
cp = json.loads((SDK / 'edge_probe_v0_1_1/DIRECTION_CHECKPOINT.json').read_text())
rows, changed = [], 0
for case in cp['accepted_complete_cases']:
    raw = SDK / 'edge_probe_v0_1_1' / case['raw']
    out = {}
    for near in (None, 25):
        D.NEAR_OVERRIDE = near
        runs = [r for r in D.read_capture(str(raw)) if r['run'] == case['run']]
        r = runs[-1]
        out[near] = (r['decision'], r['state_left'], r['state_right'], r['quality'])
    same = out[None] == out[25]
    changed += not same
    print(f"{case['setup'][:44]:44s} run {case['run']}: v2(15)={out[None]} near25={out[25]} {'SAME' if same else 'CHANGED'}")
    rows.append({'setup': case['setup'], 'run': case['run'], 'expected': case['decision'],
                 'near15': out[None], 'near25': out[25], 'same': same})
print(f"{len(rows)} accepted cases, {changed} changed by the 25 mm near allowance")
Path(sys.argv[2]).write_text(json.dumps(rows, indent=1) + '\n')
sys.exit(1 if changed else 0)
