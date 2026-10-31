#!/usr/bin/env python3
"""Summarise Buddy 5 bench console logs (one file per test case).

Reads the [b5 profile], [b5 scan] and [b5 decision] lines printed by the
obstacle task (B5_SCAN_DUMP 1) and prints one row per scan plus a per-file
summary. Rule B logs (bench v0.7 / RTOS 2026-10-31 rule B) also carry
[b5 evidence], [b5 sweep] and [b5 pings] lines; sample suffixes are
u = unstable echo, p = echo+1 fault, b = block only, w = weak no-return,
x = unknown. Usage: python3 parse_b5_bench.py logs/2026-10-31_S5-C07_*.txt
"""
import re
import sys

PROF = re.compile(r"\[b5 profile\] observed span (-?\d+) mm; model allowance (-?\d+) mm; "
                  r"planning envelope (-?\d+) mm; evidence 0x([0-9a-fA-F]+); clearance L/R (-?\d+)/(-?\d+) mm")
DEC = re.compile(r"\[b5 decision\] side (RIGHT|LEFT|NONE)[^;]*; found (\d+); confidence (\d+)%; "
                 r"closest (\d+) mm at (-?\d+) dd; edges L/R (-?\d+)/(-?\d+) dd; at arc limit L/R (\d)/(\d); "
                 r"need (\d+) mm; lateral (\d+) mm; forward (\d+) mm")
SAMPLE = re.compile(r"(-?\d+):(\d+)([xwpbu]?)")
EVID = {1: "supported", 2: "scanL", 4: "scanR", 8: "clear-side", 16: "WEAK", 32: "sweep-assumed",
        64: "sweepFAIL-L", 128: "sweepFAIL-R"}
EVL = re.compile(r"\[b5 evidence\] measured (\d+)%; coverage (\d+)%; stable-return span (\d+) mm")
SWP = re.compile(r"\[b5 sweep\] ([LR]): (not run|FAIL|PASS_ASSUMED|PASS)")
PNG = re.compile(r"\[b5 pings\] echo (\d+), no-return (\d+), fault (\d+) \(stuck-high (\d+), no-rise (\d+), "
                 r"no-fall (\d+), short (\d+)\); no-return widths (\d+)\.\.(\d+) us")


def parse(path):
    rows, cur, samples = [], None, []
    with open(path, errors="replace") as f:
        for line in f:
            m = PROF.search(line)
            if m:
                cur = {"span": int(m[1]), "allow": int(m[2]), "plan": int(m[3]),
                       "evid": int(m[4], 16), "clrL": int(m[5]), "clrR": int(m[6])}
                samples = []
                continue
            if cur is not None and "[b5 scan]" in line and "samples" not in line:
                samples += SAMPLE.findall(line.split("[b5 scan]", 1)[1])
                continue
            m = DEC.search(line)
            if m and cur is not None:
                cur.update(side=m[1], found=int(m[2]), conf=int(m[3]), closest=int(m[4]),
                           closest_dd=int(m[5]), edgeL=int(m[6]), edgeR=int(m[7]),
                           limL=int(m[8]), limR=int(m[9]), need=int(m[10]))
                cur["n"] = len(samples)
                cur["unknown"] = sum(1 for s in samples if s[2] == "x")
                cur["weak"] = sum(1 for s in samples if s[2] == "w")
                rows.append(cur)
                continue
            if rows:
                last = rows[-1]
                m = EVL.search(line)
                if m:
                    last.update(meas=int(m[1]), cov=int(m[2]), stable=int(m[3]))
                m = SWP.search(line)
                if m:
                    last["sw" + m[1]] = {"not run": "-", "FAIL": "FAIL", "PASS_ASSUMED": "ASSUMED", "PASS": "PASS"}[m[2]]
                m = PNG.search(line)
                if m:
                    last["pings"] = "E%s NR%s F%s(hi%s rise%s fall%s short%s) NRw %s..%s" % tuple(m.groups())
    return rows


def main(paths):
    for p in paths:
        rows = parse(p)
        print(f"== {p}: {len(rows)} scan(s)")
        print(" #  side   span  allow  plan  clrL  clrR  need conf unk/n closest@dd    edges L/R  lim  evidence")
        for i, r in enumerate(rows, 1):
            ev = ",".join(v for k, v in EVID.items() if r["evid"] & k) or "-"
            print(f"{i:2d}  {r['side']:5s} {r['span']:5d} {r['allow']:6d} {r['plan']:5d} {r['clrL']:5d} {r['clrR']:5d} "
                  f"{r['need']:5d} {r['conf']:4d} {r['unknown']:2d}/{r['n']:<2d} {r['closest']:4d}@{r['closest_dd']:<5d} "
                  f"{r['edgeL']:5d}/{r['edgeR']:<5d} {r['limL']}/{r['limR']}  {ev}")
            if "cov" in r:
                print(f"      rule B: measured {r['meas']}% coverage {r['cov']}% weak {r['weak']}/{r['n']} "
                      f"stable span {r['stable']} mm; sweep L {r.get('swL', '?')} R {r.get('swR', '?')}"
                      + (f"; {r['pings']}" if "pings" in r else ""))
        if rows:
            spans = [r["span"] for r in rows]
            sides = {s: sum(1 for r in rows if r["side"] == s) for s in ("LEFT", "RIGHT", "NONE")}
            print(f"   span min/mean/max {min(spans)}/{sum(spans)/len(spans):.0f}/{max(spans)} mm; "
                  f"planning {min(r['plan'] for r in rows)}-{max(r['plan'] for r in rows)} mm; sides {sides}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    main(sys.argv[1:])
