"""Read-only USB timing check for RTOS Step 1. Sends no robot commands."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
from pathlib import Path
import re
import time

PAIR = re.compile(r"([a-z_][a-z_0-9]*)=([^\s]+)")


def parse_line(line: str) -> tuple[str, dict]:
    kind = line.split(" ", 1)[0]
    values = {}
    for key, value in PAIR.findall(line):
        try:
            values[key] = int(value, 16 if value.startswith("0x") else 10)
        except ValueError:
            values[key] = value
    return kind, values


def analyze(rows: list[dict]) -> dict:
    clocks, controls = [], []
    for row in rows:
        if row["kind"] == "CLOCK" and "timer_us" in row:
            # A p/n query can repeat the same stored sample; do not count it
            # as another independent measurement.
            if not clocks or row["kernel_ms"] != clocks[-1]["kernel_ms"]:
                clocks.append(row)
        elif row["kind"] == "CONTROL" and "count" in row:
            controls.append(row)
    if len(clocks) < 2 or len(controls) < 2:
        return {"result": "INCONCLUSIVE", "reason": "Not enough CLOCK/CONTROL samples; check the UF2 and COM port."}
    first, last = clocks[0], clocks[-1]
    wall = last["wall_s"] - first["wall_s"]
    if wall < 20:
        return {"result": "INCONCLUSIVE", "reason": "Capture at least 20 seconds between independent clock samples."}
    kernel = ((last["kernel_ms"] - first["kernel_ms"]) & 0xffffffff) / 1000
    timer = ((last["timer_us"] - first["timer_us"]) & 0xffffffff) / 1e6
    control_wall = controls[-1]["wall_s"] - controls[0]["wall_s"]
    hz = ((controls[-1]["count"] - controls[0]["count"]) & 0xffffffff) / control_wall if control_wall > 0 else 0
    new_late = controls[-1]["late"] - controls[0]["late"]
    issues = []
    if not 0.97 <= kernel / wall <= 1.03:
        issues.append("Kernel clock differs from laptop wall time by more than 3%.")
    if not 0.97 <= timer / wall <= 1.03:
        issues.append("Hardware TIMER differs from laptop wall time by more than 3%.")
    if not 97 <= hz <= 103:
        issues.append("Control task is outside 97..103 updates per wall second.")
    if last.get("status") != "PASS":
        issues.append("Firmware internal clock validation has not passed.")
    if new_late:
        issues.append("Additional control intervals exceeded 15 ms during capture.")
    if controls[-1]["tx_drop"] != controls[0]["tx_drop"] or controls[-1]["rx_drop"] != controls[0]["rx_drop"]:
        issues.append("USB diagnostics were dropped during capture; recapture before interpreting timing.")
    if any(row.get("active", 0) for row in controls):
        issues.append("This idle clock check included an active motor test; repeat it idle first.")
    return {"result": "FAIL" if issues else "PASS", "issues": issues,
            "wall_seconds": round(wall, 6), "kernel_seconds": round(kernel, 6),
            "timer_seconds": round(timer, 6), "control_hz": round(hz, 3),
            "new_late_intervals": new_late, "samples": len(clocks),
            "limits": "Both clocks within 3% of laptop time; 97..103Hz; no new >15ms intervals or USB drops.",
            "scope": "USB software timing observation; not a physical stop-latency or motor-performance measurement."}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--seconds", type=int, default=40)
    parser.add_argument("--output", type=Path, default=Path("rtos_test_session"))
    args = parser.parse_args()
    if args.seconds < 25:
        parser.error("Use at least 25 seconds; 40 is recommended.")
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        raise SystemExit("Install pyserial once: py -3 -m pip install pyserial") from None
    if not args.port:
        for item in serial.tools.list_ports.comports():
            print(f"{item.device}: {item.description}")
        args.port = input("Pico COM port (e.g. COM4): ").strip()
    if args.port.isdecimal():
        args.port = "COM" + args.port
    args.output.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    raw_path = args.output / (stamp + "_usb.txt")
    rows = []
    print("Close PuTTY first. Capturing idle diagnostics; no commands or motor requests are sent.")
    try:
        with serial.Serial(args.port, 115200, timeout=0.25) as port, raw_path.open("w", encoding="utf-8") as raw:
            start = time.monotonic()
            # Old status lines may be buffered before a terminal opens. Keep
            # them in the raw evidence, but exclude the first 2s from rate
            # measurements so draining that backlog cannot create false drift.
            while time.monotonic() - start < args.seconds + 2:
                line = port.readline().decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                wall = time.monotonic() - start
                raw.write(f"{wall:.6f}\t{line}\n")
                raw.flush()
                kind, values = parse_line(line)
                if wall >= 2 and kind in ("CLOCK", "CONTROL", "CLOCK_REG", "WHEELS"):
                    rows.append({"wall_s": wall, "kind": kind, **values})
                print(line)
    except serial.SerialException as error:
        print(f"USB capture failed: {error}. Recorded data is retained; success is not assumed.")
        return 2
    result = analyze(rows)
    summary_path = args.output / (stamp + "_summary.json")
    summary_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    keys = ["wall_s", "kind"] + sorted(set().union(*(row.keys() for row in rows)) - {"wall_s", "kind"})
    with (args.output / (stamp + "_samples.csv")).open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(result, indent=2))
    print(f"Send the files in {args.output.resolve()} back for review.")
    return 0 if result["result"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
