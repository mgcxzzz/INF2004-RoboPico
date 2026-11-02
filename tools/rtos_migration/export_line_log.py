"""Export stopped Step 2 RAM history over USB. Never sends a movement command."""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import io
import json
from pathlib import Path
import re
import time

COLUMNS = ("sequence,ms,ticks_l,ticks_r,speed_l_x10,speed_r_x10,target_l,target_r,pwm_l,pwm_r,"
           "adc_l,adc_r,adc_side,level_l,level_r,phase,decision,reason,side_black,busy,white,search,"
           "pending,action,dt_us,cost_us,filtered_l,filtered_r,optical_target_l,optical_target_r,"
           "steer_gain_milli,damping_milli,stop_reason,control_flags").split(",")


def parse_capture(lines: list[str]) -> tuple[list[dict], dict]:
    rows, errors = [], []
    expected = overwritten = end_count = None
    collecting = ended = False
    calibration = None
    for line in lines:
        if line.startswith("LINE_LOG_BEGIN "):
            found = re.fullmatch(r"LINE_LOG_BEGIN version=1 rows=(\d+) overwritten=(\d+)", line)
            if not found:
                errors.append("Unrecognized log header.")
                continue
            if collecting or ended:
                errors.append("Multiple dumps in one capture.")
            expected, overwritten = map(int, found.groups())
            collecting = True
        elif line.startswith("LINE_LOG_CAL ") and collecting:
            calibration = line
        elif line.startswith("LINE_ROW ") and collecting:
            cells = next(csv.reader(io.StringIO(line[len("LINE_ROW "):])))
            try:
                if len(cells) != len(COLUMNS):
                    raise ValueError("wrong column count")
                row = dict(zip(COLUMNS, map(int, cells)))
                if any(not 0 <= row[k] <= 4095 for k in
                       ("adc_l", "adc_r", "adc_side", "filtered_l", "filtered_r")):
                    raise ValueError("ADC outside 12-bit range")
                if any(not 0 <= row[k] <= 1000 for k in
                       ("level_l", "level_r", "steer_gain_milli", "damping_milli")):
                    raise ValueError("normalized field outside range")
                if rows and row["sequence"] != rows[-1]["sequence"] + 1:
                    errors.append("Missing/duplicate/reordered sample sequence.")
                if rows and row["ms"] < rows[-1]["ms"]:
                    errors.append("Sample time moved backwards.")
                rows.append(row)
            except ValueError as error:
                errors.append(f"Malformed row: {error}.")
        elif line.startswith("LINE_LOG_END ") and collecting:
            found = re.fullmatch(r"LINE_LOG_END rows=(\d+)", line)
            if found:
                end_count = int(found.group(1))
                collecting, ended = False, True
    if expected is None:
        errors.append("No log header; stop the robot and use Step 2 firmware.")
    if not ended:
        errors.append("Incomplete dump: end marker missing.")
    if expected is not None and (len(rows) != expected or end_count != expected):
        errors.append("Row count does not match the firmware header/end marker.")
    if calibration is None:
        errors.append("Run calibration metadata missing.")
    info = {"result": "COMPLETE" if not errors else "INCOMPLETE", "issues": errors,
            "rows": len(rows), "overwritten": overwritten, "run_calibration": calibration,
            "scope": "Copied diagnostic history; COMPLETE means export integrity, not driving acceptance."}
    if rows:
        info.update(first_ms=rows[0]["ms"], last_ms=rows[-1]["ms"],
                    max_dt_us=max(r["dt_us"] for r in rows),
                    max_control_cost_us=max(r["cost_us"] for r in rows),
                    late_over_15ms=sum(r["dt_us"] > 15000 for r in rows),
                    search_rows=sum(r["search"] for r in rows),
                    phases=sorted(set(r["phase"] for r in rows)))
    return rows, info


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--input", type=Path, help="Parse a saved PuTTY dump without opening USB")
    parser.add_argument("--output", type=Path, default=Path("rtos_line_session"))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    prefix = args.output / stamp
    lines = []
    if args.input:
        lines = args.input.read_text(encoding="utf-8-sig", errors="replace").splitlines()
    else:
        try:
            import serial
            import serial.tools.list_ports
        except ImportError:
            print("Install the USB dependency first: py -3 -m pip install pyserial")
            return 2
        if not args.port:
            for item in serial.tools.list_ports.comports():
                print(f"{item.device}: {item.description}")
            args.port = input("Pico COM port (e.g. COM4): ").strip()
        if args.port.isdecimal():
            args.port = "COM" + args.port
        print("Robot must already be stopped. Close PuTTY; keep USB power connected.")
        print("Sending d only; exporting history. No Start, reset or calibration command is sent.")
        try:
            with serial.Serial(args.port, 115200, timeout=.25, write_timeout=2) as port:
                # Let initial buffered status drain without assuming a completed run.
                time.sleep(.5)
                port.write(b"d")
                deadline = time.monotonic() + 120
                while time.monotonic() < deadline:
                    line = port.readline().decode("utf-8", errors="replace").strip()
                    if not line:
                        continue
                    lines.append(line)
                    if not line.startswith("LINE_ROW "):
                        print(line)
                    if line.startswith("LINE_LOG_END ") or line.startswith("LOG rejected:"):
                        break
        except serial.SerialException as error:
            print(f"USB export interrupted: {error}. Partial evidence is retained.")
    prefix.with_name(prefix.name + "_usb.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    rows, info = parse_capture(lines)
    with prefix.with_name(prefix.name + "_line.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    prefix.with_name(prefix.name + "_summary.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(info, indent=2))
    print(f"Send the three files in {args.output.resolve()} and the run video for review.")
    return 0 if info["result"] == "COMPLETE" else 2


if __name__ == "__main__":
    raise SystemExit(main())
