"""Read-only USB + passive MQTT heartbeat capture for RTOS Step 3A."""
from __future__ import annotations
import argparse, csv, datetime as dt, json, socket, sys, threading, time, uuid
from pathlib import Path
from capture_bringup import parse_line, analyze as analyze_clock
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "dashboard"))
from check_mqtt import mqtt_connect_packet, subscribe_packet, read_packet, PacketTimeout


def heartbeat(payload: bytes):
    if len(payload) != 15 or payload[:2] != b"\x12\x0c":
        return None
    word = int.from_bytes(payload[3:7], "little")
    return {"sequence": payload[2], "uptime_s": (word >> 12) & 16383,
            "state": (word >> 2) & 7, "net": word & 3}


def analyze(rows, beats, errors):
    clock = analyze_clock(rows)
    issues = list(errors)
    net = [r for r in rows if r["kind"] == "NET"]
    if clock.get("result") != "PASS":
        issues.append("Clock/control readiness check did not pass.")
    if len(net) < 20:
        issues.append("Not enough independent network status samples.")
    else:
        first, last = net[0], net[-1]
        if last.get("polls", 0) <= first.get("polls", 0) or last.get("last_ms", 0) <= first.get("last_ms", 0):
            issues.append("Kernel network worker did not advance.")
        if any(r.get("motors") != "DISABLED" or r.get("connected") != 1 or r.get("subscribed") != 1 for r in net):
            issues.append("Bench mode or MQTT connection was not maintained throughout capture.")
        if last.get("tx_drop") != first.get("tx_drop") or last.get("rx_drop") != first.get("rx_drop"):
            issues.append("Network buffer drops increased during capture.")
        if last.get("received", 0) <= first.get("received", 0):
            issues.append("No dashboard command received during capture; use Ping/apply mapping after ten seconds.")
    gaps = [b["wall_s"] - a["wall_s"] for a, b in zip(beats, beats[1:])]
    if len(beats) < 20:
        issues.append("Insufficient MQTT heartbeats received from the car.")
    else:
        wall = beats[-1]["wall_s"] - beats[0]["wall_s"]
        uptime = (beats[-1]["uptime_s"] - beats[0]["uptime_s"]) & 16383
        if wall < 20 or abs(uptime - wall) > max(2, wall * .03):
            issues.append("Heartbeat uptime did not track laptop wall time.")
        if max(gaps) > 2:
            issues.append("Observed heartbeat gap exceeded two seconds.")
    return {"result": "PASS" if not issues else "REVIEW", "issues": issues,
            "clock": clock, "heartbeat_count": len(beats), "max_heartbeat_gap_s": max(gaps, default=None),
            "network_first": net[0] if net else None, "network_last": net[-1] if net else None,
            "scope": "Stopped network/clock test only; no driving or junction acceptance."}


def monitor(broker, stop, beats, errors, started):
    try:
        with socket.create_connection((broker, 1883), timeout=5) as sock:
            sock.sendall(mqtt_connect_packet("rtos-review-" + uuid.uuid4().hex[:8]))
            header, body = read_packet(sock, 5)
            if header != 0x20 or body != b"\x00\x00":
                raise ValueError("Broker refused the read-only monitor.")
            sock.sendall(subscribe_packet())
            last_ping = time.monotonic()
            while not stop.is_set():
                if time.monotonic() - last_ping > 20:
                    sock.sendall(b"\xc0\x00"); last_ping = time.monotonic()
                try:
                    header, body = read_packet(sock, 3)
                except PacketTimeout:
                    if stop.is_set(): break
                    raise ValueError("No MQTT traffic for three seconds during heartbeat capture.")
                if header >> 4 != 3 or header & 1 or len(body) < 2: continue
                length = int.from_bytes(body[:2], "big")
                if body[2:2 + length] != b"is28/hb": continue
                beat = heartbeat(body[2 + length:])
                if beat is not None:
                    beats.append({"wall_s": time.monotonic() - started, **beat})
    except Exception as error:
        errors.append("MQTT monitor: " + str(error))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port")
    parser.add_argument("--broker", default="127.0.0.1", help="Broker seen from this laptop; localhost is valid here")
    parser.add_argument("--seconds", type=float, default=75)
    parser.add_argument("--output", type=Path, default=Path("rtos_network_session"))
    args = parser.parse_args()
    if args.seconds < 30: parser.error("Capture at least30seconds.")
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:
        raise SystemExit("Install pyserial once: py -3 -m pip install pyserial")
    if not args.port:
        for p in serial.tools.list_ports.comports(): print(p.device, p.description)
        args.port = input("Pico port (COM4 or full /dev/cu... path): ").strip()
    if args.port.isdecimal(): args.port = "COM" + args.port
    args.output.mkdir(parents=True, exist_ok=True)
    prefix = args.output / dt.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    print("Keep broker/dashboard running; close PuTTY. No robot commands are sent.")
    print("During capture, use dashboard Ping and apply a mapping. Keep the car stopped.")
    rows, beats, errors = [], [], []
    stop = threading.Event(); started = time.monotonic()
    worker = threading.Thread(target=monitor, args=(args.broker, stop, beats, errors, started), daemon=True)
    worker.start()
    try:
        with serial.Serial(args.port, 115200, timeout=.25) as port, prefix.with_name(prefix.name + "_usb.txt").open("w", encoding="utf-8") as raw:
            while time.monotonic() - started < args.seconds + 2:
                line = port.readline().decode("utf-8", errors="replace").strip()
                if not line: continue
                wall = time.monotonic() - started
                raw.write(f"{wall:.6f}\t{line}\n"); raw.flush()
                kind, values = parse_line(line)
                if wall >= 2: rows.append({"wall_s": wall, "kind": kind, **values})
                if kind in ("NET", "CLOCK"): print(line)
    except serial.SerialException as error:
        errors.append("USB capture: " + str(error))
    finally:
        stop.set(); worker.join(4)
    result = analyze(rows, beats, errors)
    prefix.with_name(prefix.name + "_summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    with prefix.with_name(prefix.name + "_heartbeats.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["wall_s", "sequence", "uptime_s", "state", "net"])
        writer.writeheader(); writer.writerows(beats)
    print(json.dumps(result, indent=2)); print("Send the three files in", args.output.resolve())
    return 0 if result["result"] == "PASS" else 2


if __name__ == "__main__": raise SystemExit(main())
