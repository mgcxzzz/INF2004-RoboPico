#!/usr/bin/env python3
"""Decode ramp-assist (v0.1/v0.2) source-15 frames from a native mosquitto_sub capture
(lines: ISO-time topic hex) or from bare hex lines. Prints CSV to stdout.
Usage: ramp_assist_decode.py CAPTURE [FROM_LINE [TO_LINE]]
'detector' = flags bits 4-6 (0 in v0.1 firmware, reported as 1; v0.2 = 2)."""
import struct, sys

EVT = {
    0: "periodic",
    1: "activate",
    2: "end",
    3: "stall_push",
    4: "hold_end",
}  # 4: v0.5 tilt floor end, v0.6 speed hold end
END = {
    0: "none",
    1: "level",
    2: "descent",
    3: "invalid",
    4: "stale",
    5: "timeout",
    6: "not_following",
    7: "paused",
    8: "stopped",
}
FIELDS = [
    "time",
    "seq",
    "uptime_ms",
    "event",
    "build",
    "assisting",
    "latched",
    "sample_valid",
    "last_end",
    "activations",
    "alpha_milli",
    "added_pwm_left",
    "added_pwm_right",
    "height_mm",
    "peak_mm",
    "interval_max_mm",
    "interval_min_mm",
    "valid_samples",
    "invalid_samples",
    "longest_over_enter",
    "max_abs_i_left",
    "max_abs_i_right",
    "i_limit_ticks",
    "active_ms",
    "peak_added_pwm",
    "detector",
    "estimator",
]


def decode(b):
    if len(b) != 31 or b[0] != 0xF2 or b[1] != 28:
        return None
    (up, ev, fl, end, acts, alpha, pl, pr, h, pk, mx, mn, v, iv, lg, il, ir, sat, ams, ppk) = (
        struct.unpack("<IBBBBHhhbbbbBBBBBBHh", bytes(b[3:]))
    )
    return {
        "seq": b[2],
        "uptime_ms": up,
        "event": EVT.get(ev, ev),
        "build": "active" if fl & 1 else "shadow",
        "assisting": int(bool(fl & 2)),
        "latched": int(bool(fl & 4)),
        "sample_valid": int(bool(fl & 8)),
        "last_end": END.get(end, end),
        "activations": acts,
        "alpha_milli": alpha,
        "added_pwm_left": pl,
        "added_pwm_right": pr,
        "height_mm": "" if h == -128 else h,
        "peak_mm": pk,
        "interval_max_mm": "" if mx == -128 else mx,
        "interval_min_mm": "" if mn == 127 else mn,
        "valid_samples": v,
        "invalid_samples": iv,
        "longest_over_enter": lg,
        "max_abs_i_left": il,
        "max_abs_i_right": ir,
        "i_limit_ticks": sat,
        "active_ms": ams,
        "peak_added_pwm": ppk,
        "detector": ((fl >> 4) & 7) or 1,
        "estimator": 2 if fl & 0x80 else 1,
    }


def main():
    path = sys.argv[1]
    a = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    z = int(sys.argv[3]) if len(sys.argv) > 3 else 1 << 62
    print(",".join(FIELDS))
    with open(path) as f:
        for n, line in enumerate(f, 1):
            if n < a or n > z:
                continue
            parts = line.split()
            hx, t = (parts[2], parts[0]) if len(parts) >= 3 else (parts[0] if parts else "", "")
            if not hx.lower().startswith("f2"):
                continue
            try:
                d = decode(bytes.fromhex(hx))
            except ValueError:
                continue
            if d:
                d["time"] = t
                print(",".join(str(d[k]) for k in FIELDS))


if __name__ == "__main__":
    main()
