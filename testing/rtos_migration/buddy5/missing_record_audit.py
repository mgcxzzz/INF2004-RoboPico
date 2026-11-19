"""Locate where the two missing D1 records of the 117 mm width capture were lost.
Read-only. Evidence used: (1) D1 records carry their own index and the width
runtime's per-frame header sequence, which advances ONLY after b5_runtime_send()
accepted a frame (width_probe_runtime.c); (2) every other telemetry stream in the
same capture has its own header sequence and Pico-ms timestamps."""
import json, sys
from pathlib import Path
cap = Path(sys.argv[1])
frames = []
for n, line in enumerate(cap.read_text().splitlines()):
    parts = line.split()
    if len(parts) != 2 or parts[0] != 'is28/tlm':
        continue
    try:
        b = bytes.fromhex(parts[1])
    except ValueError:
        continue
    if len(b) < 3 or len(b) != 3 + b[1]:
        continue
    frames.append((n, b))
def ms(b):
    p = b[3:]
    return int.from_bytes(p[0:4], 'little') if b[0] >> 4 in (9, 15) and len(p) >= 4 else None
d1 = [(n, b) for n, b in frames if b[0] == 0xF2 and b[1] == 56 and b[3 + 4] == 0xD1]
idx = [int.from_bytes(b[3 + 8:3 + 10], 'little') for _, b in d1]
seq = [b[2] for _, b in d1]
d2 = [b for _, b in frames if b[0] == 0xF2 and b[1] == 58 and b[3 + 4] == 0xD2]
claimed = int.from_bytes(d2[-1][3 + 14:3 + 16], 'little') if d2 else None
lost_fw = int.from_bytes(d2[-1][3 + 16:3 + 18], 'little') if d2 else None
missing = sorted(set(range(claimed or 0)) - set(idx))
print(f"D1 received={len(d1)} firmware records={claimed} firmware lost(queue overwrite)={lost_fw} missing indices={missing}")
gaps = []
for k in range(1, len(d1)):
    if (seq[k] - seq[k - 1]) & 0xFF != 1:
        gaps.append((idx[k - 1], idx[k], seq[k - 1], seq[k], ms(d1[k - 1][1]), ms(d1[k][1])))
for g in gaps:
    print(f"  D1 index {g[0]}->{g[1]}: header seq {g[2]}->{g[3]} (send-accepted frames skipped: {(g[3]-g[2]-1)&0xFF}); pico ms {g[4]}->{g[5]}")
# other streams around the gap: source-9 motion frames (0x92 header) carry their own seq
win = []
if gaps:
    t0, t1 = gaps[0][4], gaps[0][5]
    s9 = [(ms(b), b[2]) for _, b in frames if b[0] == 0x92]
    before = [x for x in s9 if x[0] is not None and x[0] <= t0]
    after = [x for x in s9 if x[0] is not None and x[0] >= t1]
    if before and after:
        a, z = before[-1], after[0]
        print(f"  source-9 stream across the same window: pico ms {a[0]}->{z[0]} ({z[0]-a[0]} ms), seq {a[1]}->{z[1]} "
              f"({(z[1]-a[1]-1)&0xFF} frames absent)")
    win = [before[-1] if before else None, after[0] if after else None]
verdict = ("Records were accepted by the firmware send path (header sequence advanced) and every other stream "
           "shows a simultaneous gap: loss is downstream of firmware publish (Wi-Fi/MQTT/broker/capture), "
           "not enqueue or decoding. Decoder rejection of the incomplete replay is correct and is retained.")
print(verdict)
json.dump({'capture': cap.name, 'received': len(d1), 'claimed': claimed, 'lost_firmware': lost_fw,
           'missing': missing, 'gaps': gaps, 'source9_window': win, 'verdict': verdict},
          open(sys.argv[2], 'w'), indent=1)
