# Raised-height V1 validation - 30 October 2026

Base V2.3 commit: 8d7f3aa4935565636bba013c74e563f5e8135bcb. Original folder unchanged.

| Check | Result / limit |
| --- | --- |
| Host suite | PASS: 37 C executables, 10 JavaScript schema tests, 20 Python helpers; assertions enabled. |
| Steering | PASS: brief alternation bounds, full persistent bends, excess-yaw damping even without shrinking optical error, mirrored/brightness-independent responses, prompt old-correction release, STOP/recovery/manoeuvre bypass, scan/profile caps and non-finite inputs. |
| Baseline steering reference | PASS: original V2.3 steering header and original steering tests retained separately for comparison. |
| Fixed-input recorded replay | PASS: 256 rows, original OPT targets/calibrated levels/measured speeds. Target total variation 883 -> 780, mean target 31.04 -> 35.14 mm/s, minimum wheel target 6 -> 15 mm/s. This is not a closed-loop physical prediction. |
| Calibration storage | PASS: measured191/154,1940/2306 preset; old versions overridden; own valid29 retained; corrupt CRC rejected; c preset source immutable. |
| Diagnostics | PASS: bit packing unchanged, capture version5, corrupt or old version4 banks rejected. GAIN0..7=0..100%, DAMP0..3=0..8 mm/s absolute half-speed damping. |
| ARM Release Wi-Fi ON | PASS: warnings-as-errors, no credentials, no default Buddy5 extension. |
| Real lwIP allocator | PASS: original core-only pool exhausts; retained7+4 pool passes100 callback reschedules and1000 add/remove cycles. |
| ELF stacks | PASS: two separate8192-byte stacks, heap below both;4424-byte known driving chain and real call edges. No complete runtime worst-case bound. |
| Source/artifact/package integrity | Checked by verify_firmware.py against HEIGHT_BASELINE/HEIGHT_PATCH/SOURCE_SHA256. |
| Physical raised-height smoothness, bends, barcode and heartbeat | PENDING: needs user's robot. |

Evidence files are in logs/. The API/nav modules, network owner/transport, dashboard and binary wire protocol, barcode/optical/recovery algorithms and continuous motor PI match V2.3 byte-for-byte. Main adapts banner/calibration/save identity/initial old-log import; diagnostic storage adapts format/observational units. No physical robot commands were sent here.
