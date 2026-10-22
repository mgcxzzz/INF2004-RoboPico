# IMU hardware observations 22 October 2026

User-supplied V1 PuTTY log:

```text
DEVICE: A_addr=0x19 A_id=0x00 expected=0x33 acc_cfg=0 | M_addr=0x1e M_id=00,00,00 expected=48,34,33 mag_cfg=0 init_errors=2
BUS: SDA=1 SCL=1 before scan
SCAN: found=0 expected A=0x19 M=0x1e; SDA=1 SCL=1
SUMMARY: A_n=0 M_n=0 A_errors=0 M_errors=0 A_overruns=0 A_clips=0 HR_lowbits_nonzero=0 M_locks=0 M_overflows=0 A_max_gap_ms=0.00 M_max_gap_ms=0.00 poll_max_ms=6.97 late_polls=0 motors=OFF
```

The log continued beyond 85 seconds of uptime. No valid IMU samples or successful
ID reads were obtained. Zero ID bytes and raw zeros were unfilled placeholders;
they do not establish that the chip returned zero. Runtime sampling errors did
not increase because configuration never succeeded and sampling was disabled.
The V1 non-finite output was misleading; V2 uses explicit NA/NO_SAMPLE markers.

The user reported no illuminated IMU power indicator while computer USB powered
the robot, while GP2/GP3 indicators were lit. This motivates checking module-side
power and cable order but does not alone prove the electrical fault or a dead
sensor. The bus being high at rest is compatible with either a connected idle
bus or a disconnected module.

The user then supplied a module pinout image labelled VIN, 3.3V, GND, SCL, SDA,
INT2, INT1, DRDY. Recommended four-wire mapping for that module is VIN to Grove
2 3V3, GND to GND, SCL to GP3, SDA to GP2, with the other module pins unused.
No clear photo of the actual connected module or voltage measurement has yet
been received. No successful V2 hardware result is recorded.
