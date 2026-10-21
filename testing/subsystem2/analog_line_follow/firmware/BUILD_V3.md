# Analog line-follow V3,21 October2026

Built and software-verified. New repeat/hands-free start still requires hardware tests.

Changes requested by user:
-Start accepts calibrated LEFT150..1000 and RIGHT<=450: full left black allowed.
  All-white and right-only/both-black still rejected. Calibration/edge target unchanged.
-Repeated runs without power cycle: after settling/save, release both and remain
  below estimated5mm/s for1second, then ready1. Explicit new request always required.
-g optional USB hands-free2second countdown; f original GP21 hold-to-run remains.
  Both buttons/s/USB disconnect cancel/stop g; sensor/start gates rechecked at start.
-Specific USB rejection reasons, live ready/pending/runs, log keyg/mode label.
-Bank starts still require fresh both-button3s/full-release/tap2s sequence.
-Unchanged V2 bounded25% breakaway, PI speeds, loss/encoder/time/distance cutoffs.
-SavedLog layout/magic/version identical to V2: existing calibration readable.

Checks PASS:
-Full Pico W SDK2.2.0/Arm GCC14.2.1 Release, application -Wall -Wextra -Werror.
-Host: release/standstill/settling readiness; readiness alone doesn't call guard;
  two actual guard start/stop cycles without reboot; GP20 reset; full-black
  acceptance and white/right-dark refusal; mode-specific hold/release checks;
  countdown button/disconnect cancellation; all prior calibration/PWM/loss tests.
-Static gates: no-movement threshold, active/generation PWM commit, g recheck,
  serial/button cancellation retained; SavedLog structure compared to V2.
-UF2 RP2040 family/headers/payload addresses below flash-log sector.
-ELF text56520/data0/BSS12068. GPIO/ADC/stdio wiring unchanged.

SHA256 buddy23_analog_line_follow_v3.uf2:

```
8cc692046fe8ed194fd1d977d731bf80e08e5e4021b41172370dc741074c7c5d
```

First physical check: computer USB only, h verify calibration/profile1; left edge
or black/right white; buttons released and ready1; g starts after2s. Stop with s
or either button. After log save, release/rest until ready1, p retrieve before
another g. No automatic rerun. Provide cable slack for any floor test.
