# Captured ADC fixtures

- `barcode_Z_captured.csv`: earlier original-v2.3 Z capture; v0.3 replay decodes once through consensus.
- `barcode_Z_latest.csv`: failed physical v0.2 run3; v0.3 replay decodes Z once via complete-frame/quiet-zone fallback.
- `barcode_Y_captured.csv`: unresolved original Y capture; no decoded frame. Negative fixture is not evidence that Y support is complete.
- `barcode_Z_v03_failed.csv`: newest physical v0.3 run1, also unresolved. Actual v0.3 replay and tested threshold/hysteresis sweep produce no complete candidates. Retained explicitly for investigation; not claimed to pass the positive regression.

These are the first2048 rows of each run. Coordinates are quantised to0.01mm, timestamps are run-relative microseconds. Larger runs are truncated; see corresponding integration raw metadata/review JSONs. Printed dimensions are user-confirmed; no artwork modification was made.
