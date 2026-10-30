#!/bin/sh
# Host-only review checks. No Pico/broker connection or motor commands.
# Current integrated build: combined v0.5 (v2.3 + Buddy4 + ramp assist v0.6 + barcode peak decoder).
set -eu
ROOT=$(CDPATH= cd -- "$(dirname "$0")/../.." && pwd)
CUR="$ROOT/testing/subsystem2/interface_wifi_v2_3_b4_peak_v0_5"
PEAK="$ROOT/testing/subsystem2/interface_wifi_v2_3_barcode_peak_v0_1"
command -v node >/dev/null
command -v python3 >/dev/null
printf '%s\n' '== Combined v0.5: peak decoder, integration with the original decoder, dashboard path, ramp assist, Buddy4'
(cd "$CUR" && sh run_combined_tests.sh)
printf '%s\n' '== Standalone barcode peak decoder (v2.3 + peak only)'
LOG=$(mktemp "${TMPDIR:-/tmp}/peak-review.XXXXXX")
if (cd "$PEAK" && sh run_barcode_peak_tests.sh) > "$LOG" 2>&1; then grep -E '^(PASS|ALL)' "$LOG"; rm -f "$LOG"
else tail -30 "$LOG"; rm -f "$LOG"; echo 'FAILED standalone peak tests'; exit 1; fi
printf '%s\n' 'PASS review checks (host only).'
