#!/bin/sh
# Combined Buddy1-4 + ramp assist + barcode peak build: all host checks (no SDK, broker, Pico or car).
# From this folder:  sh run_combined_tests.sh
# Needs clang and/or gcc, python3, node. CC/CFLAGS are passed to run_tests.py (e.g. CC=gcc CFLAGS=-Wno-misleading-indentation).
set -u
cd "$(dirname "$0")"
ROOT=$(cd ../../.. && pwd)
DASH="$ROOT/tools/dashboard_merged"
fails=0
O=${TMPDIR:-/tmp}/combined_tests.$$; mkdir -p "$O"
CCS=""; for c in clang gcc; do command -v "$c" >/dev/null 2>&1 && CCS="$CCS $c"; done
first=$(echo $CCS | cut -d' ' -f1)
DEFS="-DLINE_CAL_MIN_SPAN=50 -DCIRCUIT_SWEEP_SEARCH=1 -DBUDDY_RUNTIME_MAPPING=1"
ok() { echo "PASS $*"; }
bad() { echo "FAILED $*"; fails=$((fails+1)); }

echo "== 1. peak decoder suite (captures, synthetic A-Z/0-9 both directions, harsh, negatives, telemetry) + v2.3 run_tests.py"
if sh run_barcode_peak_tests.sh > "$O/peak.txt" 2>&1; then grep -E '^(PASS|ALL)' "$O/peak.txt" | sort -u; else tail -25 "$O/peak.txt"; bad "run_barcode_peak_tests.sh"; fi

echo "== 2. combined integration: real original decoder + runtime mapping + peak harness"
for cc in $CCS; do for a in 1 0; do
  if $cc -std=c11 -Wall -Wextra -Werror -O1 -I. $DEFS -DBARCODE_PEAK_ACTIVE=$a barcode_peak_tests/test_peak_integration.c wifi_protocol.c -lm -o "$O/pi_${cc}_$a" \
     && "$O/pi_${cc}_$a" > "$O/pi_${cc}_$a.txt"; then echo "$cc: $(tail -1 "$O/pi_${cc}_$a.txt")"; else cat "$O/pi_${cc}_$a.txt" 2>/dev/null; bad "integration $cc active=$a"; fi
done; done
san=""
for cc in $CCS; do
  if $cc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -I. $DEFS -DBARCODE_PEAK_ACTIVE=1 \
       barcode_peak_tests/test_peak_integration.c wifi_protocol.c -lm -o "$O/pi_san" 2>/dev/null; then san=$cc; break; fi
done
if [ -n "$san" ]; then
  if "$O/pi_san" > "$O/pi_san.txt" 2>&1; then echo "$san ASan/UBSan: $(tail -1 "$O/pi_san.txt")"; else tail -20 "$O/pi_san.txt"; bad "integration sanitizers"; fi
else echo "SKIP sanitizers unavailable"; fi

echo "== 3. combined reporting path: real C packers -> fixture -> Python and dashboard decoders"
if $first -std=c11 -Wall -Wextra -Werror -O1 -I. $DEFS -DBARCODE_PEAK_ACTIVE=1 barcode_peak_tests/gen_dashboard_frames.c wifi_protocol.c wifi_packets.c -lm -o "$O/gen" \
   && "$O/gen" > "$O/frames.txt"; then
  if cmp -s "$O/frames.txt" "$DASH/test/fixtures/sdk_b4_peak_c_frames.txt"; then ok "C packer output matches the committed dashboard fixture ($(wc -l < "$O/frames.txt") frames)";
  else bad "dashboard fixture is stale: regenerate with barcode_peak_tests/gen_dashboard_frames.c"; fi
  sed 's/^TLM //' "$O/frames.txt" > "$O/hex.txt"
  python3 source15_decode.py "$O/hex.txt" > "$O/s15.txt"
  r=$(grep -c '^ramp,' "$O/s15.txt"); p=$(grep -c '^peak,' "$O/s15.txt"); u=$(grep -c '^unknown,' "$O/s15.txt")
  if [ "$p" = 2 ] && [ "$r" -ge 10 ] && [ "$u" = 0 ] && grep -q '^peak,.*code=Y,reason=ok,bars=15,.*combined_build=1' "$O/s15.txt" \
     && grep -q '^ramp,.*detector=2,estimator=2' "$O/s15.txt"; then ok "source15_decode.py: $r ramp + $p peak frames interleaved, 0 unknown";
  else bad "source15_decode.py mixed stream (ramp=$r peak=$p unknown=$u)"; fi
else bad "dashboard frame generator"; fi
if (cd "$DASH" && node --test test/test_source15.js test/test_merged_schema.js test/compat/v23_test_schema.js > "$O/dash.txt" 2>&1); then
  ok "dashboard decoders: $(grep -E '^# pass' "$O/dash.txt")"; else grep -E 'not ok|Error' "$O/dash.txt" | head; bad "dashboard node tests"; fi

echo "== 4. existing ramp-assist suite (detector v0.6)"
if sh run_ramp_assist_tests.sh > "$O/ra.txt" 2>&1; then grep -E '^PASS' "$O/ra.txt" | sort -u; else tail -15 "$O/ra.txt"; bad "run_ramp_assist_tests.sh"; fi

echo "== 5. existing Buddy4 suites (service/packets, powered acceptance estimator v0.2)"
if sh buddy4/host/run_host_test.sh "$O/b4" "$DASH/is28_schema.js" > "$O/b4.txt" 2>&1; then grep -E '^PASS' "$O/b4.txt"; else tail -10 "$O/b4.txt"; bad "Buddy4 service test"; fi
if $first -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O1 -Ibuddy4/host/stubs -Ibuddy4 -Ibuddy4/subsystem4 buddy4/host/test_powered_acceptance.c -lm -o "$O/pa" \
   && "$O/pa" > "$O/pa.txt" 2>&1; then ok "Buddy4 powered acceptance: $(tail -1 "$O/pa.txt")"; else tail -10 "$O/pa.txt"; bad "Buddy4 powered acceptance"; fi

rm -rf "$O"
echo "failures: $fails"; exit "$fails"
