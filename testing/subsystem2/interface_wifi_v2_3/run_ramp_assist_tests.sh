#!/bin/sh
# ramp-assist v0.2 host checks (no SDK, no hardware). From interface_wifi_v2_3/:
#   sh run_ramp_assist_tests.sh
set -u
cd "$(dirname "$0")"
fails=0
CCS=""; for c in clang gcc; do command -v "$c" >/dev/null 2>&1 && CCS="$CCS $c"; done
O=${TMPDIR:-/tmp}/ramp_assist_tests.$$; mkdir -p "$O"
for cc in $CCS; do
  echo "== $cc -Wall -Wextra -Werror"
  if $cc -std=c11 -Wall -Wextra -Werror -O1 -I. -DLINE_CAL_MIN_SPAN=50 -DCIRCUIT_SWEEP_SEARCH=1 \
       ramp_assist_tests/test_ramp_assist.c -lm -o "$O/t_$cc" && "$O/t_$cc" > "$O/out_$cc.txt"; then
    grep -E '^(replay|climb|after|PASS)' "$O/out_$cc.txt"
  else cat "$O/out_$cc.txt" 2>/dev/null; echo "FAILED ($cc)"; fails=$((fails+1)); fi
done
san=""
for cc in $CCS; do
  if $cc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -I. \
       -DLINE_CAL_MIN_SPAN=50 -DCIRCUIT_SWEEP_SEARCH=1 ramp_assist_tests/test_ramp_assist.c -lm -o "$O/t_san" 2>/dev/null; then san=$cc; break; fi
done
if [ -n "$san" ]; then
  echo "== $san ASan/UBSan"
  if "$O/t_san" > "$O/out_san.txt" 2>&1; then grep '^PASS' "$O/out_san.txt"; else tail -20 "$O/out_san.txt"; echo "FAILED (sanitizers)"; fails=$((fails+1)); fi
else echo "SKIP sanitizers unavailable"; fi
echo "== telemetry decoder round trip"
first=$(echo $CCS | cut -d' ' -f1)
grep '^f2' "$O/out_$first.txt" > "$O/hex.txt"
if python3 ramp_assist_decode.py "$O/hex.txt" | tail -1 | grep -q '^,90,16909060,end,active,1,0,1,descent,3,200,30,36,-5,9,9,-2,24,1,12,100,55,7,1234,36,2,1$'; then
  echo "PASS source-15 frame decodes to the packed values"; else echo "FAILED decoder"; fails=$((fails+1)); fi
echo "== existing v2.3 suite (run_tests.py, CC=clang)"
if CC=${CC:-clang} python3 run_tests.py > "$O/run_tests.txt" 2>&1; then tail -1 "$O/run_tests.txt"; else tail -15 "$O/run_tests.txt"; echo "FAILED run_tests.py"; fails=$((fails+1)); fi
rm -rf "$O"
echo "failures: $fails"; exit "$fails"
