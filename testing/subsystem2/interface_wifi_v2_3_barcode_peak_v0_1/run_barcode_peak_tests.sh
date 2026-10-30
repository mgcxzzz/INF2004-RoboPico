#!/bin/sh
# Barcode peak decoder v0.1 host checks (no SDK, no hardware). From this folder:
#   sh run_barcode_peak_tests.sh
set -u
cd "$(dirname "$0")"
fails=0
CCS=""; for c in clang gcc; do command -v "$c" >/dev/null 2>&1 && CCS="$CCS $c"; done
O=${TMPDIR:-/tmp}/barcode_peak_tests.$$; mkdir -p "$O"
for cc in $CCS; do
  echo "== $cc -Wall -Wextra -Werror"
  if $cc -std=c11 -Wall -Wextra -Werror -O1 -I. barcode_peak_tests/test_barcode_peak.c -lm -o "$O/t_$cc" && "$O/t_$cc" > "$O/out_$cc.txt"; then
    grep -E '^(capture|synthetic|harsh|random|latency|PASS)' "$O/out_$cc.txt"
  else cat "$O/out_$cc.txt" 2>/dev/null; echo "FAILED ($cc)"; fails=$((fails+1)); fi
done
san=""
for cc in $CCS; do
  if $cc -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -I. \
       barcode_peak_tests/test_barcode_peak.c -lm -o "$O/t_san" 2>/dev/null; then san=$cc; break; fi
done
if [ -n "$san" ]; then
  echo "== $san ASan/UBSan"
  if "$O/t_san" > "$O/out_san.txt" 2>&1; then grep '^PASS' "$O/out_san.txt"; else tail -20 "$O/out_san.txt"; echo "FAILED (sanitizers)"; fails=$((fails+1)); fi
else echo "SKIP sanitizers unavailable"; fi
echo "== telemetry decoder round trip"
first=$(echo $CCS | cut -d' ' -f1)
grep '^f2' "$O/out_$first.txt" > "$O/hex.txt"
if python3 barcode_peak_decode.py "$O/hex.txt" | tail -1 | grep -q '^,90,16909060,v0.1,active,1,0,0,0,rev,0,Z,ok,15,0,0,175,206,2287,31,1.25,3.05,3.0,141.5,7$'; then
  echo "PASS source-15 barcode-peak frame decodes to the packed values"; else echo "FAILED decoder"; python3 barcode_peak_decode.py "$O/hex.txt"; fails=$((fails+1)); fi
echo "== existing v2.3 suite (run_tests.py, CC=${CC:-clang})"
if CC=${CC:-clang} python3 run_tests.py > "$O/run_tests.txt" 2>&1; then tail -1 "$O/run_tests.txt"; else tail -15 "$O/run_tests.txt"; echo "FAILED run_tests.py"; fails=$((fails+1)); fi
rm -rf "$O"
echo "failures: $fails"; exit "$fails"
