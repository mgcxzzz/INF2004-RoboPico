#!/bin/sh
# Host-only review checks. No Pico/broker connection or motor commands.
set -eu
ROOT=$(CDPATH= cd -- "$(dirname "$0")/../.." && pwd)
SDK="$ROOT/testing/subsystem2/interface_wifi_v2_3"
BAR="$ROOT/testing/subsystem2/interface_wifi_v2_3_barcode_fine_v0_3"
OUT=$(mktemp -d "${TMPDIR:-/tmp}/v23-review.XXXXXX")
trap 'rm -rf "$OUT"' EXIT HUP INT TERM
export CC=${CC:-clang}
command -v node >/dev/null
command -v "$CC" >/dev/null
printf '%s\n' '== Integrated v2.3/ramp-assist suites'
(cd "$SDK" && sh run_ramp_assist_tests.sh)
printf '%s\n' '== Buddy4 service/packet checks'
sh "$SDK/buddy4/host/run_host_test.sh" "$OUT/service" "$ROOT/tools/dashboard_merged/is28_schema.js"
printf '%s\n' '== Buddy4 powered estimator checks'
"$CC" -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -I"$SDK/buddy4/host/stubs" -I"$SDK/buddy4" -I"$SDK/buddy4/subsystem4" "$SDK/buddy4/host/test_powered_acceptance.c" -lm -o "$OUT/estimator"
"$OUT/estimator"
printf '%s\n' '== Barcode candidate suites (current physical failure is documented, not claimed fixed)'
(cd "$BAR" && python3 run_tests.py) > "$OUT/barcode.log" 2>&1 || { cat "$OUT/barcode.log"; exit 1; }
tail -1 "$OUT/barcode.log"
node "$BAR/test_fine_reporting.js" "$OUT/barcode.log" "$ROOT/tools/dashboard_merged/is28_schema.js"
(cd "$BAR" && "$CC" -std=c11 -Wall -Wextra -Werror -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=all -DBUDDY_RUNTIME_MAPPING=1 -DBUDDY_WIFI_LOGGING=1 test_barcode_fine.c wifi_protocol.c -lm -o "$OUT/barcode" && "$OUT/barcode")
printf '%s\n' '== Merged dashboard decoder checks'
node --test "$ROOT/tools/dashboard_merged/test/test_merged_schema.js" "$ROOT/tools/dashboard_merged/test/compat/v23_test_schema.js"
printf '%s\n' 'PASS review checks. Physical barcode decoding remains unresolved.'
