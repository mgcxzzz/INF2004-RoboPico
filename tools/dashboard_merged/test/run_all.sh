#!/bin/sh
# Merged dashboard v0.1 test runner. Simulated data only: no broker, Pico or car.
# From the repository root:   sh tools/dashboard_merged/test/run_all.sh
# Browser checks need Playwright (set PLAYWRIGHT_NODE_MODULE_PATH / CHROME_PATH if not global).
set -u
D=$(cd "$(dirname "$0")" && pwd)
fails=0
run() { echo "== $*"; "$@" || { echo "FAILED: $*"; fails=$((fails + 1)); }; }

run node --test "$D/test_merged_schema.js"
run node --test "$D/compat/v23_test_schema.js"
run node "$D/compat/rtos_check_speed_dashboard.mjs"
run node "$D/compat/rtos_check_dashboard.mjs"
if command -v cc >/dev/null 2>&1 && command -v python3 >/dev/null 2>&1; then
  run python3 "$D/compat/rtos_check_imu_dashboard.py"
else
  echo "SKIP INT-03 IMU vectors (needs cc and python3)"
fi
if node -e "require(process.env.PLAYWRIGHT_NODE_MODULE_PATH || 'playwright')" >/dev/null 2>&1; then
  export IS28_PLAYWRIGHT_MODULE="${IS28_PLAYWRIGHT_MODULE:-${PLAYWRIGHT_NODE_MODULE_PATH:-playwright}}"
  run node "$D/ui_merged.mjs"
  run node "$D/compat/v23_ui_smoke.mjs"
  run node "$D/compat/rtos_check_dashboard_ui.mjs"
else
  echo "SKIP browser checks (Playwright not installed)"
fi
echo "failures: $fails"
exit "$fails"
