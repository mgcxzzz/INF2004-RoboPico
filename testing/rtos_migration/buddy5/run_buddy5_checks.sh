#!/bin/sh
# Focused Buddy 5 RTOS candidate host checks (logic only; no hardware).
# usage: sh testing/rtos_migration/buddy5/run_buddy5_checks.sh <bench_tools/buddy5_sdk> [out_dir]
# Needs gcc (ASan/UBSan), python3 and node. Frozen bench folders are read only.
set -u
SDK=$1; OUT=${2:-/tmp/b5_checks}; mkdir -p "$OUT"
ROOT=$(cd "$(dirname "$0")/../../.." && pwd); cd "$ROOT"
B=firmware/integration/buddy1_4; E=$B/extensions/buddy5_obstacles
CORE="$B/src/navigation/motion_api.c $B/src/navigation/nav_coordinator.c $B/src/navigation/nav_codec.c $B/src/motion/motion_executor.c"
CC="gcc -std=gnu11 -O1 -g -fsanitize=address,undefined -fno-sanitize-recover=undefined -DBUDDY_RUNTIME_MAPPING=1 -DBUDDY_COORDINATED_BARCODE=1"
fails=0; run() { echo "== $*"; "$@" || { echo "FAILED: $*"; fails=$((fails+1)); }; }
# 1. mission simulation + C-packed dashboard fixture (v0.2: steered approach
#    and width policy need the coordinator extension, as in the APP build)
STEER="-DMOTION_STEERED_DRIVE=1 -DLINE_CAL_MIN_SPAN=50"
$CC $STEER -Wall -Wextra -I $B -I app_program/obstacle_navigation testing/rtos_migration/test_b5_mission.c \
  app_program/obstacle_navigation/b5_mission.c $E/width_probe.c $E/width_estimate.c $E/edge_probe.c \
  $E/automatic_bypass.c $E/manual_bypass.c $E/rect_route.c $E/stripe_detect.c $E/profile/obstacle_profile.c $CORE -lm -o "$OUT/test_b5_mission"
run "$OUT/test_b5_mission" "$ROOT/dashboard/test/fixtures/b5_rtos_mission_c_frames.txt"
# 1b. NAV_STEER coordinator extension (codec, gating, bound, 100 ms lapse)
$CC $STEER -Wall -Wextra -I $B testing/rtos_migration/test_nav_steer.c $CORE -lm -o "$OUT/test_nav_steer"
run "$OUT/test_nav_steer"
# 2. frozen width v0.2 suites against the ported files
W=$SDK/width_calibrated_probe_v0_2/source/firmware/integration/buddy1_4/tests/width_only
mkdir -p "$OUT/w/tests/width_only"; cp "$W"/* "$OUT/w/tests/width_only/"; cp "$W"/test_width_calibrated.c "$W"/test_retry_speed.c "$OUT/w/"
printf '#include "src/motion/motion_port.h"\ntypedef motion_feedback_t MotionFeedback;\ntypedef motion_output_t MotionOutput;\ntypedef motion_port_t MotionPort;\ntypedef motion_event_t MotionEvent;\n' > "$OUT/w/sdk_names.h"
for t in test_width_calibrated test_retry_speed; do
  $CC -I "$OUT/w" -I $B -include "$OUT/w/sdk_names.h" "$OUT/w/$t.c" $E/width_probe.c $E/width_estimate.c $E/profile/obstacle_profile.c $CORE -lm -o "$OUT/w/$t"
  (cd "$OUT/w" && run "./$t")
done
# 3. frozen edge_probe v0.1.1 suite: exact v0.1.1 rule, then the 25 mm/v3 rule
T=$SDK/edge_probe_v0_1_1/source/firmware/integration/buddy1_4/tests/turn_only/test_edge_probe.c
mkdir -p "$OUT/e"; cp "$OUT/w/sdk_names.h" "$OUT/e/"; cp "$T" "$OUT/e/"
sed 's/fr\[3 + 5\] == 2/fr[3 + 5] == 3/g' "$T" > "$OUT/e/test_edge_probe25.c"
$CC -I "$OUT/e" -I $B -include "$OUT/e/sdk_names.h" -DB5EP_DIR_WIRE=2u -DB5EP_GATE_NEAR_MM=15 "$OUT/e/test_edge_probe.c" $E/edge_probe.c $E/profile/obstacle_profile.c $CORE -lm -o "$OUT/e/t15"
$CC -I "$OUT/e" -I $B -include "$OUT/e/sdk_names.h" "$OUT/e/test_edge_probe25.c" $E/edge_probe.c $E/profile/obstacle_profile.c $CORE -lm -o "$OUT/e/t25"
run "$OUT/e/t15"; run "$OUT/e/t25"
# 4. recorded evidence (read only)
run python3 -B testing/rtos_migration/buddy5/direction_near25_replay.py "$SDK" "$OUT/direction_near25_replay.json"
run python3 -B testing/rtos_migration/buddy5/missing_record_audit.py "$SDK/width_calibrated_probe_v0_2/hardware/2026-11-18_22-06-08_width_calibrated_v02_box117_centred_distance300_mqtt.txt" "$OUT/missing_record_audit_117.json"
# 5. dashboard decoders on the C-packed frames
run node --test dashboard/test/test_b5_rtos.js
run node dashboard/test/smoke_b5_panel.js dashboard
echo "failures: $fails"; exit "$fails"
