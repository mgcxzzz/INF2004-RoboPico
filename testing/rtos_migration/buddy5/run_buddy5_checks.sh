#!/bin/sh
# Focused Buddy 5 RTOS candidate host checks (logic only; no hardware).
# usage: sh testing/rtos_migration/buddy5/run_buddy5_checks.sh <bench_tools/buddy5_sdk> [out_dir]
# v0.2.12 long-width candidate: SDK width_long_probe v0.2 suites and decoder;
# straight-gate revision: gate unit test and the gate mission scenarios.
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
MSRC="testing/rtos_migration/test_b5_mission.c app_program/obstacle_navigation/b5_mission.c \
  app_program/obstacle_navigation/b5_straight.c $E/width_probe.c \
  $E/width_estimate.c $E/edge_probe.c $E/automatic_bypass.c $E/manual_bypass.c $E/rect_route.c $E/stripe_detect.c \
  $E/profile/obstacle_profile.c $CORE"
# APP profile: no SDK run-log copy (B5WP_RUN_ARCHIVE=0); the archive-on build
# must produce byte-identical telemetry (the copy is write-only).
$CC $STEER -DAPP_RTOS_BUDDY5=1 -DB5WP_RUN_ARCHIVE=0 -Wall -Wextra -I . -I $B -I app_program/obstacle_navigation $MSRC -lm -o "$OUT/test_b5_mission"
$CC $STEER -DAPP_RTOS_BUDDY5=1 -DB5WP_RUN_ARCHIVE=1 -Wall -Wextra -I . -I $B -I app_program/obstacle_navigation $MSRC -lm -o "$OUT/test_b5_mission_arch"
run "$OUT/test_b5_mission" "$OUT/mission_frames.txt"
run "$OUT/test_b5_mission_arch" "$OUT/mission_frames_archive_on.txt" > "$OUT/mission_archive_on.log"
run cmp "$OUT/mission_frames.txt" "$OUT/mission_frames_archive_on.txt"
# 1c. straight-only takeover gate with the real Buddy3 bend monitor
$CC -Wall -Wextra -DCONTINUOUS_BEND_ASSIST=1 -DCIRCUIT_SWEEP_SEARCH=1 -DCIRCUIT_STATIONARY_SEARCH=1 -DLINE_CAL_MIN_SPAN=50 \
  -I $B -I app_program/obstacle_navigation testing/rtos_migration/test_b5_straight_gate.c \
  app_program/obstacle_navigation/b5_straight.c -lm -o "$OUT/test_b5_straight_gate"
run "$OUT/test_b5_straight_gate"
# 1a. offline decoder: every mission run, replayed by the unchanged SDK v0.2 decoder
mkdir -p "$OUT/mission_runs"
python3 -B - "$OUT/mission_frames.txt" "$OUT/mission_runs" <<'PY'
import re, sys
out, n = None, 0
for line in open(sys.argv[1]):
    if line.startswith('SCENARIO'):
        n += 1
        out = open(f"{sys.argv[2]}/{n:02d}_{re.sub(r'[^A-Za-z0-9]+', '_', line[9:].strip()).strip('_')}.txt", 'w')
    elif out:
        out.write(line)
PY
for f in "$OUT"/mission_runs/*.txt; do
  run python3 -B tools/telemetry/b5_rtos_decode.py "$f" --sdk-width-decoder "$SDK/width_long_probe_v0_2/decode_width_long.py" \
    --json "${f%.txt}.json" > "${f%.txt}.decoded"
done
grep -h "WIDTH " "$OUT"/mission_runs/*.decoded | sed 's/:.*//' | sort | uniq -c
run python3 -B tools/telemetry/b5_rtos_decode.py "$ROOT/dashboard/test/fixtures/b5_rtos_mission_c_frames_v0_2_legacy.txt" > "$OUT/legacy_fixture.decoded"
# 1b. NAV_STEER coordinator extension (codec, gating, bound, 100 ms lapse)
$CC $STEER -Wall -Wextra -I $B testing/rtos_migration/test_nav_steer.c $CORE -lm -o "$OUT/test_nav_steer"
run "$OUT/test_nav_steer"
# 2. SDK width_long_probe v0.2 suites against the ported (byte-identical) width
#    files and the v0.2.12 RTOS motion core. The SDK USB/flash archive helper is
#    not part of the RTOS build; its header-only builder is staged into the test
#    include path only because test_width_long_v02.c exercises it.
L=$SDK/width_long_probe_v0_2
W=$L/source/firmware/integration/buddy1_4/tests/width_only
mkdir -p "$OUT/w/tests/width_only" "$OUT/w/extensions/buddy5_obstacles"
cp "$W"/*.c "$W"/*.h "$OUT/w/tests/width_only/"; cp "$W"/*.c "$W"/*.h "$OUT/w/"
cp "$L/source/firmware/integration/buddy1_4/extensions/buddy5_obstacles/width_probe_log.h" "$OUT/w/extensions/buddy5_obstacles/"
for f in width_estimate.c width_estimate.h width_probe_pack.h; do
  run cmp "$L/source/firmware/integration/buddy1_4/extensions/buddy5_obstacles/$f" "$E/$f"
done
# width_probe.c/.h: compare the SDK path with the RTOS-only retry hook removed.
for f in width_probe.c width_probe.h; do
  run python3 -B - "$L/source/firmware/integration/buddy1_4/extensions/buddy5_obstacles/$f" "$E/$f" <<'PY'
import difflib, re, sys
a, b = (open(x).read().splitlines() for x in sys.argv[1:3])
block = r'^#ifdef APP_RTOS_BUDDY5\n.*?^#endif\n'
def sdk_branch(match):
    lines = match[0].splitlines(keepends=True)
    return ''.join(lines[lines.index('#else\n') + 1:-1]) if '#else\n' in lines else ''
b = re.sub(block, sdk_branch, '\n'.join(b) + '\n', flags=re.M | re.S).splitlines()
ok = {'#if B5WP_RUN_ARCHIVE', '#endif', '#define B5WP_RUN_ARCHIVE      1'}
bad = [l for l in difflib.ndiff(a, b) if l[:2] in ('- ', '+ ') and not (l.startswith('+ ') and (
       l[2:].strip() in ok or l[2:].startswith('#ifndef B5WP_RUN_ARCHIVE') or l[2:].lstrip().startswith(('0 = no copy', 'needs the RAM'))))]
print(sys.argv[2], 'archive guard only' if not bad else bad)
sys.exit(1 if bad else 0)
PY
done
printf '#include "src/motion/motion_port.h"\ntypedef motion_feedback_t MotionFeedback;\ntypedef motion_output_t MotionOutput;\ntypedef motion_port_t MotionPort;\ntypedef motion_event_t MotionEvent;\n' > "$OUT/w/sdk_names.h"
WCC="$CC -Wall -Wextra -Wno-missing-field-initializers -DLINE_CAL_MIN_SPAN=50 -DB5WP_HOST_TEST_HOOKS"
$WCC -I "$OUT/w" -I $B "$OUT/w/test_width_estimate_near_axis.c" $E/width_estimate.c -lm -o "$OUT/w/estimator"
(cd "$OUT/w" && run ./estimator)
$WCC -I "$OUT/w" -I $B -include "$OUT/w/sdk_names.h" "$OUT/w/test_width_long_v02.c" $E/width_probe.c $E/width_estimate.c \
  $E/profile/obstacle_profile.c $CORE -lm -o "$OUT/w/probe"
(cd "$OUT/w" && run ./probe "$OUT/w/sim_rtos_frames.jsonl")
# 2b. the unchanged SDK v0.2 decoder replays the RTOS-built simulation frames
#     (independent rule-2 estimator, clearance policy, D5 limits)
run python3 -B "$L/decode_width_long.py" "$OUT/w/sim_rtos_frames.jsonl" --json "$OUT/w/sim_rtos_decoded.json"
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
