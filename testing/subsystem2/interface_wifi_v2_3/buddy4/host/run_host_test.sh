#!/bin/sh
# Host test of the v2.3 Buddy4 add-on. Usage: run_host_test.sh <out_dir> <path to is28_schema.js>
set -eu
H=$(cd "$(dirname "$0")" && pwd); O=${1:?out dir}; SCHEMA=${2:?is28_schema.js}
mkdir -p "$O"; cd "$O"
gcc -std=c11 -D_DEFAULT_SOURCE -Wall -Wextra -Werror -O2 -I"$H/stubs" -I"$H/.." -I"$H/../subsystem4" \
  -o test_buddy4_v23_service "$H/test_buddy4_v23_service.c" "$H/fake_drv.c" "$H/../buddy4_service.c" \
  "$H/../subsystem4/imu_proc.c" "$H/../subsystem4/b4_pack.c" "$H/../subsystem2/b2_pack.c" "$H/../system/sys_tlm.c" -lm
./test_buddy4_v23_service
node "$H/decode_frames.cjs" "$SCHEMA" buddy4_v23_frames.hex
