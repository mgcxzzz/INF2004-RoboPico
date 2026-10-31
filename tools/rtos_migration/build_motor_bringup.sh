#!/usr/bin/env bash
# Run from Git Bash. Paths with spaces are passed as arguments, never eval'd.
set -euo pipefail
REPO="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
TOOLS="$(cd -- "$REPO/../.buddy2-tools" && pwd -P)"
export PICO_SDK_PATH="$(cygpath -ms "$TOOLS/pico-sdk")"
export PATH="$(cygpath -u "$TOOLS/arm-gcc/bin"):$(cygpath -u "$TOOLS/make-msys/usr/bin"):/c/MinGW/bin:/usr/bin:$PATH"
cd "$REPO/build_make"
exec "$TOOLS/make-msys/usr/bin/make.exe" -j8 APP=rtos_bringup SMP=1 \
  CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 CRASH_REPORT=0 TEST=0 "$@"
