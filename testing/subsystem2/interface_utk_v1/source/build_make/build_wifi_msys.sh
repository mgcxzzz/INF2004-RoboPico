#!/usr/bin/env bash
# Build the isolated Pico W UTK image with the workspace's pinned Windows tools.
# Invoke from Git Bash: ./build_wifi_msys.sh
set -euo pipefail

BUILD_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
TOOL_ROOT="$(cd -- "$BUILD_DIR/../../../../../../.buddy2-tools" && pwd -P)"
SDK_PATH="$(cygpath -ms "$TOOL_ROOT/pico-sdk")"
ARM_BIN="$(cygpath -u "$TOOL_ROOT/arm-gcc/bin")"
MAKE_BIN="$(cygpath -u "$TOOL_ROOT/make-msys/usr/bin")"
MAKE_EXE="$TOOL_ROOT/make-msys/usr/bin/make.exe"

if [[ ! -f "$SDK_PATH/pico_sdk_init.cmake" ]]; then
    echo "Pico SDK not found at $SDK_PATH" >&2
    exit 2
fi
if [[ ! -x "$TOOL_ROOT/arm-gcc/bin/arm-none-eabi-gcc.exe" || ! -x "$MAKE_EXE" ]]; then
    echo "Workspace Arm GNU Toolchain or MSYS make is missing below $TOOL_ROOT" >&2
    exit 2
fi

export PICO_SDK_PATH="$SDK_PATH"
export PATH="$ARM_BIN:$MAKE_BIN:/usr/bin:$PATH"
cd "$BUILD_DIR"
exec "$MAKE_EXE" -j8 \
    WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 "$@"
