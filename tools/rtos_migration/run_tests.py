"""Offline migration checks; no robot or serial port is opened."""
from pathlib import Path
import os
import shlex
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
CORE = ROOT / "firmware/integration/buddy1_4"
OUT = ROOT / "testing/rtos_migration/build-host"
OUT.mkdir(parents=True, exist_ok=True)
cc = shlex.split(os.environ.get("CC", "gcc"))
# The bundled 32-bit MinGW defaults to x87 extended constants. Use stored
# float32 semantics, like Arm soft-float, for the unchanged steering tests.
host_float = ["-msse2", "-mfpmath=sse"] if os.name == "nt" else []
sources = [CORE / "src/navigation" / (n + ".c") for n in ("motion_api", "nav_coordinator", "nav_codec")]
sources.append(CORE / "src/motion/motion_executor.c")

checks = [(ROOT / "testing/rtos_migration/test_bringup_policy.c", []),
          (ROOT / "testing/rtos_migration/test_encoder_quality.c", []),
          (ROOT / "testing/rtos_migration/test_mqtt_delivery.c", []),
          (ROOT / "testing/rtos_migration/test_line_model.c", []),
          (ROOT / "testing/rtos_migration/test_line_model_angle.c", []),
          (ROOT / "testing/rtos_migration/test_adc_reader.c", []),
          (ROOT / "testing/rtos_migration/test_junction_policy.c", []),
          (ROOT / "testing/rtos_migration/test_junction_reference.c", [])]
for name, extra in [("test_continuous_drive", []), ("test_interface", sources),
                    ("test_clock", sources), ("test_coordinator_assist", sources),
                    ("test_continuous_steering", []), ("test_continuous_follow", []),
                    ("test_continuous_integration", sources), ("test_line_search", []),
                    ("test_uturn", [])]:
    checks.append((CORE / "tests/unit" / (name + ".c"), extra))
checks.extend([
    (ROOT / "testing/rtos_migration/test_connected_bridge.c", sources + [CORE / "src/network/wifi_protocol.c", CORE / "src/network/wifi_packets.c"]),
    (ROOT / "testing/rtos_migration/test_network_bench.c", [CORE / "src/network/wifi_protocol.c"]),
    (CORE / "tests/unit/test_wifi_packets.c", [CORE / "src/network/wifi_packets.c"]),
    (CORE / "tests/unit/test_wifi_protocol.c", [CORE / "src/network/wifi_protocol.c"]),
    (CORE / "tests/unit/test_wifi_settings.c", [])])
for source, extra in checks:
    exe = OUT / (source.stem + (".exe" if os.name == "nt" else ""))
    subprocess.run([*cc, *host_float, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-UNDEBUG",
                    "-DCIRCUIT_SWEEP_SEARCH=1", "-I" + str(ROOT), "-I" + str(CORE), "-I" + str(CORE / "config"),
                    source, *extra, "-lm", "-o", exe], check=True)
    subprocess.run([exe], check=True)
subprocess.run([sys.executable, ROOT / "testing/rtos_migration/test_capture_bringup.py"], check=True)
subprocess.run([sys.executable, ROOT / "testing/rtos_migration/test_export_line_log.py"], check=True)
subprocess.run([sys.executable, ROOT / "testing/rtos_migration/test_capture_adc_readiness.py"], check=True)
subprocess.run([sys.executable, ROOT / "testing/rtos_migration/test_provision_network.py"], check=True)
subprocess.run([sys.executable, ROOT / "testing/rtos_migration/test_capture_network_bench.py"], check=True)
subprocess.run([sys.executable, ROOT / "testing/rtos_migration/test_capture_connected_line.py"], check=True)
print(f"PASS: {len(checks)} production-C regression programs plus capture/export/provisioning checks. Hardware test is still required.")
