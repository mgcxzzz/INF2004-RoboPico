"""Offline migration checks; no robot or serial port is opened."""
from pathlib import Path
import os
import shlex
import subprocess
import sys
import argparse

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
          (ROOT / "testing/rtos_migration/test_imu_i2c.c", []),
          (ROOT / "testing/rtos_migration/test_usb_printf.c", []),
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
    (ROOT / "testing/rtos_migration/test_encoder_fault.c", []),
    (ROOT / "testing/rtos_migration/test_network_stop.c", []),
    (ROOT / "testing/rtos_migration/test_barcode_attempt.c", sources + [CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_release_imu.c", [CORE / "tests/imu/fake_drv.c", CORE / "src/imu/imu_service.c", CORE / "src/imu/imu_proc.c", CORE / "src/imu/b4_pack.c", CORE / "src/telemetry/b2_pack.c"]),
    (ROOT / "testing/rtos_migration/test_release_policy.c", []),
    (ROOT / "testing/rtos_migration/test_release_exports.c", sources + [CORE / "src/network/wifi_packets.c", CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_release_bridge.c", sources + [CORE / "src/network/wifi_protocol.c", CORE / "src/network/wifi_packets.c"]),
    (ROOT / "testing/rtos_migration/test_hump_odo_grace.c", []),
    (ROOT / "testing/rtos_migration/test_imu_calibration_diagnostics.c", [CORE / "tests/imu/fake_drv.c", CORE / "src/imu/imu_service.c", CORE / "src/imu/imu_proc.c", CORE / "src/imu/b4_pack.c", CORE / "src/telemetry/b2_pack.c"]),
    (ROOT / "testing/rtos_migration/test_imu_task_reset.c", [CORE / "tests/imu/fake_drv.c", CORE / "src/imu/imu_service.c", CORE / "src/imu/imu_proc.c", CORE / "src/imu/b4_pack.c", CORE / "src/telemetry/b2_pack.c"]),
    (ROOT / "testing/rtos_migration/test_rtos_ramp_bridge.c", sources + [CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_rtos_ramp.c", sources + [CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_imu_service_rtos.c", [CORE / "tests/imu/fake_drv.c", CORE / "src/imu/imu_service.c", CORE / "src/imu/imu_proc.c", CORE / "src/imu/b4_pack.c", CORE / "src/telemetry/b2_pack.c"]),
    (CORE / "tests/imu/test_powered_acceptance.c", []),
    (CORE / "tests/ramp/test_ramp_assist.c", []),
    (ROOT / "testing/rtos_migration/test_barcode_gate.c", sources + [CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_barcode_gate_reference.c", sources + [CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_junction_sensor.c", sources + [CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_rtos_barcode.c", sources + [CORE / "src/network/wifi_protocol.c"]),
    (ROOT / "testing/rtos_migration/test_reference_peak.c", [CORE / "src/network/wifi_protocol.c"]),
    (CORE / "tests/barcode/test_barcode_peak.c", []),
    (ROOT / "testing/rtos_migration/test_connected_bridge.c", sources + [CORE / "src/network/wifi_protocol.c", CORE / "src/network/wifi_packets.c"]),
    (ROOT / "testing/rtos_migration/test_network_bench.c", [CORE / "src/network/wifi_protocol.c"]),
    (CORE / "tests/unit/test_wifi_packets.c", [CORE / "src/network/wifi_packets.c"]),
    (CORE / "tests/unit/test_wifi_protocol.c", [CORE / "src/network/wifi_protocol.c"]),
    (CORE / "tests/unit/test_wifi_settings.c", [])])
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--only', nargs='+', help='Run only named production-C programs (file stems).')
parser.add_argument('--skip-python', action='store_true', help='Run selected Python checks separately.')
args = parser.parse_args()
if args.only:
    unknown = set(args.only) - {source.stem for source, _ in checks}
    if unknown:
        parser.error('Unknown programs: ' + ', '.join(sorted(unknown)))
    checks = [(source, extra) for source, extra in checks if source.stem in args.only]
for source, extra in checks:
    exe = OUT / (source.stem + (".exe" if os.name == "nt" else ""))
    profile_flags = ["-DAPP_RTOS_IMU=1"] if source.stem in ("test_release_imu", "test_hump_odo_grace", "test_imu_service_rtos", "test_imu_task_reset", "test_imu_calibration_diagnostics") else []
    if source.stem == "test_usb_printf":
        profile_flags += ["-I" + str(ROOT / "testing/rtos_migration/printf_stubs")]
    subprocess.run([*cc, *host_float, *profile_flags, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-UNDEBUG",
                    "-DCIRCUIT_SWEEP_SEARCH=1", "-I" + str(ROOT), "-I" + str(CORE), "-I" + str(CORE / "config"),
                    source, *extra, "-lm", "-o", exe], check=True)
    subprocess.run([exe], check=True, cwd=OUT if source.stem in ("test_imu_service_rtos", "test_ramp_assist") else ROOT)
if not args.skip_python:
    for name in ('test_capture_bringup', 'test_export_line_log', 'test_capture_adc_readiness',
                 'test_provision_network', 'test_capture_network_bench', 'test_capture_connected_line',
                 'test_export_barcode_log', 'test_tcp_allocation'):
        subprocess.run([sys.executable, ROOT / 'testing/rtos_migration' / (name + '.py')], check=True)
print(f"PASS: {len(checks)} production-C regression programs; Python checks {'run' if not args.skip_python else 'selected separately'}. Hardware test is still required.")
