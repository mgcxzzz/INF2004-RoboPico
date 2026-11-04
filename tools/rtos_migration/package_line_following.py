"""Build a reviewable Step 2 source/UF2 bundle. Does not access a robot."""
from pathlib import Path
import hashlib
import json
import re
import shutil
import zipfile

import package_motor_bringup as common

ROOT = common.ROOT
NAME = "robopico_rtos_line_following_v0_1_2"
CHECKPOINT = ROOT / "firmware/migration/micro_tkernel/step_02_line_following"
ART = CHECKPOINT / "firmware"
common.ART = ART


def linked_checks(elf: Path, network: bool = False, barcode: bool = False, imu: bool = False) -> dict:
    expected='rtos_imu_wifi' if imu else 'rtos_barcode_wifi' if barcode else 'rtos_line_wifi' if network else 'rtos_line'
    assert (ROOT/'build_make/.build_profile').read_text(encoding='utf-8').strip().startswith(expected+'-'), \
        'Rebuild the requested profile before packaging so stack-usage files match its ELF.'
    tool = ROOT.parent / ".buddy2-tools/arm-gcc/bin/arm-none-eabi-nm.exe"
    output = common.command(tool, "-n", elf)
    symbols = {}
    for line in output.splitlines():
        cells = line.split()
        if len(cells) == 3 and re.fullmatch(r"[0-9a-fA-F]+", cells[0]):
            symbols[cells[2]] = int(cells[0], 16)
    for name in ("usermain", "bringup_control_task", "bringup_safety_task", "bringup_nav_task",
                 "bringup_console_task", "bringup_status_task", "rtos_line_flash_task",
                 "rtos_line_follow", "rtos_line_power", "rtos_line_init",
                 "motion_submit_navigation", "motion_commit",
                 "rl_flash_park", "rl_flash_write"):
        assert name in symbols, name
    if not barcode:
        assert 'motion_submit_barcode' in symbols
    if network:
        for name in ("cyw43_utk_task", "mqtt_client_connect", "rn_app_task", "rn_drive_process", "rn_drive_telemetry"):
            assert name in symbols, name
    if barcode:
        for name in ('motion_accept_barcode','bp_window','rtos_line_side_poll'):
            assert name in symbols,name
    if imu:
        for name in ('ri_task','ri_ramp_step','b4v23_tick','b4_drv_read_fifo'):
            assert name in symbols, name
    for name in ((() if network else ("cyw43_utk_task", "mqtt_client_connect")) + ("subsystem5_start", "dev_init_adc",
                 "dev_init_ser") + (() if barcode else ("barcode_analog_sample", "barcode_peak_sample"))):
        assert name not in symbols, name
    for name in ("rl_flash_park", "rl_flash_write"):
        assert 0x20000000 <= symbols[name] < 0x20040000, name
    assembly = common.command(tool.with_name("arm-none-eabi-objdump.exe"), "-d", "-j", ".data", elf)
    # RAM parking makes no calls. Writer only calls prepared RAM/ROM pointers;
    # a direct BL/BLX target in XIP would fetch code while flash is unavailable.
    for line in assembly.splitlines():
        assert not re.search(r"\bblx?\s+10[0-9a-f]{6}\b", line), line
    assert symbols["__end"] <= 0x20030000, "Insufficient RAM left for kernel/task allocations"
    usage = {}
    for directory in (ROOT / "build_make/mtkernel_3/app_program/rtos_bringup",
                      ROOT / "build_make/mtkernel_3/app_program/line_navigation",
                      *([ROOT / "build_make/mtkernel_3/app_program/network_bridge"] if network else []),
                      *([ROOT / "build_make/mtkernel_3/app_program/imu_navigation"] if imu else [])):
        for path in directory.glob("*.su"):
            for line in path.read_text(encoding="utf-8").splitlines():
                label, size, kind = line.rsplit("\t", 2)
                usage[label.split(":")[-1]] = {"bytes": int(size), "kind": kind}
                assert int(size) < 2048, f"Large application stack frame: {line}"
    assert "bringup_control_task" in usage and "rtos_line_dump_if_requested" in usage
    return {"flash_ram_addresses": {name: hex(symbols[name]) for name in ("rl_flash_park", "rl_flash_write")},
            "static_ram_end": hex(symbols["__end"]),
            "ram_remaining_before_dynamic_kernel_stacks": 0x20040000 - symbols["__end"],
            "application_stack_frames": usage,
            "note": "Static stack frames are not a measured call-chain high-water mark; hardware checks remain required."}


def main():
    ART.mkdir(parents=True, exist_ok=True)
    firmware = ROOT / "build_make" / (NAME + ".uf2")
    elf = ROOT / "build_make" / (NAME + ".elf")
    layout = common.validate_uf2(firmware.read_bytes())
    linked = linked_checks(elf)
    copied = ART / firmware.name
    shutil.copyfile(firmware, copied)
    paths = sorted(set(common.sources()))
    hashes = {p.relative_to(ROOT).as_posix(): common.digest(p) for p in paths}
    preserved, reviewed = common.verify_shared_sources(hashes)
    manifest = {
        "name": NAME, "status": "EXPERIMENTAL Step2; hardware acceptance pending; not final Buddy1-5",
        "branch": common.command("git", "branch", "--show-current"),
        "source_commit": common.command("git", "rev-parse", "HEAD"),
        "reference_main_commit": common.BASE,
        "build_profile": "APP=rtos_line SMP=1 CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 CRASH_REPORT=0 TEST=0",
        "compiler": common.command(ROOT.parent / ".buddy2-tools/arm-gcc/bin/arm-none-eabi-gcc.exe", "--version").splitlines()[0],
        "sdk_commit": common.command("git", "-C", ROOT.parent / ".buddy2-tools/pico-sdk", "rev-parse", "HEAD"),
        "uf2_sha256": common.digest(copied), "uf2_bytes": copied.stat().st_size,
        "elf_sha256": common.digest(elf), "uf2_layout": layout,
        "source_file_count": len(paths), "unchanged_shared_files_verified": len(preserved),
        "reviewed_shared_changes": reviewed,
        "junction_policy": "JUNCTION_ANGLE_GATE_ONLY=1: >10deg for30ms; no prior white prerequisite; existing branch30ms/windows/acquisition/faults retained.",
        "calibration_priority": ["CRC-valid own RTOS calibration at 0x1a9000", "CRC-valid SDK record28 at 0x1d0000",
                                 "preset white163,160 black3457,3700"],
        "live_features": ["profile1 Continuous V2 steering/drive", "IR tracking/recovery", "USB-queued physical-cross junction turns",
                          "shared navigation API/coordinator", "calibration import/capture/save", "stopped RAM diagnostic export"],
        "not_active": ["Wi-Fi/MQTT/dashboard", "physical barcode decoders", "IMU/ramp assistance", "Buddy5"],
        "software_checks": ["15 strict production-C regression executables", "38 Python analyzer/export tests",
                            "Step1 and Step2 ARM builds", "source parity, UF2 bounds, RAM flash-code and stack-frame checks"],
        "hardware_checks": "Step1 accepted; Step2 v0.1.1 ADC/initial straight/stops user-tested; old white prerequisite blocked junction tests. v0.1.2 angle-only experiment NOT physically tested.",
        "adc_correction": "Recheck READY at original100us deadline; classify and retain first/last failure; stopped-only100ms reset/retry and3 fresh all-channel qualification frames; runtime failure latches STOP; no auto-resume.",
        "linked_checks": linked,
    }
    source_hashes = ART / "SOURCE_SHA256.json"
    source_hashes.write_text(json.dumps(hashes, indent=2) + "\n", encoding="utf-8")
    manifest_path = ART / "BUILD_MANIFEST.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    bundle = ART / (NAME + "_bundle.zip")
    with zipfile.ZipFile(bundle, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in paths:
            archive.write(path, path.relative_to(ROOT).as_posix())
        for path in (copied, source_hashes, manifest_path):
            archive.write(path, path.relative_to(ROOT).as_posix())
        archive.write(copied, "firmware/" + copied.name)
        archive.writestr("START_HERE.md", "# Step 2 v0.1.2 - angle-only junction experiment\n\n"
                         "Flash firmware/" + NAME + ".uf2.\n\n"
                         "Follow [the exact junction tests and log export steps](TEST_JUNCTIONS.md).\n\n"
                         "The complete [Step 2 setup and driving guide](firmware/migration/"
                         "micro_tkernel/step_02_line_following/README.md) is also included.\n")
        archive.writestr("TEST_ADC_FIRST.md", (CHECKPOINT / "UPDATE_V0_1_1.md").read_text(encoding="utf-8"))
        archive.writestr("TEST_JUNCTIONS.md", (CHECKPOINT / "UPDATE_V0_1_2.md").read_text(encoding="utf-8"))
        for name, script in (("Run timing check.cmd", "capture_bringup.py"),
                             ("Check ADC readiness.cmd", "capture_adc_readiness.py"),
                             ("Export line log.cmd", "export_line_log.py")):
            archive.writestr(name, '@echo off\r\ncd /d "%~dp0"\r\npy -3 tools\\rtos_migration\\' + script + '\r\npause\r\n')
    with zipfile.ZipFile(bundle) as archive:
        assert archive.testzip() is None
        for relative, expected in hashes.items():
            assert hashlib.sha256(archive.read(relative)).hexdigest() == expected
        assert archive.read("firmware/" + copied.name) == copied.read_bytes()
    (ART / "SHA256SUMS.txt").write_text("".join(f"{common.digest(p)}  {p.name}\n" for p in (copied, bundle)), encoding="utf-8")
    print(json.dumps({"uf2": str(copied), "bundle": str(bundle), "uf2_sha256": common.digest(copied),
                      "bundle_sha256": common.digest(bundle), "bundle_bytes": bundle.stat().st_size,
                      "source_files": len(paths), "unchanged_shared_files": len(preserved), **layout,
                      "ram_remaining_before_dynamic_kernel_stacks": linked["ram_remaining_before_dynamic_kernel_stacks"]}, indent=2))


if __name__ == "__main__":
    main()
