"""Package the reviewed v0.2.8 wireless-calibration candidate; never access a robot."""
import hashlib
import json
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import package_motor_bringup as common
from package_line_following import linked_checks
from package_release import collect_sources

ROOT = common.ROOT
CORE = ROOT / "firmware/integration/buddy1_4"
ART = CORE / "artifacts/v0_2_8_wireless_calibration"
NAME = "robopico_buddy1_4_rtos_v0_2_8_wireless_calibration"
BASELINE = "7b18596675e6198837851305500f6541b81774b6"
common.ART = ART


def main():
    ART.mkdir(parents=True, exist_ok=True)
    assert not common.command("git", "status", "--porcelain", "--untracked-files=no"), \
        "Commit reviewed source before packaging; commit artifacts separately."
    subprocess.run([sys.executable, ROOT / "tools/coding_standard/check.py"], check=True)
    subprocess.run([sys.executable, ROOT / "tools/coding_standard/format.py", "--check"], check=True)
    built = ROOT / "build_make" / (NAME + ".uf2")
    elf = built.with_suffix(".elf")
    layout = common.validate_uf2(built.read_bytes())
    linked = linked_checks(elf, network=True, barcode=True, imu=True, release=True)
    assert int(linked["static_ram_end"], 16) <= 0x2002ff48, "Calibration exceeded its 256-byte static growth budget."
    tool = ROOT.parent / ".buddy2-tools/arm-gcc/bin/arm-none-eabi-nm.exe"
    names = {line.split()[-1] for line in common.command(tool, "-g", elf).splitlines() if line.split()}
    for name in ("ri_lift_guard_samples", "ri_safety_packet", "ri_safety_reset_latch",
                 "ri_safety_peak_packet", "rtos_line_drive_mode_info", "ri_ramp_step",
                 "rtos_line_cal_command", "rtos_line_cal_info"):
        assert name in names, "Required tested feature missing: " + name
    paths = set(collect_sources())
    paths.add(ROOT / "CODING_STANDARD.md")
    for folder in ("tools/coding_standard", ".github"):
        paths.update(p for p in (ROOT / folder).rglob("*") if p.is_file()
                     and "__pycache__" not in p.parts and p.suffix != ".pyc")
    paths = sorted(paths)
    hashes = {p.relative_to(ROOT).as_posix(): common.digest(p) for p in paths}
    firmware = ART / built.name
    shutil.copyfile(built, firmware)
    baseline = json.loads((CORE / "artifacts/v0_2_6_rtos/BUILD_MANIFEST.json").read_text())
    manifest = {
        "name": NAME, "version": "0.2.8", "branch": common.command("git", "branch", "--show-current"),
        "status": "WIRELESS CALIBRATION CANDIDATE; hardware smoke test pending; Buddy5 NOT integrated",
        "source_commit": common.command("git", "rev-parse", "HEAD"),
        "tested_main_baseline": BASELINE, "build_profile": baseline["build_profile"],
        "compiler": common.command(tool.with_name("arm-none-eabi-gcc.exe"), "--version").splitlines()[0],
        "sdk_commit": baseline["sdk_commit"], "uf2_sha256": common.digest(firmware),
        "uf2_bytes": firmware.stat().st_size, "elf_sha256": common.digest(elf),
        "uf2_layout": layout, "source_files": len(paths),
        "change_boundary": ["Adapted Barr policy, braces/formatting, types/variables and declarations",
                            "Module contracts/guards, explicit logical grouping and equality style",
                            "Unsigned compact storage with explicit signed interpretation in seven native types",
                            "Stopped-only staged wireless front IR calibration and CC/28 readback",
                            "Verified explicit persistence through existing flash adapter; no automatic retuning"],
        "preserved": ["micro T-Kernel SMP topology/priorities, deadlines, locks and copied communication",
                      "45 default, stopped-only 60/75 modes, both barcode decoders, 7mm junction gate",
                      "Bend/recovery, ramp assistance/acceptance, pickup latch and explicit eligible Reset",
                      "Powerbank operation, network supervision, optional captures and stopped wireless exports",
                      "Existing wire schemas preserved; new CC/28 status and CONFIG key3; saved formats unchanged"],
        "calibration_priority": baseline["calibration_priority"],
        "expected_tested_car_calibration": baseline["expected_tested_car_calibration"],
        "native_layouts": "Seven baseline size/alignment assertions compiled on host and Arm; no width expansion",
        "static_ram_delta_vs_v026": int(linked["static_ram_end"], 16) - 0x2002fe48,
        "linked_checks": linked,
        "software_validation": "docs/wireless_calibration_v0_2_8.md records targeted checks and limitations",
        "hardware_validation": "Pending docs/wireless_calibration_v0_2_8.md smoke test; main not promoted",
        "standard": "RoboPico C Standard 1.0, based on Barr-C:2018 with documented exceptions",
        "compliance_scope": "Mechanical gates and targeted semantic review; not exhaustive Barr/MISRA certification"}
    for name, value in (("BUILD_MANIFEST.json", manifest), ("SOURCE_SHA256.json", hashes)):
        (ART / name).write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8", newline="\n")
    bundle = ART / (NAME + "_bundle.zip")
    with zipfile.ZipFile(bundle, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in paths:
            archive.write(path, path.relative_to(ROOT).as_posix())
        for path in (firmware, ART / "BUILD_MANIFEST.json", ART / "SOURCE_SHA256.json"):
            archive.write(path, path.relative_to(ROOT).as_posix())
        archive.write(firmware, "firmware/" + firmware.name)
        guide = (ROOT / "docs/wireless_calibration_v0_2_8.md").read_text(encoding="utf-8")
        archive.writestr("START_HERE.md", guide)
        archive.writestr("TEST_CHANGES.md", guide)
        for title, script, arguments in (
            ("Start dashboard.cmd", "tools/dashboard/run_dashboard.py", ""),
            ("Setup Wi-Fi.cmd", "tools/rtos_migration/provision_network.py", ""),
            ("Capture wireless test.cmd", "tools/rtos_migration/capture_wireless.py", ""),
            ("Capture connected test.cmd", "tools/rtos_migration/capture_connected_line.py", " --network-supervised"),
            ("Export line log.cmd", "tools/rtos_migration/export_line_log.py", ""),
            ("Export barcode log.cmd", "tools/rtos_migration/export_barcode_log.py", "")):
            archive.writestr(title, '@echo off\r\ncd /d "%~dp0"\r\npy -3 '
                             + script.replace("/", "\\") + arguments + "\r\npause\r\n")
    with zipfile.ZipFile(bundle) as archive:
        assert archive.testzip() is None
        assert archive.read("firmware/" + firmware.name) == firmware.read_bytes()
        for name, expected in hashes.items():
            assert hashlib.sha256(archive.read(name)).hexdigest() == expected, name
        assert not any(n.endswith("wifi_secrets.h") or ".local." in n for n in archive.namelist())
    (ART / "SHA256SUMS.txt").write_text("".join(f"{common.digest(p)}  {p.name}\n"
        for p in (firmware, bundle)), encoding="utf-8", newline="\n")
    print(json.dumps({"uf2": str(firmware), "bundle": str(bundle),
                      "bundle_bytes": bundle.stat().st_size, "uf2_sha256": common.digest(firmware),
                      "static_ram_delta_vs_v026": manifest["static_ram_delta_vs_v026"]}, indent=2))


if __name__ == "__main__":
    main()
