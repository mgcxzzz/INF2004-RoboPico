"""Package the combined RTOS image and its matching dashboard. No car access."""

import hashlib
import json
import shutil
import subprocess
import sys
import zipfile

import package_motor_bringup as common
from package_line_following import linked_checks
from package_release import collect_sources

ROOT = common.ROOT
ART = ROOT / "firmware/integration/buddy1_4/artifacts/v0_2_13_combined"
NAME = "robopico_buddy1_5_rtos_v0_2_13"
GUIDE = ROOT / "docs/buddy1_5_v0_2_13.md"
PROFILE = "rtos_buddy1_5"
ARM = ROOT.parent / ".buddy2-tools/arm-gcc/bin"
SDK = ROOT.parent / ".buddy2-tools/pico-sdk"


def bundle_inputs():
    tracked = {ROOT / path for path in common.command("git", "ls-files").splitlines()}
    paths = set(collect_sources())
    for folder in ("firmware/integration/buddy1_4/extensions/buddy5_obstacles",
                   "tools/coding_standard", "tools/provisioning", "tools/telemetry",
                   "firmware/windows", ".github"):
        paths.update(path for path in (ROOT / folder).rglob("*") if path.is_file())
    paths.add(ROOT / "CODING_STANDARD.md")
    return sorted(paths & tracked)


def main():
    assert not common.command("git", "status", "--porcelain", "--untracked-files=no"), \
        "Commit the reviewed source before packaging."
    for args in (("check.py",), ("format.py", "--check")):
        subprocess.run([sys.executable, ROOT / "tools/coding_standard" / args[0], *args[1:]],
                       check=True)

    built = ROOT / "build_make" / f"{NAME}.uf2"
    elf = built.with_suffix(".elf")
    layout = common.validate_uf2(built.read_bytes())
    linked = linked_checks(elf, network=True, barcode=True, imu=True, release=True, buddy5=True)
    pool = common.command(sys.executable, ROOT / "testing/rtos_migration/buddy5/kernel_pool_budget.py",
                          elf, PROFILE, "--nm", ARM / "arm-none-eabi-nm.exe")
    print(pool)

    paths = bundle_inputs()
    hashes = {path.relative_to(ROOT).as_posix(): common.digest(path) for path in paths}
    ART.mkdir(parents=True, exist_ok=True)
    firmware = ART / built.name
    shutil.copyfile(built, firmware)
    manifest = {
        "name": NAME,
        "version": "0.2.13",
        "status": "Integration candidate, hardware verification pending",
        "source_commit": common.command("git", "rev-parse", "HEAD"),
        "buddy1_4_baseline": "81a994f57e3828364492cc2806fcbea124dfa18a",
        "buddy5_review": "89e7fc69e47d895029ca6d5151ab5ac4f4073c24",
        "build_profile": f"APP={PROFILE} SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0",
        "compiler": common.command(ARM / "arm-none-eabi-gcc.exe", "--version").splitlines()[0],
        "sdk_commit": common.command("git", "-C", SDK, "rev-parse", "HEAD"),
        "uf2_sha256": common.digest(firmware),
        "uf2_bytes": firmware.stat().st_size,
        "uf2_layout": layout,
        "elf_sha256": common.digest(elf),
        "linked_checks": linked,
        "kernel_pool_estimate": pool.splitlines()[-1],
        "standard": "RoboPico C Standard 1.0 with frozen SDK algorithm exception CEX-08",
    }
    for filename, value in (("BUILD_MANIFEST.json", manifest), ("SOURCE_SHA256.json", hashes)):
        (ART / filename).write_text(json.dumps(value, indent=2) + "\n", encoding="utf-8", newline="\n")

    bundle = ART / f"{NAME}_bundle.zip"
    firmware_name = f"firmware/{firmware.name}"
    with zipfile.ZipFile(bundle, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in paths:
            archive.write(path, path.relative_to(ROOT).as_posix())
        for filename in ("BUILD_MANIFEST.json", "SOURCE_SHA256.json"):
            archive.write(ART / filename, filename)
        archive.write(firmware, firmware_name)
        archive.write(GUIDE, "START_HERE.md")
    with zipfile.ZipFile(bundle) as archive:
        assert archive.testzip() is None
        assert archive.read(firmware_name) == firmware.read_bytes()
        for name, digest in hashes.items():
            assert hashlib.sha256(archive.read(name)).hexdigest() == digest, name
        assert not any(n.endswith("wifi_secrets.h") or ".local." in n for n in archive.namelist())

    checksums = "".join(f"{common.digest(p)}  {p.name}\n" for p in (firmware, bundle))
    (ART / "SHA256SUMS.txt").write_text(checksums, encoding="utf-8", newline="\n")
    print(checksums, end="")


if __name__ == "__main__":
    main()
