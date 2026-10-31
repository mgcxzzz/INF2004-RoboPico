"""Package Step1 firmware/source; never reads a robot, credentials or flash."""
from pathlib import Path
import hashlib
import json
import shutil
import struct
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[2]
CHECKPOINT = ROOT / "firmware/migration/micro_tkernel/step_01_motor_bringup"
ART = CHECKPOINT / "firmware"
NAME = "robopico_rtos_motor_bringup_v0_1"
BASE = "7843f45fbe45d5a774bc18c93be9bb11084d1b64"
SHARED = ROOT / "firmware/integration/buddy1_4"
EXCLUDED = {"__pycache__", ".git", "build-host", "node_modules", "rtos_test_session"}


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def command(*args):
    return subprocess.check_output(list(map(str, args)), cwd=ROOT, text=True, encoding="utf-8").strip()


def validate_uf2(data):
    if not data or len(data) % 512:
        raise ValueError("Invalid UF2 length")
    blocks = len(data) // 512
    addresses = set()
    for number in range(blocks):
        offset = number * 512
        magic0, magic1, flags, address, size, index, total, family = struct.unpack_from("<8I", data, offset)
        assert (magic0, magic1, flags, size, index, total, family) == (
            0x0a324655, 0x9e5d5157, 0x2000, 256, number, blocks, 0xe48bff56)
        assert struct.unpack_from("<I", data, offset + 508)[0] == 0x0ab16f30
        assert 0x10000000 <= address and address + size <= 0x10100000
        assert address % 256 == 0 and address not in addresses
        addresses.add(address)
    assert 0x10000000 in addresses
    assert data[32:288] != b"\x00" * 256
    return {"blocks": blocks, "first_xip": hex(min(addresses)),
            "end_xip_exclusive": hex(max(addresses) + 256), "flash_ceiling": "0x10100000"}


def sources():
    directories = ["app_program", "kernel", "lib", "include", "config", "device", "etc", "build_make",
                   "tools/elf2uf2", "tools/rtos_migration", "testing/rtos_migration",
                   "firmware/integration/buddy1_4/src", "firmware/integration/buddy1_4/include",
                   "firmware/integration/buddy1_4/config", "firmware/integration/buddy1_4/tests/unit",
                   "firmware/migration/micro_tkernel", "docs"]
    for directory in directories:
        for path in sorted((ROOT / directory).rglob("*")):
            if not path.is_file() or ART in path.parents:
                continue
            relative = path.relative_to(ROOT)
            if any(part in EXCLUDED or part.startswith("build-") for part in relative.parts):
                continue
            if path.suffix.lower() in {".o", ".obj", ".d", ".elf", ".map", ".uf2", ".exe", ".pyc", ".zip"}:
                continue
            if path.name in {"wifi_secrets.h", ".build_profile"} or ".local." in path.name:
                continue
            yield path
    for name in ["README.md", "TEAM.md", ".clang-format", ".gitignore", ".gitattributes"]:
        yield ROOT / name


def main():
    ART.mkdir(parents=True, exist_ok=True)
    firmware = ROOT / "build_make" / (NAME + ".uf2")
    elf = ROOT / "build_make" / (NAME + ".elf")
    assert firmware.is_file() and elf.is_file()
    info = validate_uf2(firmware.read_bytes())
    copied = ART / firmware.name
    shutil.copyfile(firmware, copied)
    paths = sorted(set(sources()))
    hashes = {path.relative_to(ROOT).as_posix(): digest(path) for path in paths}
    # Shared algorithm and calibration sources must remain byte-identical to main.
    baseline_files = command("git", "ls-tree", "-r", "--name-only", BASE, "firmware/integration/buddy1_4").splitlines()
    preserved = []
    for relative in baseline_files:
        if any(relative.startswith("firmware/integration/buddy1_4/" + part + "/") for part in ("src", "include", "config")):
            original = subprocess.check_output(["git", "show", f"{BASE}:{relative}"], cwd=ROOT)
            assert hashlib.sha256(original).hexdigest() == hashes[relative], relative
            preserved.append(relative)
    source_hashes = ART / "SOURCE_SHA256.json"
    source_hashes.write_text(json.dumps(hashes, indent=2) + "\n", encoding="utf-8")
    tool = ROOT.parent / ".buddy2-tools/arm-gcc/bin/arm-none-eabi-gcc.exe"
    sdk = ROOT.parent / ".buddy2-tools/pico-sdk"
    symbols = command(tool.with_name("arm-none-eabi-nm.exe"), elf)
    defined = {line.split()[-1] for line in symbols.splitlines() if line.strip()}
    for name in ["usermain", "bringup_control_task", "bringup_safety_task", "bringup_nav_task",
                 "bringup_console_task", "bringup_status_task", "motion_submit_navigation", "motion_commit"]:
        assert name in defined, name
    for name in ["cyw43_utk_task", "mqtt_client_connect", "subsystem5_start", "dev_init_adc", "dev_init_ser", "flash_range_program"]:
        assert name not in defined, name
    manifest = {
        "name": NAME, "status": "EXPERIMENTAL; hardware acceptance pending; Step1 bench only",
        "branch": command("git", "branch", "--show-current"),
        "source_commit": command("git", "rev-parse", "HEAD"), "reference_main_commit": BASE,
        "build_profile": "APP=rtos_bringup SMP=1 CONSOLE=usb_cdc WIFI=none NET=none MQTT=0 CRASH_REPORT=0 TEST=0",
        "compiler": command(tool, "--version").splitlines()[0],
        "sdk_commit": command("git", "-C", sdk, "rev-parse", "HEAD"),
        "uf2_sha256": digest(copied), "uf2_bytes": copied.stat().st_size,
        "elf_sha256": digest(elf), "uf2_layout": info,
        "source_file_count": len(paths), "unchanged_shared_files_verified": len(preserved),
        "linked_symbol_check": "Five application tasks, coordinator API and commit present; Wi-Fi, sample ADC/UART, Buddy5 and flash programming absent.",
        "software_checks": ["5 strict production-C regression executables", "10 Python capture-analysis tests",
                            "Strict ARM build of new application", "UF2 block/family/bounds and source parity checks"],
        "hardware_checks": "Not run: laptop-vs-kernel/TIMER timing, wheels, encoders and emergency stop require the user test.",
        "calibration": "Not read or written in Step1; saved reference sectors outside this image remain untouched.",
    }
    manifest_path = ART / "BUILD_MANIFEST.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    bundle = ART / (NAME + "_bundle.zip")
    with zipfile.ZipFile(bundle, "w", zipfile.ZIP_DEFLATED, compresslevel=6) as archive:
        for path in paths:
            archive.write(path, path.relative_to(ROOT).as_posix())
        for path in (copied, source_hashes, manifest_path):
            archive.write(path, path.relative_to(ROOT).as_posix())
        archive.write(copied, "firmware/" + copied.name)
        archive.writestr("START_HERE.md", (CHECKPOINT / "README.md").read_text(encoding="utf-8"))
        archive.writestr("Run timing check.cmd", '@echo off\r\ncd /d "%~dp0"\r\npy -3 tools\\rtos_migration\\capture_bringup.py\r\npause\r\n')
    with zipfile.ZipFile(bundle) as archive:
        assert archive.testzip() is None
        for relative, expected in hashes.items():
            assert hashlib.sha256(archive.read(relative)).hexdigest() == expected
        assert archive.read(copied.relative_to(ROOT).as_posix()) == copied.read_bytes()
    (ART / "SHA256SUMS.txt").write_text("".join(f"{digest(p)}  {p.name}\n" for p in (copied, bundle)), encoding="utf-8")
    print(json.dumps({"uf2": str(copied), "bundle": str(bundle), "uf2_sha256": digest(copied),
                      "bundle_bytes": bundle.stat().st_size, "source_files": len(paths),
                      "unchanged_shared_files": len(preserved), **info}, indent=2))


if __name__ == "__main__":
    main()
