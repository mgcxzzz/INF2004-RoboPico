#!/usr/bin/env python3
"""Validate and package the verified UTK Drive V1 firmware release.

Run from any directory after the owner has confirmed the final firmware build:
    python release.py

The packager deliberately refuses stale build metrics, malformed UF2 images,
source drift from the recorded calibration/steering baselines, or firmware
that reaches the reserved WCF1 settings sector.
"""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import os
import re
import shutil
import struct
import subprocess
import sys
import zipfile
from pathlib import Path


ROOT = Path(__file__).resolve().parent
REPO = Path(subprocess.check_output(
    ["git", "-C", str(ROOT), "rev-parse", "--show-toplevel"], text=True
).strip())
REL_ROOT = ROOT.relative_to(REPO).as_posix()
PROFILE_NAME = "mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt"
PROFILE_UF2 = ROOT / "source" / "build_make" / f"{PROFILE_NAME}.uf2"
PROFILE_ELF = ROOT / "source" / "build_make" / f"{PROFILE_NAME}.elf"
PROFILE_MAP = ROOT / "source" / "build_make" / f"{PROFILE_NAME}.map"
PROFILE_FILE = ROOT / "source" / "build_make" / ".build_profile"
FIRMWARE_DIR = ROOT / "firmware"
GENERIC_UF2 = FIRMWARE_DIR / "buddy123_utk_drive_v1.uf2"
BUNDLE_ZIP = FIRMWARE_DIR / "buddy123_utk_drive_v1_bundle.zip"
SUMS_FILE = FIRMWARE_DIR / "SHA256SUMS.txt"
SOURCE_MANIFEST = ROOT / "SOURCE_SHA256.json"
METRICS_FILE = ROOT / "BUILD_METRICS.json"

XIP_BASE = 0x10000000
SETTINGS_OFFSET = 0x101AA000
RP2040_SRAM_START = 0x20000000
RP2040_SRAM_END = 0x20042000
UF2_MAGIC0 = 0x0A324655
UF2_MAGIC1 = 0x9E5D5157
UF2_MAGIC_END = 0x0AB16F30
UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
RP2040_FAMILY_ID = 0xE48BFF56
EXPECTED_SIZE_METRICS = {"text": 475528, "data": 0, "bss": 225108}
SOURCE_ORIGINS = {
    "kernel_wifi_service": "12f2f66b16ffc2b9c68b624629ce0859772b38bc",
    "buddy1_upstream": "b09403074939f45cf4d7c11b7606148f6c2a4e73",
    "wifi_v2_upstream": "10d7886b1ef9c5f5df681467fdf788566edeab6e",
}
PARITY_HEADERS = (
    "continuous_drive.h",
    "continuous_follow.h",
    "continuous_steering.h",
)
RAM_FLASH_SYMBOLS = (
    "utk_flash_service",
    "utk_flash_end",
    "flash_range_erase",
    "flash_range_program",
)
SKIP_DIRS = {
    ".git", ".pytest_cache", ".mypy_cache", ".ruff_cache", "__pycache__",
    "build", "build-unpacked", "build-debug", "node_modules", ".venv", "venv",
}
SKIP_SUFFIXES = {
    ".o", ".d", ".exe", ".elf", ".map", ".bin", ".hex", ".uf2", ".zip",
    ".pyc", ".pdb", ".log", ".tmp", ".bak",
}


def fail(message: str) -> None:
    raise SystemExit(f"release check failed: {message}")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def run_checked(command: list[str], *, cwd: Path | None = None) -> str:
    try:
        return subprocess.check_output(command, cwd=cwd, text=True, stderr=subprocess.STDOUT)
    except (OSError, subprocess.CalledProcessError) as error:
        output = getattr(error, "output", "")
        fail(f"command failed ({Path(command[0]).name}): {output.strip() or error}")


def locate_tool(name: str) -> str:
    tool_dir = REPO.parent / ".buddy2-tools" / "arm-gcc" / "bin"
    windows_name = f"{name}.exe"
    candidate = tool_dir / windows_name
    if candidate.is_file():
        return str(candidate)
    candidate = tool_dir / name
    if candidate.is_file():
        return str(candidate)
    found = shutil.which(windows_name) or shutil.which(name)
    if found:
        return found
    fail(f"cannot find {name}; expected it below {tool_dir} or on PATH")


def verify_origins() -> dict[str, str]:
    names = {
        "kernel_wifi_service": "micro T-Kernel kernel/Wi-Fi service snapshot",
        "buddy1_upstream": "reviewed Buddy 1 protocol, mapping, and dashboard snapshot",
        "wifi_v2_upstream": "Wi-Fi V2 packet and dashboard baseline",
    }
    for label, commit in SOURCE_ORIGINS.items():
        run_checked(["git", "-C", str(REPO), "cat-file", "-e", f"{commit}^{{commit}}"])
    return {label: f"{names[label]}: {commit}" for label, commit in SOURCE_ORIGINS.items()}


def list_release_sources() -> list[Path]:
    result = subprocess.run(
        ["git", "-C", str(REPO), "ls-files", "--cached", "--others", "--exclude-standard", "-z", "--", REL_ROOT],
        check=True, stdout=subprocess.PIPE,
    )
    prefix = REL_ROOT.rstrip("/") + "/"
    candidates = sorted({os.fsdecode(item) for item in result.stdout.split(b"\0") if item})
    selected: list[Path] = []
    for repo_rel in candidates:
        if not repo_rel.startswith(prefix):
            continue
        rel = Path(repo_rel[len(prefix):])
        if not rel.parts or rel.parts[0] == "firmware":
            continue
        if any(part in SKIP_DIRS or part.startswith("build-") for part in rel.parts[:-1]):
            continue
        if rel.suffix.lower() in SKIP_SUFFIXES:
            continue
        if rel.name in {"SOURCE_SHA256.json", "BUILD_METRICS.json"}:
            continue
        if rel.name in {"wifi_secrets.h", ".env", ".env.local"}:
            fail(f"refusing to package secret/config file: {rel.as_posix()}")
        path = ROOT / rel
        if not path.is_file():
            fail(f"source list contains a missing file: {rel.as_posix()}")
        selected.append(path)
    if not selected:
        fail("git source enumeration returned no package files")
    return selected


def source_manifest(paths: list[Path]) -> dict:
    files = {
        path.relative_to(ROOT).as_posix(): sha256(path)
        for path in paths
    }
    return {
        "release": "interface_utk_drive_v1",
        "files": dict(sorted(files.items())),
    }


def verify_calibration() -> dict:
    preset = ROOT / "source/app_program/subsystem2/controller/successful_calibration.h"
    selftest = ROOT / "test_successful_calibration.c"
    main = ROOT / "source/app_program/subsystem2/controller/v12_2/baseline/testing/subsystem2/circuit_with_barcode/main.c"
    preset_text = preset.read_text(encoding="utf-8")
    test_text = selftest.read_text(encoding="utf-8")
    main_text = main.read_text(encoding="utf-8")
    for literal in ("white={163,160}", "black={3457,3700}"):
        if literal not in preset_text:
            fail(f"calibration preset does not contain {literal}")
    for literal in ("good.white[0]==163", "good.white[1]==160", "good.black[0]==3457", "good.black[1]==3700", "chosen,27,true", "chosen,27,false"):
        if literal not in test_text:
            fail(f"successful-calibration test is missing {literal}")
    for literal in ("recorded_version=27", "ch=='w' || ch=='l' || ch=='r'", "ch=='k'", "w/l/r/k remain available"):
        if literal not in main_text:
            fail(f"mutable calibration/log workflow is missing {literal}")
    return {
        "preset_adc": {"white_left": 163, "white_right": 160, "black_left": 3457, "black_right": 3700},
        "saved_log_record_version": 27,
        "mutable_controls": ["w", "l", "r", "k", "c"],
        "startup_behavior": "valid saved calibration overrides preset; otherwise use preset",
        "verification": "test_successful_calibration.c checks preset values, record 27 override and restore behavior",
    }


def verify_steering() -> dict:
    parity_path = ROOT / "CONTINUOUS_V2_PARITY.json"
    parity = json.loads(parity_path.read_text(encoding="utf-8"))
    baseline_dir = ROOT.parent / "interface_continuous_v2"
    verified = {}
    for name in PARITY_HEADERS:
        entry = parity.get(name)
        current = ROOT / "source/app_program/subsystem2/controller" / name
        baseline = baseline_dir / name
        if not entry or entry.get("matches_continuous_v2") is not True or not current.is_file():
            fail(f"missing Continuous V2 parity record or controller header: {name}")
        current_hash = sha256(current)
        if current_hash != entry.get("sha256"):
            fail(f"controller header changed from recorded parity: {name}")
        if baseline.is_file() and sha256(baseline) != current_hash:
            fail(f"controller header no longer matches original Continuous V2: {name}")
        verified[name] = current_hash
    return {"byte_identical_headers": verified, "baseline": "interface_continuous_v2"}


def verify_uf2(path: Path) -> dict:
    if not path.is_file():
        fail(f"firmware UF2 is missing: {path}")
    raw = path.read_bytes()
    if len(raw) == 0 or len(raw) % 512 != 0:
        fail("UF2 length must be a non-zero multiple of 512 bytes")
    blocks = len(raw) // 512
    ranges = []
    image = bytearray()
    for index in range(blocks):
        block = raw[index * 512:(index + 1) * 512]
        magic0, magic1, flags, target, payload_size, block_no, block_count, family = struct.unpack_from("<8I", block)
        magic_end, = struct.unpack_from("<I", block, 508)
        if (magic0, magic1, magic_end) != (UF2_MAGIC0, UF2_MAGIC1, UF2_MAGIC_END):
            fail(f"invalid UF2 magic in block {index}")
        if block_no != index or block_count != blocks:
            fail(f"UF2 block index/count mismatch in block {index}")
        if flags & UF2_FLAG_FAMILY_ID_PRESENT == 0 or family != RP2040_FAMILY_ID:
            fail(f"block {index} is not tagged for RP2040")
        if payload_size == 0 or payload_size > 476:
            fail(f"invalid UF2 payload length in block {index}")
        end = target + payload_size
        if target < XIP_BASE or end > SETTINGS_OFFSET:
            fail(f"UF2 block {index} reaches outside firmware flash or overlaps WCF1 settings at 0x{SETTINGS_OFFSET:08x}")
        ranges.append((target, end))
        image.extend(block[32:32 + payload_size])
    max_end = max(end for _, end in ranges)
    banner = b"BUDDY123 UTK DRIVE V1"
    if banner not in image:
        fail("UF2 does not contain the expected UTK Drive V1 firmware banner")
    return {"bytes": len(raw), "blocks": blocks, "max_flash_end": f"0x{max_end:08x}", "settings_sector_start": f"0x{SETTINGS_OFFSET:08x}"}


def parse_nm(nm_output: str) -> dict[str, int]:
    symbols: dict[str, int] = {}
    for line in nm_output.splitlines():
        match = re.match(r"^([0-9a-fA-F]+)\s+\S\s+(\S+)$", line.strip())
        if match:
            symbols[match.group(2)] = int(match.group(1), 16)
    return symbols


def verify_build() -> dict:
    if not PROFILE_ELF.is_file() or not PROFILE_MAP.is_file() or not PROFILE_FILE.is_file():
        fail("expected ELF, linker map, or .build_profile output is missing")
    profile = PROFILE_FILE.read_text(encoding="ascii").strip()
    if profile != "smp1-usb_cdc-wificyw43-netlwip-mqtt1-crash0-test0":
        fail(f"unexpected build profile {profile!r}")

    size_tool = locate_tool("arm-none-eabi-size")
    nm_tool = locate_tool("arm-none-eabi-nm")
    gcc_tool = locate_tool("arm-none-eabi-gcc")
    size_output = run_checked([size_tool, str(PROFILE_ELF)])
    size_lines = [line.split() for line in size_output.splitlines() if line.split() and line.split()[0].isdigit()]
    if not size_lines or len(size_lines[-1]) < 3:
        fail(f"could not parse ARM size output: {size_output.strip()}")
    metrics = {"text": int(size_lines[-1][0]), "data": int(size_lines[-1][1]), "bss": int(size_lines[-1][2])}
    if metrics != EXPECTED_SIZE_METRICS:
        fail(f"built section sizes {metrics} do not match root-verified build metrics {EXPECTED_SIZE_METRICS}")

    nm_output = run_checked([nm_tool, "-n", str(PROFILE_ELF)])
    symbols = parse_nm(nm_output)
    required = [*RAM_FLASH_SYMBOLS, "__flash_binary_end"]
    missing = [name for name in required if name not in symbols]
    if missing:
        fail(f"ELF is missing required symbols: {', '.join(missing)}")
    for name in RAM_FLASH_SYMBOLS:
        if not (RP2040_SRAM_START <= symbols[name] < RP2040_SRAM_END):
            fail(f"flash-critical function {name} is not in RP2040 SRAM: 0x{symbols[name]:08x}")
    if symbols["__flash_binary_end"] >= SETTINGS_OFFSET:
        fail("ELF image end overlaps the WCF1 settings sector")

    compiler = run_checked([gcc_tool, "--version"]).splitlines()[0]
    return {
        "profile": profile,
        "compiler": compiler,
        "elf": PROFILE_ELF.relative_to(ROOT).as_posix(),
        "size_bytes": metrics,
        "ram_flash_symbols": {name: f"0x{symbols[name]:08x}" for name in RAM_FLASH_SYMBOLS},
        "__flash_binary_end": f"0x{symbols['__flash_binary_end']:08x}",
        "settings_sector_start": f"0x{SETTINGS_OFFSET:08x}",
    }


def write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8", newline="\n")


def make_bundle(source_files: list[Path]) -> None:
    FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(BUNDLE_ZIP, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for path in source_files:
            archive.write(path, (ROOT.name + "/" + path.relative_to(ROOT).as_posix()))
        for path in (SOURCE_MANIFEST, METRICS_FILE):
            archive.write(path, ROOT.name + "/" + path.name)
        archive.write(GENERIC_UF2, ROOT.name + "/firmware/" + GENERIC_UF2.name)
        inside_sums = f"{sha256(GENERIC_UF2)}  {GENERIC_UF2.name}\n"
        archive.writestr(ROOT.name + "/firmware/SHA256SUMS.txt", inside_sums)
    outside_sums = "\n".join(
        f"{sha256(path)}  {path.name}" for path in (GENERIC_UF2, BUNDLE_ZIP)
    ) + "\n"
    SUMS_FILE.write_text(outside_sums, encoding="ascii", newline="\n")

    with zipfile.ZipFile(BUNDLE_ZIP, "r") as archive:
        bad = archive.testzip()
        if bad:
            fail(f"ZIP CRC verification failed for {bad}")
        manifest = json.loads(SOURCE_MANIFEST.read_text(encoding="utf-8"))
        source_entries = {
            ROOT.name + "/" + rel: digest
            for rel, digest in manifest["files"].items()
        }
        expected = {
            ROOT.name + "/firmware/" + GENERIC_UF2.name,
            ROOT.name + "/" + SOURCE_MANIFEST.name,
            ROOT.name + "/" + METRICS_FILE.name,
            *source_entries.keys(),
        }
        entries = set(archive.namelist())
        missing = expected.difference(entries)
        if missing:
            fail(f"bundle is missing required entries (first 8): {sorted(missing)[:8]}")
        for name, digest in source_entries.items():
            if hashlib.sha256(archive.read(name)).hexdigest() != digest:
                fail(f"source file hash mismatch inside bundle: {name}")
        bundled_uf2 = archive.read(ROOT.name + "/firmware/" + GENERIC_UF2.name)
        if hashlib.sha256(bundled_uf2).hexdigest() != sha256(GENERIC_UF2):
            fail("bundled UF2 does not match staged UF2")
        for path in (SOURCE_MANIFEST, METRICS_FILE):
            name = ROOT.name + "/" + path.name
            if archive.read(name) != path.read_bytes():
                fail(f"bundle metadata differs from release metadata: {path.name}")
    checksum_lines = SUMS_FILE.read_text(encoding="ascii").splitlines()
    checksums = {line.split(maxsplit=1)[1]: line.split(maxsplit=1)[0] for line in checksum_lines if len(line.split(maxsplit=1)) == 2}
    for path in (GENERIC_UF2, BUNDLE_ZIP):
        if checksums.get(path.name) != sha256(path):
            fail(f"SHA256SUMS mismatch for {path.name}")


def main() -> None:
    # The caller stages only the firmware image whose build has been approved.
    input_uf2 = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else PROFILE_UF2
    if len(sys.argv) > 2:
        fail("usage: python release.py [verified-profile.uf2]")
    origins = verify_origins()
    build = verify_build()
    uf2 = verify_uf2(input_uf2)
    calibration = verify_calibration()
    steering = verify_steering()
    source_files = list_release_sources()
    manifest = source_manifest(source_files)

    FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(input_uf2, GENERIC_UF2)
    write_json(SOURCE_MANIFEST, manifest)
    write_json(METRICS_FILE, {
        "artifact": "firmware/buddy123_utk_drive_v1.uf2",
        "bundle": "firmware/buddy123_utk_drive_v1_bundle.zip",
        "generated_utc": dt.datetime.now(dt.timezone.utc).replace(microsecond=0).isoformat(),
        "source_origins": origins,
        "build": build,
        "firmware": uf2,
        "calibration": calibration,
        "controller_preservation": steering,
        "validation_scope": {
            "wifi_network_bench": "reported by user",
            "physical_driving": "not validated",
            "embedded_credentials": False,
        },
        "source_manifest": SOURCE_MANIFEST.name,
        "source_file_count": len(manifest["files"]),
    })
    make_bundle(source_files)
    print(f"Packaged: {GENERIC_UF2.relative_to(ROOT)} ({uf2['bytes']} bytes, {uf2['blocks']} UF2 blocks)")
    print(f"Bundle:   {BUNDLE_ZIP.relative_to(ROOT)} ({BUNDLE_ZIP.stat().st_size} bytes)")
    print(f"Hashes:   {SUMS_FILE.relative_to(ROOT)}")
    print(f"Sources:  {SOURCE_MANIFEST.relative_to(ROOT)} ({len(manifest['files'])} files)")
    print(f"Metrics:  {METRICS_FILE.relative_to(ROOT)}")


if __name__ == "__main__":
    main()
