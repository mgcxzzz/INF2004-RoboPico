"""Package the provisional SDK integration; never flashes or provisions a board."""

from pathlib import Path
import argparse, hashlib, json, os, re, shutil, struct, subprocess, zipfile

REPO = Path(__file__).resolve().parents[2]
PROJECT = REPO / "firmware/integration/buddy1_4"
NAME = "robopico_buddy1_4_integration_v0_1"
EXCLUDE = {"build-host", "__pycache__", "dashboard_session", ".git", "node_modules"}


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def run(*args):
    return subprocess.check_output(
        list(map(str, args)), text=True, encoding="utf-8", errors="replace"
    )


def source_files():
    for folder in [
        PROJECT,
        REPO / "dashboard",
        REPO / "tools/dashboard",
        REPO / "tools/provisioning",
        REPO / "tools/telemetry",
        REPO / "tools/firmware",
        REPO / "docs",
        REPO / "testing/evidence",
    ]:
        for p in sorted(folder.rglob("*")):
            if not p.is_file() or any(
                x in EXCLUDE or x.startswith("build-") for x in p.relative_to(REPO).parts
            ):
                continue
            if PROJECT / "artifacts" in p.parents and p.name != "README.md":
                continue
            if p.name in ["wifi_config.local.h", "wifi_settings.local.json"] or p.suffix in [
                ".pyc",
                ".exe",
                ".o",
                ".obj",
            ]:
                continue
            yield p
    for n in [
        "README.md",
        "BUNDLE_README.md",
        "Start dashboard.cmd",
        "Configure WiFi.cmd",
        "Run offline tests.cmd",
        ".gitignore",
        ".gitattributes",
        ".clang-format",
    ]:
        p = REPO / n
        if p.is_file():
            yield p


def main():
    a = argparse.ArgumentParser(description=__doc__)
    a.add_argument("--build-dir", type=Path, required=True)
    a.add_argument("--nm", default=shutil.which("arm-none-eabi-nm"))
    a.add_argument("--verify-only", action="store_true")
    args = a.parse_args()
    out = PROJECT / "artifacts"
    out.mkdir(exist_ok=True)
    if args.verify_only:
        m = json.loads((out / "BUILD_MANIFEST.json").read_text())
        verify(out, m)
        return
    build = args.build_dir.resolve()
    binary = build / (NAME + ".bin")
    elf = build / (NAME + ".elf")
    if not args.nm:
        raise SystemExit("Provide --nm <arm-none-eabi-nm path> or add the ARM toolchain to PATH.")
    assert binary.is_file() and elf.is_file(), "Build the Release ELF and BIN first."
    assert not (
        PROJECT / "config/wifi_config.local.h"
    ).exists(), "Public release must not contain a local credentials header."
    ninja = (build / "build.ninja").read_text(encoding="utf-8")
    for flag in [
        "BUDDY_WIFI_LOCAL_CONFIG=0",
        "BUDDY_WIFI_ENABLED=1",
        "RAMP_ASSIST_ACTIVE=1",
        "BARCODE_PEAK_ACTIVE=1",
        "BUDDY_COORDINATED_BARCODE=1",
    ]:
        assert flag in ninja, flag
    data = binary.read_bytes()
    assert data and 0x10000000 + len(data) < 0x101AA000, "Firmware overlaps settings/log sectors."
    for forbidden in [b"replace-with-wifi-password", b"replace-with-laptop-hotspot-ssid"]:
        assert forbidden not in data, "Example credentials were embedded."
    symbols = {}
    for line in run(args.nm, "-n", "--defined-only", elf).splitlines():
        cells = line.split()
        if len(cells) == 3:
            try:
                symbols[cells[2]] = int(cells[0], 16)
            except ValueError:
                pass
    needed = [
        "__StackTop",
        "__StackBottom",
        "__StackOneTop",
        "__StackOneBottom",
        "__bss_end__",
        "__HeapLimit",
        "__StackLimit",
        "__flash_binary_end",
    ]
    s = {n: symbols[n] for n in needed}
    assert (
        s["__StackTop"] - s["__StackBottom"] == 8192
        and s["__StackOneTop"] - s["__StackOneBottom"] == 8192
    )
    assert s["__bss_end__"] <= s["__HeapLimit"] == s["__StackLimit"] <= s["__StackOneBottom"]
    assert s["__StackOneTop"] <= s["__StackBottom"] < s["__StackTop"] <= 0x20040000
    assert s["__flash_binary_end"] < 0x101AA000
    assert (
        struct.unpack_from("<I", data, 256)[0] == s["__StackTop"]
    ), "Boot vector stack pointer differs."
    assert b"PROVISIONAL" in data and b"BUDDY5 NOT INTEGRATED" in data
    blocks = (len(data) + 255) // 256
    uf2 = bytearray()
    for i in range(blocks):
        b = bytearray(512)
        struct.pack_into(
            "<8I",
            b,
            0,
            0x0A324655,
            0x9E5D5157,
            0x2000,
            0x10000000 + 256 * i,
            256,
            i,
            blocks,
            0xE48BFF56,
        )
        b[32:288] = data[i * 256 : (i + 1) * 256].ljust(256, b"\0")
        struct.pack_into("<I", b, 508, 0x0AB16F30)
        uf2.extend(b)
    artifact = out / (NAME + ".uf2")
    artifact.write_bytes(uf2)
    source = {str(p.relative_to(REPO)).replace("\\", "/"): sha(p) for p in source_files()}
    (out / "SOURCE_SHA256.json").write_text(
        json.dumps(source, indent=2), encoding="utf-8", newline="\n"
    )
    frames = []
    for p in build.rglob("*.su"):
        for line in p.read_text(encoding="utf-8").splitlines():
            v = line.split("\t")
            if len(v) > 1:
                try:
                    frames.append({"function": v[0].split(":")[-1], "bytes": int(v[1])})
                except ValueError:
                    pass
    cache = (build / "CMakeCache.txt").read_text(encoding="utf-8")
    match = re.search(r"^PICO_SDK_PATH:[^=]*=(.*)$", cache, re.M)
    sdk_commit = (
        run("git", "-C", match.group(1).strip(), "rev-parse", "HEAD").strip()
        if match
        else "unknown"
    )
    assert sdk_commit == "98a542c1a62fb549ffb5d66a3e5892b06276b670"
    assert re.search(
        r"^BUDDY_EXTENSION_SOURCE:FILEPATH=\s*$", cache, re.M
    ), "Buddy5 extension must be empty for this package."
    m = {
        "name": NAME,
        "status": "PROVISIONAL integration; hardware acceptance pending",
        "implemented_buddies": [1, 2, 3, 4],
        "buddy5_integrated": False,
        "source_baseline": "ac161d3bd9e0a7361419747755e6834c4fc98744",
        "pico_sdk_commit": sdk_commit,
        "compiler": "ARM GNU 14.2.1",
        "modes": {
            "ramp_assist_active": True,
            "barcode_peak_active": True,
            "coordinated_physical_barcode": True,
        },
        "embedded_credentials": False,
        "calibration": {
            "record": 28,
            "builtin_white": [163, 160],
            "builtin_black": [3457, 3700],
            "saved_record28_preserved": True,
            "tilted_front_capture_is_default": False,
        },
        "artifact": artifact.name,
        "sha256": sha(artifact),
        "bytes": len(uf2),
        "blocks": blocks,
        "bin_bytes": len(data),
        "bin_sha256": sha(binary),
        "elf_sha256": sha(elf),
        "source_ledger_sha256": sha(out / "SOURCE_SHA256.json"),
        "symbols": s,
        "compiler_frame_sizes": frames,
        "stack_analysis_limit": "Frame measurements and layout checks are not a complete worst-case call-chain bound.",
        "bundle": NAME + "_bundle.zip",
        "bundle_scope": "SDK integration source, tests, dashboard, setup tools/docs, selected evidence and UF2; separate RTOS project excluded",
        "checks": {
            "host_tests": "testing/evidence/current/host_tests.txt",
            "hardware_tested_this_image": False,
        },
    }
    (out / "BUILD_MANIFEST.json").write_text(
        json.dumps(m, indent=2), encoding="utf-8", newline="\n"
    )
    with zipfile.ZipFile(out / m["bundle"], "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for p in source_files():
            z.write(p, NAME + "/" + str(p.relative_to(REPO)).replace("\\", "/"))
        for p in [artifact, out / "BUILD_MANIFEST.json", out / "SOURCE_SHA256.json"]:
            z.write(p, NAME + "/" + str(p.relative_to(REPO)).replace("\\", "/"))
    (out / "SHA256SUMS.txt").write_text(
        "\n".join(
            sha(out / n) + "  " + n
            for n in [artifact.name, m["bundle"], "BUILD_MANIFEST.json", "SOURCE_SHA256.json"]
        )
        + "\n",
        encoding="utf-8",
        newline="\n",
    )
    verify(out, m)


def verify(out, m):
    artifact = out / m["artifact"]
    data = artifact.read_bytes()
    assert sha(artifact) == m["sha256"]
    payload = bytearray()
    assert len(data) == m["blocks"] * 512 == m["bytes"]
    for i in range(m["blocks"]):
        b = data[i * 512 : (i + 1) * 512]
        fields = struct.unpack_from("<8I", b)
        assert fields == (
            0x0A324655,
            0x9E5D5157,
            0x2000,
            0x10000000 + i * 256,
            256,
            i,
            m["blocks"],
            0xE48BFF56,
        )
        assert fields[3] + fields[4] <= 0x101AA000
        assert struct.unpack_from("<I", b, 508)[0] == 0x0AB16F30
        payload.extend(b[32:288])
    assert hashlib.sha256(payload[: m["bin_bytes"]]).hexdigest() == m["bin_sha256"]
    assert sha(out / "SOURCE_SHA256.json") == m["source_ledger_sha256"]
    ledger = json.loads((out / "SOURCE_SHA256.json").read_text())
    for rel, expected in ledger.items():
        assert sha(REPO / rel) == expected, "Working source differs from package: " + rel
    with zipfile.ZipFile(out / m["bundle"]) as z:
        names = z.namelist()
        assert len(names) == len(set(names))
        for n in names:
            assert n.startswith(NAME + "/") and ".." not in Path(n).parts
            assert not any(x in EXCLUDE or x.startswith("build-") for x in Path(n).parts)
            assert Path(n).name not in ["wifi_config.local.h", "wifi_settings.local.json"]
        for rel, h in ledger.items():
            assert hashlib.sha256(z.read(NAME + "/" + rel)).hexdigest() == h, rel
        entry = NAME + "/firmware/integration/buddy1_4/artifacts/" + m["artifact"]
        assert hashlib.sha256(z.read(entry)).hexdigest() == m["sha256"]
    print(
        "PASS: RP2040 UF2 blocks/flash, stack/heap/vector, source ledger and complete bundle. PROVISIONAL; Buddy5 not integrated."
    )
    print("UF2:", artifact.name, len(data), "bytes", m["sha256"])


if __name__ == "__main__":
    main()
