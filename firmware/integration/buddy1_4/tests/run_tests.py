"""Portable offline regression suite. No robot, serial port, or broker access."""

from pathlib import Path
import os, subprocess, shlex, sys, json

P = Path(__file__).resolve().parents[1]
REPO = P.parents[2]
B = P / "build-host"
B.mkdir(exist_ok=True)
CC = shlex.split(os.environ.get("CC", "gcc"))
NODE = os.environ.get("NODE", "node")
FLAGS = shlex.split(os.environ.get("CFLAGS", ""))
core = [P / "src/navigation" / n for n in ["motion_api.c", "nav_coordinator.c", "nav_codec.c"]] + [
    P / "src/motion/motion_executor.c"
]
defs = ["LINE_CAL_MIN_SPAN=50", "CIRCUIT_SWEEP_SEARCH=1"]
count = 0


def call(cmd, cwd=P):
    subprocess.run(list(map(str, cmd)), cwd=cwd, check=True)


def run(name, extra=(), define=(), suffix=""):
    global count
    hits = list((P / "tests").rglob(name))
    assert len(hits) == 1, (name, hits)
    out = B / (Path(name).stem + suffix + (".exe" if os.name == "nt" else ""))
    call(
        [
            *CC,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-O0",
            "-UNDEBUG",
            *FLAGS,
            "-I" + str(P),
            "-I" + str(P / "config"),
            *["-D" + x for x in defs + list(define)],
            hits[0],
            *extra,
            "-lm",
            "-o",
            out,
        ]
    )
    call([out], B)
    count += 1


original = json.loads((P / "SOURCE_ORIGINS.json").read_text())["relocations"]
for old in sorted(original):
    if old.startswith("v12_2/") and Path(old).name.startswith("test_") and old.endswith(".c"):
        run(Path(old).name)
for name in ["test_interface.c", "test_clock.c"]:
    run(name, core)
    run(name, core, ["INTERFACE_UNPACKED_REFERENCE=1"], "_unpacked")
for name, extra in [
    ("test_storage.c", [P / "src/navigation/nav_codec.c"]),
    ("test_drive_diag.c", []),
    ("test_diag_parity.c", []),
    ("test_cruise_handover.c", []),
    ("test_continuous_drive.c", []),
    ("test_continuous_steering.c", []),
    ("test_continuous_follow.c", []),
    ("test_continuous_integration.c", core),
    ("test_buddy5.c", core + [P / "extensions/buddy5_obstacles/buddy5_extension.c"]),
    ("test_wifi_protocol.c", [P / "src/network/wifi_protocol.c"]),
    ("test_wifi_packets.c", [P / "src/network/wifi_packets.c"]),
    ("test_wifi_settings.c", []),
    ("test_wifi_link_state.c", []),
    ("test_wifi_storage.c", []),
    (
        "test_wifi_harness.c",
        core + [P / "src/network/wifi_protocol.c", P / "src/network/wifi_packets.c"],
    ),
    ("test_successful_calibration.c", []),
    ("test_wifi_routes.c", core + [P / "src/network/wifi_protocol.c"]),
    ("test_parity.c", core),
]:
    special = name in ["test_wifi_harness.c", "test_successful_calibration.c", "test_wifi_routes.c"]
    run(name, extra, ["BUDDY_RUNTIME_MAPPING=1", "BUDDY_WIFI_LOGGING=1"] if special else [])
# Include paths are compiler arguments, not macro definitions.
FLAGS += ["-I" + str(P / "tests/transport_test_stubs")]
run("test_wifi_transport.c")
FLAGS = FLAGS[:-1]
run("test_barcode_peak.c")
for active in [1, 0]:
    run(
        "test_peak_integration.c",
        [P / "src/network/wifi_protocol.c"],
        ["BUDDY_RUNTIME_MAPPING=1", "BARCODE_PEAK_ACTIVE=" + str(active)],
        "_" + str(active),
    )
run("test_ramp_assist.c")
for filename in ["test_coordinator_assist.c", "test_barcode_handover.c"]:
    if (P / "tests/unit" / filename).exists():
        run(filename, core + [P / "src/network/wifi_protocol.c"], ["BUDDY_RUNTIME_MAPPING=1"])
H = P / "tests/imu"
I = P / "src/imu"
FLAGS += ["-I" + str(H / "stubs"), "-I" + str(I)]
run(
    "test_buddy4_v23_service.c",
    [
        H / "fake_drv.c",
        I / "imu_service.c",
        I / "imu_proc.c",
        I / "b4_pack.c",
        P / "src/telemetry/b2_pack.c",
        P / "src/telemetry/sys_tlm.c",
    ],
    ["_DEFAULT_SOURCE"],
)
run("test_powered_acceptance.c", define=["_DEFAULT_SOURCE"])
FLAGS = FLAGS[:-2]
os.environ["PYTHONPATH"] = os.pathsep.join(
    [str(REPO / "tools/dashboard"), str(REPO / "tools/provisioning")]
)
call(
    [sys.executable, "-m", "unittest", "test_setup_helpers.py", "test_network_diagnostics.py"],
    P / "tests/unit",
)
call(
    [
        NODE,
        "--test",
        "test/test_source15.js",
        "test/test_ramp_targets.js",
        "test/test_merged_schema.js",
        "test/compat/v23_test_schema.js",
    ],
    REPO / "dashboard",
)
print("ALL OFFLINE CHECKS PASSED; C executables:", count)
