"""Verify real C packet vectors against the current bundled JS decoder."""

from pathlib import Path
import os, shlex, shutil, subprocess

P = Path(__file__).resolve().parents[1]
B = P / "build-host"
B.mkdir(exist_ok=True)
exe = B / ("packet_vectors.exe" if os.name == "nt" else "packet_vectors")
cc = shlex.split(os.environ.get("CC", shutil.which("gcc") or "cc"))
flags = shlex.split(os.environ.get("CFLAGS", ""))
sources = [
    P / "tests/unit/wifi_vectors.c",
    P / "src/network/wifi_protocol.c",
    P / "src/network/wifi_packets.c",
]
subprocess.run(
    [
        *cc,
        "-std=c11",
        "-Wall",
        "-Wextra",
        "-Werror",
        *flags,
        "-I" + str(P),
        *map(str, sources),
        "-o",
        str(exe),
    ],
    check=True,
)
subprocess.run(
    [os.environ.get("NODE", "node"), str(P / "tests/unit/test_wifi_vectors.mjs"), str(exe)],
    check=True,
)
