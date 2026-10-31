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
sources = [CORE / "src/navigation" / (n + ".c") for n in ("motion_api", "nav_coordinator", "nav_codec")]
sources.append(CORE / "src/motion/motion_executor.c")

checks = [(ROOT / "testing/rtos_migration/test_bringup_policy.c", [])]
for name, extra in [("test_continuous_drive", []), ("test_interface", sources),
                    ("test_clock", sources), ("test_coordinator_assist", sources)]:
    checks.append((CORE / "tests/unit" / (name + ".c"), extra))
for source, extra in checks:
    exe = OUT / (source.stem + (".exe" if os.name == "nt" else ""))
    subprocess.run([*cc, "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-UNDEBUG",
                    "-I" + str(ROOT), "-I" + str(CORE), "-I" + str(CORE / "config"),
                    source, *extra, "-lm", "-o", exe], check=True)
    subprocess.run([exe], check=True)
subprocess.run([sys.executable, ROOT / "testing/rtos_migration/test_capture_bringup.py"], check=True)
print("PASS: 5 production-C regression programs and 10 capture-analyzer checks. Hardware test is still required.")
