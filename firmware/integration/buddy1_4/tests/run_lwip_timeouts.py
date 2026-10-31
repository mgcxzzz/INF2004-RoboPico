"""Real SDK lwIP pool regression; offline and never opens the network."""

from pathlib import Path
import os, re, shlex, shutil, subprocess

P = Path(__file__).resolve().parents[1]
sdk = Path(os.environ.get("PICO_SDK_PATH", ""))
lwip = sdk / "lib/lwip/src"
if not (lwip / "core/timeouts.c").is_file():
    raise SystemExit("Set PICO_SDK_PATH to the SDK used for this build.")
cc = shlex.split(os.environ.get("CC", shutil.which("gcc") or "cc"))
flags = shlex.split(os.environ.get("CFLAGS", ""))
B = P / "build-host/lwip"
B.mkdir(parents=True, exist_ok=True)
default = B / "default"
default.mkdir(exist_ok=True)
(default / "lwipopts.h").write_text(
    re.sub(
        r"^#define MEMP_NUM_SYS_TIMEOUT[^\n]*\n",
        "",
        (P / "config/lwipopts.h").read_text(),
        flags=re.M,
    )
)
for name, config, args in [
    ("default", default, ["--expect-exhaustion"]),
    ("fixed", P / "config", []),
]:
    exe = B / (name + (".exe" if os.name == "nt" else ""))
    subprocess.run(
        [
            *cc,
            "-std=c11",
            "-Wall",
            "-Wextra",
            "-Werror",
            *flags,
            "-I" + str(P / "tests/lwip_test_port"),
            "-I" + str(config),
            "-I" + str(lwip / "include"),
            str(P / "tests/unit/test_lwip_timeouts.c"),
            str(lwip / "core/timeouts.c"),
            str(lwip / "core/memp.c"),
            "-o",
            str(exe),
        ],
        check=True,
    )
    subprocess.run([str(exe), *args], check=True)
print(
    "PASS real lwIP: original pool exhaustion reproduced; +4 capacity passes; no physical networking tested."
)
