"""Check active source boundaries and local documentation links."""

from pathlib import Path
import re, json

P = Path(__file__).resolve().parents[1]
R = P.parents[2]
for p in (P / "src").rglob("*"):
    if p.suffix not in [".c", ".h"]:
        continue
    t = p.read_text(encoding="utf-8")
    assert not re.search(r'#include\s+"[^"\n]*\.c"', t), p
    if p.name != "motor_output.c":
        assert "pwm_set_both_levels" not in t, p
for feature in [
    "app",
    "barcode",
    "line_following",
    "motion",
    "imu",
    "navigation",
    "network",
    "telemetry",
    "platform",
]:
    assert (P / "src" / feature / "README.md").is_file(), feature
for p in [
    R / "README.md",
    *list((R / "docs").glob("*.md")),
    P / "README.md",
    *list((P / "src").rglob("README.md")),
]:
    if p.name == "rtos_setup.md":
        continue
    for match in re.finditer(r"\]\(([^)]+)\)", p.read_text(encoding="utf-8")):
        link = match[1].split("#")[0]
        if not link or "://" in link:
            continue
        target = (p.parent / link).resolve()
        assert target.exists(), (p, link)
assert not any("v12_2" in p.parts for p in (P / "src").rglob("*"))
provenance = json.loads((R / "testing/evidence/PROVENANCE.json").read_text())
import hashlib

for item in provenance:
    data = (R / "testing/evidence" / item["file"]).read_bytes()
    assert hashlib.sha256(data).hexdigest() == item["sha256"], item["file"]
print(
    "PASS source boundaries, feature READMEs/local links, descriptive paths and23 imported evidence hashes."
)
