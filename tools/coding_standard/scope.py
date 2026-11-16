"""Explicit application ownership boundary; no dependency or artifact rewriting."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PREFIXES = (
    "app_program/rtos_bringup",
    "app_program/line_navigation",
    "app_program/barcode_navigation",
    "app_program/network_bridge",
    "app_program/imu_navigation",
    "firmware/integration/buddy1_4/src",
    "firmware/integration/buddy1_4/include",
    "firmware/integration/buddy1_4/config",
)


def source_files():
    return sorted(p for folder in PREFIXES for p in (ROOT / folder).rglob("*")
                  if p.is_file() and p.suffix in {".c", ".h", ".inc"}
                  and ".local." not in p.name)
