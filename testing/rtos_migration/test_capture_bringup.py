import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("capture", ROOT / "tools/rtos_migration/capture_bringup.py")
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)


def sample_rows(kernel_rate=1.0, timer_rate=1.0, hz=100, offset=0):
    rows = []
    for second in range(31):
        rows.append({"kind": "CLOCK", "wall_s": second, "kernel_ms": int(second * 1000 * kernel_rate),
                     "timer_us": (offset + int(second * 1000000 * timer_rate)) & 0xffffffff, "status": "PASS"})
        rows.append({"kind": "CONTROL", "wall_s": second, "count": second * hz,
                     "late": 0, "tx_drop": 0, "rx_drop": 0, "active": 0})
    return rows


class CaptureTest(unittest.TestCase):
    def test_parse(self):
        kind, row = capture.parse_line("CONTROL count=42 flags=0xc late=0")
        self.assertEqual((kind, row), ("CONTROL", {"count": 42, "flags": 12, "late": 0}))

    def test_normal(self):
        self.assertEqual(capture.analyze(sample_rows())["result"], "PASS")

    def test_slow_kernel_and_timer(self):
        self.assertEqual(capture.analyze(sample_rows(1 / 42.7, 1 / 42.7, 2))["result"], "FAIL")

    def test_disagreement(self):
        self.assertEqual(capture.analyze(sample_rows(1, 1.525))["result"], "FAIL")

    def test_wrap(self):
        self.assertEqual(capture.analyze(sample_rows(offset=0xffff0000))["result"], "PASS")

    def test_inconclusive(self):
        self.assertEqual(capture.analyze([])["result"], "INCONCLUSIVE")
        self.assertEqual(capture.analyze(sample_rows()[:10])["result"], "INCONCLUSIVE")

    def test_late_and_dropped(self):
        rows = sample_rows()
        rows[-1]["late"] = 1
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")
        rows[-1]["late"] = 0
        rows[-1]["tx_drop"] = 20
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")

    def test_internal_clock_not_passed(self):
        rows = sample_rows()
        rows[-2]["status"] = "CHECK"
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")

    def test_active_motor_run(self):
        rows = sample_rows()
        rows[-1]["active"] = 1
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")

    def test_duplicate_query(self):
        rows = sample_rows()
        rows.insert(20, dict(rows[18], wall_s=9.5))
        self.assertEqual(capture.analyze(rows)["result"], "PASS")


if __name__ == "__main__":
    unittest.main()
