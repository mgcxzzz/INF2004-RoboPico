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
                     "late": 0, "tx_drop": 0, "rx_drop": 0, "active": 0, "flags": 12, "usb": 3})
        rows.append({"kind": "WHEELS", "wall_s": second, "gp20": 0, "gp21": 0, "irq_stop": 0})
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
        rows[-2]["late"] = 1
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")
        rows[-2]["late"] = 0
        rows[-2]["tx_drop"] = 20
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")

    def test_internal_clock_not_passed(self):
        rows = sample_rows()
        rows[-3]["status"] = "CHECK"
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")

    def test_active_motor_run(self):
        rows = sample_rows()
        rows[-2]["active"] = 1
        self.assertEqual(capture.analyze(rows)["result"], "FAIL")

    def test_connected_clock_can_include_motion_without_hiding_fault(self):
        rows = sample_rows()
        rows[-2]["active"] = 1
        self.assertEqual(capture.analyze(rows, allow_active=True)["timing_result"], "PASS")
        rows[-2]["flags"] = 76
        result = capture.analyze(rows, allow_active=True)
        self.assertEqual(result["timing_result"], "PASS")
        self.assertEqual(result["motor_readiness"], "BLOCKED")
        self.assertTrue(any("encoder feedback/quality" in s for s in result["motor_blockers"]))

    def test_duplicate_query(self):
        rows = sample_rows()
        rows.insert(1, dict(rows[0], wall_s=0.5))
        self.assertEqual(capture.analyze(rows)["result"], "PASS")

    def test_historical_fault_blocks_despite_stable_timing(self):
        for flag in (2, 16, 32, 64):
            rows = sample_rows()
            for row in rows:
                if row["kind"] == "CONTROL":
                    row.update(flags=12 | flag, late=1)
            result = capture.analyze(rows)
            self.assertEqual(result["result"], "BLOCKED")
            self.assertEqual(result["timing_result"], "PASS")
            self.assertEqual(result["motor_readiness"], "BLOCKED")

    def test_old_gap_can_be_recorded_without_a_latch(self):
        rows = sample_rows()
        for row in rows:
            if row["kind"] == "CONTROL":
                row.update(late=1, max_dt_us=132256)
        self.assertEqual(capture.analyze(rows)["result"], "PASS")

    def test_missing_or_invalid_readiness_is_not_pass(self):
        for flags in (None, -1, 128):
            rows = sample_rows()
            rows[-2]["flags"] = flags
            self.assertEqual(capture.analyze(rows)["result"], "BLOCKED")
        rows = [r for r in sample_rows() if r["kind"] != "WHEELS"]
        self.assertEqual(capture.analyze(rows)["result"], "BLOCKED")

    def test_buttons_irq_and_usb(self):
        for key in ("gp20", "gp21", "irq_stop"):
            rows = sample_rows()
            rows[-1][key] = 1
            self.assertEqual(capture.analyze(rows)["result"], "BLOCKED")
        rows = sample_rows()
        rows[-2]["usb"] = 2
        self.assertEqual(capture.analyze(rows)["result"], "BLOCKED")

    def test_stale_readiness(self):
        rows = sample_rows()
        rows[-2]["wall_s"] -= 3
        self.assertEqual(capture.analyze(rows)["motor_readiness"], "BLOCKED")


if __name__ == "__main__":
    unittest.main()
