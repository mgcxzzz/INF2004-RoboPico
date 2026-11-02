import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("export_line_log", ROOT / "tools/rtos_migration/export_line_log.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ExportTests(unittest.TestCase):
    def capture(self, n=2):
        rows = ["LINE_LOG_BEGIN version=1 rows=%d overwritten=5" % n,
                "LINE_LOG_CAL white=163,160 black=3457,3700 origin=1"]
        for j in range(n):
            row = dict.fromkeys(module.COLUMNS, 0)
            row.update(sequence=j + 6, ms=100 + j * 10, adc_l=4000, adc_side=220,
                       filtered_l=3999, level_l=1000, dt_us=10000, cost_us=2000)
            rows.append("LINE_ROW " + ",".join(str(row[k]) for k in module.COLUMNS))
        rows.append("LINE_LOG_END rows=%d" % n)
        return rows

    def test_complete(self):
        rows, info = module.parse_capture(self.capture())
        self.assertEqual(info["result"], "COMPLETE")
        self.assertEqual(len(rows), 2)
        self.assertEqual(info["overwritten"], 5)

    def test_missing_end(self):
        self.assertEqual(module.parse_capture(self.capture()[:-1])[1]["result"], "INCOMPLETE")

    def test_missing_row(self):
        lines = self.capture(3)
        del lines[3]
        self.assertEqual(module.parse_capture(lines)[1]["result"], "INCOMPLETE")

    def test_bad_column(self):
        lines = self.capture()
        lines[2] += ",99"
        self.assertEqual(module.parse_capture(lines)[1]["result"], "INCOMPLETE")

    def test_invalid_range(self):
        lines = self.capture()
        lines[2] = lines[2].replace(",4000,", ",5000,")
        self.assertEqual(module.parse_capture(lines)[1]["result"], "INCOMPLETE")

    def test_repeated_dump(self):
        self.assertEqual(module.parse_capture(self.capture() * 2)[1]["result"], "INCOMPLETE")

    def test_status_interleaving(self):
        lines = self.capture()
        lines.insert(3, "CLOCK kernel_ms=100 status=PASS")
        self.assertEqual(module.parse_capture(lines)[1]["result"], "COMPLETE")

    def test_empty_history(self):
        self.assertEqual(module.parse_capture(self.capture(0))[1]["result"], "COMPLETE")

    def test_no_dump(self):
        self.assertEqual(module.parse_capture(["LOG rejected: stop before d."])[1]["result"], "INCOMPLETE")


if __name__ == "__main__":
    unittest.main()
