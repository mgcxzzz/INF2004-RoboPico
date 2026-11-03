import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("export_line_log", ROOT / "tools/rtos_migration/export_line_log.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class ExportTests(unittest.TestCase):
    def capture(self, n=2, version=2):
        rows = ["LINE_LOG_BEGIN version=%d rows=%d overwritten=5" % (version, n),
                "LINE_LOG_CAL white=163,160 black=3457,3700 origin=1"]
        if version == 2:
            rows.append("LINE_LOG_POLICY name=angle_only departure_deg_gt=10 departure_us=30000 branch_us=30000")
        columns = module.COLUMNS if version == 2 else module.COLUMNS_V1
        for j in range(n):
            row = dict.fromkeys(module.COLUMNS, 0)
            row.update(sequence=j + 6, ms=100 + j * 10, adc_l=4000, adc_side=220,
                       filtered_l=3999, level_l=1000, dt_us=10000, cost_us=2000)
            rows.append("LINE_ROW " + ",".join(str(row[k]) for k in columns))
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
        del lines[4]
        self.assertEqual(module.parse_capture(lines)[1]["result"], "INCOMPLETE")

    def test_bad_column(self):
        lines = self.capture()
        lines[3] += ",99"
        self.assertEqual(module.parse_capture(lines)[1]["result"], "INCOMPLETE")

    def test_invalid_range(self):
        lines = self.capture()
        lines[3] = lines[3].replace(",4000,", ",5000,")
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

    def test_old_firmware_export(self):
        rows, info = module.parse_capture(self.capture(version=1))
        self.assertEqual(info["result"], "COMPLETE")
        self.assertNotIn("junction_prior_white", rows[0])

    def test_missing_policy(self):
        lines = self.capture()
        del lines[2]
        self.assertEqual(module.parse_capture(lines)[1]["result"], "INCOMPLETE")

    def test_junction_acceptance(self):
        lines = self.capture()
        fields = dict.fromkeys(module.COLUMNS, 0)
        fields.update(sequence=6, ms=100, turn_degrees_x10=780, action=70,
                      junction_branch_accepted=1, junction_accept_event=1,
                      junction_accepted_without_white=1)
        lines[3] = "LINE_ROW " + ",".join(str(fields[k]) for k in module.COLUMNS)
        info = module.parse_capture(lines)[1]
        self.assertEqual(info["result"], "COMPLETE")
        self.assertEqual(info["junction_acceptances"][0]["turn_degrees"], 78)
        self.assertEqual(info["junction_acceptances"][0]["accepted_without_white"], 1)

    def test_invalid_junction_flag(self):
        lines = self.capture()
        cells = lines[3][len("LINE_ROW "):].split(",")
        cells[-1] = "2"
        lines[3] = "LINE_ROW " + ",".join(cells)
        self.assertEqual(module.parse_capture(lines)[1]["result"], "INCOMPLETE")


if __name__ == "__main__":
    unittest.main()
