from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'tools/rtos_migration'))
import capture_adc_readiness as module


def fixture():
    rows = []
    for n in range(66):
        wall = float(n + 2)
        rows += [dict(kind='CLOCK', wall_s=wall, kernel_ms=10000 + n * 1000,
                      timer_us=10000000 + n * 1000000, status='PASS'),
                 dict(kind='CONTROL', wall_s=wall, count=1000 + n * 100, late=0,
                      flags=12, usb=3, active=0, tx_drop=0, rx_drop=0),
                 dict(kind='WHEELS', wall_s=wall, gp20=0, gp21=0, irq_stop=0),
                 dict(kind='LINE', wall_s=wall, flags=3, raw='122,138,180'),
                 dict(kind='ADC', wall_s=wall, state=2, good=3, frames=1000 + n * 100,
                      conversions=10000 + n * 9000, ch0=1000 + n * 100,
                      ch1=5000 + n * 500, ch2=1000 + n * 100,
                      timeouts=0, errors=0, init_errors=0, bad_arguments=0,
                      fail0=0, fail1=0, fail2=0, avoided_timeout=n)]
    return rows


class CaptureTests(unittest.TestCase):
    def test_ready(self):
        self.assertEqual(module.analyze(fixture())['result'], 'PASS')
    def test_missing(self):
        self.assertEqual(module.analyze([])['result'], 'INCONCLUSIVE')
    def test_short(self):
        self.assertEqual(module.analyze(fixture()[:150])['result'], 'REVIEW')
    def test_lost_ready(self):
        rows = fixture()
        rows[-1]['state'] = 0
        self.assertEqual(module.analyze(rows)['result'], 'REVIEW')
    def test_recovered_error_still_review(self):
        rows = fixture()
        rows[-1]['errors'] = 1
        self.assertEqual(module.analyze(rows)['result'], 'REVIEW')
    def test_frozen_channel(self):
        rows = fixture()
        rows[-1]['ch2'] = rows[4]['ch2']
        self.assertEqual(module.analyze(rows)['result'], 'REVIEW')
    def test_line_flags(self):
        rows = fixture()
        rows[-2]['flags'] = 2
        self.assertEqual(module.analyze(rows)['result'], 'REVIEW')
    def test_moving(self):
        rows = fixture()
        rows[1]['active'] = 1
        self.assertEqual(module.analyze(rows)['result'], 'REVIEW')
    def test_clock_slow(self):
        rows = fixture()
        rows[-5]['timer_us'] = rows[0]['timer_us'] + 1000000
        self.assertEqual(module.analyze(rows)['result'], 'REVIEW')
    def test_dropped_usb(self):
        rows = fixture()
        rows[-4]['tx_drop'] = 1
        self.assertEqual(module.analyze(rows)['result'], 'REVIEW')


if __name__ == '__main__':
    unittest.main()
