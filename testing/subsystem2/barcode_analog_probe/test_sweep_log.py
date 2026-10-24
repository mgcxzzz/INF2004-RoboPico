import unittest
import zlib
from analyze_sweep_log import parse

ROWS = ['0,0,200,0,0,0,0.000,0.000,0.000',
        '1,2000,3000,771,-10,10,0.792,0.791,0.791',
        '2,4000,210,0,-20,20,1.585,1.582,1.583']

def trace(rows=ROWS, count=3, crc_rows=ROWS, newline='\n'):
    checksum = zlib.crc32(('\n'.join(crc_rows)+'\n').encode())
    return newline.join([f'ANALOG_SWEEP v5: samples={count} duration_ms=5', *rows,
                         f'SWEEP_DUMP_END: rows={count} crc32={checksum:08x}', 'END ANALOG_SWEEP.'])

class AuditTests(unittest.TestCase):
    def test_complete_and_crlf(self):
        for newline in ['\n', '\r\n']:
            s = parse(trace(newline=newline))[0].summary()
            self.assertTrue(s['checksum_verified'])
            self.assertEqual(s['ADC_range'], [200, 3000])

    def test_dropped_row_with_end_marker(self):
        s = parse(trace([ROWS[0], ROWS[2]]))[0].summary()
        self.assertFalse(s['complete'])
        self.assertFalse(s['indexes_ok'])
        self.assertFalse(s['checksum_verified'])

    def test_changed_voltage(self):
        changed = ROWS.copy()
        changed[1] = changed[1].replace('3000', '3100')
        s = parse(trace(changed))[0].summary()
        self.assertTrue(s['complete'])
        self.assertFalse(s['checksum_verified'])

    def test_truncation_and_joined_row(self):
        self.assertFalse(parse(trace().split('SWEEP_DUMP_END')[0])[0].summary()['complete'])
        s = parse(trace([ROWS[0], ROWS[1]+ROWS[2]]))[0].summary()
        self.assertFalse(s['complete'])
        self.assertEqual(s['malformed'], 1)

    def test_multiple_versions_without_cross_contamination(self):
        old = 'ANALOG_SWEEP v4: samples=8241 duration_ms=17010\n75,187,0,0,0,0,0,0\nEND ANALOG_SWEEP.\n'
        scans = parse('Old help text\n'+old+trace())
        self.assertEqual(len(scans), 2)
        self.assertFalse(scans[0].summary()['complete'])
        self.assertTrue(scans[1].summary()['checksum_verified'])

    def test_duplicate_or_reversed_time(self):
        rows = [ROWS[0], ROWS[1], ROWS[2].replace('4000', '1000')]
        self.assertFalse(parse(trace(rows, crc_rows=rows))[0].summary()['complete'])

if __name__ == '__main__':
    unittest.main()
