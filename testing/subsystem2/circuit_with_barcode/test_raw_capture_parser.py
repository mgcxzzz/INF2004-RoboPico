import struct
import unittest
import zlib
from analyze_raw_capture import parse_capture


def fixture():
    records = [(2000, -25, 181), (4000, 0, 218), (6000, 4, 205)]
    crc = zlib.crc32(b''.join(struct.pack('<IiHH', *r, 0) for r in records))
    text = f'help text\nRAW_ADC_BEGIN: rows=3 data_crc={crc:08x} stored=1 matches_motion=1\nsample,us,path_mm,adc\n'
    text += ''.join(f'{n},{us},{path/100:.2f},{adc}\n' for n, (us, path, adc) in enumerate(records))
    return text + f'RAW_ADC_END: rows=3 data_crc={crc:08x}\n'


class RawParserTests(unittest.TestCase):
    def test_valid_lf_and_crlf(self):
        for separator in ['\n', '\r\n']:
            fields, rows = parse_capture(fixture().replace('\n', separator))
            self.assertEqual(rows[0], (2000, -25, 181))
            self.assertEqual(len(rows), 3)

    def test_missing_row(self):
        with self.assertRaises(ValueError):
            parse_capture(fixture().replace('1,4000,0.00,218\n', ''))

    def test_corrupted_adc(self):
        with self.assertRaises(ValueError):
            parse_capture(fixture().replace(',218\n', ',219\n'))

    def test_missing_end(self):
        with self.assertRaises(ValueError):
            parse_capture(fixture().split('RAW_ADC_END:')[0])

    def test_truncated_line(self):
        with self.assertRaises(ValueError):
            parse_capture(fixture().replace('1,4000,0.00,218', '1,4000,0.00'))

    def test_last_dump_must_be_complete(self):
        with self.assertRaises(ValueError):
            parse_capture(fixture() + fixture().split('RAW_ADC_END:')[0])


if __name__ == '__main__':
    unittest.main()
