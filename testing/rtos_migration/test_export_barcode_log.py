import unittest,sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/rtos_migration'))
from export_barcode_log import parse_capture
def sample():
    return ['BARCODE_LOG_BEGIN version=1 events=1 raw=2 overwritten=3 frozen=1',
            'BARCODE_LOG_CAL white=122,138 black=246,267 origin=1',
            'BARCODE_EVENT 5,1500,500,14000,3,65,76,1,64,148,265,148',
            'BARCODE_RAW 0,1000,100,175','BARCODE_RAW 1,1002,109,2200',
            'BARCODE_LOG_END events=1 raw=2']
class ExportTests(unittest.TestCase):
    def test_valid_and_interleaved(self):
        a=sample();a.insert(3,'NET connected=1');events,raw,s=parse_capture(a)
        self.assertEqual(s['result'],'COMPLETE');self.assertEqual(s['max_raw_gap_ms'],2)
        self.assertEqual(s['decoded'][0]['letter_ascii'],65);self.assertTrue(s['raw_frozen_on_decode'])
    def test_partial_or_duplicate(self):
        self.assertEqual(parse_capture(sample()[:-1])[2]['result'],'INCOMPLETE')
        self.assertEqual(parse_capture(sample()+sample())[2]['result'],'INCOMPLETE')
    def test_invalid_adc_sequence_pattern_and_counts(self):
        for index,value in [(4,'BARCODE_RAW 2,999,1,5000'),(2,'BARCODE_EVENT 5,1,0,0,3,65,76,1,0,999,1,148'),(-1,'BARCODE_LOG_END events=1 raw=0')]:
            a=sample();a[index]=value;self.assertEqual(parse_capture(a)[2]['result'],'INCOMPLETE')
    def test_empty_stationary(self):
        a=[sample()[0].replace('events=1 raw=2','events=0 raw=0'),sample()[1], 'BARCODE_LOG_END events=0 raw=0']
        self.assertEqual(parse_capture(a)[2]['result'],'COMPLETE')
if __name__=='__main__':unittest.main()
