import sys,unittest,struct
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/rtos_migration'))
from capture_wireless import decode_release,summarize

def runtime(reads,up):
    values=[up,reads,0,0,0,0,0,reads*2,0,2,1,0,0,1]
    return bytes([0xf2,60,2,0xc3])+struct.pack('<14I',*values)+bytes([0,1,12])
class WirelessTests(unittest.TestCase):
    def test_runtime_and_wall_deltas(self):
        frames=[{'topic':'is28/tlm','hex':runtime(100+50*k,2000+1000*k).hex(),'wall_s':2+k,'retained':False} for k in range(3)]
        rows,r=summarize(frames,[]);self.assertEqual(len(rows),3)
        self.assertEqual(r['runtime_windows'][0]['imu_hz_wall'],50)
        self.assertEqual(r['runtime_windows'][0]['control_hz_wall'],100)
        self.assertEqual(r['result'],'REVIEW_PHYSICAL_TEST')
    def test_retained_and_wrong_layout_ignored(self):
        rows,r=summarize([{'topic':'is28/tlm','hex':runtime(100,2000).hex(),'wall_s':1,'retained':True}],[])
        self.assertEqual(rows,[]);self.assertIsNone(r['runtime_first'])
        self.assertIsNone(decode_release(b'\xf2\x02\x00\xc3'))
    def test_actual_c_metadata_when_available(self):
        p=Path(__file__).parent/'build-host/release_metadata.bin'
        if not p.exists():self.skipTest('Run test_release_exports first for C fixture')
        d=decode_release(p.read_bytes());self.assertEqual(d['version'],'0.2.0')
        self.assertEqual(d['white'],[122,138]);self.assertEqual(d['black'],[246,267])
        self.assertTrue(d['network_supervised'] and d['stored_logs'])
if __name__=='__main__':unittest.main()
