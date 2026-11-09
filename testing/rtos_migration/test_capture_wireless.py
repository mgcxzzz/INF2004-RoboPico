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
    def test_actual_c_diagnostic_packets(self):
        base=Path(__file__).parent/'build-host'
        required=['encoder_fault.bin','encoder_fault_clear.bin','barcode_attempt.bin','network_stop.bin']
        if not all((base/name).is_file() for name in required):self.skipTest('Run C diagnostic fixture tests first')
        encoder=decode_release((base/'encoder_fault.bin').read_bytes())
        self.assertEqual((encoder['trip_kind'],encoder['wheel']),(2,2))
        self.assertEqual(encoder['targets'],[-45,70]);self.assertEqual(encoder['pwm'],[-33,400])
        self.assertFalse(decode_release((base/'encoder_fault_clear.bin').read_bytes())['valid'])
        barcode=decode_release((base/'barcode_attempt.bin').read_bytes())
        self.assertEqual((barcode['attempt'],barcode['code'],barcode['bars']),(3,'F',14))
        self.assertTrue(barcode['original_accepted'] and barcode['truncated'])
        self.assertEqual(barcode['raw_count'],2048)
        network=decode_release((base/'network_stop.bin').read_bytes())
        self.assertTrue(network['valid']);self.assertEqual(network['link'],-2)
        frames=[{'topic':'is28/tlm','hex':(base/name).read_bytes().hex(),'wall_s':k,'retained':False} for k,name in enumerate(required)]
        _,summary=summarize(frames,[])
        self.assertIsNone(summary['release_first'])
        self.assertEqual(summary['diagnostic_latest']['barcode-attempt']['code'],'F')
    def test_actual_c_metadata_when_available(self):
        p=Path(__file__).parent/'build-host/release_metadata.bin'
        if not p.exists():self.skipTest('Run test_release_exports first for C fixture')
        d=decode_release(p.read_bytes());self.assertEqual(d['version'],'0.2.2')
        self.assertEqual(d['white'],[122,138]);self.assertEqual(d['black'],[246,267])
        self.assertTrue(d['network_supervised'] and d['stored_logs'])
    def test_drive_and_junction_c_fixtures(self):
        base=Path(__file__).parent/'build-host'
        j=decode_release((base/'junction_decision.bin').read_bytes())
        d=decode_release((base/'drive_diagnostics.bin').read_bytes())
        self.assertEqual(j['width_mm'],10.4);self.assertEqual(j['result'],4)
        self.assertEqual(j['threshold'],1100);self.assertEqual(j['hysteresis'],90)
        self.assertEqual(d['target_left'],-35);self.assertEqual(d['target_right'],35)
        self.assertEqual(d['recovery_yaw_deg'],-12.3);self.assertEqual(d['bend_phase'],2)
if __name__=='__main__':unittest.main()
