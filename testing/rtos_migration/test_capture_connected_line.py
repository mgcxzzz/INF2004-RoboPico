import sys, unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'tools/rtos_migration'))
from capture_connected_line import summarize

class CaptureTests(unittest.TestCase):
    def test_no_automatic_driving_pass(self):
        r=summarize([],[],[])
        self.assertEqual(r['result'],'REVIEW_PHYSICAL_TEST')
        self.assertEqual(r['motion_frames'],0)
        self.assertIsNone(r['max_heartbeat_gap_s'])
    def test_real_packet_shapes_and_gap(self):
        hb=bytes([0x12,12,1])+bytes(12)
        motion=bytes([0x92,45,2])+bytes(45)
        frames=[{'wall_s':t,'topic':'is28/hb','hex':hb.hex()} for t in [1,2,5]]
        frames += [{'wall_s':2,'topic':'is28/tlm','hex':motion.hex()},
                   {'wall_s':2,'topic':'is28/tlm','hex':'92'}]
        r=summarize([],frames,[{'error':'intentional broker loss'}])
        self.assertEqual(r['heartbeats'],3)
        self.assertEqual(r['max_heartbeat_gap_s'],3)
        self.assertEqual(r['motion_frames'],1)
        self.assertEqual(len(r['monitor_interruptions']),1)

if __name__=='__main__':unittest.main()
