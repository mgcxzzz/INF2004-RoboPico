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
    def test_delivery_and_guard_are_preserved_in_summary(self):
        rows=[{'kind':'NET_TX','mem':5,'coalesced':10},
              {'kind':'NET_TX','mem':6,'coalesced':20},
              {'kind':'ENCODER_GUARD','trip_kind':1},
              {'kind':'ENCODER_TRIP','wheel':2,'trip_kind':1}]
        r=summarize(rows,[],[])
        self.assertEqual(r['delivery_first']['mem'],5)
        self.assertEqual(r['delivery_last']['coalesced'],20)
        self.assertEqual(r['encoder_guard_last']['trip_kind'],1)
        self.assertEqual(len(r['encoder_trips']),1)
    def test_full_imu_counters_and_reset_epochs_are_preserved(self):
        rows=[{'kind':'IMU_ACQ','failed':65,'epoch':1,'ready_epoch':1},
              {'kind':'IMU_ACQ','failed':66,'epoch':2,'ready_epoch':1},
              {'kind':'IMU_I2C','timeout_stop':4,'aborts':2}]
        r=summarize(rows,[],[])
        self.assertEqual(r['imu_acquisition_first']['failed'],65)
        self.assertEqual(r['imu_acquisition_last']['epoch'],2)
        self.assertEqual(r['imu_acquisition_last']['ready_epoch'],1)
        self.assertEqual(r['imu_i2c_last']['timeout_stop'],4)
    def test_recent_timing_windows_and_logging_are_preserved(self):
        rows=[{'kind':'IMU_TIMING','win_seq':1,'win_max_us':20000},
              {'kind':'IMU_TIMING','win_seq':1,'win_max_us':31000},
              {'kind':'IMU_TIMING','win_seq':2,'prev_max_us':31000,'win_max_us':20000},
              {'kind':'LOGGING','atomic':1,'dropped':0,'max_copy_us':20}]
        r=summarize(rows,[],[])
        self.assertEqual(len(r['imu_timing_windows']),2)
        self.assertEqual(r['imu_timing_windows'][0]['win_max_us'],31000)
        self.assertEqual(r['imu_timing_last']['prev_max_us'],31000)
        self.assertEqual(r['logging_last']['max_copy_us'],20)
    def test_startup_calibration_fault_and_service_timing_retained(self):
        rows=[{'kind':'IMU_CAL','epoch':1,'core':1,'fault':3,'last_reason':2},
              {'kind':'IMU_CAL','epoch':2,'core':1,'fault':0,'progress':100},
              {'kind':'NET_SERVICE','max_radio_us':14000,'max_total_us':15000}]
        r=summarize(rows,[],[])
        self.assertEqual(r['imu_calibration_faults'][0]['last_reason'],2)
        self.assertEqual(r['imu_calibration_last']['progress'],100)
        self.assertEqual(r['network_service_last']['max_total_us'],15000)
    def test_v014_phase_budget_and_all_poll_snapshots_are_retained(self):
        rows=[{'kind':'IMU_PHASE','read_us':1700,'process_us':400},
              {'kind':'IMU_PHASE','read_us':1900,'process_us':450},
              {'kind':'IMU_BUDGET','fifo_exhausted':1},
              {'kind':'SIDE_SERVICE','polls':500,'poll_total_us':80000}]
        r=summarize(rows,[],[])
        self.assertEqual(r['imu_phase_first']['read_us'],1700)
        self.assertEqual(r['imu_phase_last']['process_us'],450)
        self.assertEqual(r['imu_budget_last']['fifo_exhausted'],1)
        self.assertEqual(r['side_service_last']['polls'],500)
        self.assertEqual(r['result'],'REVIEW_PHYSICAL_TEST')

if __name__=='__main__':unittest.main()
