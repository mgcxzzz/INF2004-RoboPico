import importlib.util
from pathlib import Path
import sys, unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'tools/rtos_migration'))
import capture_network_bench as capture


class NetworkCaptureTests(unittest.TestCase):
    def samples(self):
        rows, beats = [], []
        for t in range(41):
            rows.extend([
                dict(kind='CLOCK', wall_s=t, kernel_ms=t*1000, timer_us=t*1000000, status='PASS'),
                dict(kind='CONTROL', wall_s=t, count=t*100, late=0, flags=12, active=0, usb=3, tx_drop=0, rx_drop=0),
                dict(kind='WHEELS', wall_s=t, gp20=0, gp21=0, irq_stop=0),
                dict(kind='NET', wall_s=t, motors='DISABLED', connected=1, subscribed=1, polls=t*500,
                     last_ms=t*1000, tx_drop=0, rx_drop=0, received=t)])
            beats.append(dict(wall_s=t, uptime_s=t, sequence=t, state=1, net=2))
        return rows, beats

    def test_complete(self):
        r,b = self.samples(); self.assertEqual(capture.analyze(r,b,[])['result'], 'PASS')

    def test_frozen_worker(self):
        r,b = self.samples()
        for x in r:
            if x['kind']=='NET': x.update(polls=2,last_ms=2)
        self.assertEqual(capture.analyze(r,b,[])['result'], 'REVIEW')

    def test_slow_uptime(self):
        r,b = self.samples()
        for x in b: x['uptime_s'] = 1
        self.assertEqual(capture.analyze(r,b,[])['result'], 'REVIEW')

    def test_gap_and_disconnect(self):
        r,b = self.samples()
        self.assertEqual(capture.analyze(r,b[:10]+b[14:],[])['result'], 'REVIEW')
        self.assertEqual(capture.analyze(r,b,['broker disconnected'])['result'], 'REVIEW')

    def test_real_heartbeat_layout(self):
        word = 2 | (1 << 2) | (123 << 12)
        packet = b'\x12\x0c\x07' + word.to_bytes(4,'little') + bytes(8)
        self.assertEqual(capture.heartbeat(packet)['uptime_s'],123)
        self.assertEqual(capture.heartbeat(packet)['net'],2)
        self.assertIsNone(capture.heartbeat(packet[:-1]))


if __name__ == '__main__': unittest.main()
