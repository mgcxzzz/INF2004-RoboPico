import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock
from provision_wifi import configuration_line
from run_dashboard import start_broker_if_needed


class ProvisionTests(unittest.TestCase):
    def test_hidden_wire_and_open_network(self):
        self.assertEqual(configuration_line('test','','192.168.1.2'),b'!net|test||192.168.1.2\n')
    def test_invalid(self):
        for a,b,c in [('', '', '192.168.1.2'),('x'*33,'','192.168.1.2'),('test','x'*65,'192.168.1.2'),('test','secret\n','192.168.1.2'),('test','','127.0.0.1'),('test','','256.1.1.1')]:
            with self.assertRaises(ValueError):configuration_line(a,b,c)


class FakeProcess:
    def __init__(self, exit_code=None):
        self.exit_code=exit_code
        self.terminated=False
        self.killed=False

    def poll(self):return self.exit_code
    def terminate(self):self.terminated=True;self.exit_code=0
    def wait(self,timeout=None):return self.exit_code
    def kill(self):self.killed=True;self.exit_code=-9


class DashboardBrokerTests(unittest.TestCase):
    def test_partial_existing_listener_fails_without_starting_child(self):
        for active_port in (1883,9001):
            popen=Mock()
            with tempfile.TemporaryDirectory() as temporary:
                with self.assertRaisesRegex(RuntimeError,'Stop or reconfigure the existing Mosquitto service'):
                    start_broker_if_needed('mosquitto',Path(temporary)/'dashboard',Path(temporary),
                        port_probe=lambda port:port==active_port,popen=popen)
            popen.assert_not_called()

    def test_child_exit_before_listeners_is_reported_and_log_closed(self):
        child=FakeProcess(exit_code=2)
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(RuntimeError,'exited before both ports'):
                start_broker_if_needed('mosquitto',Path(temporary)/'dashboard',Path(temporary),
                    port_probe=lambda _port:False,popen=Mock(return_value=child))
            self.assertTrue((Path(temporary)/'dashboard_session'/'broker.log').exists())

    def test_startup_timeout_terminates_child_and_closes_log(self):
        child=FakeProcess()
        now=[0.0]
        def sleep(seconds):now[0]+=seconds
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(RuntimeError,'startup timeout'):
                start_broker_if_needed('mosquitto',Path(temporary)/'dashboard',Path(temporary),
                    port_probe=lambda _port:False,popen=Mock(return_value=child),
                    monotonic=lambda:now[0],sleep=sleep,startup_timeout=.25,poll_interval=.1)
        self.assertTrue(child.terminated)
        self.assertFalse(child.killed)

    def test_started_child_is_returned_only_after_both_ports_open(self):
        child=FakeProcess()
        probes=[False,False,True,True]
        def probe(_port):return probes.pop(0)
        with tempfile.TemporaryDirectory() as temporary:
            process,log=start_broker_if_needed('mosquitto',Path(temporary)/'dashboard',Path(temporary),
                port_probe=probe,popen=Mock(return_value=child),sleep=lambda _seconds:None)
            self.assertIs(process,child)
            self.assertIsNotNone(log)
            self.assertIsNone(child.poll())
            log.close()


if __name__=='__main__':unittest.main()
