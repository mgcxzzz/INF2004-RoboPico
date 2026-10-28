import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock
from provision_wifi import configuration_line,wait_for_confirmation
from run_dashboard import start_broker_if_needed


class ProvisionTests(unittest.TestCase):
    def test_hidden_wire_and_open_network(self):
        self.assertEqual(configuration_line('test','','192.168.1.2'),b'!net|test||192.168.1.2\n')
    def test_invalid(self):
        for a,b,c in [('', '', '192.168.1.2'),('x'*33,'','192.168.1.2'),('test','x'*65,'192.168.1.2'),('test','secret\n','192.168.1.2'),('test','','127.0.0.1'),('test','','256.1.1.1')]:
            with self.assertRaises(ValueError):configuration_line(a,b,c)

    def test_ack_is_consumed_before_reboot_discards_partial_read(self):
        class Port:
            remaining=bytearray(b'noise\nWIFI_PROVISIONED:')
            def read(self,size):
                if size!=1:raise OSError('Windows reset discarded partial read')
                if not self.remaining:raise OSError('reboot')
                return bytes([self.remaining.pop(0)])
        wait_for_confirmation(Port(),OSError)

    def test_disconnect_without_ack_is_not_success(self):
        port=Mock();port.read.side_effect=OSError('USB reset')
        with self.assertRaisesRegex(SystemExit,'Success was not confirmed'):
            wait_for_confirmation(port,OSError)

    def test_rejected_configuration_is_not_success(self):
        port=Mock();port.read.side_effect=[bytes([x]) for x in b'WIFI_SETUP_REJECTED:']
        with self.assertRaisesRegex(SystemExit,'Setup rejected'):
            wait_for_confirmation(port,OSError)

    def test_missing_ack_times_out(self):
        port=Mock();port.read.return_value=b''
        clock=Mock(side_effect=[0,0,9])
        with self.assertRaisesRegex(SystemExit,'No verified setup response'):
            wait_for_confirmation(port,OSError,clock=clock)


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
