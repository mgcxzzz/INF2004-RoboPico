import socket
import unittest
from unittest.mock import Mock, patch

import check_mqtt
import run_dashboard


class FakeSocket:
    def __init__(self, chunks=(), error=None):
        self.chunks = list(chunks)
        self.error = error
        self.timeouts = []

    def settimeout(self, value):
        self.timeouts.append(value)

    def recv(self, count):
        if self.error:
            raise self.error
        if not self.chunks:
            return b""
        data = self.chunks.pop(0)
        if len(data) > count:
            self.chunks.insert(0, data[count:])
            return data[:count]
        return data


class NetworkDiagnosticTests(unittest.TestCase):
    def test_only_fresh_schema_heartbeat_counts(self):
        topic = b"is28/hb"
        body = len(topic).to_bytes(2, "big") + topic + bytes([0x12, 12, 1]) + bytes(12)
        self.assertTrue(check_mqtt.fresh_heartbeat(0x30, body))
        self.assertFalse(check_mqtt.fresh_heartbeat(0x31, body))  # retained old value
        self.assertFalse(check_mqtt.fresh_heartbeat(0x32, body))  # unsupported QoS
        self.assertFalse(check_mqtt.fresh_heartbeat(0x30, body[:-1]))
        self.assertFalse(check_mqtt.fresh_heartbeat(0x30, body[:-15] + b"not a heartbeat"))

    def test_diagnose_requires_connack_suback_then_fresh_heartbeat(self):
        topic = b"is28/hb"
        heartbeat = len(topic).to_bytes(2, "big") + topic + bytes([0x12, 12, 1]) + bytes(12)
        sock = Mock()
        packets = iter(
            [(0x20, b"\x00\x00"), (0x90, b"\x00\x01\x00"), (0x31, heartbeat), (0x30, heartbeat)]
        )
        result = check_mqtt.diagnose(
            "10.1.2.3", connector=lambda *a, **k: sock, packet_reader=lambda *a: next(packets)
        )
        self.assertIn("fresh Pico heartbeat frame received", result)
        self.assertEqual(sock.sendall.call_args_list[0].args[0][0], 0x10)
        self.assertEqual(sock.sendall.call_args_list[1].args[0][0], 0x82)
        sock.close.assert_called_once()

    def test_diagnose_reports_broker_refusal(self):
        sock = Mock()
        result = check_mqtt.diagnose(
            "10.1.2.3", connector=lambda *a, **k: sock, packet_reader=lambda *a: (0x20, b"\x00\x05")
        )
        self.assertIn("not authorized", result)
        sock.close.assert_called_once()

    def test_diagnose_reports_tcp_failure(self):
        def fail(*a, **k):
            raise OSError("blocked")

        self.assertIn("TCP connection failed", check_mqtt.diagnose("10.1.2.3", connector=fail))

    def test_frame_reader_handles_partial_reads(self):
        fake = FakeSocket([b"\x20", b"\x02", b"\x00\x00"])
        self.assertEqual(check_mqtt.read_packet(fake), (0x20, b"\x00\x00"))

    def test_frame_reader_rejects_more_than_four_remaining_length_bytes(self):
        fake = FakeSocket([b"\x20", b"\x80", b"\x80", b"\x80", b"\x80"])
        with self.assertRaisesRegex(check_mqtt.DiagnosticError, "more than four bytes"):
            check_mqtt.read_packet(fake)

    def test_frame_reader_times_out(self):
        fake = FakeSocket(error=socket.timeout())
        with self.assertRaises(check_mqtt.PacketTimeout):
            check_mqtt.read_packet(fake, timeout_s=0.1)

    @patch("check_mqtt.socket.socket")
    def test_mqtt_route_selection_uses_pico_peer_without_sending(self, socket_factory):
        sock = socket_factory.return_value.__enter__.return_value
        sock.getsockname.return_value = ("192.168.43.20", 0)
        self.assertEqual(check_mqtt.local_ipv4_for_peer("192.168.43.31"), "192.168.43.20")
        sock.connect.assert_called_once_with(("192.168.43.31", 9))
        sock.send.assert_not_called()
        sock.sendall.assert_not_called()

    @patch("run_dashboard.socket.socket")
    def test_dashboard_route_selection_uses_pico_peer_without_sending(self, socket_factory):
        sock = socket_factory.return_value.__enter__.return_value
        sock.getsockname.return_value = ("10.0.0.7", 0)
        self.assertEqual(run_dashboard.local_ipv4_for_peer("10.0.0.23"), "10.0.0.7")
        sock.connect.assert_called_once_with(("10.0.0.23", 9))
        sock.send.assert_not_called()
        sock.sendall.assert_not_called()


if __name__ == "__main__":
    unittest.main()
