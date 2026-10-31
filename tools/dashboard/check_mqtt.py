"""Read-only TCP/MQTT connectivity diagnostic for the Pico W broker path."""

from __future__ import annotations

import argparse
import ipaddress
import socket
import time
import uuid


BROKER_PORT = 1883
HEARTBEAT_TOPIC = "is28/hb"
DEFAULT_WAIT_S = 20.0


class DiagnosticError(Exception):
    pass


class PacketTimeout(DiagnosticError):
    pass


def usable_ipv4(value: str, option: str = "broker") -> str:
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise ValueError(f"--{option}-ip must be an IPv4 address") from exc
    if address.is_unspecified or address.is_loopback or address.is_multicast or address.is_reserved:
        raise ValueError(f"--{option}-ip must be a usable unicast IPv4 address")
    return str(address)


def local_ipv4_for_peer(peer_ip: str) -> str:
    """Use the OS route table to identify the source interface; sends no payload."""
    peer_ip = usable_ipv4(peer_ip, "pico")
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect((peer_ip, 9))
        return usable_ipv4(sock.getsockname()[0], "local")


def recv_exact(sock, count: int, deadline: float, monotonic=time.monotonic) -> bytes:
    chunks = bytearray()
    while len(chunks) < count:
        remaining = deadline - monotonic()
        if remaining <= 0:
            raise PacketTimeout("timed out waiting for MQTT packet data")
        sock.settimeout(remaining)
        try:
            part = sock.recv(count - len(chunks))
        except socket.timeout as exc:
            raise PacketTimeout("timed out waiting for MQTT packet data") from exc
        if not part:
            raise DiagnosticError("broker closed the TCP connection")
        chunks.extend(part)
    return bytes(chunks)


def read_packet(sock, timeout_s: float = 5.0, monotonic=time.monotonic) -> tuple[int, bytes]:
    """Read one bounded MQTT packet and reject malformed Remaining Length."""
    deadline = monotonic() + timeout_s
    fixed = recv_exact(sock, 1, deadline, monotonic)[0]
    multiplier = 1
    remaining_length = 0
    for index in range(4):
        byte = recv_exact(sock, 1, deadline, monotonic)[0]
        remaining_length += (byte & 0x7F) * multiplier
        if not (byte & 0x80):
            break
        if index == 3:
            raise DiagnosticError("malformed MQTT Remaining Length (more than four bytes)")
        multiplier *= 128
    if remaining_length > 1024 * 1024:
        raise DiagnosticError("MQTT packet exceeds the 1 MiB diagnostic limit")
    return fixed, recv_exact(sock, remaining_length, deadline, monotonic)


def mqtt_connect_packet(client_id: str) -> bytes:
    client = client_id.encode("utf-8")
    variable = b"\x00\x04MQTT\x04\x02\x00\x14"
    payload = len(client).to_bytes(2, "big") + client
    body = variable + payload
    return b"\x10" + encode_remaining_length(len(body)) + body


def encode_remaining_length(value: int) -> bytes:
    if value < 0 or value > 268435455:
        raise ValueError("invalid MQTT Remaining Length")
    output = bytearray()
    while True:
        digit = value % 128
        value //= 128
        if value:
            digit |= 0x80
        output.append(digit)
        if not value:
            return bytes(output)


def subscribe_packet(packet_id: int = 1, topic: str = HEARTBEAT_TOPIC) -> bytes:
    encoded = topic.encode("utf-8")
    body = packet_id.to_bytes(2, "big") + len(encoded).to_bytes(2, "big") + encoded + b"\x00"
    return b"\x82" + encode_remaining_length(len(body)) + body


def _packet_timeout(sock, timeout_s: float) -> tuple[int, bytes]:
    return read_packet(sock, timeout_s=timeout_s)


def diagnose(
    broker_ip: str,
    *,
    pico_ip: str | None = None,
    wait_s: float = DEFAULT_WAIT_S,
    tcp_timeout_s: float = 3.0,
    connector=socket.create_connection,
    route_selector=local_ipv4_for_peer,
    monotonic=time.monotonic,
    packet_reader=_packet_timeout,
) -> str:
    """Connect, subscribe read-only to heartbeat, and return a concise outcome."""
    broker_ip = usable_ipv4(broker_ip)
    if not 0.1 <= wait_s <= DEFAULT_WAIT_S:
        raise ValueError(f"--wait must be between 0.1 and {DEFAULT_WAIT_S:g} seconds")
    source = route_selector(pico_ip) if pico_ip else None
    if source:
        print(
            f"Route to Pico {pico_ip} selects laptop interface {source}; no UDP payload was sent."
        )
    sock = None
    try:
        try:
            sock = connector((broker_ip, BROKER_PORT), timeout=tcp_timeout_s)
        except OSError as exc:
            return f"TCP connection failed to {broker_ip}:{BROKER_PORT}: {exc}"
        client_id = "pico-netdiag-" + uuid.uuid4().hex[:12]
        sock.sendall(mqtt_connect_packet(client_id))
        try:
            header, body = packet_reader(sock, 5.0)
        except PacketTimeout:
            return "TCP connected, but no MQTT CONNACK arrived within 5 seconds."
        except DiagnosticError as exc:
            return f"TCP connected, but MQTT CONNACK was not readable: {exc}"
        if header >> 4 != 2 or len(body) != 2:
            return "TCP connected, but broker returned an invalid MQTT CONNACK."
        if body[0] & 0xFE:
            return "Broker returned an invalid MQTT CONNACK flags byte."
        reason = body[1]
        if reason != 0:
            reasons = {
                1: "unsupported protocol version",
                2: "client ID rejected",
                3: "server unavailable",
                4: "bad username/password (diagnostic sends none)",
                5: "not authorized",
            }
            return f'MQTT CONNACK refused connection ({reason}: {reasons.get(reason, "reserved/refused")}).'
        sock.sendall(subscribe_packet())
        deadline = monotonic() + wait_s
        suback = False
        next_ping = monotonic() + 10.0
        while monotonic() < deadline:
            remaining = deadline - monotonic()
            try:
                header, body = packet_reader(sock, min(1.0, remaining))
            except PacketTimeout:
                if monotonic() >= next_ping:
                    sock.sendall(b"\xC0\x00")
                    next_ping = monotonic() + 10.0
                continue
            except DiagnosticError as exc:
                return f"MQTT CONNACK accepted, then broker disconnected before heartbeat: {exc}"
            packet_type = header >> 4
            if packet_type == 9:
                if len(body) < 3 or int.from_bytes(body[:2], "big") != 1:
                    return "Broker returned a malformed SUBACK."
                if body[2] == 0x80:
                    return "MQTT connected, but broker refused the is28/hb subscription."
                suback = True
            elif packet_type == 3:
                if len(body) < 2:
                    return "Broker returned a malformed PUBLISH packet."
                topic_len = int.from_bytes(body[:2], "big")
                if len(body) < 2 + topic_len:
                    return "Broker returned a malformed PUBLISH topic."
                try:
                    topic = body[2 : 2 + topic_len].decode("utf-8")
                except UnicodeDecodeError:
                    continue
                if suback and topic == HEARTBEAT_TOPIC and fresh_heartbeat(header, body):
                    return "MQTT connected, is28/hb subscription accepted, fresh Pico heartbeat frame received."
            elif packet_type == 14:
                return "Broker disconnected the MQTT diagnostic client before a heartbeat arrived."
        if not suback:
            return f"MQTT CONNECT accepted, but no SUBACK or Pico heartbeat arrived within {wait_s:g} seconds."
        return f"MQTT connection and is28/hb subscription succeeded; no Pico heartbeat arrived within {wait_s:g} seconds."
    finally:
        if sock is not None:
            try:
                sock.sendall(b"\xE0\x00")
            except OSError:
                pass
            sock.close()


def fresh_heartbeat(header: int, body: bytes) -> bool:
    """Require a non-retained QoS0 v2 heartbeat, not arbitrary topic data."""
    if header != 0x30 or len(body) < 2:
        return False
    n = int.from_bytes(body[:2], "big")
    if len(body) < 2 + n:
        return False
    if body[2 : 2 + n] != HEARTBEAT_TOPIC.encode("utf8"):
        return False
    payload = body[2 + n :]
    return len(payload) == 15 and payload[0] == 0x12 and payload[1] == 12


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--broker", required=True, help="broker IPv4 address (TCP 1883)")
    parser.add_argument("--pico-ip", help="optional Pico IPv4 for route/interface selection")
    parser.add_argument(
        "--wait",
        type=float,
        default=DEFAULT_WAIT_S,
        help=f"max seconds to wait for heartbeat (default {DEFAULT_WAIT_S:g})",
    )
    args = parser.parse_args()
    try:
        print(diagnose(args.broker, pico_ip=args.pico_ip, wait_s=args.wait))
    except (ValueError, OSError, RuntimeError) as exc:
        raise SystemExit(str(exc)) from exc


if __name__ == "__main__":
    main()
