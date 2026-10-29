"""Run the local dashboard and, when available, a local Mosquitto broker."""
from __future__ import annotations

import argparse
import functools
import http.server
import ipaddress
import os
import shutil
import socket
import subprocess
import time
import webbrowser
from pathlib import Path
from typing import Callable, TextIO


BROKER_PORTS = (1883, 9001)
BROKER_START_TIMEOUT_S = 8.0
BROKER_POLL_INTERVAL_S = 0.1


def usable_host_ipv4(value: str) -> str:
    try:
        address = ipaddress.IPv4Address(value)
    except ipaddress.AddressValueError as exc:
        raise ValueError('--broker-ip and --pico-ip must be IPv4 addresses') from exc
    if address.is_unspecified or address.is_loopback or address.is_multicast or address.is_reserved:
        raise ValueError('Use a usable unicast IPv4 address, not loopback or a reserved address')
    return str(address)


def local_ipv4_for_peer(peer_ip: str) -> str:
    """Select the local interface routed to a peer, without sending a packet."""
    peer_ip = usable_host_ipv4(peer_ip)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect((peer_ip, 9))
            local_ip = sock.getsockname()[0]
    except OSError as exc:
        raise RuntimeError(f'Could not select a local route to Pico at {peer_ip}: {exc}') from exc
    return usable_host_ipv4(local_ip)


def listening(port: int) -> bool:
    try:
        with socket.create_connection(('127.0.0.1', port), timeout=0.3):
            return True
    except OSError:
        return False


def stop_broker(broker: subprocess.Popen[bytes]) -> None:
    if broker.poll() is not None:
        return
    broker.terminate()
    try:
        broker.wait(timeout=5)
    except subprocess.TimeoutExpired:
        broker.kill()
        broker.wait()


def start_broker_if_needed(
    binary: str | None,
    dashboard: Path,
    root: Path,
    *,
    port_probe: Callable[[int], bool] = listening,
    popen: Callable[..., subprocess.Popen[bytes]] = subprocess.Popen,
    monotonic: Callable[[], float] = time.monotonic,
    sleep: Callable[[float], None] = time.sleep,
    startup_timeout: float = BROKER_START_TIMEOUT_S,
    poll_interval: float = BROKER_POLL_INTERVAL_S,
) -> tuple[subprocess.Popen[bytes] | None, TextIO | None]:
    """Reuse a complete broker or start and verify its local listeners.

    Raises RuntimeError when a partial existing broker would conflict, or when
    a newly started process exits or fails to open both local listeners in time.
    """
    ports_ready = {port: port_probe(port) for port in BROKER_PORTS}
    if all(ports_ready.values()):
        print('Found local TCP listeners on ports 1883 and 9001; remote Pico reachability is not tested.')
        return None, None

    if any(ports_ready.values()):
        open_ports = ', '.join(str(port) for port, is_open in ports_ready.items() if is_open)
        missing_ports = ', '.join(str(port) for port, is_open in ports_ready.items() if not is_open)
        raise RuntimeError(
            f'A broker listener is already using port(s) {open_ports}, but port(s) {missing_ports} '
            'are unavailable. Stop or reconfigure the existing Mosquitto service so the supplied '
            'configuration can listen locally on TCP 1883 and WebSockets 9001, then rerun this helper. '
            'This helper will not change Windows services or firewall settings.'
        )

    if not binary:
        return None, None

    session = root / 'dashboard_session'
    session.mkdir(exist_ok=True)
    log = (session / 'broker.log').open('a', encoding='utf-8')
    broker: subprocess.Popen[bytes] | None = None
    try:
        broker = popen(
            [binary, '-c', str(dashboard / 'mosquitto.conf'), '-v'],
            stdout=log,
            stderr=subprocess.STDOUT,
            creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0),
        )
        deadline = monotonic() + startup_timeout
        while True:
            if broker.poll() is not None:
                raise RuntimeError(
                    'Mosquitto exited before local listeners on ports 1883 and 9001 became available. '
                    f'Check {session / "broker.log"}; stop any existing broker using those ports.'
                )
            if all(port_probe(port) for port in BROKER_PORTS) and broker.poll() is None:
                print('Started Mosquitto and confirmed local listeners on ports 1883 and 9001; remote Pico reachability is not tested.')
                return broker, log
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    'Mosquitto did not open local listeners on ports 1883 and 9001 before the startup timeout. '
                    f'Check {session / "broker.log"} and stop/reconfigure any conflicting service.'
                )
            sleep(min(poll_interval, remaining))
    except Exception:
        if broker is not None:
            stop_broker(broker)
        log.close()
        raise


def find_mosquitto(explicit_path: Path | None) -> str | None:
    binary = str(explicit_path) if explicit_path else shutil.which('mosquitto')
    if binary:
        return binary
    for folder in ('ProgramFiles', 'ProgramFiles(x86)'):
        candidate = Path(os.environ.get(folder, '')) / 'mosquitto' / 'mosquitto.exe'
        if candidate.is_file():
            return str(candidate)
    return None


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mosquitto', type=Path, help='Path to mosquitto.exe if it is not on PATH')
    parser.add_argument('--pico-ip', help='Pico IPv4; use it to select the laptop interface routed to the Pico')
    parser.add_argument('--broker-ip', help='Explicit laptop IPv4 to provision as the Pico broker address')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    try:
        pico_ip = usable_host_ipv4(args.pico_ip) if args.pico_ip else None
        broker_ip = usable_host_ipv4(args.broker_ip) if args.broker_ip else None
        route_ip = local_ipv4_for_peer(pico_ip) if pico_ip else None
    except (RuntimeError, ValueError) as exc:
        raise SystemExit(str(exc)) from exc
    root = Path(__file__).resolve().parent
    dashboard = root / 'dashboard'
    binary = find_mosquitto(args.mosquitto)

    try:
        broker, log = start_broker_if_needed(binary, dashboard, root)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    if not binary and broker is None and not all(listening(port) for port in BROKER_PORTS):
        print('Mosquitto is not installed. Install the Windows package from https://mosquitto.org/download/ and rerun this helper.')
        print('The dashboard will still open; local broker listeners on TCP 1883 and WebSockets 9001 are unavailable.')
    ip = broker_ip or route_ip
    if broker_ip:
        print(f'Pico broker setting (explicit laptop IPv4): {broker_ip}.')
    elif route_ip:
        print(f'Pico broker setting candidate (interface routed to Pico {pico_ip}): {route_ip}.')
    else:
        print('No laptop broker IPv4 guessed. Rerun with --pico-ip <Pico IPv4> to select the routed interface,')
        print('or --broker-ip <laptop IPv4> to choose the address to provision explicitly.')
    print(f'Laptop dashboard: http://localhost:{args.port}/')
    if ip:
        print(f'Phone/dashboard WebSocket candidate: http://{ip}:{args.port}/ and ws://{ip}:9001.')
    print('Local browser WebSocket (ws://localhost:9001) and Pico TCP MQTT (TCP 1883) are separate paths.')
    print('Local listener checks do not prove that the Pico can reach TCP 1883; use check_mqtt.py from this laptop.')
    print('Press Ctrl+C here to close the dashboard server and the broker started by this helper.')
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=str(dashboard))
    try:
        with http.server.ThreadingHTTPServer(('0.0.0.0', args.port), handler) as server:
            if not args.no_browser:
                webbrowser.open(f'http://localhost:{args.port}/')
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        if broker is not None:
            stop_broker(broker)
        if log is not None:
            log.close()


if __name__ == '__main__':
    main()
