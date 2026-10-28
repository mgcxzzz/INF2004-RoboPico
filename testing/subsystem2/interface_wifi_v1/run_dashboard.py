"""Run the local dashboard and, when available, a local Mosquitto broker."""
from __future__ import annotations

import argparse
import functools
import http.server
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


def lan_ip() -> str:
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(('192.0.2.1', 9))
            return sock.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())


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
    """Reuse a complete broker or start and verify a local one.

    Raises RuntimeError when a partial existing broker would conflict, or when
    a newly started process exits or fails to open both listeners in time.
    """
    ports_ready = {port: port_probe(port) for port in BROKER_PORTS}
    if all(ports_ready.values()):
        print('Using the broker already listening on ports 1883 and 9001.')
        return None, None

    if any(ports_ready.values()):
        open_ports = ', '.join(str(port) for port, is_open in ports_ready.items() if is_open)
        missing_ports = ', '.join(str(port) for port, is_open in ports_ready.items() if not is_open)
        raise RuntimeError(
            f'A broker listener is already using port(s) {open_ports}, but port(s) {missing_ports} '
            'are unavailable. Stop or reconfigure the existing Mosquitto service so the supplied '
            'configuration can listen on both TCP 1883 and WebSockets 9001, then rerun this helper. '
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
                    'Mosquitto exited before both ports 1883 and 9001 became available. '
                    f'Check {session / "broker.log"}; stop any existing broker using those ports.'
                )
            if all(port_probe(port) for port in BROKER_PORTS) and broker.poll() is None:
                print('Started Mosquitto and confirmed ports 1883 and 9001 are listening.')
                return broker, log
            remaining = deadline - monotonic()
            if remaining <= 0:
                raise RuntimeError(
                    'Mosquitto did not open both ports 1883 and 9001 before the startup timeout. '
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
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--no-browser', action='store_true')
    args = parser.parse_args()
    root = Path(__file__).resolve().parent
    dashboard = root / 'dashboard'
    binary = find_mosquitto(args.mosquitto)

    try:
        broker, log = start_broker_if_needed(binary, dashboard, root)
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc

    if not binary and broker is None and not all(listening(port) for port in BROKER_PORTS):
        print('Mosquitto is not installed. Install the Windows package from https://mosquitto.org/download/ and rerun this helper.')
        print('The dashboard will still open; it will show disconnected until the broker runs on ports 1883 and 9001.')

    ip = lan_ip()
    print(f'Pico broker setting: {ip} (LAN address; use the same Wi-Fi/hotspot).')
    print(f'Laptop dashboard: http://localhost:{args.port}/')
    print(f'Phone on the same LAN: http://{ip}:{args.port}/ ; broker WebSocket ws://{ip}:9001')
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
