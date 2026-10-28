#!/usr/bin/env python3
"""Opt-in Buddy 1 hardware checks against a running local MQTT broker.

Uses only Python's standard library. It changes the test mapping and
telemetry setting; run with a simulated/stationary Buddy 1 image.
Publisher-side TCP splits do not prove callback boundaries on the Pico.
"""
import argparse
import json
import socket
import struct
import time
from pathlib import Path


def text(value):
    data = value.encode()
    return struct.pack('!H', len(data)) + data


def packet(header, body):
    size = len(body)
    encoded = bytearray([header])
    while True:
        byte = size % 128
        size //= 128
        encoded.append(byte | (128 if size else 0))
        if not size:
            break
    return bytes(encoded) + body


class Client:
    def __init__(self, host, capture):
        self.sock = socket.create_connection((host, 1883), timeout=4)
        self.sock.settimeout(.2)
        self.buf = bytearray()
        self.events = []
        self.capture = capture.open('w')
        self.send(0x10, text('MQTT') + bytes([4, 2, 0, 30]) +
                  text('is28-live-check-' + str(time.time_ns())))
        self.wait(lambda e: e['kind'] == 2)
        self.send(0x82, b'\x00\x01' + text('is28/#') + b'\x00')
        self.wait(lambda e: e['kind'] == 9)
        self.request_id = 20

    def send(self, header, body):
        self.sock.sendall(packet(header, body))

    def publish(self, payload, split=False):
        data = packet(0x30, text('is28/cmd') + payload)
        if split:
            for start in range(0, len(data), 13):
                self.sock.sendall(data[start:start + 13])
                time.sleep(.002)
        else:
            self.sock.sendall(data)
        self.capture.write(f'{time.time():.6f} SENT is28/cmd {payload.hex()}\n')
        self.capture.flush()

    def pump(self):
        try:
            data = self.sock.recv(65536)
            if not data:
                raise ConnectionError('Broker closed connection')
            self.buf.extend(data)
        except socket.timeout:
            return
        while len(self.buf) >= 2:
            count, multiplier, pos = 0, 1, 1
            while True:
                if pos >= len(self.buf):
                    return
                byte = self.buf[pos]
                count += (byte & 127) * multiplier
                pos += 1
                multiplier *= 128
                if not byte & 128:
                    break
            if len(self.buf) < pos + count:
                return
            header = self.buf[0]
            body = bytes(self.buf[pos:pos + count])
            del self.buf[:pos + count]
            event = {'time': time.time(), 'kind': header >> 4}
            if event['kind'] == 3:
                length = struct.unpack('!H', body[:2])[0]
                topic = body[2:2 + length].decode()
                offset = 2 + length + (2 if (header >> 1) & 3 else 0)
                payload = body[offset:]
                event.update(topic=topic, hex=payload.hex())
                self.capture.write(f"{event['time']:.6f} {topic} {payload.hex()}\n")
                self.capture.flush()
            self.events.append(event)

    def wait(self, predicate, timeout=5, start=0):
        deadline = time.monotonic() + timeout
        while True:
            for event in self.events[start:]:
                if predicate(event):
                    return event
            if time.monotonic() > deadline:
                raise TimeoutError('Expected MQTT response not received')
            self.pump()

    def command(self, code, body=b'\x00\x00', expected=0, split=False):
        self.request_id = (self.request_id + 1) & 255
        request_id = self.request_id
        start = len(self.events)
        self.publish(bytes([(code << 4) | 2, request_id]) + body, split)
        ack = self.wait(lambda e: e.get('topic') == 'is28/ack' and
                        len(bytes.fromhex(e['hex'])) == 4 and
                        bytes.fromhex(e['hex'])[:2] == bytes([(code << 4) | 2, request_id]), start=start)
        actual = bytes.fromhex(ack['hex'])[2]
        assert actual == expected, f'command {code} id {request_id}: result {actual}, expected {expected}'
        return request_id, start

    def mapping(self, word=None, expected=0, body=None, split=False):
        if body is None and word is not None:
            body = word.to_bytes(3, 'little')
        request_id, start = self.command(9 if body is not None else 10,
                                         body if body is not None else b'\x00\x00', expected, split)
        if expected:
            return None
        event = self.wait(lambda e: e.get('topic') == 'is28/tlm' and
                          len(bytes.fromhex(e['hex'])) == 8 and
                          bytes.fromhex(e['hex'])[0] == 0x82 and
                          bytes.fromhex(e['hex'])[7] == request_id, start=start)
        return int.from_bytes(bytes.fromhex(event['hex'])[3:7], 'little')

    def observe(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.pump()

    def close(self):
        self.send(0xe0, b'')
        self.sock.close()
        self.capture.close()


def word(letters):
    return sum((ord(c) - 65) << (5 * i) for i, c in enumerate(letters))


def run(client, checks):
    baseline = word('ZAQM')
    changed = word('ZAMQ')

    def check(name, condition=True, **detail):
        checks.append({'name': name, 'pass': bool(condition), **detail})
        print(('PASS ' if condition else 'FAIL ') + name, flush=True)
        assert condition, name

    client.command(4)
    client.command(5)
    snapshot = client.mapping(baseline)
    check('baseline mapping configured on Pico', snapshot & 0xfffff == baseline and snapshot & 0x100000 != 0)
    for name, body in [
        ('duplicate letter', word('ZAZM').to_bytes(3, 'little')),
        ('index outside A-Z', ((baseline & ~31) | 26).to_bytes(3, 'little')),
        ('reserved bit set', (baseline | 0x100000).to_bytes(3, 'little')),
        ('short mapping', baseline.to_bytes(3, 'little')[:2]),
        ('extra mapping byte', baseline.to_bytes(3, 'little') + b'\x00'),
        ('1024-byte mapping with valid prefix', baseline.to_bytes(3, 'little') + b'\x00' * 1019),
        ('split TCP writes of oversized mapping', changed.to_bytes(3, 'little') + b'\xff' * 1019),
    ]:
        client.mapping(body=body, expected=3, split=name.startswith('split'))
        check(name + ' rejected; full mapping/revision retained', client.mapping() == snapshot)

    try:
        client.command(7, b'\x00\x10')
        before = len(client.events)
        client.observe(3)
        observed = client.events[before:]
        check('telemetry off keeps heartbeat and suppresses ordinary frames',
              any(e.get('topic') == 'is28/hb' for e in observed) and
              not any(e.get('topic') == 'is28/tlm' for e in observed))
        updated = client.mapping(changed)
        check('SET and GET work while telemetry off', updated & 0xfffff == changed and client.mapping() == updated)
        check('identical SET while telemetry off retains revision', client.mapping(changed) == updated)
        snapshot = client.mapping(baseline)
        check('test mapping restored while telemetry off', client.mapping() == snapshot)
    finally:
        client.command(7, b'\x01\x10')
    client.command(8)
    client.command(5)
    check('mapping preserved through RESET', client.mapping() == snapshot)


def stress(client, results):
    snapshot = client.mapping()
    client.observe(2)

    def heartbeat(event):
        return event.get('topic') == 'is28/hb' and event.get('hex', '').startswith('120c')

    before = next(e for e in reversed(client.events) if heartbeat(e))
    before_bytes = bytes.fromhex(before['hex'])
    lost_before = int.from_bytes(before_bytes[9:11], 'little')
    start = len(client.events)
    # One finite burst of GET requests; no movement commands or new map.
    count = 1024
    body = b''.join(packet(0x30, text('is28/cmd') + bytes([0xa2, i & 255, 0, 0]))
                    for i in range(count))
    sent = time.time()
    client.sock.sendall(body)
    client.capture.write(f'{sent:.6f} BURST {count} BARCODE_GET requests\n')
    client.capture.flush()
    client.observe(8)
    burst_events = client.events[start:]
    hb = [e for e in burst_events if heartbeat(e)]
    assert hb, 'No heartbeat after finite burst'
    after = bytes.fromhex(hb[-1]['hex'])
    lost_after = int.from_bytes(after[9:11], 'little')
    delta = (lost_after - lost_before) & 65535
    acks = [e for e in burst_events if e.get('topic') == 'is28/ack']
    results['burst'] = {'sentRequests': count, 'acknowledgementsObserved': len(acks),
                        'lostBefore': lost_before, 'lostAfter': lost_after,
                        'lostDelta': delta, 'heartbeatsObserved': len(hb),
                        'queueFillAfterPct': after[14] & 127,
                        'limits': 'Shared lost counter combines incoming/outgoing queue and network drops; no attribution to a specific queue.'}
    assert delta > 0, 'No counted drops: saturation not established'
    client.command(8)
    retained = client.mapping() == snapshot
    assert retained, 'Mapping changed during read-only burst'
    results['checks'].append({'name': 'finite GET burst causes counted drops, heartbeat survives, queue recovers and mapping retained', 'pass': True})
    print('PASS finite burst: counted drops, responsive Ping/GET, mapping retained', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='store_true', help='explicitly run against real Pico')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--stress-only', action='store_true', help='finite 1024-request GET burst')
    args = parser.parse_args()
    if not args.run:
        parser.error('--run required: this sends commands to the real Pico')
    results = {'scope': 'Real Pico MQTT mapping edge cases', 'checks': [],
               'limits': 'No motor timing claim. Publisher TCP splits do not prove Pico callback boundaries. No queue-saturation claim.'}
    client = None
    try:
        client = Client(args.host, args.output.with_suffix('.txt'))
        if args.stress_only:
            results['scope'] = 'Real Pico finite MQTT request burst and queue recovery'
            results['limits'] = 'No motor timing or integrated-load claim; only this finite burst and shared counter observed.'
            stress(client, results)
        else:
            run(client, results['checks'])
        results['pass'] = True
    except Exception as error:
        results.update({'pass': False, 'error': str(error)})
        raise
    finally:
        args.output.with_suffix('.json').write_text(json.dumps(results, indent=2) + '\n')
        if client:
            client.close()
