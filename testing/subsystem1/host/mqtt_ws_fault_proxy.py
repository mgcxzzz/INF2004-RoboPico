#!/usr/bin/env python3
"""Local test-only WebSocket proxy, 9002 -> 9001.

When the arm file exists, removes complete source-8 mapping MQTT PUBLISH
packets from uncompressed binary server frames. Other traffic is forwarded.
Packet boundaries spanning WebSocket frames are forwarded without filtering;
the event log must show an actual drop before claiming a test result.
"""
import argparse
import asyncio
import json
import struct
import time


def mqtt_packets(data):
    packets = []
    offset = 0
    while offset < len(data):
        pos, size, mult = offset + 1, 0, 1
        while True:
            if pos >= len(data) or mult > 128 ** 3:
                return None
            byte = data[pos]
            pos += 1
            size += (byte & 127) * mult
            mult *= 128
            if not byte & 128:
                break
        end = pos + size
        if end > len(data):
            return None
        raw = data[offset:end]
        mapping = False
        if data[offset] >> 4 == 3 and size >= 2:
            length = int.from_bytes(data[pos:pos + 2], 'big')
            topic = data[pos + 2:pos + 2 + length]
            body = pos + 2 + length + (2 if (data[offset] >> 1) & 3 else 0)
            mapping = topic == b'is28/tlm' and data[body:end].startswith(b'\x82\x05')
        packets.append((raw, mapping))
        offset = end
    return packets


async def main(args):
    def log(event, **fields):
        with open(args.log, 'a') as output:
            output.write(json.dumps({'epoch': time.time(), 'event': event, **fields}) + '\n')

    async def connection(reader, writer):
        upstream_reader, upstream = await asyncio.open_connection('127.0.0.1', 9001)
        log('client_connected')

        async def client_to_broker():
            while data := await reader.read(65536):
                upstream.write(data)
                await upstream.drain()

        async def broker_to_client():
            response = await upstream_reader.readuntil(b'\r\n\r\n')
            writer.write(response)
            await writer.drain()
            while True:
                header = await upstream_reader.readexactly(2)
                length = header[1] & 127
                extension = b''
                if length in (126, 127):
                    extension = await upstream_reader.readexactly(2 if length == 126 else 8)
                    length = int.from_bytes(extension, 'big')
                mask = await upstream_reader.readexactly(4) if header[1] & 128 else b''
                payload = await upstream_reader.readexactly(length)
                # No filtering unless this is a complete, uncompressed,
                # unmasked binary frame containing whole MQTT packets.
                if header[0] == 0x82 and not mask and args.arm.exists():
                    packets = mqtt_packets(payload)
                    if packets is not None:
                        kept = []
                        for raw, mapping in packets:
                            if mapping:
                                log('mapping_readback_dropped', mqttHex=raw.hex())
                            else:
                                kept.append(raw)
                        changed = b''.join(kept)
                        if len(changed) != len(payload):
                            if not changed:
                                continue
                            payload = changed
                            size = len(payload)
                            header = bytes([0x82, size if size < 126 else 126 if size < 65536 else 127])
                            extension = b'' if size < 126 else struct.pack('!H' if size < 65536 else '!Q', size)
                writer.write(header + extension + mask + payload)
                await writer.drain()

        tasks = [asyncio.create_task(client_to_broker()), asyncio.create_task(broker_to_client())]
        try:
            done, pending = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in done:
                try:
                    task.result()
                except (asyncio.IncompleteReadError, ConnectionError) as error:
                    log('connection_ended', reason=type(error).__name__)
            for task in pending:
                task.cancel()
        finally:
            upstream.close()
            writer.close()

    server = await asyncio.start_server(connection, '127.0.0.1', 9002)
    log('proxy_listening', listen='127.0.0.1:9002', upstream='127.0.0.1:9001')
    print('Test proxy listening on 127.0.0.1:9002', flush=True)
    async with server:
        await server.serve_forever()


if __name__ == '__main__':
    from pathlib import Path
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--arm', type=Path, required=True)
    parser.add_argument('--log', type=Path, required=True)
    asyncio.run(main(parser.parse_args()))
