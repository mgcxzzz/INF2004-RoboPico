"""Read-only Step3B USB timing + MQTT frames. Dashboard supplies test commands."""
from __future__ import annotations
import argparse, csv, datetime as dt, json, socket, sys, threading, time, uuid
from pathlib import Path
from capture_bringup import parse_line, analyze as analyze_clock
from capture_network_bench import heartbeat
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'dashboard'))
from check_mqtt import mqtt_connect_packet, subscribe_packet, read_packet, DiagnosticError


def summarize(rows, frames, interruptions):
    clock = analyze_clock(rows, allow_active=True)
    beats = [dict(wall_s=f['wall_s'], **b) for f in frames
             if f['topic']=='is28/hb' and (b := heartbeat(bytes.fromhex(f['hex']))) is not None]
    gaps = [b['wall_s']-a['wall_s'] for a,b in zip(beats,beats[1:])]
    motion = [f for f in frames if f['topic']=='is28/tlm' and
              len(p := bytes.fromhex(f['hex']))==48 and p[0]==0x92]
    net = [r for r in rows if r['kind']=='NET']
    delivery = [r for r in rows if r['kind']=='NET_TX']
    guard = [r for r in rows if r['kind']=='ENCODER_GUARD']
    memory = [r for r in rows if r['kind']=='NET_MEMORY']
    return {'result':'REVIEW_PHYSICAL_TEST', 'clock':clock, 'heartbeats':len(beats),
            'max_heartbeat_gap_s':max(gaps,default=None), 'motion_frames':len(motion),
            'network_first':net[0] if net else None, 'network_last':net[-1] if net else None,
            'delivery_first':delivery[0] if delivery else None,
            'delivery_last':delivery[-1] if delivery else None,
            'encoder_guard_last':guard[-1] if guard else None,
            'encoder_trips':[r for r in rows if r['kind']=='ENCODER_TRIP'],
            'memory_first':memory[0] if memory else None,'memory_last':memory[-1] if memory else None,
            'barcode_decodes':[r for r in rows if r['kind']=='BARCODE_DECODE'],
            'monitor_interruptions':interruptions,
            'scope':'Clock and passive receive evidence only. Review video and stopped USB line log for tracking/stops; intentional outage gaps are not a baseline heartbeat pass.'}


def monitor(broker, stop, frames, interruptions, started):
    while not stop.is_set():
        try:
            with socket.create_connection((broker,1883),timeout=3) as sock:
                sock.sendall(mqtt_connect_packet('rtos-drive-review-'+uuid.uuid4().hex[:8]))
                header,body=read_packet(sock,3)
                if header!=0x20 or body!=b'\0\0': raise ValueError('MQTT monitor refused')
                sock.sendall(subscribe_packet(topic='is28/#'))
                last_ping=time.monotonic()
                while not stop.is_set():
                    if time.monotonic()-last_ping>20:
                        sock.sendall(b'\xc0\0'); last_ping=time.monotonic()
                    # Preserve the passive observer through short stalls. The
                    # measured heartbeat gap still includes every silent second;
                    # this changes no car/dashboard keepalive or acceptance limit.
                    header,body=read_packet(sock,6)
                    if header>>4!=3 or len(body)<2: continue
                    n=int.from_bytes(body[:2],'big')
                    if n>len(body)-2: raise ValueError('Truncated MQTT topic')
                    frames.append({'wall_s':round(time.monotonic()-started,6),
                        'topic':body[2:2+n].decode('utf-8',errors='replace'),
                        'hex':body[2+n:].hex(), 'retained':bool(header&1)})
        except (OSError,ValueError,DiagnosticError) as error:
            if not stop.is_set():
                interruptions.append({'wall_s':round(time.monotonic()-started,6),'error':str(error)})
                stop.wait(1) # Read-only reconnect; never sends a robot command.


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port');p.add_argument('--broker',default='127.0.0.1')
    p.add_argument('--seconds',type=float,default=300)
    p.add_argument('--output',type=Path,default=Path('rtos_connected_session'))
    a=p.parse_args()
    if a.seconds<30:p.error('Capture at least30seconds.')
    try:
        import serial
        import serial.tools.list_ports
    except ImportError:raise SystemExit('Install once: py -3 -m pip install pyserial')
    if not a.port:
        for item in serial.tools.list_ports.comports():print(item.device,item.description)
        a.port=input('Pico port (COM4 or full /dev/cu...): ').strip()
    if a.port.isdecimal():a.port='COM'+a.port
    a.output.mkdir(parents=True,exist_ok=True)
    prefix=a.output/dt.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    print('Close PuTTY; keep USB attached. Keep broker/dashboard open.')
    print('This capture holds USB supervision open. Use dashboard commands for the test.')
    print('It sends NO Start/Stop/calibration commands. Closing it ends USB supervision and stops motion.')
    print(f'Capture runs for {a.seconds:g} seconds. Press dashboard Start near the beginning, not at the end.')
    frames,rows,interruptions=[],[],[];stop=threading.Event();started=time.monotonic()
    worker=threading.Thread(target=monitor,args=(a.broker,stop,frames,interruptions,started),daemon=True)
    worker.start()
    try:
        with serial.Serial(a.port,115200,timeout=.2) as port, prefix.with_name(prefix.name+'_usb.txt').open('w',encoding='utf-8') as raw:
            while time.monotonic()-started<a.seconds:
                line=port.readline().decode('utf-8',errors='replace').strip()
                if not line:continue
                wall=time.monotonic()-started;raw.write(f'{wall:.6f}\t{line}\n');raw.flush()
                kind,values=parse_line(line)
                if wall>=2:rows.append({'wall_s':wall,'kind':kind,**values})
                if kind in ('CONTROL','NET_COMMAND','STOP','CLOCK'):print(line)
    except (serial.SerialException,KeyboardInterrupt) as error:
        interruptions.append({'wall_s':time.monotonic()-started,'error':'USB capture ended: '+str(error)})
    finally:
        stop.set();worker.join(7)
    print('USB SERIAL SUPERVISION IS NOW CLOSED. To Start again, reopen capture or PuTTY.')
    with prefix.with_name(prefix.name+'_mqtt.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.DictWriter(f,fieldnames=['wall_s','topic','hex','retained']);writer.writeheader();writer.writerows(frames)
    result=summarize(rows,frames,interruptions)
    prefix.with_name(prefix.name+'_summary.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    print(json.dumps(result,indent=2));print('Send all three files from',a.output.resolve())

if __name__=='__main__':main()
