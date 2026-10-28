"""Configure the Pico over USB. Password is entered locally and never printed."""
from __future__ import annotations
import argparse,getpass,ipaddress,json,time
from pathlib import Path

def configuration_line(ssid:str,password:str,broker:str)->bytes:
    fields=[ssid,password,broker]
    if any(any(c in v for c in '|\r\n\0') for v in fields):raise ValueError('Settings cannot contain pipe, newline or NUL characters.')
    if not ssid or len(ssid.encode('utf-8'))>32:raise ValueError('SSID must contain 1..32 UTF-8 bytes.')
    if len(password.encode('utf-8'))>64:raise ValueError('Password exceeds 64 UTF-8 bytes.')
    address=ipaddress.IPv4Address(broker)
    if address.is_unspecified or address.is_multicast or address.is_loopback:raise ValueError('Use the laptop LAN IPv4 address, not localhost.')
    return ('!net|'+ssid+'|'+password+'|'+str(address)+'\n').encode('utf-8')

def wait_for_confirmation(port,serial_error,*,timeout=8,clock=time.monotonic)->None:
    """Consume the short ACK before the intentional USB reboot.

    Windows can discard a partial multi-byte read when the device resets.
    Read one byte at a time and never infer success merely from disconnect.
    """
    deadline=clock()+timeout;buffer=b'';rejected=False
    while clock()<deadline:
        try:buffer=(buffer+port.read(1))[-1024:]
        except serial_error:
            if rejected:raise SystemExit('Setup rejected by Pico; USB disconnected before its full reason was received. Reopen PuTTY and press n for WIFI_SETUP.') from None
            raise SystemExit('USB disconnected while waiting for confirmation. The Pico may have rebooted after saving. Wait a few seconds, reopen PuTTY and press n: configured=1 means settings were loaded. Success was not confirmed by this helper; do not reflash just for this error.') from None
        if b'WIFI_SETUP_REJECTED:' in buffer:
            rejected=True
            detail=buffer.split(b'WIFI_SETUP_REJECTED:',1)[1]
            if b'\n' in detail:
                # Firmware sends only a reason code here; never echo config data.
                import re
                match=re.fullmatch(rb'\s*code=(\d{1,3}) reason=([A-Za-z0-9_]+)\s*',detail.split(b'\n',1)[0])
                reason=('code='+match[1].decode()+' reason='+match[2].decode()) if match else 'unspecified (press n in PuTTY for status)'
                raise SystemExit('Setup rejected by Pico: '+reason)
        if b'WIFI_PROVISIONED:' in buffer:return
    if rejected:raise SystemExit('Setup rejected by Pico; incomplete reason. Press n in PuTTY for WIFI_SETUP.')
    raise SystemExit('No verified setup response. Check the port and firmware banner, then retry. No credentials were printed.')

def main()->None:
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',help='Pico COM port, for example COM7')
    parser.add_argument('--broker',help='Laptop LAN IPv4 address')
    parser.add_argument('--settings',type=Path,help='Optional local JSON with ssid/password/broker; keep it gitignored')
    args=parser.parse_args()
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as exc:
        raise SystemExit('Install the USB helper once: py -m pip install pyserial') from exc
    values=json.loads(args.settings.read_text(encoding='utf-8')) if args.settings else {}
    if not args.port:
        for p in list_ports.comports():print(f'{p.device}: {p.description}')
        args.port=input('Pico COM port: ').strip()
    ssid=values.get('ssid') or input('Wi-Fi / 2.4 GHz hotspot name: ')
    password=values.get('password') if 'password' in values else getpass.getpass('Wi-Fi password (hidden; Enter for open network): ')
    broker=args.broker or values.get('broker') or input('Laptop broker LAN IPv4 address: ').strip()
    line=configuration_line(ssid,password,broker)
    print('Close PuTTY first. Robot must be stopped, buttons released, and its run saved.')
    with serial.Serial(args.port,115200,timeout=.2,write_timeout=2) as port:
        time.sleep(1);port.reset_input_buffer();port.write(b's');port.flush();time.sleep(5)
        port.write(line);port.flush();wait_for_confirmation(port,serial.SerialException)
        print('Settings verified on the Pico. It is rebooting; open PuTTY again and press n to check Wi-Fi.')

if __name__=='__main__':main()
