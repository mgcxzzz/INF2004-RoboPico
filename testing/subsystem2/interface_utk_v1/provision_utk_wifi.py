"""Write only the WCF1 settings sector through the RP2040 BOOTSEL drive.

No live RTOS flash write, serial port, pyserial, or stored password file.
The aligned full sector avoids RP2040-E14 sparse/partial-sector behaviour.
"""
from __future__ import annotations
import argparse,binascii,ctypes,getpass,ipaddress,os,struct,string,time
from pathlib import Path

SETTINGS_ADDRESS=0x101AA000
FAMILY=0xE48BFF56

def settings_page(ssid:str,password:str,broker:str)->bytes:
    address=ipaddress.IPv4Address(broker)
    if address.is_unspecified or address.is_loopback or address.is_multicast:
        raise ValueError('Use the laptop Wi-Fi LAN IPv4, not localhost or a gateway example.')
    fields=[(ssid,33,False),(password,65,True),(str(address),16,False),('is28-utk-network-v1',32,False)]
    encoded=bytearray()
    for text,capacity,empty in fields:
        raw=text.encode('utf-8')
        if (not raw and not empty) or len(raw)>=capacity or any(c<32 or c==124 for c in raw):
            raise ValueError('A setting is empty, too long, or contains a control/pipe character.')
        encoded.extend(raw+b'\0'*(capacity-len(raw)))
    page=bytearray(b'\xff'*256);page[:4]=b'WCF1';page[8:8+len(encoded)]=encoded
    struct.pack_into('<I',page,4,binascii.crc32(page[8:])&0xffffffff)
    return bytes(page)

def settings_uf2(page:bytes)->bytes:
    if len(page)!=256 or page[:4]!=b'WCF1' or struct.unpack_from('<I',page,4)[0]!=(binascii.crc32(page[8:])&0xffffffff):
        raise ValueError('Invalid WCF1 page')
    sector=page+b'\xff'*(4096-len(page));blocks=[]
    for i in range(16):
        block=bytearray(512)
        struct.pack_into('<8I',block,0,0x0a324655,0x9e5d5157,0x2000,SETTINGS_ADDRESS+i*256,256,i,16,FAMILY)
        block[32:288]=sector[i*256:(i+1)*256]
        struct.pack_into('<I',block,508,0x0ab16f30);blocks.append(bytes(block))
    return b''.join(blocks)

def bootsel_drive(path:Path)->bool:
    try:
        text=(path/'INFO_UF2.TXT').read_text(encoding='utf-8',errors='replace')
        return 'Board-ID: RPI-RP2' in text and 'UF2 Bootloader' in text
    except OSError:return False

def find_drives()->list[Path]:
    if os.name=='nt':
        mask=ctypes.windll.kernel32.GetLogicalDrives()
        roots=[Path(letter+':/') for i,letter in enumerate(string.ascii_uppercase) if mask&(1<<i)]
    else:
        roots=[p for parent in (Path('/media')/getpass.getuser(),Path('/run/media')/getpass.getuser(),Path('/Volumes'))
               if parent.exists() for p in parent.iterdir()]
    return [p for p in roots if bootsel_drive(p)]

def main()->None:
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--drive',type=Path)
    args=parser.parse_args()
    print('UTK network setup. First flash buddy123_utk_network_v1.uf2. Keep the robot stopped.')
    ssid=input('2.4 GHz Wi-Fi / hotspot name: ')
    password=getpass.getpass('Wi-Fi password (hidden; Enter for open network): ')
    broker=input('Laptop broker LAN IPv4 address: ').strip()
    try:payload=settings_uf2(settings_page(ssid,password,broker))
    except (ValueError,UnicodeError):raise SystemExit('Invalid settings: check IPv4 and field lengths. Nothing written.') from None
    print('Close PuTTY. Disconnect power, hold BOOTSEL while reconnecting Pico USB, then release BOOTSEL.')
    input('When RPI-RP2 appears in File Explorer, press Enter here: ')
    drives=[args.drive] if args.drive else find_drives()
    if len(drives)!=1 or not bootsel_drive(drives[0]):
        raise SystemExit('Expected exactly one RP2040 RPI-RP2 drive. Reconnect in BOOTSEL, or rerun with --drive E:/ using its actual letter.')
    try:
        with (drives[0]/'NETWORK.UF2').open('wb',buffering=0) as file:
            count=file.write(payload)
            if count!=len(payload):raise OSError('short UF2 write')
    except OSError:
        raise SystemExit('BOOTSEL write was not confirmed. Check the Pico console after reboot; settings may or may not have been applied. No password was printed.') from None
    print('Settings UF2 sent. Pico should reboot. This is not a device readback confirmation.')
    print('Reopen PuTTY at 115200 and look for cfg=1, the new broker IP, then mqtt: Operational.')
    print('No private UF2 or password file was saved on the laptop. You may close this window.')

if __name__=='__main__':main()
