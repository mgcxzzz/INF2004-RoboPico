import binascii,struct,tempfile,unittest
from pathlib import Path
from provision_utk_wifi import settings_page,settings_uf2,bootsel_drive,SETTINGS_ADDRESS

class ProvisionTests(unittest.TestCase):
    def test_wcf1_layout_crc_and_utf8(self):
        p=settings_page('TEST-NET','test-password','10.236.162.47')
        self.assertEqual(p[:4],b'WCF1');self.assertEqual(p[8:16],b'TEST-NET')
        self.assertEqual(p[106:122].split(b'\0',1)[0],b'10.236.162.47')
        self.assertEqual(struct.unpack_from('<I',p,4)[0],binascii.crc32(p[8:])&0xffffffff)
        self.assertEqual(len(settings_page('test-\u00e9','','192.168.1.2')),256)
    def test_full_aligned_sector_only(self):
        page=settings_page('test','','192.168.1.2');uf2=settings_uf2(page);self.assertEqual(len(uf2),8192)
        for i in range(16):
            b=uf2[i*512:(i+1)*512];a,c,f,addr,n,index,total,family=struct.unpack_from('<8I',b)
            self.assertEqual((a,c,f,addr,n,index,total,family),(0x0a324655,0x9e5d5157,0x2000,SETTINGS_ADDRESS+i*256,256,i,16,0xe48bff56))
            self.assertEqual(struct.unpack_from('<I',b,508)[0],0x0ab16f30)
            self.assertEqual(b[32:288],page if i==0 else b'\xff'*256)
    def test_invalid_fields_and_corrupt_page(self):
        for ssid,pw,ip in [('', '', '192.168.1.2'),('x'*33,'','192.168.1.2'),('x','x'*65,'192.168.1.2'),('x','bad\n','192.168.1.2'),('x','','127.0.0.1'),('x','','0.0.0.0')]:
            with self.assertRaises(ValueError):settings_page(ssid,pw,ip)
        p=bytearray(settings_page('x','','192.168.1.2'));p[200]^=1
        with self.assertRaises(ValueError):settings_uf2(p)
    def test_only_rp2040_bootsel_mount_accepted(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);self.assertFalse(bootsel_drive(p))
            (p/'INFO_UF2.TXT').write_text('UF2 Bootloader v1\nBoard-ID: RP2350\n');self.assertFalse(bootsel_drive(p))
            (p/'INFO_UF2.TXT').write_text('UF2 Bootloader v1\nBoard-ID: RPI-RP2\n');self.assertTrue(bootsel_drive(p))

if __name__=='__main__':unittest.main()
