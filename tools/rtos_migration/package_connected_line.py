"""Package the public Step3B firmware, source, dashboard and exact test tools."""
import hashlib, json, shutil, zipfile
import package_motor_bringup as common
from package_line_following import linked_checks
ROOT=common.ROOT
CHECKPOINT=ROOT/'firmware/migration/micro_tkernel/step_03_connected_line'
ART=CHECKPOINT/'firmware/v0_1_2'
NAME='robopico_rtos_connected_line_v0_1_2'
common.ART=ART

def main():
    ART.mkdir(parents=True,exist_ok=True)
    source=ROOT/'build_make'/(NAME+'.uf2');elf=source.with_suffix('.elf')
    layout=common.validate_uf2(source.read_bytes());linked=linked_checks(elf,network=True)
    paths=set(common.sources())
    for folder in ['dashboard','tools/dashboard']:
        paths.update(p for p in (ROOT/folder).rglob('*') if p.is_file() and
            not any(s in common.EXCLUDED for s in p.parts) and p.suffix not in {'.pyc','.exe'})
    paths=sorted(paths)
    hashes={p.relative_to(ROOT).as_posix():common.digest(p) for p in paths}
    preserved,reviewed=common.verify_shared_sources(hashes)
    firmware=ART/source.name;shutil.copyfile(source,firmware)
    manifest={'name':NAME,'status':'EXPERIMENTAL Step3B USB-supervised connected line; physical acceptance pending; not final Buddy1-5',
        'branch':common.command('git','branch','--show-current'),
        'source_commit':common.command('git','rev-parse','HEAD'),'reference_main_commit':common.BASE,
        'step2_firmware_reference':'49244d256b80cf64c10fbec750c8531a0a58907c',
        'step3a_acceptance_reference':'8866729',
        'build_profile':'APP=rtos_line_wifi SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0',
        'compiler':common.command(ROOT.parent/'.buddy2-tools/arm-gcc/bin/arm-none-eabi-gcc.exe','--version').splitlines()[0],
        'uf2_sha256':common.digest(firmware),'uf2_bytes':firmware.stat().st_size,'uf2_layout':layout,
        'elf_sha256':common.digest(elf),'source_files':len(paths),
        'unchanged_shared_files_verified':len(preserved),'reviewed_shared_changes':reviewed,
        'calibration_priority':['CRC-valid RTOS saved0x1a9000','CRC-valid SDK record28 at0x1d0000','original163/160/3457/3700 fallback'],
        'last_user_hardware_calibration':'white122,138 black246,267 importedSDK origin1; read back actual car before driving; no forced overwrite',
        'features':['unchanged Continuous V2 profile1 and Step2 angle-only junction experiment',
            'recent encoder burst/density guard instead of lifetime8 errors; original reverse-feedback guard retained',
            'MQTT16 requests/1024-byte ring, Nagle disabled, ACK/HB reserved capacity, latest motion/nav snapshots',
            'exact-size TCP pbuf allocation; copied NET_MEMORY heap/TCP diagnostics; NET_TX and encoder guard diagnostics',
            'kernel-only lwIP/MQTT with copied bounded TMO_POL buffers','dashboard Start2s/Stop/E-stop/Pause/qualified Resume/Reset',
            'mandatory USB DTR and reported network/poll-loss stop; no automatic restart',
            '20Hz motion +4Hz coordinator +1Hz heartbeat; RAM mapping readback',
            'stopped USB100Hz1536-row history export'],
        'deferred':['barcode decoders/runtime mapped junction actions Step4','IMU/ramp assist Step5','Buddy5','dashboard stored-row download','untethered driving acceptance'],
        'software_checks':['25 strict C regression programs including barcode software tests; barcode remains disabled in this comparison image',
            '55 Python capture/export/provisioning checks plus pinned upstream TCP allocation experiment','27 dashboard protocol compatibility checks and 53 browser simulation assertions',
            'ARM Step2/Step3A/Step3B builds, UF2 bounds, baseline source audit, RAM flash-code/static stack checks'],
        'hardware_checks':'User reports Step3B v0.1.1 motion/safety checks passed. New network correction pending; extended-mat L/R and sharp bends pending.',
        'linked_checks':linked}
    for name,data in [('SOURCE_SHA256.json',hashes),('BUILD_MANIFEST.json',manifest)]:
        (ART/name).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8',newline='\n')
    bundle=ART/(NAME+'_bundle.zip')
    with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in paths:z.write(p,p.relative_to(ROOT).as_posix())
        for p in [firmware,ART/'SOURCE_SHA256.json',ART/'BUILD_MANIFEST.json']:z.write(p,p.relative_to(ROOT).as_posix())
        z.write(firmware,'firmware/'+firmware.name)
        z.writestr('START_HERE.md','# Step3B connected line v0.1.2\n\nExtract first. Flash firmware/'+NAME+'.uf2. Read [UPDATE_V0_1_2.md](UPDATE_V0_1_2.md) for the exact encoder/network retests and log export. The complete setup is in [TEST_CONNECTED_LINE.md](TEST_CONNECTED_LINE.md). USB serial supervision is mandatory; calibration/Wi-Fi sectors are retained. Barcode/IMU remain later steps.\n')
        z.writestr('UPDATE_V0_1_2.md',(CHECKPOINT/'UPDATE_V0_1_2.md').read_text(encoding='utf-8'))
        z.writestr('TEST_CONNECTED_LINE.md',(CHECKPOINT/'README.md').read_text(encoding='utf-8'))
        for name,script in [('Start dashboard.cmd','tools/dashboard/run_dashboard.py'),
                            ('Setup Wi-Fi.cmd','tools/rtos_migration/provision_network.py'),
                            ('Capture connected test.cmd','tools/rtos_migration/capture_connected_line.py'),
                            ('Export line log.cmd','tools/rtos_migration/export_line_log.py')]:
            z.writestr(name,'@echo off\r\ncd /d "%~dp0"\r\npy -3 '+script.replace('/','\\')+'\r\npause\r\n')
    with zipfile.ZipFile(bundle) as z:
        assert z.testzip() is None
        for name,digest in hashes.items():assert hashlib.sha256(z.read(name)).hexdigest()==digest,name
        assert z.read('firmware/'+firmware.name)==firmware.read_bytes()
    (ART/'SHA256SUMS.txt').write_text(''.join(f'{common.digest(p)}  {p.name}\n' for p in [firmware,bundle]),encoding='utf-8',newline='\n')
    print(json.dumps({'uf2':str(firmware),'bundle':str(bundle),'uf2_sha256':common.digest(firmware),
        'bundle_sha256':common.digest(bundle),'bundle_bytes':bundle.stat().st_size,**layout,**linked},indent=2))

if __name__=='__main__':main()
