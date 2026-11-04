"""Package Step5 IMU/ramp test, source/dashboard/test tools. No robot I/O."""
import hashlib,json,shutil,zipfile
from pathlib import Path
import package_motor_bringup as common
from package_line_following import linked_checks
ROOT=common.ROOT
CHECKPOINT=ROOT/'firmware/migration/micro_tkernel/step_05_imu'
ART=CHECKPOINT/'firmware/v0_1_2'
NAME='robopico_rtos_connected_imu_v0_1_2'
common.ART=ART
def main():
    ART.mkdir(parents=True,exist_ok=True)
    source=ROOT/'build_make'/(NAME+'.uf2');elf=source.with_suffix('.elf')
    layout=common.validate_uf2(source.read_bytes());linked=linked_checks(elf,network=True,barcode=True,imu=True)
    paths=set(common.sources())
    for folder in ('dashboard','tools/dashboard','firmware/integration/buddy1_4/tests/barcode','firmware/integration/buddy1_4/tests/imu','firmware/integration/buddy1_4/tests/ramp'):
        paths.update(p for p in (ROOT/folder).rglob('*') if p.is_file() and
            not any(s in common.EXCLUDED for s in p.parts) and p.suffix not in {'.pyc','.exe'})
    paths=sorted(paths);hashes={p.relative_to(ROOT).as_posix():common.digest(p) for p in paths}
    preserved,reviewed=common.verify_shared_sources(hashes)
    firmware=ART/source.name;shutil.copyfile(source,firmware)
    manifest={'name':NAME,'status':'EXPERIMENTAL Step5 Buddy1-4; Step4 v0.1.2 and Step5 physical acceptance pending; Buddy5 NOT integrated',
        'branch':common.command('git','branch','--show-current'),'source_commit':common.command('git','rev-parse','HEAD'),
        'baseline_tested_step3b':'ba7cfe08ef26775d3305108a75bb5b05978c7f48',
        'reference_main_commit':common.BASE,'build_profile':'APP=rtos_imu_wifi SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0',
        'uf2_sha256':common.digest(firmware),'uf2_bytes':firmware.stat().st_size,'uf2_layout':layout,
        'compiler':common.command(ROOT.parent/'.buddy2-tools/arm-gcc/bin/arm-none-eabi-gcc.exe','--version').splitlines()[0],
        'sdk_commit':common.command('git','-C',ROOT.parent/'.buddy2-tools/pico-sdk','rev-parse','HEAD'),
        'elf_sha256':common.digest(elf),
        'source_files':len(paths),'unchanged_shared_files_verified':len(preserved),'reviewed_shared_changes':reviewed,
        'calibration_priority':['CRC-valid RTOS saved0x1a9000','CRC-valid SDK record28 at0x1d0000','original fallback'],
        'expected_current_car_calibration':'white122,138 black246,267 origin1; read back actual car; no forced overwrite',
        'step4_parent':'78c42c050d6bbd464a3ceef255ce1ec597b60ada',
        'features':['core0 priority13 sole I2C1 GP2/3 owner; fixed20ms releases; original50Hz service and400Hz FIFO; copied odometry/tilt',
            'RTOS-only preemption-safe transfer completion/deadline recheck; bounded2ms per wait, abort-source diagnostics, no post-timeout queue writes',
            'full32bit lifetime acquisition counters separate failed reads, late gaps and FIFO overruns; wake/transfer maxima; legacy MQTT formats retained',
            'Step5 USB-only printf formats outside locks; bounded768-byte atomic ring records, whole-record drops, copy critical-section duration diagnostics',
            'recent IMU last/late gaps and10-second current/previous timing buckets; small bucket counters use9-bit fields; full counters remain lifetime',
            'stopped Reset reinitialises IMU even from Idle; calibration epoch and ready_epoch distinguish acceptance from calibration completion',
            'original detector6 ramp requests via Buddy4 nav API with50ms TTL; ordinary drive/limits retained',
            'source2/4 original binary packets, source15 ramp/C1 share sequence with peak; copied USB IMU/ramp diagnostics','unchanged reference63-bank + peak Code39 decisions; independent verified-frame midpoint junction classifier with5% hysteresis',
            'fresh4mm white re-arm after decode/Resume;2ms debounce;11..30mm stripe and4mm trailing white; confirmation-time visibility/search/clearance gate; explicit rejection flags; no300ms/3deg/min-wheel-speed dwell',
            'measured GP27 forward axle offset110mm; SDK/other steps keep92mm; track width/55mm rolling offset unchanged',
            'Buddy3 coordinator approval; frozen action/map revision at decode; next separate junction',
            'dashboard live A-Z mapping and decoded/queued/dispatch/completed events; separate source sequence ownership',
            'Continuous V2/profile1/line calibration/Step2 junction experiment retained',
            'kernel-only networking; TCP_OVERSIZE=0 SINGLE_PBUF=0; heap/delivery diagnostics',
            '640 copied100Hz line rows;2048 ten-byte raw rows frozen at first decode;192 reference barcode events',
            'stopped USB exports, mandatory USB DTR and network-loss stop/no automatic restart'],
        'deferred':['Buddy5','powerbank/untethered supervision (deferred until final integration)','dashboard stored-row downloads'],
        'software_checks':['36 strict C regression programs including actual USB formatter/ring bounds and nested producer integrity, recent timing buckets, RTOS transport failures, task/service Reset and barcode regression',
            'Python capture/export/provisioning checks plus pinned upstream TCP allocation experiment',
            'ARM Step4/Step5; source parity/UF2/stack/RAM gates; dashboard Step5 profile/IMU packet checks; network controls unchanged from Step4'],
        'hardware_checks':'Step4 barcode/dispatch user-reported working; L/R line rejoin needs sufficient mat. Step5 v0.1.1 passed reads/Reset but had9/10 late gaps in two settled minutes; v0.1.2 logging correction requires stationary retest before driving/ramp.',
        'linked_checks':linked}
    for name,data in [('BUILD_MANIFEST.json',manifest),('SOURCE_SHA256.json',hashes)]:
        (ART/name).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8',newline='\n')
    bundle=ART/(NAME+'_bundle.zip')
    with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in paths:z.write(p,p.relative_to(ROOT).as_posix())
        for p in (firmware,ART/'BUILD_MANIFEST.json',ART/'SOURCE_SHA256.json'):z.write(p,p.relative_to(ROOT).as_posix())
        z.write(firmware,'firmware/'+firmware.name)
        z.writestr('START_HERE.md',(CHECKPOINT/'README.md').read_text(encoding='utf-8').replace('../../../../app_program/','app_program/').replace('firmware/v0_1_2/','firmware/'))
        for title,script in [('Start dashboard.cmd','tools/dashboard/run_dashboard.py'),
            ('Setup Wi-Fi.cmd','tools/rtos_migration/provision_network.py'),
            ('Capture connected test.cmd','tools/rtos_migration/capture_connected_line.py'),
            ('Export line log.cmd','tools/rtos_migration/export_line_log.py'),
            ('Export barcode log.cmd','tools/rtos_migration/export_barcode_log.py')]:
            z.writestr(title,'@echo off\r\ncd /d "%~dp0"\r\npy -3 '+script.replace('/','\\')+'\r\npause\r\n')
    with zipfile.ZipFile(bundle) as z:
        assert z.testzip() is None
        assert z.read('firmware/'+firmware.name)==firmware.read_bytes()
        for name,digest in hashes.items():assert hashlib.sha256(z.read(name)).hexdigest()==digest,name
        assert not any(n.endswith('wifi_secrets.h') or '.local.' in n for n in z.namelist())
    (ART/'SHA256SUMS.txt').write_text(''.join(f'{common.digest(p)}  {p.name}\n' for p in [firmware,bundle]),encoding='utf-8',newline='\n')
    print(json.dumps({'uf2':str(firmware),'bundle':str(bundle),'uf2_sha256':common.digest(firmware),
        'bundle_sha256':common.digest(bundle),'bundle_bytes':bundle.stat().st_size,
        'static_ram_end':linked['static_ram_end'],'ram_remaining':linked['ram_remaining_before_dynamic_kernel_stacks']},indent=2))
if __name__=='__main__':main()
