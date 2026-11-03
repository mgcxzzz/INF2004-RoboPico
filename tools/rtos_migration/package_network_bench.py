"""Package the public, motor-disabled Step3A checkpoint; never reads credentials."""
from pathlib import Path
import hashlib, json, re, shutil, zipfile
import package_motor_bringup as common

ROOT = common.ROOT
CHECKPOINT = ROOT / 'firmware/migration/micro_tkernel/step_03_wifi_mqtt'
ART = CHECKPOINT / 'firmware'
NAME = 'robopico_rtos_network_bench_v0_1'
common.ART = ART


def main():
    ART.mkdir(parents=True, exist_ok=True)
    source = ROOT / 'build_make' / (NAME + '.uf2')
    elf = source.with_suffix('.elf')
    layout = common.validate_uf2(source.read_bytes())
    tool = ROOT.parent / '.buddy2-tools/arm-gcc/bin/arm-none-eabi-nm.exe'
    listing = common.command(tool, '-n', elf)
    symbols = {c[2]: int(c[0],16) for line in listing.splitlines()
               if len(c := line.split()) == 3 and re.fullmatch('[0-9a-fA-F]+',c[0])}
    for name in ['usermain','bringup_control_task','bringup_safety_task','rn_app_task',
                 'rn_tx_take','rn_rx_push','mqtt_utk_start','cyw43_utk_task','motion_stop',
                 'bringup_stop_from','mqtt_client_connect']:
        assert name in symbols,name
    for name in ['rtos_line_init','rl_flash_write','board_line_save','barcode_analog_sample',
                 'barcode_peak_sample','subsystem5_start','dev_init_adc','dev_init_ser']:
        assert name not in symbols,name
    assert symbols['__end'] < 0x20030000
    assembly = common.command(tool.with_name('arm-none-eabi-objdump.exe'), '-d', elf)
    match = re.search(r'<apply_outputs>:\n(.*?)(?=\n[0-9a-f]+ <)', assembly, re.S)
    assert match and 'board_motor_off' in match[1] and 'board_motor_write' not in match[1]
    paths = set(common.sources())
    for folder in ['dashboard','tools/dashboard']:
        paths.update(p for p in (ROOT / folder).rglob('*') if p.is_file() and
                     not any(s in common.EXCLUDED for s in p.parts) and p.suffix not in {'.pyc','.exe'})
    paths = sorted(paths)
    hashes = {p.relative_to(ROOT).as_posix():common.digest(p) for p in paths}
    preserved,reviewed = common.verify_shared_sources(hashes)
    firmware = ART / source.name
    shutil.copyfile(source,firmware)
    manifest = {'name':NAME,'status':'EXPERIMENTAL Step3A motor-disabled network bench; physical acceptance pending',
        'branch':common.command('git','branch','--show-current'),'source_commit':common.command('git','rev-parse','HEAD'),
        'reference_main_commit':common.BASE,'previous_step2_commit':'49244d256b80cf64c10fbec750c8531a0a58907c',
        'build_profile':'APP=rtos_network SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0',
        'compiler':common.command(tool.with_name('arm-none-eabi-gcc.exe'),'--version').splitlines()[0],
        'uf2_sha256':common.digest(firmware),'uf2_bytes':firmware.stat().st_size,'uf2_layout':layout,
        'elf_sha256':common.digest(elf),'source_files':len(paths),'unchanged_shared_files_verified':len(preserved),
        'reviewed_shared_changes':reviewed,'static_ram_end':hex(symbols['__end']),
        'ram_remaining_before_dynamic_kernel_stacks':0x20040000-symbols['__end'],
        'motor_gate':'apply_outputs linked only to board_motor_off; USB starts rejected; MQTT Start/Resume unsupported',
        'settings':'WCF1 at0x1aa000, BOOTSEL settings-only4KiB sector; no live calibration/flash writes',
        'software_checks':['19 strict C programs','47 Python checks','Step1/Step2/Step3A ARM builds',
                           'UF2 family/bounds, source audit, required/forbidden symbols and output-gate disassembly'],
        'hardware_checks':'No hardware claim; user must verify clock and actual received heartbeat timing, commands and broker reconnect.'}
    for name,data in [('SOURCE_SHA256.json',hashes),('BUILD_MANIFEST.json',manifest)]:
        (ART/name).write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8',newline='\n')
    bundle = ART/(NAME+'_bundle.zip')
    with zipfile.ZipFile(bundle,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
        for p in paths: z.write(p,p.relative_to(ROOT).as_posix())
        for p in [firmware,ART/'SOURCE_SHA256.json',ART/'BUILD_MANIFEST.json']:
            z.write(p,p.relative_to(ROOT).as_posix())
        z.write(firmware,'firmware/'+firmware.name)
        z.writestr('START_HERE.md','# Step3A network bench\n\nMotors disabled. Extract this bundle first.\n\nFollow [setup and exact tests](TEST_NETWORK.md). Keep v0.1.2 for tomorrow\'s mat tests.\n')
        z.writestr('TEST_NETWORK.md',(CHECKPOINT/'README.md').read_text(encoding='utf-8'))
        for name,script in [('Start dashboard.cmd','tools/dashboard/run_dashboard.py'),
                            ('Setup Wi-Fi.cmd','tools/rtos_migration/provision_network.py'),
                            ('Capture network test.cmd','tools/rtos_migration/capture_network_bench.py')]:
            z.writestr(name,'@echo off\r\ncd /d "%~dp0"\r\npy -3 '+script.replace('/','\\')+'\r\npause\r\n')
    with zipfile.ZipFile(bundle) as z:
        assert z.testzip() is None
        for name,digest in hashes.items(): assert hashlib.sha256(z.read(name)).hexdigest()==digest,name
        assert z.read('firmware/'+firmware.name)==firmware.read_bytes()
    (ART/'SHA256SUMS.txt').write_text(''.join(f'{common.digest(p)}  {p.name}\n' for p in [firmware,bundle]),encoding='utf-8',newline='\n')
    print(json.dumps({'uf2':str(firmware),'bundle':str(bundle),'uf2_sha256':common.digest(firmware),
                      'bundle_sha256':common.digest(bundle),'bundle_bytes':bundle.stat().st_size,**layout},indent=2))


if __name__=='__main__':main()
