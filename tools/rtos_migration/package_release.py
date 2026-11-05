"""Package the provisional main Buddy1-4 RTOS release; no robot or credential I/O."""
import hashlib
import json
import shutil
import subprocess
import zipfile
from pathlib import Path

import package_motor_bringup as common
from package_line_following import linked_checks

ROOT = common.ROOT
CORE = ROOT / 'firmware/integration/buddy1_4'
ART = CORE / 'artifacts/v0_2_0_rtos'
NAME = 'robopico_buddy1_4_rtos_v0_2_0'
TESTED = '75325f29ad604c75a9d9ab8111658aa1536543d5'
common.ART = ART


def collect_sources():
    paths = set(common.sources())
    for folder in ('dashboard', 'tools/dashboard', 'firmware/integration/buddy1_4/tests/imu',
                   'firmware/integration/buddy1_4/tests/ramp', 'testing/evidence/rtos_buddy1_4',
                   'firmware/extensions'):
        paths.update(p for p in (ROOT / folder).rglob('*') if p.is_file()
                     and not any(s in common.EXCLUDED for s in p.parts)
                     and p.suffix not in {'.pyc', '.exe', '.zip'}
                     and p.name != 'wifi_secrets.h' and '.local.' not in p.name)
    paths.update(p for p in CORE.glob('*.md'))
    paths.add(CORE / 'artifacts/README.md')
    return sorted(paths)


def algorithm_checks():
    """Preserve all tested shared executable inputs, plus RTOS decision models."""
    folders = ('firmware/integration/buddy1_4/src/', 'firmware/integration/buddy1_4/include/',
               'firmware/integration/buddy1_4/config/')
    models = ('app_program/line_navigation/line_model.h',
              'app_program/barcode_navigation/barcode_model.h',
              'app_program/barcode_navigation/junction_sensor.h',
              'app_program/imu_navigation/hump_odo_policy.h',
              'app_program/imu_navigation/ramp_bridge.c')
    names = common.command('git', 'ls-tree', '-r', '--name-only', TESTED).splitlines()
    preserved = []
    for name in names:
        if (name.startswith(folders) and Path(name).suffix in {'.c', '.h'}) or name in models:
            original = subprocess.check_output(['git', 'show', f'{TESTED}:{name}'], cwd=ROOT)
            assert (ROOT / name).read_bytes() == original, 'Tested algorithm changed: ' + name
            preserved.append(name)
    assert len(preserved) > 50
    return preserved


def main():
    ART.mkdir(parents=True, exist_ok=True)
    built = ROOT / 'build_make' / (NAME + '.uf2')
    elf = built.with_suffix('.elf')
    layout = common.validate_uf2(built.read_bytes())
    linked = linked_checks(elf, network=True, barcode=True, imu=True, release=True)
    algorithms = algorithm_checks()
    paths = collect_sources()
    hashes = {p.relative_to(ROOT).as_posix(): common.digest(p) for p in paths}
    preserved, reviewed = common.verify_shared_sources(hashes)
    firmware = ART / built.name
    shutil.copyfile(built, firmware)
    manifest = {
        'name': NAME, 'version': '0.2.0', 'target_branch': 'main',
        'status': 'PROVISIONAL Buddy1-4 RTOS; Buddy5 NOT integrated',
        'source_commit': common.command('git', 'rev-parse', 'HEAD'),
        'tested_step5_baseline': TESTED, 'previous_sdk_main': common.BASE,
        'build_profile': 'APP=rtos_buddy1_4 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0',
        'compiler': common.command(ROOT.parent / '.buddy2-tools/arm-gcc/bin/arm-none-eabi-gcc.exe', '--version').splitlines()[0],
        'sdk_commit': common.command('git', '-C', ROOT.parent / '.buddy2-tools/pico-sdk', 'rev-parse', 'HEAD'),
        'uf2_sha256': common.digest(firmware), 'uf2_bytes': firmware.stat().st_size,
        'elf_sha256': common.digest(elf), 'uf2_layout': layout,
        'calibration_priority': ['CRC-valid RTOS save at0x1a9000', 'CRC-valid SDK record28 at0x1d0000',
                                 'fallback white163,160 black3457,3700'],
        'expected_tested_car_calibration': 'white122,138 black246,267; existing saves preserved, not forced',
        'profile': 'Tested Profile1 only, normal45mm/s; no speed/gain/decoder/ramp tuning',
        'supervision': ['No USB/DTR requirement in production; explicit Start with existing2s countdown',
                        'Kernel MQTT link/subscription and poll/app freshness required',
                        'Recent TCP publish-ACK activity within3000ms and from current connection',
                        'Network loss/stale delivery stops; reconnect never automatically restarts',
                        'Boot OFF, GP20 ESTOP/GP21 Stop, existing clocks/ADC/late/encoder/coordinator guards'],
        'network_ownership': 'Kernel Wi-Fi task and mqtt_utk.c only; copied fixed TX/RX queues; no subsystem lwIP calls',
        'wireless_logs': {'condition': 'Stopped, no pending Start/USB export; bounded5s refreshed transfer lease blocks Start',
                          'rows': {'motion': '48 bytes, run-relative time/nonnegative center displacement',
                                   'diagnostics': '56-byte RTOS LineLogRow explicit LE',
                                   'raw_sensors': '10 bytes time/signed position/GP27 ADC',
                                   'navigation': '31-byte coordinator event'},
                          'capacity': '640 line rows,2048 raw rows frozen at first decode, bounded navigation history',
                          'new_source15': 'C2 release/calibration15 bytes; C3 runtime63 bytes;1Hz, C3 telemetry-enabled only'},
        'software_checks': ['6 focused strict native C programs: legacy/release policy, real release exports, legacy/release bridge, release IMU',
                            '28 focused Python observer/connected/line-export/barcode-export tests',
                            '17 existing Node schema regressions and actual C-to-browser release fixtures',
                            'Headless browser stopped56-byte row transfer and binary/CSV/JSON metadata integrity',
                            'ARM release build, UF2 bounds/RAM flash code/stack gates, source fingerprint and ZIP integrity'],
        'hardware_evidence': 'Step5v0.1.5: flat0 humps; one nominal10mm one-sided ramp1 accepted/13mm max; assist active; about50Hz IMU/100Hz control; no new measured motion-window timing/acquisition/logging faults. Not a repeatability claim.',
        'pending': ['First physical powerbank Start/Pause/Resume/Stop and network-loss check',
                    'Full supported-mat left/right outgoing-line rejoin and sharp bends', 'Buddy5 integration'],
        'source_files': len(paths), 'unchanged_shared_vs_sdk': len(preserved),
        'reviewed_shared_vs_sdk': reviewed,
        'tested_executable_files_unchanged': algorithms,
        'ram_comparison': {'baseline': 'Step5v0.1.5 same ARM toolchain',
                           'baseline_static_ram_end': '0x2002e700',
                           'static_ram_growth_bytes': int(linked['static_ram_end'],16) - 0x2002e700,
                           'message_extensions_bytes_per_second': 78,
                           'scope': 'Linked static RAM only, not dynamic stack high-water or whole-firmware Tribunal completion'},
        'linked_checks': linked,
    }
    for name, data in [('BUILD_MANIFEST.json', manifest), ('SOURCE_SHA256.json', hashes)]:
        (ART / name).write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8', newline='\n')
    bundle = ART / (NAME + '_bundle.zip')
    with zipfile.ZipFile(bundle, 'w', zipfile.ZIP_DEFLATED, compresslevel=6) as z:
        for p in paths:
            z.write(p, p.relative_to(ROOT).as_posix())
        for p in (firmware, ART / 'BUILD_MANIFEST.json', ART / 'SOURCE_SHA256.json'):
            z.write(p, p.relative_to(ROOT).as_posix())
        z.write(firmware, 'firmware/' + firmware.name)
        z.writestr('START_HERE.md', (ROOT / 'docs/setup.md').read_text(encoding='utf-8'))
        commands = [('Start dashboard.cmd', 'tools/dashboard/run_dashboard.py', ''),
                    ('Setup Wi-Fi.cmd', 'tools/rtos_migration/provision_network.py', ''),
                    ('Capture wireless test.cmd', 'tools/rtos_migration/capture_wireless.py', ''),
                    ('Capture connected test.cmd', 'tools/rtos_migration/capture_connected_line.py', ' --network-supervised'),
                    ('Export line log.cmd', 'tools/rtos_migration/export_line_log.py', ''),
                    ('Export barcode log.cmd', 'tools/rtos_migration/export_barcode_log.py', '')]
        for title, script, args in commands:
            z.writestr(title, '@echo off\r\ncd /d "%~dp0"\r\npy -3 ' + script.replace('/', '\\') + args + '\r\npause\r\n')
    with zipfile.ZipFile(bundle) as z:
        assert z.testzip() is None
        assert z.read('firmware/' + firmware.name) == firmware.read_bytes()
        for name, digest in hashes.items():
            assert hashlib.sha256(z.read(name)).hexdigest() == digest, name
        assert not any(n.endswith('wifi_secrets.h') or '.local.' in n for n in z.namelist())
    (ART / 'SHA256SUMS.txt').write_text(''.join(f'{common.digest(p)}  {p.name}\n' for p in (firmware,bundle)), encoding='utf-8', newline='\n')
    print(json.dumps({'uf2':str(firmware), 'bundle':str(bundle), 'bundle_bytes':bundle.stat().st_size,
                      'uf2_sha256':common.digest(firmware), 'bundle_sha256':common.digest(bundle),
                      'unchanged_tested_executable_files':len(algorithms),
                      'static_ram_growth':manifest['ram_comparison']['static_ram_growth_bytes'],
                      'ram_remaining':linked['ram_remaining_before_dynamic_kernel_stacks']}, indent=2))

if __name__ == '__main__':
    main()
