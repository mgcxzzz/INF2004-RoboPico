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
ART = CORE / 'artifacts/v0_2_5_rtos'
NAME = 'robopico_buddy1_4_rtos_v0_2_5'
TESTED = '75325f29ad604c75a9d9ab8111658aa1536543d5'
PREVIOUS = 'f4be2bea044f73960546adb4da80b720e84c0f2a'
# Exact fingerprints for the reviewed v0.2.5 change boundary. Other tested
# algorithms must remain identical to their accepted baseline.
REVIEWED = {'firmware/integration/buddy1_4/src/line_following/continuous_steering.h': {'sha256': '0391ea3504e66aca15c4c2a98ae8f964b0710f31a3d9b217983623016885ed30',
                                                                            'change': 'Opt-in60/75 cruise '
                                                                                      'only;45 baseline and '
                                                                                      'scan/turn/zero-inner '
                                                                                      'policies retained, '
                                                                                      'bend scaling '
                                                                                      'bounded.'},
 'firmware/integration/buddy1_4/src/line_following/line_search.h': {'sha256': '65b1542d3b277473af769b662ad37638e7fd87e6fc782f97796dd7a3461c26f7',
                                                                    'change': 'Production opt-in signed '
                                                                              'in-place search, stationary '
                                                                              'reversals/reacquisition, '
                                                                              'explicit '
                                                                              'input/angle/path/time/stall '
                                                                              'limits; legacy rolling search '
                                                                              'retained for default builds.'},
 'firmware/integration/buddy1_4/src/line_following/circuit_recovery.h': {'sha256': 'd2b5f184f17e0618cbee5ac80fbcfdf9eaa418a463720afa878b1bbf2a340601',
                                                                         'change': 'Production stationary '
                                                                                   'search orchestration; '
                                                                                   'original brief-gap/loss '
                                                                                   'confirmation and optical '
                                                                                   'reacquisition durations '
                                                                                   'retained.'},
 'firmware/integration/buddy1_4/src/line_following/circuit_logic.h': {'sha256': 'a03d4da7b20d50bec405aa38454d2cb1378d8f241e07c04b38fc04ec22e5c7c5',
                                                                      'change': 'Opt-in measured wheel/speed '
                                                                                'recovery inputs with '
                                                                                'fail-closed validation and '
                                                                                'existing compatibility '
                                                                                'entry point.'},
 'app_program/line_navigation/line_model.h': {'sha256': '65801a11c0ae31a6c543efef1d4cd717cd38845fcabe4594d0e5a22c2e0eb177',
                                              'change': 'Production-only load reserve in ordinary following; '
                                                        'runtime cruise wrapper; old baseline facade and '
                                                        'logs retained.'},
 'app_program/barcode_navigation/junction_sensor.h': {'sha256': '7405da62dbfa7649671c709e3196acba5376d2b9d47c2f08e041034a30a62444',
                                                      'change': 'Production7mm minimum width, unchanged30mm '
                                                                'maximum/fresh-white gates; frozen bounded '
                                                                'width/threshold/reason C7 snapshot.'},
 'firmware/integration/buddy1_4/src/imu/imu_service.c': {'sha256': 'fa94ef1b261130cfcfa40a8fb70a08039958221521c9b8fba02adfc61f64e1a7',
                                                         'change': 'Production-only calibrated raw FIFO hook '
                                                                   'feeds independent lift guard even idle; '
                                                                   'historical SDK/migration service path '
                                                                   'retained.'},
 'firmware/integration/buddy1_4/src/imu/imu_proc.c': {'sha256': '995007231edecc95d5b6c92a80988d2dcd6534352da263a2ecf0ae08302c71d8',
                                                      'change': 'Production removes collision Impact '
                                                                'classification, retains high-dynamic '
                                                                'terrain exclusion and all baseline '
                                                                'height/ramp thresholds; legacy collision '
                                                                'branch remains default.'},
 'app_program/rtos_bringup/bringup_policy.h': {'sha256': '966634c0995a510365e0c252f4015db4a94550c8908a02e1e3445648ae908f4e',
                                               'change': 'Add packed eighth supervisor bit and '
                                                         'lift/overturn/unavailable reasons with safety '
                                                         'reset/motion gates; original clock/motor/network '
                                                         'limits unchanged.'},
 'app_program/barcode_navigation/barcode_model.h': {'sha256': '2236f6bb803318a908516c3ec98d42c2f84f52a4335a02ace01c3478ca5544ab',
                                                    'change': 'Cruise-aware following wrapper only; old '
                                                              'facade retained; decoder and junction '
                                                              'algorithms unchanged.'},
 'app_program/imu_navigation/imu_lift_packet.h': {'sha256': '85aede0cba9be40a593b942572cc075bfc54273bfaa5cb95c9221e7b7086af81',
                                                  'change': 'Explicit C9 codec25 extended dwell and CA '
                                                            'retained-peak encoding with validated compact '
                                                            'ranges.'},
 'app_program/imu_navigation/imu_service.c': {'sha256': '985355ff214e7a5d921f06fd981a0cb884799620d24603a3a77cc5ca6d96dc65',
                                              'change': 'Protected sample-before-clock freshness; single '
                                                        'detector writer/reset queue; copied retained '
                                                        'evidence.'},
 'app_program/imu_navigation/imu_service.h': {'sha256': '0d24afd6ec0a4522980c0e03bcf06400febbd4c94020c549c4eae2c5a734b8c0',
                                              'change': 'Expose retained evidence packet API; existing '
                                                        'motion ownership retained.'},
 'app_program/line_navigation/line_service.c': {'sha256': '54f45fa554a6f45c04a54f80a42760b8e8a2982b24e1a2415e6a694d08032a94',
                                                'change': 'Owner-side stopped/countdown/export mode gates '
                                                          'and copied CB load status; unchanged '
                                                          'ADC/calibration/log histories.'},
 'app_program/line_navigation/line_service.h': {'sha256': '98776fe563c997fdab51f8655759260b39dca5e100286e166bcf30269cb1586e',
                                                'change': 'Declare stopped-only cruise and copied CB status '
                                                          'APIs.'},
 'app_program/line_navigation/release_exports.inc': {'sha256': '6273b279010a290186469664741feb148e1c4cefc7f2e88c35ba279268514e4a',
                                                     'change': 'Advertise release0.2.5; existing export '
                                                               'schema and calibration fields retained.'},
 'app_program/network_bridge/connected_control.c': {'sha256': '7f7f1ab420e65ed3d111a4f69f4e9b15474821b270c936647685ffd32f2ac413',
                                                    'change': 'Route validated CONFIG mode requests through '
                                                              'shared coordinator; packed atomic result '
                                                              'snapshot; existing network stops retained.'},
 'app_program/network_bridge/drive_mode_telemetry.h': {'sha256': 'f6eb075fc88335f4a22c4c700ece9e411ac7d33d50f36f76df19d092221528be',
                                                       'change': 'Explicit compact CB encoder with '
                                                                 'reserved-zero wire fields; no raw '
                                                                 'native-struct serialization.'},
 'app_program/network_bridge/network_bridge.c': {'sha256': '727b6a76e628acc8721a1701fe074440722dbf80b3df868a12a4f7346a548fd0',
                                                 'change': 'Queue mode command to owner; keep legacy '
                                                           'telemetry CONFIG; emit CA/CB at1Hz through '
                                                           'bounded TX.'},
 'app_program/network_bridge/network_bridge.h': {'sha256': 'd9e9600865452046dac20a41d936020b71fed6fe9aa1d077a6aa2e68cb08d24c',
                                                 'change': 'Declare copied drive mode request/result APIs.'},
 'app_program/rtos_bringup/app_main.c': {'sha256': '2d042f56135bee67ee6398efa62cc0a9ef57e7e915cd41a350db627f3f035038',
                                         'change': 'Release0.2.5 banner only; RTOS task topology unchanged.'},
 'firmware/integration/buddy1_4/include/interfaces/motion_api.h': {'sha256': '01537b965109a7ba0e16c6cdd657722cb9f627acf8d903493fd32602a2428048',
                                                                   'change': 'Bounded speed mode status and '
                                                                             'shared request API; existing '
                                                                             'ownership and motion semantics '
                                                                             'retained.'},
 'firmware/integration/buddy1_4/include/interfaces/nav_api.h': {'sha256': '8f7b9437d5ba0fefb6d369a8133989a2f7e1687459fdef6327d48e928c1577cf',
                                                                'change': 'Expose nav_request_speed_mode '
                                                                          'facade for Buddy1; no private '
                                                                          'motor access.'},
 'firmware/integration/buddy1_4/src/imu/imu_lift_guard.h': {'sha256': 'f8d77f9d4ea46b2f13755e05e1711e2e1b235bd6c932481e150cdf70df31a8ef',
                                                            'change': '45degree100ms tilt;400mg30ms upward '
                                                                      'cue retained; bounded integer '
                                                                      'peaks/rejection history since Reset.'},
 'firmware/integration/buddy1_4/src/line_following/continuous_follow.h': {'sha256': '53498b8d3c245e31cad9dbc85e5e38ca4ef1f33e538dcfd22e2975054f017d19',
                                                                          'change': 'Cruise-aware wrapper '
                                                                                    'retains baseline entry '
                                                                                    'point, barcode activity '
                                                                                    'and turn selection.'},
 'firmware/integration/buddy1_4/src/motion/continuous_drive.h': {'sha256': '7ad1b4a568008ba05c34cc75bc672f5afddc68c5d9e2045f881209935d07653c',
                                                                 'change': 'Bounded40PWM smooth per-wheel '
                                                                           'own-reference reserve,180ms '
                                                                           'dwell/300ms cooldown; '
                                                                           'cap/slew/startup aid and stop '
                                                                           'guards retained.'},
 'firmware/integration/buddy1_4/src/motion/drive_mode.h': {'sha256': 'f8aadcaa7f795587020a2b6ab2e8f0f3225d057bce431ff87a6c86343635966e',
                                                           'change': 'RAM modes0/1/2 map to45/60/75 and '
                                                                     'define bounded load policy constants.'},
 'firmware/integration/buddy1_4/src/navigation/motion_api.c': {'sha256': '9f65ae4024c1dff68025c466354e6b7376d51a347754fa7501c59dc06d394090',
                                                               'change': 'Canonical NAV_SPEED_MODE request '
                                                                         'dispatch and copied status '
                                                                         'facade.'},
 'firmware/integration/buddy1_4/src/navigation/nav_codec.c': {'sha256': '17930199b973c521a1745dc66653d54ea12030a91c6ddee561ac707e7a6982bc',
                                                              'change': 'Canonical2-bit speed mode request '
                                                                        'kind12; existing request bitstreams '
                                                                        'retained.'},
 'firmware/integration/buddy1_4/src/navigation/nav_codec.h': {'sha256': '387223085e75f3682a8e11c31fd8ae939c74d745b451ae71e7cb44eadd63340b',
                                                              'change': 'New request kind12 only; existing '
                                                                        'kinds and packet capacity '
                                                                        'retained.'},
 'firmware/integration/buddy1_4/src/navigation/nav_coordinator.c': {'sha256': '22be22e2a0d49595e15fd090612c0d0ec3d2e79eaa68e54594bf2ba3e3b832cf',
                                                                    'change': 'Buddy1 stopped OFF '
                                                                              'ended-session mode '
                                                                              'ownership;32-bit revision; '
                                                                              'original motion/ramp safety '
                                                                              'unchanged.'},
 'firmware/integration/buddy1_4/src/navigation/nav_coordinator.h': {'sha256': '4fd7023d96d6558f296dcba78992f236030dacafea8e7c863bcc3434fff99502',
                                                                    'change': 'Declare coordinator mode '
                                                                              'request/status APIs.'},
 'firmware/integration/buddy1_4/src/network/wifi_protocol.c': {'sha256': '5202e6b5bebe8c8e56f3e545f9d96d631afd6dc1dfed3f6246ede77360818d84',
                                                               'change': 'CONFIG key2 mode0..2 validation; '
                                                                         'legacy0x1000/1001 telemetry '
                                                                         'preserved.'},
 'firmware/integration/buddy1_4/src/network/wifi_protocol.h': {'sha256': 'ac2c22a1dc0520d82b59d2dc290660204e16e788df33d6ba6bbc0ade99c97bf8',
                                                               'change': 'Name CONFIG drive mode key2.'}}
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
    paths.update(p for p in (ROOT / 'testing/evidence/rtos_buddy1_4').rglob('*')
                 if p.is_file() and p.suffix in {'.md', '.txt', '.csv', '.json'})
    paths.update(p for p in CORE.glob('*.md'))
    paths.add(CORE / 'artifacts/README.md')
    return sorted(paths)


def algorithm_checks():
    """Preserve all tested shared executable inputs, plus RTOS decision models."""
    for name, entry in REVIEWED.items():
        assert common.digest(ROOT/name) == entry['sha256'], 'Reviewed source changed: ' + name
    folders = ('firmware/integration/buddy1_4/src/', 'firmware/integration/buddy1_4/include/',
               'firmware/integration/buddy1_4/config/')
    models = ('app_program/line_navigation/line_model.h',
              'app_program/rtos_bringup/encoder_quality.h',
              'app_program/barcode_navigation/junction_sensor.h',
              'app_program/imu_navigation/hump_odo_policy.h',
              'app_program/imu_navigation/ramp_bridge.c')
    names = common.command('git', 'ls-tree', '-r', '--name-only', TESTED).splitlines()
    preserved = []
    for name in names:
        if (name.startswith(folders) and Path(name).suffix in {'.c', '.h'}) or name in models:
            original = subprocess.check_output(['git', 'show', f'{TESTED}:{name}'], cwd=ROOT)
            if name in REVIEWED:
                assert common.digest(ROOT/name)==REVIEWED[name]['sha256'], 'Reviewed algorithm changed: '+name
                continue
            assert (ROOT / name).read_bytes() == original, 'Tested algorithm changed: ' + name
            preserved.append(name)
    assert len(preserved) > 50
    for name in ('app_program/rtos_bringup/bringup_policy.h', 'app_program/network_bridge/connected_policy.h'):
        if name in REVIEWED:
            assert common.digest(ROOT/name)==REVIEWED[name]['sha256'], 'Reviewed safety policy changed: '+name
            continue
        original = subprocess.check_output(['git','show',f'f4a367eaacbeb6afb7418aabc507687349613437:{name}'],cwd=ROOT)
        assert (ROOT/name).read_bytes()==original, 'Main supervision policy changed: '+name
        preserved.append(name)
    return preserved


def main():
    ART.mkdir(parents=True, exist_ok=True)
    built = ROOT / 'build_make' / (NAME + '.uf2')
    elf = built.with_suffix('.elf')
    layout = common.validate_uf2(built.read_bytes())
    linked = linked_checks(elf, network=True, barcode=True, imu=True, release=True)
    nm = common.command(ROOT.parent / '.buddy2-tools/arm-gcc/bin/arm-none-eabi-nm.exe', '-g', elf)
    required = ('rtos_line_junction_info', 'rtos_line_drive_info', 'ri_lift_guard_samples', 'ri_safety_packet', 'ri_safety_reset_latch', 'ri_safety_peak_packet', 'rtos_line_drive_mode_info')
    names = {line.split()[-1] for line in nm.splitlines() if line.split()}
    for name in required:
        assert name in names, 'Release diagnostic missing from image: ' + name
    linked['drive_refinement_symbols'] = list(required)
    algorithms = algorithm_checks()
    paths = collect_sources()
    hashes = {p.relative_to(ROOT).as_posix(): common.digest(p) for p in paths}
    preserved, reviewed = common.verify_shared_sources(hashes, REVIEWED)
    firmware = ART / built.name
    shutil.copyfile(built, firmware)
    manifest = {
        'name': NAME, 'version': '0.2.5', 'target_branch': 'main',
        'status': 'PROVISIONAL Buddy1-4 RTOS; Buddy5 NOT integrated',
        'source_commit': common.command('git', 'rev-parse', 'HEAD'),
        'tested_step5_baseline': TESTED, 'previous_sdk_main': common.BASE,
        'previous_rtos_main': PREVIOUS,
        'reviewed_release_algorithms': REVIEWED,
        'change_boundary': 'Live demo overview; stopped-only45/60/75 cruise modes through coordinator; bounded per-wheel load reserve; IMU45degree/100ms tilt qualification, snapshot clock ordering and retained peak evidence. Preserve RTOS ownership, barcode/junction/recovery policies and calibration.',
        'build_profile': 'APP=rtos_buddy1_4 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CRASH_REPORT=0 TEST=0',
        'compiler': common.command(ROOT.parent / '.buddy2-tools/arm-gcc/bin/arm-none-eabi-gcc.exe', '--version').splitlines()[0],
        'sdk_commit': common.command('git', '-C', ROOT.parent / '.buddy2-tools/pico-sdk', 'rev-parse', 'HEAD'),
        'uf2_sha256': common.digest(firmware), 'uf2_bytes': firmware.stat().st_size,
        'elf_sha256': common.digest(elf), 'uf2_layout': layout,
        'calibration_priority': ['CRC-valid RTOS save at0x1a9000', 'CRC-valid SDK record28 at0x1d0000',
                                 'fallback white163,160 black3457,3700'],
        'expected_tested_car_calibration': 'white122,138 black246,267; existing saves preserved, not forced',
        'profile': 'Profile1 Baseline45 default, stopped-only Normal60/Fastertrial75; scan/turn/recovery speed policies retained; bounded per-wheel own-target load reserve; IMU45degree/100ms tilt and400mg/30ms upward qualification',
        'supervision': ['No USB/DTR requirement in production; explicit Start with existing2s countdown',
                        'Kernel MQTT link/subscription and poll/app freshness required',
                        'Release IMU requires fresh calibrated data; qualified lift/overturn latches and cancels all motion; explicit eligible Reset and new Start',
                        'Recent TCP publish-ACK activity within3000ms and from current connection',
                        'Network loss/stale delivery stops; reconnect never automatically restarts',
                        'Boot OFF, GP20 ESTOP/GP21 Stop, existing clocks/ADC/late/encoder/coordinator guards'],
        'network_ownership': 'Kernel Wi-Fi task and mqtt_utk.c only; copied fixed TX/RX queues; no subsystem lwIP calls',
        'wireless_logs': {'condition': 'Stopped, no pending Start/USB export; bounded5s refreshed transfer lease blocks Start',
                          'rows': {'motion': '48 bytes, run-relative time/nonnegative center displacement',
                                   'diagnostics': '56-byte RTOS LineLogRow explicit LE',
                                   'raw_sensors': '10 bytes time/signed position/GP27 ADC',
                                   'navigation': '31-byte coordinator event'},
                          'capacity': '640 line rows,2048 raw rows for latest reportable attempt,512-row preroll, diagnostic5ms decimation, bounded navigation history',
                          'new_source15': 'C2 release/calibration15,C3 runtime63,C4 encoder fault48,C5 barcode attempt30,C6 network stop32,C7 junction20,C8 drive18,C9 IMU lift30,CA retained IMU peaks28,CB drive mode17 bytes;1Hz; C3/C5/C7/C8 ordinary telemetry enabled only; C9/CA/CB always'},
        'software_checks': ['Focused production-C IMU dwell/freshness/reset/retained-peak and wire fixtures',
                            'Production drive-mode coordinator/protocol/owner gates and bounded load-reserve regressions',
                            'Actual C-to-browser decoding, legacy codec compatibility and dashboard controls/freshness',
                            'ARM image, required symbols, flash/RAM/stack limits, reviewed protected source hashes and archive integrity'],
        'hardware_evidence': 'Step5v0.1.5: flat0 humps; one nominal10mm one-sided ramp1 accepted/13mm max; assist active; about50Hz IMU/100Hz control; no new measured motion-window timing/acquisition/logging faults. Not a repeatability claim.',
        'pending': ['Physical revised pickup stop and ramp false-positive check; software checks cannot certify lift discrimination or physical stop latency', '60/75 cruise mode and load-reserve physical acceptance; Baseline45 remains default',
                    'Bend untested; stationary recovery reported not sweeping, awaiting logs; v0.2.2 logic retained',
                    'Full supported-mat left/right outgoing-line rejoin and sharp bends', 'Buddy5 integration'],
        'source_files': len(paths), 'unchanged_shared_vs_sdk': len(preserved),
        'reviewed_shared_vs_sdk': reviewed,
        'tested_executable_files_unchanged': algorithms,
        'ram_comparison': {'baseline': 'Main RTOS v0.2.0 same ARM toolchain',
                           'baseline_static_ram_end': '0x2002e710',
                           'static_ram_growth_bytes': int(linked['static_ram_end'],16) - 0x2002e710,
                           'previous_v022_static_ram_end': '0x2002fd90',
                           'previous_v023_static_ram_end': '0x2002fda8',
                           'growth_vs_v023_bytes': int(linked['static_ram_end'],16) - 0x2002fda8,
                           'growth_vs_v022_bytes': int(linked['static_ram_end'],16) - 0x2002fd90,
                           'message_extensions_bytes_per_second': 301,
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
        z.writestr('TEST_CHANGES.md', (ROOT/'docs/demo_console_v0_2_5.md').read_text(encoding='utf-8'))
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
