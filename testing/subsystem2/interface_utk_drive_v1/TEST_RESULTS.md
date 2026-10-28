# V1.1 validation, 28 October 2026

- Real `sys_hal_init` and clock-validation regression: PASS for running/wrong divider, correct and disabled tick, rejected writes, wrong reference clock, frozen/slow/fast timer and wraparound. MMIO and kernel waits are test models.
- Actual application startup regression: PASS; register-init or time-check failures cannot create/start the driving task, and motor GPIOs are held off.
- V1.1 ARM build: PASS, text 476328, BSS 225124. One linker warning for the RAM read/write/execute segment is expected because flash-writing helpers execute from SRAM.
- New clock-check hardware result is pending. Earlier V1 hardware timing was wrong despite its software build passing.

## V1 baseline checks (did not validate hardware timer rate)

- ARM kernel Wi-Fi/MQTT + driving build: passed. ELF text 475528 bytes, BSS 225108 bytes. RAM-executable flash helpers are counted in text by GNU size; the linker map records their RAM VMA and flash load address.
- Original Continuous V2 drive, follow, steering and coordinator integration regressions: passed. Three steering/drive headers are byte-identical to interface_continuous_v2. Host steering tests use MinGW SSE single-precision arithmetic; x87 extended precision did not meet a boundary assertion, so it is not used as an RP2040 arithmetic model.
- Calibration test: passed. The preset is white 163/160, black 3457/3700; legacy records cannot replace the initial preset. CRC-valid record27 saves deliberate custom calibration; invalid records fall back to the preset. c restores the immutable preset; k saves it.
- Bounded kernel bridge test: passed. Covers copied 64-byte topic packets, original RX oversize length, TMO_POL busy behavior, queue limits, publish counters and drop counts. Host kernel calls are test stand-ins, not a kernel scheduling test.
- Dashboard schema: 10 tests passed. Browser smoke test passed without page errors (mock transport). The four actual stored datasets are motion, diagnostics, raw sensors and navigation.
- Kernel timeout override and compile guard: INTERNAL + 4 retained. The earlier actual-allocator regression for the same kernel options reproduced default exhaustion and passed with core6/capacity10.
- Earlier UTK Network V1: the user confirmed live heartbeats, command subscription and mapping readback on their Pico/broker. That was the motors-off image.
- This combined driving image has not been flashed here. Driving, real-time sampling gaps, post-stop flash saving and concurrent live MQTT must be validated on the physical robot. No speed or steering gains were changed to hide a timing problem.
