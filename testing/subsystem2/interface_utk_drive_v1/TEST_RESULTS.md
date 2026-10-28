# Validation, 28 October 2026

- ARM kernel Wi-Fi/MQTT + driving build: passed. ELF text 475528 bytes, BSS 225108 bytes. RAM-executable flash helpers are counted in text by GNU size; the linker map records their RAM VMA and flash load address.
- Original Continuous V2 drive, follow, steering and coordinator integration regressions: passed. Three steering/drive headers are byte-identical to interface_continuous_v2. Host steering tests use MinGW SSE single-precision arithmetic; x87 extended precision did not meet a boundary assertion, so it is not used as an RP2040 arithmetic model.
- Calibration test: passed. The preset is white 163/160, black 3457/3700; legacy records cannot replace the initial preset. CRC-valid record27 saves deliberate custom calibration; invalid records fall back to the preset. c restores the immutable preset; k saves it.
- Bounded kernel bridge test: passed. Covers copied 64-byte topic packets, original RX oversize length, TMO_POL busy behavior, queue limits, publish counters and drop counts. Host kernel calls are test stand-ins, not a kernel scheduling test.
- Dashboard schema: 10 tests passed. Browser smoke test passed without page errors (mock transport). The four actual stored datasets are motion, diagnostics, raw sensors and navigation.
- Kernel timeout override and compile guard: INTERNAL + 4 retained. The earlier actual-allocator regression for the same kernel options reproduced default exhaustion and passed with core6/capacity10.
- Earlier UTK Network V1: the user confirmed live heartbeats, command subscription and mapping readback on their Pico/broker. That was the motors-off image.
- This combined driving image has not been flashed here. Driving, real-time sampling gaps, post-stop flash saving and concurrent live MQTT must be validated on the physical robot. No speed or steering gains were changed to hide a timing problem.
