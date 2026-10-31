# Checks for provisional Buddy1–4 integration v0.1

These checks concern the newly structured source, not the older hardware runs in the parent archive. No robot, serial port or physical broker was used.

- `host_tests.txt`:43 C test executables,27 merged dashboard decoder tests and22 Python helper tests. Includes real captured barcode replay, dual-reader mapping/conflict/junction checks,360,906 ramp checks, coordinator assist/expiry and barcode handback regression.
- `dashboard_browser.txt`:16 headless Chrome checks with simulated MQTT, including target-floor display, explicit provisional/Buddy5 status and restart clearing.
- `lwip_timeouts.txt`:actual SDK allocator reproduces the original7-slot exhaustion and passes with11 slots,100 application callbacks and1,000 add/remove cycles.
- `source_preservation.json`:17 algorithm token streams match the pinned combined baseline after ignoring includes/comments/format.
- `arm_build.txt`:Release ARM build with warnings treated as errors for application source.
- `structure.txt`:single entrypoint/source boundaries, feature README links and23 byte-preserved historical evidence hashes.
- `packet_vectors.txt`:actual C packet vectors decoded by the bundled dashboard schema.

`artifacts/BUILD_MANIFEST.json` records the final UF2 hash, source ledger, compile modes and linker stack/heap limits. Layout checks and compiler frame measurements are not a full worst-case stack call-chain proof. Hardware acceptance remains pending: flat/bend, mapped barcode-to-junction, ramp, stop/takeover, Pause/Resume and heartbeat under motion. Buddy5 remains unimplemented.
