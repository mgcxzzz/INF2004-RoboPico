# Buddy 1–4 RTOS v0.2.8 wireless-calibration candidate

Archived v0.2.8 wireless calibration candidate. The current combined downloads are in the [v0.4.3 release directory](../../../../releases/v0.4.3/).
Buddy 5 is not integrated.

- [Complete bundle](robopico_buddy1_4_rtos_v0_2_8_wireless_calibration_bundle.zip): use this for the matching dashboard wizard.
- [UF2](robopico_buddy1_4_rtos_v0_2_8_wireless_calibration.uf2).
- [Instructions, wire contract and validation](../../../../../docs/wireless_calibration_v0_2_8.md).

Open **Diagnostics → Front IR calibration** while fully stopped. Capture white,
left black and right black. Review, then explicitly Use in RAM or Save and use.
Existing saved calibration stays active until that explicit commit. No USB
console is needed. Existing steering, barcode, junction and IMU behavior remains.

`BUILD_MANIFEST.json` identifies the source commit and linked checks;
`SOURCE_SHA256.json` records bundled sources; `SHA256SUMS.txt` hashes UF2/bundle.
