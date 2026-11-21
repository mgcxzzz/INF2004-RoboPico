# Historical SDK runtime Wi-Fi setup

For the current RTOS release, use the [combined setup guide](../../docs/setup.md) and the [BOOTSEL network launcher](../../firmware/windows/Configure%20WiFi.cmd). The SDK USB-serial helper is
`python tools/provisioning/provision_wifi.py`. It prompts locally for SSID,
hidden password and the laptop broker's LAN IPv4, then waits for an explicit
firmware confirmation. Numeric Windows port 4 is normalized to COM4. Full
Mac and Linux device paths are also accepted.

Close PuTTY first. Stop and settle the robot, keep both buttons released and let saving finish. The helper may be closed after a verified save/reboot. Do not commit local settings JSON or a credentials header. See docs/setup.md for exact steps and network checks.
