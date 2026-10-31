# Runtime Wi-Fi setup

Run `python tools/provisioning/provision_wifi.py` or the root Configure WiFi.cmd. This uses the running Pico's USB serial port, not BOOTSEL. It prompts locally for SSID, hidden password and laptop broker LAN IPv4, then waits for an explicit firmware confirmation. A disconnect alone is not proof of success. Numeric Windows port4 is normalized to COM4; full Mac/Linux device paths remain valid.

Close PuTTY first. Stop and settle the robot, keep both buttons released and let saving finish. The helper may be closed after a verified save/reboot. Do not commit local settings JSON or a credentials header. See docs/setup.md for exact steps and network checks.
