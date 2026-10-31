# Dashboard tools

`run_dashboard.py` serves the repository/package's single dashboard/ directory and starts/checks a local Mosquitto broker. Run it directly or use the root Start dashboard.cmd. `check_mqtt.py` diagnoses TCP/MQTT and fresh heartbeat separately. No Python package is needed for dashboard hosting. Install Mosquitto separately; Python helpers do not silently replace an existing Windows service or firewall policy.

The broker log and session files are local and gitignored. Keep the launcher running during wireless use. Feature logic and export instructions are in docs/setup.md and the firmware feature READMEs.
