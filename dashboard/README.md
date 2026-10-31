# Provisional Buddy1–4 dashboard

Run `python tools/dashboard/run_dashboard.py` from repository root or double-click Start dashboard.cmd. The helper serves this folder, starts a local Mosquitto broker if available and keeps its broker log under dashboard_session/. Existing listeners are checked, not silently replaced. Install Mosquitto separately.

Connect to ws://localhost:9001 on the laptop. The Pico uses the laptop's LAN IPv4 on MQTT TCP1883, not localhost. Car-online requires fresh heartbeat, independently of the browser-to-broker connection. The A–Z map applies through MQTT and uses confirmed readback.

This view retains the merged SDK/RTOS decoders and adds requested/effective ramp targets for provisional source15 C1. It does not implement Buddy5. Detailed feature logic lives under firmware/integration/buddy1_4/src/*/README.md; export instructions are in docs/setup.md. Vendored MQTT.js and its license are included for offline use.
