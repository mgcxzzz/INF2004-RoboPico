@echo off
cd /d "%~dp0"
py -3 tools\provisioning\provision_wifi.py
pause
