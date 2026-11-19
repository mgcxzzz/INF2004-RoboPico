@echo off
cd /d "%~dp0"
py -3 tools\rtos_migration\provision_network.py
pause
