@echo off
cd /d "%~dp0../../../.."
py -3 tools\rtos_migration\capture_bringup.py
pause
