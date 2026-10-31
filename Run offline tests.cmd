@echo off
cd /d "%~dp0"
py -3 firmware\integration\buddy1_4\tests\run_tests.py
pause
