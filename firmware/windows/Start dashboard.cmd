@echo off
cd /d "%~dp0..\.."
py -3 "tools\dashboard\run_dashboard.py" %*
set result=%errorlevel%
if "%~1"=="" pause
exit /b %result%
