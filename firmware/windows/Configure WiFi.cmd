@echo off
cd /d "%~dp0..\.."
if exist "tools\rtos_migration\provision_network.py" (
    py -3 "tools\rtos_migration\provision_network.py" %*
) else (
    py -3 "tools\provisioning\provision_wifi.py" %*
)
set result=%errorlevel%
if "%~1"=="" pause
exit /b %result%
