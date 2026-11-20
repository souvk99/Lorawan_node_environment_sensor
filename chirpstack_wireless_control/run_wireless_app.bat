@echo off
title ChirpStack Wireless Node Configurator
cd /d "%~dp0"
echo ===================================================
echo Starting ChirpStack Wireless Node Configurator...
echo Target ChirpStack Server: http://172.16.50.30:8080
echo Target DevEUI: ac1f09fffe22024e
echo ===================================================
python wireless_configurator_gui.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Application exited with an error. Press any key to exit.
    pause >nul
)
