@echo off
title WisBlock RAK4631 Node Alert Configurator
echo ========================================================
echo   WisBlock RAK4631 Node Alert Configurator
echo ========================================================
echo Starting GUI application...
python "%~dp0node_configurator_gui.py"
if %errorlevel% neq 0 (
    echo.
    echo An error occurred while launching the configurator.
    pause
)
