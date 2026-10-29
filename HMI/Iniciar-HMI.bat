@echo off
rem TCM HMI — doble clic: abre Chrome y deja esta ventana abierta.
rem Para cerrar el HMI: Ctrl+C (o cerrar la ventana). Al volver a abrir, reinicia limpio.
title TCM HMI
cd /d "%~dp0"

set "PY=..\.venv\Scripts\python.exe"
if not exist "%PY%" set "PY=python"

"%PY%" -u "%~dp0iniciar_hmi.py" --restart
if errorlevel 1 pause
