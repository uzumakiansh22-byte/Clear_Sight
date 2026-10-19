@echo off
cd /d "%~dp0"
python campus_navigator.py
if errorlevel 1 (
  py campus_navigator.py
)
