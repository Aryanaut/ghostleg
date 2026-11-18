@echo off
cd /d "%~dp0\.."
python -m pip install pyserial numpy --quiet
python experiments\imu_check.py
