@echo off
REM Double-click to start the SPM controller GUI (opens your browser).
cd /d "%~dp0"
python -m pip install -r requirements.txt --quiet
python server.py
pause
