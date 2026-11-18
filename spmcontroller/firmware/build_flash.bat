@echo off
REM =====================================================================
REM  Build + flash the SPM USB-stream firmware with the EXISTING ESP-IDF
REM  v5.5.2 install (Espressif Installation Manager layout).
REM  Double-click it. Everything is also logged to build_flash_log.txt.
REM    build_flash.bat          build, then flash the board it finds
REM    build_flash.bat build    build only
REM =====================================================================
setlocal
cd /d "%~dp0"
call :main %* > "%~dp0build_flash_log.txt" 2>&1
type "%~dp0build_flash_log.txt"
echo.
echo (log saved to build_flash_log.txt)
pause
exit /b

:main
echo ==== %DATE% %TIME% ====
REM ---- 1. point at the installed ESP-IDF, its tools and Python env ----
set "IDF_PATH=C:\Users\Penelope\esp\v5.5.2\esp-idf"
set "IDF_TOOLS_PATH=C:\Users\Penelope\.espressif"
set "IDF_PYTHON_ENV_PATH=C:\Users\Penelope\.espressif\python_env\idf5.5_py3.11_env"
set "PATH=%IDF_PYTHON_ENV_PATH%\Scripts;C:\Users\Penelope\.espressif\tools\idf-git\2.39.2\cmd;%PATH%"
call "%IDF_PATH%\export.bat"
if errorlevel 1 ( echo EXPORT_FAILED & exit /b 1 )
echo IDF_PATH=%IDF_PATH%

REM ---- 2. build ----
call idf.py build
if errorlevel 1 ( echo BUILD_FAILED & exit /b 1 )
echo BUILD_OK
if /i "%~1"=="build" exit /b 0

REM ---- 3. find the board: Espressif USB Serial/JTAG = VID 303A ----
set "PORT="
for /f "usebackq delims=" %%P in (`powershell -NoProfile -Command "Get-PnpDevice -Class Ports -PresentOnly | Where-Object { $_.InstanceId -match 'VID_303A' } | ForEach-Object { if ($_.FriendlyName -match '\((COM\d+)\)') { $Matches[1] } } | Select-Object -First 1"`) do set "PORT=%%P"
echo All serial ports:
powershell -NoProfile -Command "Get-PnpDevice -Class Ports -PresentOnly | Format-Table -AutoSize FriendlyName,InstanceId"
if "%PORT%"=="" ( echo NO_BOARD_FOUND - is the board plugged in by USB? & exit /b 1 )
echo BOARD_PORT=%PORT%

REM ---- 4. flash ----
call idf.py -p %PORT% flash
if errorlevel 1 ( echo FLASH_FAILED & exit /b 1 )
echo FLASH_OK
exit /b 0
