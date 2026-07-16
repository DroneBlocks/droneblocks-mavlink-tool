@echo off
title DEXI-3 Flight Controller Flasher

set "REPO=%USERPROFILE%\droneblocks-mavlink-tool"

if not exist "%REPO%\venv\Scripts\python.exe" (
  echo.
  echo   [X] Flasher is not installed at:
  echo       %REPO%
  echo   Ask Dennis to run the one-time setup on this PC.
  echo.
  pause
  exit /b 1
)

cd /d "%REPO%"

REM dfu-util lives in %USERPROFILE%\bin. Add it here so this works even when
REM Explorer's PATH is stale (a fresh install's setx isn't seen until re-login).
set "PATH=%USERPROFILE%\bin;%PATH%"

echo Checking for firmware / tool updates...
git pull --ff-only 1>nul 2>nul
if errorlevel 1 (
  echo   ^(offline or local changes - using the version already on this PC^)
) else (
  echo   Up to date.
)

echo.
echo ================================================================
echo   DEXI-3 FC FLASHER
echo ----------------------------------------------------------------
echo   BARE board   : HOLD the BOOT button while plugging in USB.
echo   Flashed board : just plug it in normally ^(no button^).
echo.
echo   Plug a board, wait for the green DONE line, unplug, repeat.
echo   Press Ctrl-C or close this window when you are finished.
echo ================================================================
echo.

venv\Scripts\python flash_batch.py

echo.
echo ----------------------------------------------------------------
echo  Flashing session ended. You can close this window.
pause
