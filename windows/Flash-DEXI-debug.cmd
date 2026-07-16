@echo off
title DEXI-3 Flasher (DEBUG - one board, verbose)

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

echo.
echo ================================================================
echo   DEXI-3 FC FLASHER  --  DEBUG  (one board, full tool output)
echo ----------------------------------------------------------------
echo   BARE board    : HOLD BOOT while plugging in  ^(Zadig req'd once^)
echo   Flashed board : just plug it in normally
echo ================================================================
echo.

venv\Scripts\python -u flash_batch.py --count 1 --verbose

echo.
echo ================================================================
echo   Flash finished. This window stays open so you can read the
echo   full log above - copy any errors to share. Press a key to close.
echo ================================================================
pause
