@echo off
chcp 65001 >nul
cd /d "%~dp0"
rem 360度カメラ -> PICO ブラウザ の中継サーバーを起動し、送信ページを開く
set "PY="
py -3 -c "import sys" >nul 2>nul && set "PY=py -3"
if not defined PY if exist "%USERPROFILE%\miniforge3\python.exe" set "PY=%USERPROFILE%\miniforge3\python.exe"
if not defined PY if exist "%USERPROFILE%\anaconda3\python.exe" set "PY=%USERPROFILE%\anaconda3\python.exe"
if not defined PY python -c "import sys" >nul 2>nul && set "PY=python"
if not defined PY (
  echo Python が見つかりません。Python 3.8 以上を入れてください。
  pause
  exit /b 1
)
echo Python: %PY%
if "%PY%"=="py -3" (
  py -3 pano_server.py --open %*
) else if "%PY%"=="python" (
  python pano_server.py --open %*
) else (
  "%PY%" pano_server.py --open %*
)
pause
