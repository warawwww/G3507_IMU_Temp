@echo off
setlocal

set "PROJECT_ROOT=%~dp0.."
set "ENV_FILE=%PROJECT_ROOT%\.eide\env.ini"

if exist "%ENV_FILE%" (
  if not defined SYSCONFIG_ROOT (
    for /f "usebackq tokens=1,* delims==" %%A in ("%ENV_FILE%") do (
      if /i "%%A"=="SYSCONFIG_ROOT" set "SYSCONFIG_ROOT=%%B"
    )
  )
  if not defined MSPM0_SDK_ROOT (
    for /f "usebackq tokens=1,* delims==" %%A in ("%ENV_FILE%") do (
      if /i "%%A"=="MSPM0_SDK_ROOT" set "MSPM0_SDK_ROOT=%%B"
    )
  )
)

if not defined SYSCONFIG_ROOT (
  echo [ERROR] SYSCONFIG_ROOT is not defined in .eide\env.ini
  pause
  exit /b 1
)

if not defined MSPM0_SDK_ROOT (
  echo [ERROR] MSPM0_SDK_ROOT is not defined in .eide\env.ini
  pause
  exit /b 1
)

if not exist "%SYSCONFIG_ROOT%\sysconfig_gui.bat" (
  echo [ERROR] SysConfig GUI not found: %SYSCONFIG_ROOT%\sysconfig_gui.bat
  pause
  exit /b 1
)

if not exist "%MSPM0_SDK_ROOT%\.metadata\product.json" (
  echo [ERROR] MSPM0 SDK product file not found: %MSPM0_SDK_ROOT%\.metadata\product.json
  pause
  exit /b 1
)

call "%SYSCONFIG_ROOT%\sysconfig_gui.bat" ^
  --product "%MSPM0_SDK_ROOT%\.metadata\product.json" ^
  --script "%PROJECT_ROOT%\cmsis_dsp_empty.syscfg"

exit /b %ERRORLEVEL%
